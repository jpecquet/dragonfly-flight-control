#!/usr/bin/env python3
"""
Blender script to render dragonfly frames with transparent backgrounds.

Usage:
    blender --background --python post/blender/render_dragonfly.py -- \
        --data frame_data.json --output-dir /tmp/frames --config viz.json

This script renders the dragonfly body and wings at each timestep,
outputting PNG files with transparent backgrounds for compositing.

The frame data JSON file should contain pre-extracted simulation data
(positions and wing vectors) since Blender's Python doesn't have h5py.
"""

import argparse
import json
import math
import sys
from pathlib import Path

# Blender imports (only available when running inside Blender)
try:
    import bpy
    import bmesh
    from mathutils import Matrix, Vector
    BLENDER_AVAILABLE = True
except ImportError:
    BLENDER_AVAILABLE = False

import numpy as np

# Add parent directory to path for imports
script_dir = Path(__file__).parent.parent.parent
sys.path.insert(0, str(script_dir))

from post.constants import get_wing_offsets
from post.wing_geometry import DEFAULT_ROOT_CHORD_RATIO, composite_ellipse_polygon_local


def setup_scene(width, height):
    """
    Configure scene for transparent rendering.

    Args:
        width: Render width in pixels
        height: Render height in pixels
    """
    scene = bpy.context.scene

    # Clear existing objects
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete()

    # Use Workbench renderer (fast, clean look)
    scene.render.engine = 'BLENDER_WORKBENCH'

    scene.render.resolution_x = width
    scene.render.resolution_y = height
    scene.render.resolution_percentage = 100
    scene.render.film_transparent = True
    scene.render.image_settings.file_format = 'PNG'
    scene.render.image_settings.color_mode = 'RGBA'

    # Workbench settings
    scene.display.shading.light = 'STUDIO'
    scene.display.shading.studio_light = 'Default'
    scene.display.shading.color_type = 'MATERIAL'

    # World (no background needed for transparent render)
    if scene.world is None:
        scene.world = bpy.data.worlds.new("World")
    scene.world.use_nodes = False


def setup_camera(elevation, azimuth, center, ortho_scale, width, height,
                 shift_x=0.0, shift_y=0.0):
    """
    Create orthographic camera looking at the specified center point.

    Args:
        elevation: Camera elevation in degrees
        azimuth: Camera azimuth in degrees
        center: 3D center point (viewport center)
        ortho_scale: World units visible in the render
        width: Render width in pixels
        height: Render height in pixels
        shift_x: Camera sensor shift in X (fraction of sensor width)
        shift_y: Camera sensor shift in Y (fraction of sensor height)

    Returns:
        bpy.types.Object: The camera object
    """
    cam_data = bpy.data.cameras.new("Camera")
    cam_data.type = 'ORTHO'
    cam_data.ortho_scale = ortho_scale
    cam_data.shift_x = shift_x
    cam_data.shift_y = shift_y
    cam_data.clip_start = 0.1
    cam_data.clip_end = ortho_scale * 9  # dist * 3

    cam_obj = bpy.data.objects.new("Camera", cam_data)
    bpy.context.scene.collection.objects.link(cam_obj)
    bpy.context.scene.camera = cam_obj

    update_camera_position(cam_obj, elevation, azimuth, center, ortho_scale)

    return cam_obj


def update_camera_position(cam_obj, elevation, azimuth, center, ortho_scale):
    """
    Position camera at spherical coordinates around center, pointing at it.

    Args:
        cam_obj: Camera object to update
        elevation: Camera elevation in degrees
        azimuth: Camera azimuth in degrees
        center: 3D center point
        ortho_scale: World units visible
    """
    elev = math.radians(elevation)
    azim = math.radians(azimuth)
    dist = ortho_scale * 3

    x = dist * math.cos(elev) * math.cos(azim)
    y = dist * math.cos(elev) * math.sin(azim)
    z = dist * math.sin(elev)

    center_vec = Vector(center)
    cam_pos = center_vec + Vector((x, y, z))
    cam_obj.location = cam_pos

    direction = center_vec - cam_pos
    rot_quat = direction.to_track_quat('-Z', 'Y')
    cam_obj.rotation_euler = rot_quat.to_euler()


def setup_lighting():
    """Create basic lighting for the scene."""
    # Key light
    key_data = bpy.data.lights.new("KeyLight", 'SUN')
    key_data.energy = 3.0
    key_obj = bpy.data.objects.new("KeyLight", key_data)
    bpy.context.scene.collection.objects.link(key_obj)
    key_obj.rotation_euler = (math.radians(45), 0, math.radians(-45))

    # Fill light
    fill_data = bpy.data.lights.new("FillLight", 'SUN')
    fill_data.energy = 1.5
    fill_obj = bpy.data.objects.new("FillLight", fill_data)
    bpy.context.scene.collection.objects.link(fill_obj)
    fill_obj.rotation_euler = (math.radians(30), 0, math.radians(135))


def _parse_rgba(color, default_rgba):
    """
    Parse a color token into Blender RGBA tuple.

    Accepts #RRGGBB, [r,g,b], [r,g,b,a], or returns default for unknown input.
    """
    if isinstance(color, str):
        value = color.strip()
        if value.startswith('#') and len(value) == 7:
            try:
                r = int(value[1:3], 16) / 255.0
                g = int(value[3:5], 16) / 255.0
                b = int(value[5:7], 16) / 255.0
                return (r, g, b, 1.0)
            except ValueError:
                return default_rgba
        return default_rgba

    if isinstance(color, (list, tuple)):
        if len(color) == 3:
            try:
                return (float(color[0]), float(color[1]), float(color[2]), 1.0)
            except (TypeError, ValueError):
                return default_rgba
        if len(color) == 4:
            try:
                return (float(color[0]), float(color[1]), float(color[2]), float(color[3]))
            except (TypeError, ValueError):
                return default_rgba

    return default_rgba


_LIGHT_BODY_HEX = "#111111"
_LIGHT_WING_HEX = "#d3d3d3"
_DARK_BODY_HEX = "#f2f5f7"
_DARK_WING_HEX = "#8f9aa6"


def _resolve_material_color(style_cfg, key, default_rgba):
    """
    Resolve a style color token for Blender materials.

    For default light-theme body/wing tokens, use the dark-theme material colors
    so light/dark themes share the same mesh material appearance.
    """
    if not isinstance(style_cfg, dict):
        return _parse_rgba(None, default_rgba)

    value = style_cfg.get(key)
    theme = str(style_cfg.get('theme', '')).strip().lower()

    if theme == 'light' and isinstance(value, str):
        normalized = value.strip().lower()
        if key == 'body_color' and normalized == _LIGHT_BODY_HEX:
            value = _DARK_BODY_HEX
        elif key == 'wing_color' and normalized == _LIGHT_WING_HEX:
            value = _DARK_WING_HEX

    return _parse_rgba(value, default_rgba)


def create_body_material(style_cfg=None):
    """Create material for dragonfly body (Workbench-compatible)."""
    mat = bpy.data.materials.new(name="DragonBody")
    mat.diffuse_color = _resolve_material_color(style_cfg, 'body_color', (0.88, 0.88, 0.88, 1.0))
    return mat


def create_wing_material(style_cfg=None):
    """Create opaque material for wings (Workbench-compatible)."""
    mat = bpy.data.materials.new(name="DragonWing")
    mat.diffuse_color = _resolve_material_color(style_cfg, 'wing_color', (0.92, 0.92, 0.92, 1.0))
    return mat


def create_stroke_plane_material(color=None, alpha=0.22):
    """Create semi-transparent pastel material for stroke-plane guide planes."""
    rgba = list(_parse_rgba(color, (0.96, 0.58, 0.58, 1.0)))
    rgba[3] = float(np.clip(alpha, 0.0, 1.0))
    mat = bpy.data.materials.new(name="StrokePlane")
    mat.diffuse_color = tuple(rgba)
    try:
        mat.blend_method = 'BLEND'
    except Exception:
        pass
    try:
        mat.shadow_method = 'NONE'
    except Exception:
        pass
    return mat


def load_body_mesh(filepath, style_cfg=None):
    """
    Load dragonfly body mesh from OBJ file.

    The OBJ is modeled with origin at the wing attachment center (x=0, y=0, z=0),
    matching the simulation reference point used for body_pos.

    Args:
        filepath: Path to body OBJ file
        style_cfg: Optional style configuration dict

    Returns:
        bpy.types.Object: The body mesh
    """
    bpy.ops.wm.obj_import(filepath=str(filepath))
    body = bpy.context.selected_objects[0]
    body.name = "Body"

    # Anchor origin to the wing attachment center (x=0, y=0, z=0)
    # so that body.location == simulation body_pos
    bpy.context.view_layer.objects.active = body
    saved_cursor = bpy.context.scene.cursor.location.copy()
    bpy.context.scene.cursor.location = (0, 0, 0)
    bpy.ops.object.origin_set(type='ORIGIN_CURSOR')
    bpy.context.scene.cursor.location = saved_cursor

    # Apply material
    body.data.materials.clear()
    mat = create_body_material(style_cfg)
    body.data.materials.append(mat)

    return body


def create_composite_ellipse_wing(name, span, root_chord, style_cfg=None, n_span=32):
    """
    Create a thin wing mesh procedurally from a composite-ellipse planform.

    The local pitching axis is the +Y/-Y span axis at x=0 (quarter-chord station).
    """
    mesh = bpy.data.meshes.new(f"{name}_mesh")
    obj = bpy.data.objects.new(name, mesh)
    bpy.context.scene.collection.objects.link(obj)

    poly = composite_ellipse_polygon_local(
        span=float(span),
        root_chord=float(root_chord),
        n_span=int(n_span),
        span_sign=-1.0,  # keep legacy local convention (tip at -Y)
    )

    bm = bmesh.new()
    verts = [bm.verts.new((float(v[0]), float(v[1]), float(v[2]))) for v in poly]
    bm.verts.ensure_lookup_table()
    bm.faces.new(verts)
    bm.to_mesh(mesh)
    bm.free()

    # Apply wing material
    obj.data.materials.clear()
    mat = create_wing_material(style_cfg)
    obj.data.materials.append(mat)

    return obj


def create_quad_object(name, corners_xyz, material):
    """Create a single-quad mesh object from 4 corners (world/body-frame coords)."""
    mesh = bpy.data.meshes.new(f"{name}_mesh")
    obj = bpy.data.objects.new(name, mesh)
    bpy.context.scene.collection.objects.link(obj)

    bm = bmesh.new()
    verts = [bm.verts.new((float(p[0]), float(p[1]), float(p[2]))) for p in corners_xyz]
    bm.verts.ensure_lookup_table()
    bm.faces.new(verts)
    bm.to_mesh(mesh)
    bm.free()

    obj.data.materials.clear()
    obj.data.materials.append(material)
    return obj


def _normalize_name(name):
    return str(name).strip().lower().replace('-', '_')


def _resolve_stroke_plane_wings(wing_names, requested=None, side='right'):
    """Resolve default fore/hind wing pair for overlays (right side by default)."""
    if isinstance(requested, list) and requested:
        resolved = [str(w) for w in requested if str(w) in wing_names]
        if resolved:
            return resolved
    side = str(side).strip().lower()
    side = 'left' if side == 'left' else 'right'
    fore = None
    hind = None
    for name in wing_names:
        n = _normalize_name(name)
        if fore is None and "fore" in n and side in n:
            fore = name
        if hind is None and "hind" in n and side in n:
            hind = name
    if fore is None or hind is None:
        for name in wing_names:
            n = _normalize_name(name)
            if fore is None and "fore" in n:
                fore = name
            if hind is None and "hind" in n:
                hind = name
    return [w for w in (fore, hind) if w is not None]


def _stroke_plane_quad_corners(wing_name, lb0, gamma, cone, half_width):
    """
    Build a rectangular stroke-plane patch in body coordinates.

    The plane is spanned by:
      - stroke direction in the XZ plane: d = (-cos(gamma), 0, sin(gamma))
      - body lateral axis +/-Y
    and offset by lb0*sin(cone) along the stroke-plane normal in XZ.
    """
    xoff, yoff, zoff = get_wing_offsets(wing_name)
    root = np.array([xoff, yoff, zoff], dtype=float)
    d = np.array([-math.cos(gamma), 0.0, math.sin(gamma)], dtype=float)
    d_norm = np.linalg.norm(d)
    if d_norm <= 1e-12:
        return None
    d = d / d_norm
    n = np.array([math.sin(gamma), 0.0, math.cos(gamma)], dtype=float)
    center = root + (float(lb0) * math.sin(float(cone))) * n
    span = float(lb0) * d
    yv = np.array([0.0, float(half_width), 0.0], dtype=float)
    p00 = center - span - yv
    p01 = center - span + yv
    p11 = center + span + yv
    p10 = center + span - yv
    return [p00, p01, p11, p10]


def create_stroke_plane_objects(body_obj, wing_names, frame_data, blender_cfg):
    """Create optional transparent stroke-plane plane meshes and parent them to body."""
    spec = blender_cfg.get('stroke_planes')
    if not isinstance(spec, dict) or not bool(spec.get('enabled', False)):
        return {}

    wing_lb0 = frame_data.get('wing_lb0', {})
    wing_gamma_mean = frame_data.get('wing_gamma_mean', {})
    wing_cone_angle = frame_data.get('wing_cone_angle', {})
    if not isinstance(wing_lb0, dict) or not isinstance(wing_gamma_mean, dict):
        return {}

    selected_wings = _resolve_stroke_plane_wings(
        wing_names, requested=spec.get('wings'), side=spec.get('side', 'right')
    )
    if not selected_wings:
        return {}

    color = spec.get('color', '#f2a3a3')
    alpha = float(spec.get('alpha', 0.22))
    half_width_default = spec.get('half_width')
    width_scale = float(spec.get('half_width_scale', 0.28))
    material = create_stroke_plane_material(color=color, alpha=alpha)

    plane_objects = {}
    for wname in selected_wings:
        if wname not in wing_lb0 or wname not in wing_gamma_mean:
            continue
        lb0 = float(wing_lb0[wname])
        gamma = float(wing_gamma_mean[wname])
        cone = float(wing_cone_angle.get(wname, 0.0))
        half_width = float(half_width_default) if half_width_default is not None else (width_scale * lb0)
        corners = _stroke_plane_quad_corners(wname, lb0, gamma, cone, half_width)
        if corners is None:
            continue
        obj = create_quad_object(f"StrokePlane_{wname}", corners, material)
        # Body origin is the simulation reference point, so body-frame coords can be parented directly.
        obj.parent = body_obj
        try:
            obj.matrix_parent_inverse = body_obj.matrix_world.inverted()
        except Exception:
            pass
        plane_objects[wname] = obj

    return plane_objects


def create_cone_surface_object(
    name, apex, axis_dir, height, radius, material, n_theta=48, half_y=None
):
    """Create an open cone surface mesh (no base cap) in body/world coordinates."""
    h = float(height)
    r = float(radius)
    if abs(h) <= 1e-8 or r <= 1e-8:
        return None

    a = np.asarray(axis_dir, dtype=float)
    a_norm = float(np.linalg.norm(a))
    if a_norm <= 1e-12:
        return None
    a = a / a_norm

    center = np.asarray(apex, dtype=float) + h * a
    ref = np.array([0.0, 1.0, 0.0], dtype=float)
    if abs(float(np.dot(ref, a))) > 0.95:
        ref = np.array([1.0, 0.0, 0.0], dtype=float)
    u = np.cross(a, ref)
    u_norm = float(np.linalg.norm(u))
    if u_norm <= 1e-12:
        return None
    u = u / u_norm
    v = np.cross(a, u)

    n_theta = max(8, int(n_theta))
    thetas = np.linspace(0.0, 2.0 * math.pi, n_theta, endpoint=False)
    ring = [
        center + r * (math.cos(th) * u + math.sin(th) * v)
        for th in thetas
    ]

    mesh = bpy.data.meshes.new(f"{name}_mesh")
    obj = bpy.data.objects.new(name, mesh)
    bpy.context.scene.collection.objects.link(obj)

    bm = bmesh.new()
    v_apex = bm.verts.new((float(apex[0]), float(apex[1]), float(apex[2])))
    v_ring = [bm.verts.new((float(p[0]), float(p[1]), float(p[2]))) for p in ring]
    bm.verts.ensure_lookup_table()
    half_y_mode = None if half_y is None else str(half_y).strip().lower()
    y_cut = float(apex[1])
    for i in range(len(v_ring)):
        v0 = v_ring[i]
        v1 = v_ring[(i + 1) % len(v_ring)]
        if half_y_mode in ("positive", "negative"):
            y0 = float(ring[i][1])
            y1 = float(ring[(i + 1) % len(ring)][1])
            eps = 1e-9
            if half_y_mode == "positive" and (y0 < y_cut - eps or y1 < y_cut - eps):
                continue
            if half_y_mode == "negative" and (y0 > y_cut + eps or y1 > y_cut + eps):
                continue
        try:
            bm.faces.new((v_apex, v0, v1))
        except ValueError:
            pass
    bm.to_mesh(mesh)
    bm.free()

    obj.data.materials.clear()
    obj.data.materials.append(material)
    return obj


def create_stroke_cone_objects(body_obj, wing_names, frame_data, blender_cfg):
    """Create optional transparent wing-cone surfaces and parent them to body."""
    spec = blender_cfg.get('stroke_cones')
    if not isinstance(spec, dict) or not bool(spec.get('enabled', False)):
        return {}

    wing_lb0 = frame_data.get('wing_lb0', {})
    wing_gamma_mean = frame_data.get('wing_gamma_mean', {})
    wing_cone_angle = frame_data.get('wing_cone_angle', {})
    if not isinstance(wing_lb0, dict) or not isinstance(wing_gamma_mean, dict):
        return {}

    selected_wings = _resolve_stroke_plane_wings(
        wing_names, requested=spec.get('wings'), side=spec.get('side', 'right')
    )
    if not selected_wings:
        return {}

    color = spec.get('color', '#ff6b6b')
    alpha = float(spec.get('alpha', 0.28))
    n_theta = int(spec.get('n_theta', 48))
    half_y = spec.get('half_y')
    material = create_stroke_plane_material(color=color, alpha=alpha)

    cone_objects = {}
    for wname in selected_wings:
        if wname not in wing_lb0 or wname not in wing_gamma_mean:
            continue
        lb0 = float(wing_lb0[wname])
        gamma = float(wing_gamma_mean[wname])
        cone = float(wing_cone_angle.get(wname, 0.0))
        xoff, yoff, zoff = get_wing_offsets(wname)
        apex = np.array([xoff, yoff, zoff], dtype=float)
        axis = np.array([math.sin(gamma), 0.0, math.cos(gamma)], dtype=float)
        height = lb0 * math.sin(cone)
        radius = abs(lb0 * math.cos(cone))
        obj = create_cone_surface_object(
            f"StrokeCone_{wname}",
            apex=apex,
            axis_dir=axis,
            height=height,
            radius=radius,
            material=material,
            n_theta=n_theta,
            half_y=half_y,
        )
        if obj is None:
            continue
        obj.parent = body_obj
        try:
            obj.matrix_parent_inverse = body_obj.matrix_world.inverted()
        except Exception:
            pass
        cone_objects[wname] = obj

    return cone_objects


def transform_wing(wing_obj, origin, e_r, e_c, mirror_y=True):
    """
    Transform wing mesh to world coordinates.

    Args:
        wing_obj: Wing mesh object
        origin: Wing attachment point (3D array)
        e_r: Span direction vector (toward tip)
        e_c: Chord direction vector
        mirror_y: If True, mirror mesh Y (base mesh has tip at -Y)
    """
    e_r = np.array(e_r)
    e_c = np.array(e_c)
    e_n = np.cross(e_c, e_r)

    # Rotation matrix: local [X, Y, Z] -> world [e_c, e_r, e_n]
    rotation = np.column_stack([e_c, e_r, e_n])

    # Convert to Blender Matrix
    rot_mat = Matrix([
        [rotation[0, 0], rotation[0, 1], rotation[0, 2], 0],
        [rotation[1, 0], rotation[1, 1], rotation[1, 2], 0],
        [rotation[2, 0], rotation[2, 1], rotation[2, 2], 0],
        [0, 0, 0, 1]
    ])

    # Mirror matrix (flip Y)
    if mirror_y:
        mirror_mat = Matrix([
            [1, 0, 0, 0],
            [0, -1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
    else:
        mirror_mat = Matrix.Identity(4)

    # Translation
    trans_mat = Matrix.Translation(Vector(origin))

    # Apply: first mirror, then rotate, then translate
    wing_obj.matrix_world = trans_mat @ rot_mat @ mirror_mat


def capture_wing_deform_data(wing_obj, span):
    """Capture local-space wing vertices and per-vertex span stations."""
    verts = wing_obj.data.vertices
    base = np.array([[v.co.x, v.co.y, v.co.z] for v in verts], dtype=float)
    span_abs = max(abs(float(span)), 1e-12)
    eta = np.clip(np.abs(base[:, 1]) / span_abs, 0.0, 1.0)
    return {
        'base_vertices': base,
        'eta': eta,
    }


def apply_wing_twist(wing_obj, deform_data, twist_model, frame_idx, wing_name=None):
    """
    Deform wing mesh in local coordinates to visualize spanwise pitch twist.

    The deformation rotates each vertex around local +Y by
    delta_psi(eta,t) = (scale(eta)-1) * psi_h1(t), matching the simulator model.
    """
    base = deform_data['base_vertices']
    eta = deform_data['eta']
    coords = base

    if twist_model:
        ref_eta = float(twist_model.get('ref_eta', 0.0))
        root_coeff = float(twist_model.get('root_coeff', 0.0))
        ref_coeff = float(twist_model.get('ref_coeff', 0.0))
        psi_h1_series = twist_model.get('psi_h1', [])
        if ref_eta > 0.0 and abs(ref_coeff) > 1e-12 and frame_idx < len(psi_h1_series):
            psi_h1_value = float(psi_h1_series[frame_idx])
            coeff_eta = ((ref_coeff - root_coeff) * (eta / ref_eta)) + root_coeff
            scale_eta = coeff_eta / ref_coeff
            delta = (scale_eta - 1.0) * psi_h1_value
            # The simulator's left/right pitch conventions use opposite signs
            # for increasing psi. Match that convention so mirrored wings
            # receive mirrored spanwise twist in world coordinates.
            if isinstance(wing_name, str) and "left" in wing_name:
                delta = -delta
            cos_delta = np.cos(delta)
            sin_delta = np.sin(delta)

            x0 = base[:, 0]
            z0 = base[:, 2]
            x_new = x0 * cos_delta + z0 * sin_delta
            z_new = -x0 * sin_delta + z0 * cos_delta
            coords = np.column_stack([x_new, base[:, 1], z_new])

    wing_obj.data.vertices.foreach_set("co", coords.reshape(-1))
    wing_obj.data.update()


def render_frame(frame_idx, output_path):
    """
    Render current frame to file.

    Args:
        frame_idx: Frame index
        output_path: Output directory path
    """
    filepath = Path(output_path) / f"frame_{frame_idx:06d}.png"
    bpy.context.scene.render.filepath = str(filepath)
    bpy.ops.render.render(write_still=True)


def main():
    """Main entry point when running in Blender."""
    # Parse arguments after "--"
    argv = sys.argv
    if "--" in argv:
        argv = argv[argv.index("--") + 1:]
    else:
        argv = []

    parser = argparse.ArgumentParser(description="Render dragonfly frames")
    parser.add_argument("--data", required=True, help="Frame data JSON file")
    parser.add_argument("--output-dir", required=True, help="Output directory for frames")
    parser.add_argument("--config", required=True, help="Configuration JSON file")
    parser.add_argument("--start", type=int, default=0, help="Start frame index")
    parser.add_argument("--end", type=int, default=-1, help="End frame index (-1 for all)")
    args = parser.parse_args(argv)

    # Load configuration
    with open(args.config, 'r') as f:
        config = json.load(f)

    camera_cfg = config.get('camera', {})
    viewport_cfg = config.get('viewport', {})
    blender_cfg = config.get('blender', {})
    style_cfg = config.get('style', {})

    # Load pre-extracted frame data
    with open(args.data, 'r') as f:
        frame_data = json.load(f)

    n_frames = frame_data['n_frames']
    states = frame_data['states']  # List of [x, y, z, ux, uy, uz]
    wing_names = frame_data['wing_names']
    wing_vectors = frame_data['wing_vectors']  # {wname: {e_r: [...], e_c: [...]}}
    wing_lb0 = frame_data.get('wing_lb0', {})
    wing_twist_h1 = frame_data.get('wing_twist_h1', {})

    # Determine frame range
    start_frame = args.start
    end_frame = args.end if args.end >= 0 else n_frames

    # Create output directory
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    # Get render resolution from camera config (match matplotlib output)
    figsize_width = camera_cfg.get('figsize_width', 6.5)
    figsize_height = camera_cfg.get('figsize_height', 4.333333333333333)
    dpi = camera_cfg.get('dpi', 300)
    render_width = int(figsize_width * dpi)
    render_height = int(figsize_height * dpi)

    # Get viewport extent and scale factor
    viewport_extent = viewport_cfg.get('extent', 2.0)
    extent_xyz = viewport_cfg.get('extent_xyz')
    if isinstance(extent_xyz, list) and len(extent_xyz) == 3:
        viewport_extent = max(float(extent_xyz[0]), float(extent_xyz[1]), float(extent_xyz[2]))
    viewport_center = viewport_cfg.get('center', [0, 0, 0])
    scale_factor = blender_cfg.get('scale_factor', 1.8)

    # Use pre-computed ortho_scale if available, otherwise fall back to scale_factor
    ortho_scale = blender_cfg.get('computed_ortho_scale')
    if ortho_scale is None:
        # Fallback: larger scale_factor = smaller dragonfly
        ortho_scale = viewport_extent * scale_factor

    # Get center offset (from matplotlib projection measurement)
    # This tells us how many pixels the matplotlib projection shifts the
    # viewport center away from the render center
    center_offset_x = blender_cfg.get('center_offset_x', 0.0)
    center_offset_y = blender_cfg.get('center_offset_y', 0.0)

    # Convert pixel offset to camera shift (fraction of sensor size)
    # shift_x/y are in fractions of the sensor dimension
    # Positive shift moves the image in the positive direction
    shift_x = -center_offset_x / render_width
    shift_y = -center_offset_y / render_height

    # Setup scene with full resolution
    setup_scene(render_width, render_height)

    # Setup camera at viewport center (fixed position for all frames)
    elevation = camera_cfg.get('elevation', 30.0)
    azimuth = camera_cfg.get('azimuth', -60.0)
    cam_obj = setup_camera(
        elevation, azimuth, viewport_center, ortho_scale,
        render_width, render_height, shift_x, shift_y
    )

    setup_lighting()

    # Load body mesh
    assets_dir = script_dir / "assets"
    body = load_body_mesh(assets_dir / "dragonfly_body.obj", style_cfg=style_cfg)
    _stroke_plane_objects = create_stroke_plane_objects(body, wing_names, frame_data, blender_cfg)
    _stroke_cone_objects = create_stroke_cone_objects(body, wing_names, frame_data, blender_cfg)

    # Wing configuration
    wing_info = {}
    for wname in wing_names:
        xoffset, yoffset, zoffset = get_wing_offsets(wname)
        span = float(wing_lb0.get(wname, 0.75))
        root_chord = DEFAULT_ROOT_CHORD_RATIO * span
        wing_info[wname] = {
            'offset': np.array([xoffset, yoffset, zoffset]),
            'span': span,
            'root_chord': root_chord,
        }

    # Create wing meshes
    wing_objects = {}
    wing_deform_data = {}
    for wname, info in wing_info.items():
        wing_obj = create_composite_ellipse_wing(
            wname,
            span=info['span'],
            root_chord=info['root_chord'],
            style_cfg=style_cfg,
            n_span=32,
        )
        wing_objects[wname] = wing_obj
        wing_deform_data[wname] = capture_wing_deform_data(wing_obj, info['span'])

    # Render frames (camera stays fixed at viewport center)
    for frame_idx in range(start_frame, end_frame):
        # Get body position for this frame
        body_pos = states[frame_idx][0:3]

        # Move body to world position
        body.location = Vector(body_pos)

        # Position wings relative to body
        for wname, wing_obj in wing_objects.items():
            info = wing_info[wname]
            # Wing origin is body position + offset
            origin = np.array(body_pos) + info['offset']
            e_r = wing_vectors[wname]['e_r'][frame_idx]
            e_c = wing_vectors[wname]['e_c'][frame_idx]
            apply_wing_twist(
                wing_obj,
                wing_deform_data[wname],
                wing_twist_h1.get(wname),
                frame_idx,
                wing_name=wname,
            )
            transform_wing(wing_obj, origin, e_r, e_c)

        # Render
        render_frame(frame_idx, output_dir)

        if (frame_idx - start_frame) % 100 == 0:
            print(f"Rendered frame {frame_idx}/{end_frame}")

    print(f"Rendering complete. Frames saved to {output_dir}")


if __name__ == "__main__":
    if BLENDER_AVAILABLE:
        main()
    else:
        print("This script must be run inside Blender:")
        print("  blender --background --python render_dragonfly.py -- --help")
        sys.exit(1)

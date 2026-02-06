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
    Create orthographic camera following the dragonfly position.

    The camera is positioned to look at the specified center point,
    with the ortho_scale determining visible world extent.

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
    # Create camera
    cam_data = bpy.data.cameras.new("Camera")
    cam_data.type = 'ORTHO'
    cam_data.ortho_scale = ortho_scale

    # Apply sensor shift to match matplotlib's projection offset
    cam_data.shift_x = shift_x
    cam_data.shift_y = shift_y

    cam_obj = bpy.data.objects.new("Camera", cam_data)
    bpy.context.scene.collection.objects.link(cam_obj)
    bpy.context.scene.camera = cam_obj

    # Convert angles to radians
    elev = math.radians(elevation)
    azim = math.radians(azimuth)

    # Distance (for positioning, affects clipping)
    dist = ortho_scale * 3

    # Compute camera position (spherical coordinates) relative to center
    x = dist * math.cos(elev) * math.cos(azim)
    y = dist * math.cos(elev) * math.sin(azim)
    z = dist * math.sin(elev)

    center_vec = Vector(center)
    cam_pos = center_vec + Vector((x, y, z))
    cam_obj.location = cam_pos

    # Point at center
    direction = center_vec - cam_pos
    rot_quat = direction.to_track_quat('-Z', 'Y')
    cam_obj.rotation_euler = rot_quat.to_euler()

    # Clip planes
    cam_data.clip_start = 0.1
    cam_data.clip_end = dist * 3

    return cam_obj


def update_camera_position(cam_obj, elevation, azimuth, center, ortho_scale):
    """
    Update camera position to follow a new center point.

    Args:
        cam_obj: Camera object to update
        elevation: Camera elevation in degrees
        azimuth: Camera azimuth in degrees
        center: New 3D center point
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

    # Point at center
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


def create_ellipsoid(name, radii, location):
    """
    Create an ellipsoid mesh.

    Args:
        name: Object name
        radii: (rx, ry, rz) radii
        location: (x, y, z) center position

    Returns:
        bpy.types.Object: The mesh object
    """
    bpy.ops.mesh.primitive_uv_sphere_add(
        radius=1.0,
        segments=24,
        ring_count=16,
        location=location
    )
    obj = bpy.context.active_object
    obj.name = name
    obj.scale = radii
    bpy.ops.object.transform_apply(scale=True)
    return obj


def create_cylinder(name, radius, height, location, direction=(1, 0, 0)):
    """
    Create a cylinder mesh.

    Args:
        name: Object name
        radius: Cylinder radius
        height: Cylinder height
        location: Center position
        direction: Axis direction

    Returns:
        bpy.types.Object: The mesh object
    """
    # Default cylinder is along Z, we want it along X
    bpy.ops.mesh.primitive_cylinder_add(
        radius=radius,
        depth=height,
        location=location
    )
    obj = bpy.context.active_object
    obj.name = name

    # Rotate to align with X axis
    if direction == (1, 0, 0):
        obj.rotation_euler = (0, math.radians(90), 0)
        bpy.ops.object.transform_apply(rotation=True)

    return obj


def create_body_material():
    """Create material for dragonfly body (Workbench-compatible)."""
    mat = bpy.data.materials.new(name="DragonBody")
    mat.diffuse_color = (0.88, 0.88, 0.88, 1.0)  # light gray
    return mat


def create_wing_material():
    """Create opaque material for wings (Workbench-compatible)."""
    mat = bpy.data.materials.new(name="DragonWing")
    mat.diffuse_color = (0.92, 0.92, 0.92, 1.0)  # lighter gray, opaque
    return mat


def create_body_mesh():
    """
    Create dragonfly body mesh at origin.

    Returns:
        bpy.types.Object: The body mesh
    """
    # Body segment lengths
    Lh = 0.1
    Lt = 0.25
    La = 1 - Lh - Lt  # 0.65

    # Body segment radii
    Rh = 0.07
    Rt = 0.05
    Ra = 0.03

    # Wing attachment positions
    dw = 0.06
    fw_x0 = dw / 2
    hw_x0 = -dw / 2

    # Body segment centers
    a_xc = hw_x0 - La / 2
    t_xc = a_xc + La / 2 + Lt / 2
    h_xc = t_xc + Lt / 2 + Lh / 2

    # Create body parts
    head = create_ellipsoid("Head", (Lh / 2, Rh, Lh / 2), (h_xc, 0, 0))
    thorax = create_ellipsoid("Thorax", (Lt / 2, Rt, Rt * 1.5), (t_xc, 0, 0))
    abdomen = create_ellipsoid("Abdomen", (La / 2, Ra, Ra), (a_xc, 0, 0))
    connector = create_cylinder(
        "Connector", Ra, t_xc - a_xc,
        ((t_xc + a_xc) / 2, 0, 0)
    )

    # Join all parts into single mesh
    bpy.ops.object.select_all(action='DESELECT')
    for obj in [head, thorax, abdomen, connector]:
        obj.select_set(True)
    bpy.context.view_layer.objects.active = head
    bpy.ops.object.join()

    body = bpy.context.active_object
    body.name = "Body"

    # Recenter origin to the wing attachment center (x=0, y=0, z=0)
    # This ensures body.location corresponds to the center of mass position
    # used in the simulation trajectory
    saved_cursor = bpy.context.scene.cursor.location.copy()
    bpy.context.scene.cursor.location = (0, 0, 0)
    bpy.ops.object.origin_set(type='ORIGIN_CURSOR')
    bpy.context.scene.cursor.location = saved_cursor

    # Apply material
    mat = create_body_material()
    body.data.materials.append(mat)

    return body


def load_wing_mesh(filepath, name):
    """
    Load wing mesh from OBJ file.

    Args:
        filepath: Path to OBJ file
        name: Name for the imported object

    Returns:
        bpy.types.Object: The imported mesh
    """
    bpy.ops.wm.obj_import(filepath=str(filepath))
    obj = bpy.context.selected_objects[0]
    obj.name = name

    # Apply wing material
    obj.data.materials.clear()
    mat = create_wing_material()
    obj.data.materials.append(mat)

    return obj


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

    # Load pre-extracted frame data
    with open(args.data, 'r') as f:
        frame_data = json.load(f)

    n_frames = frame_data['n_frames']
    states = frame_data['states']  # List of [x, y, z, ux, uy, uz]
    wing_names = frame_data['wing_names']
    wing_vectors = frame_data['wing_vectors']  # {wname: {e_r: [...], e_c: [...]}}

    # Determine frame range
    start_frame = args.start
    end_frame = args.end if args.end >= 0 else n_frames

    # Create output directory
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    # Get render resolution from camera config (match matplotlib output)
    figsize_width = camera_cfg.get('figsize_width', 6.0)
    figsize_height = camera_cfg.get('figsize_height', 4.0)
    dpi = camera_cfg.get('dpi', 300)
    render_width = int(figsize_width * dpi)
    render_height = int(figsize_height * dpi)

    # Get viewport extent and scale factor
    viewport_extent = viewport_cfg.get('extent', 2.0)
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

    # Create body (will be moved each frame)
    body = create_body_mesh()

    # Load wing meshes
    assets_dir = script_dir / "assets"
    wing_templates = {}
    for base_name in ['fore', 'hind']:
        filepath = assets_dir / f"{base_name}wing.obj"
        if filepath.exists():
            mesh = load_wing_mesh(filepath, f"{base_name}_template")
            wing_templates[base_name] = mesh
            # Hide template
            mesh.hide_render = True
            mesh.hide_viewport = True
        else:
            print(f"Warning: Wing mesh not found: {filepath}")

    # Wing configuration
    dw = 0.06
    fw_x0 = dw / 2
    hw_x0 = -dw / 2

    wing_info = {}
    for wname in wing_names:
        base = 'fore' if 'fore' in wname else 'hind'
        xoffset = fw_x0 if 'fore' in wname else hw_x0
        yoffset = -0.02 if 'right' in wname else 0.02
        wing_info[wname] = {
            'base': base,
            'offset': np.array([xoffset, yoffset, 0]),
        }

    # Create wing instances
    wing_objects = {}
    for wname, info in wing_info.items():
        if info['base'] in wing_templates:
            # Duplicate template
            template = wing_templates[info['base']]
            wing_copy = template.copy()
            wing_copy.data = template.data.copy()
            wing_copy.name = wname
            wing_copy.hide_render = False
            wing_copy.hide_viewport = False
            bpy.context.scene.collection.objects.link(wing_copy)
            wing_objects[wname] = wing_copy

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

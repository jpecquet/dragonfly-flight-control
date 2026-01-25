"""
3D visualization of dragonfly body and wings using PyVista.
"""

from pathlib import Path

import numpy as np
import pyvista as pv

# Asset directory relative to this file
ASSETS_DIR = Path(__file__).parent.parent / "assets"

# Base wing mesh filenames (modeled as right wings, mirrored for left)
WING_MESHES = {
    'fore': 'forewing.obj',
    'hind': 'hindwing.obj',
}


def plot_dragonfly(states, wing_vectors, params, outfile):
    """
    Animate the dragonfly simulation.

    Args:
        states: list of state arrays [x, y, z, ux, uy, uz]
        wing_vectors: list of wing vector dicts (one per timestep)
        params: parameter dict
        outfile: output filename (.mp4)
    """
    p = params

    # Get per-wing lb0 values
    wing_lb0 = p.get('wing_lb0', {})

    # Validate exactly 4 wings with expected names
    expected_wings = {'fore_left', 'fore_right', 'hind_left', 'hind_right'}
    wing_names = set(wing_vectors[0].keys())

    if wing_names != expected_wings:
        missing = expected_wings - wing_names
        unexpected = wing_names - expected_wings
        msg = "Dragonfly visualization requires exactly 4 wings: fore_left, fore_right, hind_left, hind_right."
        if missing:
            msg += f" Missing: {missing}."
        if unexpected:
            msg += f" Unexpected: {unexpected}."
        raise ValueError(msg)

    wing_names = list(wing_vectors[0].keys())

    # Calculate average lb0 for fore and hind wings
    fore_lb0 = [wing_lb0.get(w, 0.75) for w in wing_names if 'fore' in w]
    hind_lb0 = [wing_lb0.get(w, 0.75) for w in wing_names if 'hind' in w]
    avg_fore = sum(fore_lb0) / len(fore_lb0) if fore_lb0 else 0.75
    avg_hind = sum(hind_lb0) / len(hind_lb0) if hind_lb0 else 0.75

    # Body segment lengths
    Lh = 0.1
    Lt = 0.25
    La = 1 - Lh - Lt

    # Body segment radii
    Rh = 0.07
    Rt = 0.05
    Ra = 0.03

    # Wing attachment positions
    dw = 0.2 * (avg_fore + avg_hind) / 2
    fw_x0 = dw / 2
    hw_x0 = -dw / 2

    # Body segment centers
    a_xc = hw_x0 - La / 2
    t_xc = a_xc + La / 2 + Lt / 2
    h_xc = t_xc + Lt / 2 + Lh / 2

    # Determine wing names and their properties
    wing_info = []
    for wname in wing_names:
        lb0 = wing_lb0.get(wname, 0.75)
        if 'fore' in wname:
            offset = fw_x0
        else:
            offset = hw_x0
        wing_info.append((wname, offset, lb0))

    # Pre-create body mesh at origin (will be copied and translated each frame)
    body_template = _make_body_template(h_xc, t_xc, a_xc, Lh, Lt, La, Rh, Rt, Ra)

    # Load wing meshes
    wing_templates = _load_wing_meshes()

    # Create plotter
    plotter = pv.Plotter(off_screen=True, window_size=[800, 800])
    plotter.set_background('white')

    plotter.open_movie(outfile, framerate=30)

    for i in range(len(states)):
        plotter.clear_actors()

        xb = states[i][0:3]
        v = wing_vectors[i]

        # Translate body template
        body = body_template.copy()
        body.translate(xb, inplace=True)
        plotter.add_mesh(body, color='tan', smooth_shading=True)

        # Collect all wings into one mesh
        all_wings = None
        lift_lines = []
        drag_lines = []

        for wname, offset, lb0 in wing_info:
            origin = xb + np.array([offset, 0, 0])
            wing_mesh = _transform_wing_mesh(wing_templates[wname], origin, v[wname])

            if all_wings is None:
                all_wings = wing_mesh
            else:
                all_wings = all_wings + wing_mesh

            # Collect force vector endpoints
            cp = origin + 0.67 * lb0 * v[wname]['e_r']
            lift_mag = np.linalg.norm(v[wname]['lift'])
            drag_mag = np.linalg.norm(v[wname]['drag'])
            if lift_mag > 1e-10:
                lift_lines.append((cp, cp + 0.05 * v[wname]['lift']))
            if drag_mag > 1e-10:
                drag_lines.append((cp, cp + 0.05 * v[wname]['drag']))

        # Add all wings as single mesh
        plotter.add_mesh(all_wings, color='lightblue', opacity=0.4,
                         smooth_shading=True, show_edges=True, edge_color='gray')

        # Add force lines as batched meshes
        if lift_lines:
            plotter.add_mesh(_make_lines(lift_lines), color='blue', line_width=3)
        if drag_lines:
            plotter.add_mesh(_make_lines(drag_lines), color='red', line_width=3)

        # Camera and title
        ux, uz = states[i][3], states[i][5]
        plotter.add_text(f"ux = {ux:.2f}, uz = {uz:.2f}",
                         position='upper_edge', font_size=12)

        # Set camera to follow body
        plotter.camera.position = (xb[0] + 1.5, xb[1] + 1.5, xb[2] + 0.8)
        plotter.camera.focal_point = tuple(xb)
        plotter.camera.up = (0, 0, 1)

        plotter.write_frame()

    plotter.close()


def _load_wing_meshes():
    """Load wing meshes from assets directory.

    Base meshes are modeled as right wings. The same mesh is used for both
    left and right; mirroring is handled in _transform_wing_mesh.
    """
    meshes = {}
    for base_name, filename in WING_MESHES.items():
        path = ASSETS_DIR / filename
        if not path.exists():
            raise FileNotFoundError(f"Wing mesh not found: {path}")
        mesh = pv.read(str(path))
        # Use same base mesh for both left and right
        meshes[f'{base_name}_left'] = mesh
        meshes[f'{base_name}_right'] = mesh
    return meshes


def _transform_wing_mesh(template, origin, vecs):
    """
    Transform wing mesh from local coordinates to world coordinates.

    Mesh convention (Blender export with forward=X, up=Z):
    - +X along chord (leading to trailing edge)
    - -Y along span (root to tip, since meshes are modeled as right wings)
    - +Z normal (dorsal)

    All meshes are mirrored so local +Y points toward tip, then rotation
    maps local +Y to world e_r (which points outward for both sides).
    """
    e_r = vecs['e_r']  # span direction
    e_c = vecs['e_c']  # chord direction
    e_n = np.cross(e_c, e_r)  # normal, right-hand rule: chord × span

    # Rotation matrix: local [X, Y, Z] -> world [e_c, e_r, e_n]
    rotation = np.column_stack([e_c, e_r, e_n])

    wing = template.copy()
    points = wing.points.copy()

    # Mirror Y so local +Y points toward tip (base mesh has tip at -Y)
    points[:, 1] = -points[:, 1]

    # Suppress spurious BLAS warnings on large matmul (results are valid)
    with np.errstate(divide='ignore', over='ignore', invalid='ignore'):
        wing.points = (rotation @ points.T).T + origin
    return wing


def _make_body_template(h_xc, t_xc, a_xc, Lh, Lt, La, Rh, Rt, Ra):
    """Create body mesh centered at origin (will be translated per frame)."""
    # Head
    head = pv.ParametricEllipsoid(Lh / 2, Rh, Lh / 2)
    head.translate([h_xc, 0, 0], inplace=True)

    # Thorax
    thorax = pv.ParametricEllipsoid(Lt / 2, Rt, Rt * 1.5)
    thorax.translate([t_xc, 0, 0], inplace=True)

    # Abdomen
    abdomen = pv.ParametricEllipsoid(La / 2, Ra, Ra)
    abdomen.translate([a_xc, 0, 0], inplace=True)

    # Connector cylinder between thorax and abdomen
    connector = pv.Cylinder(
        center=[(t_xc + a_xc) / 2, 0, 0],
        direction=[1, 0, 0],
        radius=Ra,
        height=t_xc - a_xc
    )

    # Combine all body parts
    body = head + thorax + abdomen + connector
    return body


def _make_lines(segments):
    """Create a single PolyData with multiple line segments."""
    points = []
    lines = []
    for start, end in segments:
        idx = len(points)
        points.extend([start, end])
        lines.extend([2, idx, idx + 1])
    return pv.PolyData(np.array(points), lines=lines)

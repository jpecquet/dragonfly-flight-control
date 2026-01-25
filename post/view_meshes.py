#!/usr/bin/env python3
"""
View wing meshes from assets directory.

Usage:
    python -m post.view_meshes [output.png]
"""
import sys
from pathlib import Path

import pyvista as pv

ASSETS_DIR = Path(__file__).parent.parent / "assets"

# Base meshes (modeled as right wings)
WING_MESHES = {
    'fore': 'forewing.obj',
    'hind': 'hindwing.obj',
}

# Wing display config: (x_offset, color)
WING_CONFIG = {
    'fore': (0.1, 'lightblue', 'lightgreen'),   # x_offset, left_color, right_color
    'hind': (-0.1, 'lightyellow', 'lightcoral'),
}


def main():
    output_file = sys.argv[1] if len(sys.argv) > 1 else None

    plotter = pv.Plotter(off_screen=(output_file is not None))
    plotter.set_background('white')

    for base_name, filename in WING_MESHES.items():
        path = ASSETS_DIR / filename
        if not path.exists():
            print(f"Warning: {path} not found")
            continue

        base_mesh = pv.read(str(path))
        x_off, left_color, right_color = WING_CONFIG[base_name]

        # Right wing (original mesh)
        right = base_mesh.copy()
        right.translate([x_off, 0, 0], inplace=True)
        plotter.add_mesh(right, color=right_color, opacity=0.7,
                         show_edges=True, edge_color='gray', label=f'{base_name}_right')

        # Left wing (mirror Y)
        left = base_mesh.copy()
        left.points[:, 1] = -left.points[:, 1]
        left.translate([x_off, 0, 0], inplace=True)
        plotter.add_mesh(left, color=left_color, opacity=0.7,
                         show_edges=True, edge_color='gray', label=f'{base_name}_left')

    plotter.add_axes()
    plotter.add_legend()
    plotter.camera_position = [(1.5, 1.5, 1.0), (0, 0, 0), (0, 0, 1)]

    if output_file:
        plotter.screenshot(output_file)
        print(f"Saved: {output_file}")
    else:
        plotter.show()

    plotter.close()


if __name__ == "__main__":
    main()

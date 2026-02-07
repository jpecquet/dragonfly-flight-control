#!/usr/bin/env python3
"""
Visualize trajectory tracking simulation output.

Shows the dragonfly, target trajectory, and actual trajectory with tracking error.

Usage:
    python -m post.plot_tracking <input.h5> [output.mp4] [options]

Options:
    --renderer hybrid|pyvista  Rendering backend (default: hybrid)
    --config <file.json>       Custom visualization config (hybrid only)
"""
import argparse
import sys
from pathlib import Path

import numpy as np

from post.constants import (
    Lh, Lt, La, Rh, Rt, Ra,
    H_XC, T_XC, A_XC,
    FORCE_CENTER_FRACTION, FORCE_SCALE, FORCE_THRESHOLD,
    DEFAULT_LB0, get_wing_info,
)
from post.io import read_tracking


def plot_tracking_pyvista(states, wing_vectors, params, controller, outfile,
                          trail_length=100, show_target_trail=True):
    """
    Animate trajectory tracking simulation using PyVista.

    Args:
        states: list of state arrays [x, y, z, ux, uy, uz]
        wing_vectors: list of wing vector dicts (one per timestep)
        params: parameter dict
        controller: dict with target_position, position_error arrays
        outfile: output filename (.mp4)
        trail_length: number of past positions to show in trail
        show_target_trail: if True, show full target trajectory
    """
    import pyvista as pv
    from post.dragonfly import (
        _load_wing_meshes, _transform_wing_mesh, _make_body_template, _make_lines
    )

    p = params
    wing_lb0 = p.get('wing_lb0', {})

    # Validate 4 wings
    expected_wings = {'fore_left', 'fore_right', 'hind_left', 'hind_right'}
    wing_names = set(wing_vectors[0].keys())
    if wing_names != expected_wings:
        missing = expected_wings - wing_names
        unexpected = wing_names - expected_wings
        msg = "Tracking visualization requires exactly 4 wings."
        if missing:
            msg += f" Missing: {missing}."
        if unexpected:
            msg += f" Unexpected: {unexpected}."
        raise ValueError(msg)

    wing_names = list(wing_vectors[0].keys())

    wing_info = get_wing_info(wing_vectors[0], wing_lb0)

    # Pre-create templates
    body_template = _make_body_template(H_XC, T_XC, A_XC, Lh, Lt, La, Rh, Rt, Ra)
    wing_templates = _load_wing_meshes()

    # Extract trajectory data
    target_positions = controller['target_position']
    position_errors = controller['position_error']

    # Compute actual positions from states
    actual_positions = states[:, 0:3]

    # Create full target trajectory line (if showing)
    target_trajectory = None
    if show_target_trail:
        # Find unique points (for hover, they're all the same)
        unique_targets = np.unique(target_positions, axis=0)
        if len(unique_targets) == 1:
            # Hover: show as a single point/sphere
            target_trajectory = pv.Sphere(radius=0.05, center=unique_targets[0])
        else:
            # Moving trajectory: show as a line
            target_trajectory = pv.Spline(target_positions, n_points=len(target_positions))

    # Calculate scene bounds for camera
    all_positions = np.vstack([actual_positions, target_positions])
    center = all_positions.mean(axis=0)
    extent = np.max(all_positions.max(axis=0) - all_positions.min(axis=0))
    cam_dist = max(2.0, extent * 1.5)

    # Create plotter
    plotter = pv.Plotter(off_screen=True, window_size=[800, 600])
    plotter.set_background('white')
    plotter.open_movie(outfile, framerate=30)

    n_frames = len(states)

    for i in range(n_frames):
        plotter.clear_actors()

        xb = states[i][0:3]
        v = wing_vectors[i]
        target = target_positions[i]
        error = position_errors[i]
        error_mag = np.linalg.norm(error)

        # --- Draw target trajectory ---
        if target_trajectory is not None:
            if isinstance(target_trajectory, pv.PolyData) and target_trajectory.n_points == 1:
                # Hover point
                plotter.add_mesh(target_trajectory, color='green', opacity=0.5)
            else:
                # Trajectory line
                plotter.add_mesh(target_trajectory, color='green', line_width=3, opacity=0.5)

        # --- Draw current target marker ---
        target_marker = pv.Sphere(radius=0.03, center=target)
        plotter.add_mesh(target_marker, color='lime', opacity=0.8)

        # --- Draw actual trajectory trail ---
        trail_start = max(0, i - trail_length)
        if i > trail_start:
            trail_points = actual_positions[trail_start:i+1]
            if len(trail_points) >= 2:
                trail = pv.Spline(trail_points, n_points=len(trail_points))
                plotter.add_mesh(trail, color='blue', line_width=2, opacity=0.7)

        # --- Draw error line (actual to target) ---
        if error_mag > 0.01:
            error_line = pv.Line(xb, target)
            plotter.add_mesh(error_line, color='red', line_width=2, style='wireframe')

        # --- Draw body ---
        body = body_template.copy()
        body.translate(xb, inplace=True)
        plotter.add_mesh(body, color='tan', smooth_shading=True)

        # --- Draw wings ---
        all_wings = None
        lift_lines = []
        drag_lines = []

        for wname, xoffset, yoffset, lb0 in wing_info:
            origin = xb + np.array([xoffset, yoffset, 0])
            wing_mesh = _transform_wing_mesh(wing_templates[wname], origin, v[wname])

            if all_wings is None:
                all_wings = wing_mesh
            else:
                all_wings = all_wings + wing_mesh

            # Force vectors
            cp = origin + FORCE_CENTER_FRACTION * lb0 * v[wname]['e_r']
            lift_mag = np.linalg.norm(v[wname]['lift'])
            drag_mag = np.linalg.norm(v[wname]['drag'])
            if lift_mag > FORCE_THRESHOLD:
                lift_lines.append((cp, cp + FORCE_SCALE * v[wname]['lift']))
            if drag_mag > FORCE_THRESHOLD:
                drag_lines.append((cp, cp + FORCE_SCALE * v[wname]['drag']))

        plotter.add_mesh(all_wings, color='lightgray', opacity=0.8,
                         smooth_shading=True, show_edges=True, edge_color='k')

        if lift_lines:
            plotter.add_mesh(_make_lines(lift_lines), color='blue', line_width=3)
        if drag_lines:
            plotter.add_mesh(_make_lines(drag_lines), color='red', line_width=3)

        # --- Add text overlay ---
        ux, uz = states[i][3], states[i][5]
        text = (f"Position error: {error_mag:.3f}\n"
                f"Target: ({target[0]:.2f}, {target[1]:.2f}, {target[2]:.2f})\n"
                f"Actual: ({xb[0]:.2f}, {xb[1]:.2f}, {xb[2]:.2f})\n"
                f"Velocity: ux={ux:.2f}, uz={uz:.2f}")
        plotter.add_text(text, position='upper_left', font_size=10, color='black')

        # Frame counter
        plotter.add_text(f"Frame {i+1}/{n_frames}", position='lower_right',
                         font_size=8, color='gray')

        # --- Legend ---
        plotter.add_text("Target", position=(0.85, 0.95), font_size=9, color='green')
        plotter.add_text("Actual", position=(0.85, 0.91), font_size=9, color='blue')
        plotter.add_text("Error", position=(0.85, 0.87), font_size=9, color='red')

        # --- Camera ---
        # Follow the dragonfly but keep some scene context
        plotter.camera.position = (xb[0] + cam_dist * 0.7,
                                   xb[1] + cam_dist * 0.7,
                                   xb[2] + cam_dist * 0.4)
        plotter.camera.focal_point = tuple(xb)
        plotter.camera.up = (0, 0, 1)

        plotter.write_frame()

    plotter.close()


def plot_tracking_summary(states, controller, time, outfile):
    """
    Create a static summary plot showing trajectories and control parameters.

    Args:
        states: list of state arrays
        controller: dict with controller data
        time: time array
        outfile: output filename (.png)
    """
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D

    actual_pos = states[:, 0:3]
    target_pos = controller['target_position']
    errors = controller['position_error']
    error_mag = np.linalg.norm(errors, axis=1)

    fig = plt.figure(figsize=(16, 10))

    # 3D trajectory plot
    ax1 = fig.add_subplot(2, 3, 1, projection='3d')
    ax1.plot(actual_pos[:, 0], actual_pos[:, 1], actual_pos[:, 2],
             'b-', linewidth=1.5, label='Actual')
    ax1.plot(target_pos[:, 0], target_pos[:, 1], target_pos[:, 2],
             'g--', linewidth=1.5, label='Target')
    ax1.scatter(*actual_pos[0], c='b', s=50, marker='o', label='Start')
    ax1.scatter(*actual_pos[-1], c='r', s=50, marker='x', label='End')
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Z')
    ax1.set_title('3D Trajectory')
    ax1.legend()

    # XZ projection
    ax2 = fig.add_subplot(2, 3, 2)
    ax2.plot(actual_pos[:, 0], actual_pos[:, 2], 'b-', linewidth=1.5, label='Actual')
    ax2.plot(target_pos[:, 0], target_pos[:, 2], 'g--', linewidth=1.5, label='Target')
    ax2.set_xlabel('X (forward)')
    ax2.set_ylabel('Z (up)')
    ax2.set_title('XZ Projection')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    ax2.set_aspect('equal')

    # XY projection
    ax3 = fig.add_subplot(2, 3, 3)
    ax3.plot(actual_pos[:, 0], actual_pos[:, 1], 'b-', linewidth=1.5, label='Actual')
    ax3.plot(target_pos[:, 0], target_pos[:, 1], 'g--', linewidth=1.5, label='Target')
    ax3.set_xlabel('X (forward)')
    ax3.set_ylabel('Y (left)')
    ax3.set_title('XY Projection')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    ax3.set_aspect('equal')

    # Position error over time
    ax4 = fig.add_subplot(2, 3, 4)
    ax4.plot(time, error_mag, 'r-', linewidth=1.5)
    ax4.set_xlabel('Time')
    ax4.set_ylabel('Position Error')
    ax4.set_title('Tracking Error')
    ax4.grid(True, alpha=0.3)
    ax4.axhline(y=0.1, color='k', linestyle='--', alpha=0.5, label='Target (<0.1)')
    ax4.legend()

    # Control parameters
    ax5 = fig.add_subplot(2, 3, 5)
    ax5.plot(time, controller['gamma_mean'], 'b-', label='gamma_mean')
    ax5.plot(time, controller['psi_mean'], 'r-', label='psi_mean')
    ax5.set_xlabel('Time')
    ax5.set_ylabel('Angle (rad)')
    ax5.set_title('Control Parameters (angles)')
    ax5.legend()
    ax5.grid(True, alpha=0.3)

    ax6 = fig.add_subplot(2, 3, 6)
    ax6.plot(time, controller['phi_amp'], 'g-', label='phi_amp')
    ax6.set_xlabel('Time')
    ax6.set_ylabel('Amplitude (rad)')
    ax6.set_title('Control Parameters (amplitude)')
    ax6.legend()
    ax6.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(outfile, dpi=150)
    plt.close()


def main():
    parser = argparse.ArgumentParser(
        description="Visualize trajectory tracking simulation output"
    )
    parser.add_argument("input", help="Input HDF5 file")
    parser.add_argument("output", nargs="?", help="Output file (default: <input>_tracking.mp4)")
    parser.add_argument(
        "--renderer",
        choices=["hybrid", "pyvista"],
        default="hybrid",
        help="Rendering backend (default: hybrid)"
    )
    parser.add_argument(
        "--config",
        help="Custom visualization config JSON (hybrid renderer only)"
    )

    args = parser.parse_args()

    input_file = args.input

    # Determine output type based on extension
    if args.output:
        output_file = args.output
    else:
        output_file = Path(input_file).stem + "_tracking.mp4"

    print(f"Reading tracking data from {input_file}...")
    params, time, states, wings, controller = read_tracking(input_file)

    if controller is None:
        print("Error: No controller data found in file. Is this a tracking simulation?")
        sys.exit(1)

    print(f"Loaded {len(states)} timesteps, {len(params['wing_lb0'])} wings")
    print(f"Final position error: {np.linalg.norm(controller['position_error'][-1]):.4f}")
    print(f"Renderer: {args.renderer}")

    def _render_tracking(out):
        if args.renderer == "hybrid":
            from post.composite import (
                check_blender_available,
                render_hybrid_tracking,
                render_mpl_only_tracking
            )
            from post.hybrid_config import HybridConfig

            config = None
            if args.config:
                config = HybridConfig.load(args.config)

            if check_blender_available():
                render_hybrid_tracking(
                    states, wings, params, controller,
                    input_file, out, config
                )
            else:
                print("Warning: Blender not available, using matplotlib-only fallback")
                render_mpl_only_tracking(
                    states, wings, params, controller, out, config
                )
        else:
            plot_tracking_pyvista(states, wings, params, controller, out)

    # Generate both animation and summary plot
    if output_file.endswith('.mp4'):
        print(f"Creating animation: {output_file}")
        _render_tracking(output_file)

        # Also create summary plot
        summary_file = output_file.replace('.mp4', '_summary.png')
        print(f"Creating summary plot: {summary_file}")
        plot_tracking_summary(states, controller, time, summary_file)

    elif output_file.endswith('.png'):
        print(f"Creating summary plot: {output_file}")
        plot_tracking_summary(states, controller, time, output_file)
    else:
        print(f"Unknown output format. Creating animation: {output_file}")
        _render_tracking(output_file)

    print("Done.")


if __name__ == "__main__":
    main()

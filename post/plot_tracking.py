#!/usr/bin/env python3
"""
Visualize trajectory tracking simulation output.

Shows the dragonfly, target trajectory, and actual trajectory with tracking error.

Usage:
    python -m post.plot_tracking <input.h5> [output.mp4] [options]

Options:
    --config <file.json>  Custom visualization config
    --theme <light|dark>  Override plot theme
    --no-blender          Force matplotlib-only fallback rendering
    --frame-step N        Render every Nth frame (default: 1)
"""
import argparse
import sys
from pathlib import Path

import numpy as np

from post.io import read_tracking
from post.style import apply_matplotlib_style, resolve_style


def plot_tracking_summary(states, controller, time, outfile, style=None):
    """
    Create a static summary plot showing trajectories and control parameters.

    Args:
        states: list of state arrays
        controller: dict with controller data
        time: time array
        outfile: output filename (.png)
    """
    import matplotlib.pyplot as plt

    style = resolve_style(style)
    apply_matplotlib_style(style)

    actual_pos = states[:, 0:3]
    target_pos = controller['target_position']
    errors = controller['position_error']
    error_mag = np.linalg.norm(errors, axis=1)

    fig = plt.figure(figsize=(16, 10))

    # 3D trajectory plot
    ax1 = fig.add_subplot(2, 3, 1, projection='3d')
    ax1.plot(actual_pos[:, 0], actual_pos[:, 1], actual_pos[:, 2],
             color=style.trajectory_color, linewidth=1.5, label='Actual')
    ax1.plot(target_pos[:, 0], target_pos[:, 1], target_pos[:, 2],
             color=style.target_color, linestyle='--', linewidth=1.5, label='Target')
    ax1.scatter(*actual_pos[0], c=style.trajectory_color, s=50, marker='o', label='Start')
    ax1.scatter(*actual_pos[-1], c=style.error_color, s=50, marker='x', label='End')
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Z')
    ax1.set_title('3D Trajectory')
    ax1.legend()

    # XZ projection
    ax2 = fig.add_subplot(2, 3, 2)
    ax2.plot(actual_pos[:, 0], actual_pos[:, 2], color=style.trajectory_color, linewidth=1.5, label='Actual')
    ax2.plot(target_pos[:, 0], target_pos[:, 2], color=style.target_color, linestyle='--', linewidth=1.5, label='Target')
    ax2.set_xlabel('X (forward)')
    ax2.set_ylabel('Z (up)')
    ax2.set_title('XZ Projection')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    ax2.set_aspect('equal')

    # XY projection
    ax3 = fig.add_subplot(2, 3, 3)
    ax3.plot(actual_pos[:, 0], actual_pos[:, 1], color=style.trajectory_color, linewidth=1.5, label='Actual')
    ax3.plot(target_pos[:, 0], target_pos[:, 1], color=style.target_color, linestyle='--', linewidth=1.5, label='Target')
    ax3.set_xlabel('X (forward)')
    ax3.set_ylabel('Y (left)')
    ax3.set_title('XY Projection')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    ax3.set_aspect('equal')

    # Position error over time
    ax4 = fig.add_subplot(2, 3, 4)
    ax4.plot(time, error_mag, color=style.error_color, linewidth=1.5)
    ax4.set_xlabel('Time')
    ax4.set_ylabel('Position Error')
    ax4.set_title('Tracking Error')
    ax4.grid(True, alpha=0.3)
    ax4.axhline(y=0.1, color=style.muted_text_color, linestyle='--', alpha=0.5, label='Target (<0.1)')
    ax4.legend()

    # Control parameters
    ax5 = fig.add_subplot(2, 3, 5)
    ax5.plot(time, controller['gamma_mean'], color=style.trajectory_color, label='gamma_mean')
    ax5.plot(time, controller['psi_mean'], color=style.error_color, label='psi_mean')
    ax5.set_xlabel('Time')
    ax5.set_ylabel('Angle (rad)')
    ax5.set_title('Control Parameters (angles)')
    ax5.legend()
    ax5.grid(True, alpha=0.3)

    ax6 = fig.add_subplot(2, 3, 6)
    ax6.plot(time, controller['phi_amp'], color=style.target_color, label='phi_amp')
    ax6.set_xlabel('Time')
    ax6.set_ylabel('Amplitude (rad)')
    ax6.set_title('Control Parameters (amplitude)')
    ax6.legend()
    ax6.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(outfile, dpi=300)
    plt.close()


def main():
    parser = argparse.ArgumentParser(
        description="Visualize trajectory tracking simulation output"
    )
    parser.add_argument("input", help="Input HDF5 file")
    parser.add_argument("output", nargs="?", help="Output file (default: <input>_tracking.mp4)")
    parser.add_argument(
        "--config",
        help="Custom visualization config JSON"
    )
    parser.add_argument(
        "--theme",
        choices=["light", "dark"],
        help="Override visualization theme"
    )
    parser.add_argument(
        "--no-blender",
        action="store_true",
        help="Force matplotlib-only fallback rendering"
    )
    parser.add_argument(
        "--frame-step",
        type=int,
        default=1,
        help="Render every Nth frame (default: 1)"
    )

    args = parser.parse_args()
    if args.frame_step < 1:
        parser.error("--frame-step must be >= 1")

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

    from post.hybrid_config import HybridConfig
    from post.style import apply_theme_to_config

    config = HybridConfig.load(args.config) if args.config else None
    config = apply_theme_to_config(config, args.theme)

    def _render_tracking(out):
        from post.composite import (
            check_blender_available,
            render_hybrid,
            render_mpl_only
        )

        if args.no_blender:
            print("Blender disabled via --no-blender; using matplotlib-only fallback")
            render_mpl_only(
                states, wings, params, out, controller=controller, config=config,
                frame_step=args.frame_step
            )
        elif check_blender_available():
            render_hybrid(
                states, wings, params,
                input_file, out, controller=controller, config=config,
                frame_step=args.frame_step
            )
        else:
            print("Warning: Blender not available, using matplotlib-only fallback")
            render_mpl_only(
                states, wings, params, out, controller=controller, config=config,
                frame_step=args.frame_step
            )

    # Generate both animation and summary plot
    if output_file.endswith('.mp4'):
        print(f"Creating animation: {output_file}")
        _render_tracking(output_file)

        # Also create summary plot
        summary_file = output_file.replace('.mp4', '_summary.png')
        print(f"Creating summary plot: {summary_file}")
        plot_tracking_summary(states, controller, time, summary_file, style=config.style)

    elif output_file.endswith('.png'):
        print(f"Creating summary plot: {output_file}")
        plot_tracking_summary(states, controller, time, output_file, style=config.style)
    else:
        print(f"Unknown output format. Creating animation: {output_file}")
        _render_tracking(output_file)

    print("Done.")


if __name__ == "__main__":
    main()

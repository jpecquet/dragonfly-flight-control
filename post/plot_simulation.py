#!/usr/bin/env python3
"""
Visualize dragonfly simulation output.

Usage:
    python -m post.plot_simulation <input.h5> [output.mp4] [options]

Options:
    --config <file.json>  Custom visualization config
    --theme <light|dark>  Override plot theme
    --no-blender          Force matplotlib-only fallback rendering
    --frame-step N        Render every Nth frame (default: 1)
"""
import argparse
from pathlib import Path

from post.io import read_simulation


def main():
    parser = argparse.ArgumentParser(
        description="Visualize dragonfly simulation output"
    )
    parser.add_argument("input", help="Input HDF5 file")
    parser.add_argument("output", nargs="?", help="Output video file (default: <input>.mp4)")
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
    output_file = args.output or (Path(input_file).stem + ".mp4")

    print(f"Reading simulation data from {input_file}...")
    params, time, states, wings = read_simulation(input_file)

    print(f"Loaded {len(states)} timesteps, {len(params['wing_lb0'])} wings")
    print(f"Wings: {list(params['wing_lb0'].keys())}")

    print(f"Creating animation: {output_file}")
    from post.composite import (
        check_blender_available,
        render_hybrid,
        render_mpl_only
    )
    from post.hybrid_config import HybridConfig
    from post.style import apply_theme_to_config

    # Load custom config if provided
    config = HybridConfig.load(args.config) if args.config else None
    config = apply_theme_to_config(config, args.theme)

    if args.no_blender:
        print("Blender disabled via --no-blender; using matplotlib-only fallback")
        render_mpl_only(
            states, wings, params, output_file, config=config, frame_step=args.frame_step
        )
    elif check_blender_available():
        render_hybrid(
            states, wings, params, input_file, output_file, config=config, frame_step=args.frame_step
        )
    else:
        print("Warning: Blender not available, using matplotlib-only fallback")
        render_mpl_only(
            states, wings, params, output_file, config=config, frame_step=args.frame_step
        )

    print("Done.")


if __name__ == "__main__":
    main()

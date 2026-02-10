#!/usr/bin/env python3
"""
Visualize dragonfly simulation output.

Usage:
    python -m post.plot_simulation <input.h5> [output.mp4] [options]

Options:
    --config <file.json>  Custom visualization config
    --no-blender          Force matplotlib-only fallback rendering
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
        "--no-blender",
        action="store_true",
        help="Force matplotlib-only fallback rendering"
    )

    args = parser.parse_args()

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

    # Load custom config if provided
    config = None
    if args.config:
        config = HybridConfig.load(args.config)

    if args.no_blender:
        print("Blender disabled via --no-blender; using matplotlib-only fallback")
        render_mpl_only(
            states, wings, params, output_file, config=config
        )
    elif check_blender_available():
        render_hybrid(
            states, wings, params, input_file, output_file, config=config
        )
    else:
        print("Warning: Blender not available, using matplotlib-only fallback")
        render_mpl_only(
            states, wings, params, output_file, config=config
        )

    print("Done.")


if __name__ == "__main__":
    main()

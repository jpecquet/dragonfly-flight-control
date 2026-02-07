#!/usr/bin/env python3
"""
Visualize dragonfly simulation output.

Usage:
    python -m post.plot_simulation <input.h5> [output.mp4] [options]

Options:
    --renderer hybrid|pyvista  Rendering backend (default: hybrid)
    --config <file.json>       Custom visualization config (hybrid only)
"""
import argparse
import sys
from pathlib import Path

from post.io import read_simulation


def main():
    parser = argparse.ArgumentParser(
        description="Visualize dragonfly simulation output"
    )
    parser.add_argument("input", help="Input HDF5 file")
    parser.add_argument("output", nargs="?", help="Output video file (default: <input>.mp4)")
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
    output_file = args.output or (Path(input_file).stem + ".mp4")

    print(f"Reading simulation data from {input_file}...")
    params, time, states, wings = read_simulation(input_file)

    print(f"Loaded {len(states)} timesteps, {len(params['wing_lb0'])} wings")
    print(f"Wings: {list(params['wing_lb0'].keys())}")

    print(f"Creating animation: {output_file}")
    print(f"Renderer: {args.renderer}")

    if args.renderer == "hybrid":
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

        if check_blender_available():
            render_hybrid(
                states, wings, params, input_file, output_file, config=config
            )
        else:
            print("Warning: Blender not available, using matplotlib-only fallback")
            render_mpl_only(
                states, wings, params, output_file, config=config
            )
    else:
        # PyVista renderer
        from post.dragonfly import plot_dragonfly
        plot_dragonfly(states, wings, params, output_file)

    print("Done.")


if __name__ == "__main__":
    main()

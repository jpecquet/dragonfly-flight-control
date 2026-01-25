#!/usr/bin/env python3
"""
Visualize dragonfly simulation output.

Usage:
    python -m post.plot_simulation <input.h5> [output.mp4]
"""
import sys
from pathlib import Path

from post.io import read_simulation
from post.dragonfly import plot_dragonfly


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    input_file = sys.argv[1]

    # Default output name based on input
    if len(sys.argv) >= 3:
        output_file = sys.argv[2]
    else:
        output_file = Path(input_file).stem + ".mp4"

    print(f"Reading simulation data from {input_file}...")
    params, time, states, wings = read_simulation(input_file)

    print(f"Loaded {len(states)} timesteps, {len(params['wing_lb0'])} wings")
    print(f"Wings: {list(params['wing_lb0'].keys())}")

    print(f"Creating animation: {output_file}")
    plot_dragonfly(states, wings, params, output_file)

    print("Done.")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
Postprocessing entry point for dragonfly simulation.

Usage:
    python main.py <input.h5> [output.png|output.mp4]

If output ends with .mp4, creates an animation.
Otherwise, creates a static image.
"""
import sys
from pathlib import Path

from read_output import read_simulation
from visual import plotDragonfly


def main():
    if len(sys.argv) < 2:
        print("Usage: python main.py <input.h5> [output.png|output.mp4]")
        sys.exit(1)

    input_file = sys.argv[1]

    # Default output name based on input
    if len(sys.argv) >= 3:
        output_file = sys.argv[2]
    else:
        output_file = Path(input_file).stem + ".png"

    print(f"Reading simulation data from {input_file}...")
    params, time, states, wings = read_simulation(input_file)

    print(f"Loaded {len(states)} timesteps")
    print(f"Parameters: lb0_f={params['lb0_f']}, lb0_h={params['lb0_h']}, omg0={params['omg0']:.2f}")

    animate = output_file.endswith(".mp4")
    print(f"Creating {'animation' if animate else 'static image'}: {output_file}")

    plotDragonfly(states, wings, params, output_file, animate=animate)

    print("Done.")


if __name__ == "__main__":
    main()

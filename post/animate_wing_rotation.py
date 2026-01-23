#!/usr/bin/env python3
"""
Animate wing rotation test results.

Visualizes the wing basis vectors (e_s, e_r, e_c) as they rotate
through three successive phases:
  Phase 1: Stroke plane (gam) 0° → 90°
  Phase 2: Flapping (phi) 0° → 25°
  Phase 3: Pitch (psi) 0° → 45°

Usage:
    python post/animate_wing_rotation.py wing_rotation.h5 output.mp4
"""

import sys
import h5py
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as ani
from mpl_toolkits.mplot3d import Axes3D

plt.rcParams["font.family"] = "serif"
plt.rcParams["font.serif"] = ["Times New Roman"]
plt.rcParams['mathtext.fontset'] = 'stix'
plt.rcParams['font.size'] = 11


def read_data(filename):
    """Read wing rotation data from HDF5 file."""
    with h5py.File(filename, 'r') as f:
        data = {
            'frames_per_phase': f['/parameters/frames_per_phase'][()],
            'total_frames': f['/parameters/total_frames'][()],
            'phase_boundaries': f['/parameters/phase_boundaries'][:],
            'gam_range': (f['/parameters/gam_start'][()], f['/parameters/gam_end'][()]),
            'phi_range': (f['/parameters/phi_start'][()], f['/parameters/phi_end'][()]),
            'psi_range': (f['/parameters/psi_start'][()], f['/parameters/psi_end'][()]),
            'is_left': f['/parameters/is_left'][()] == 1,
            'gam': f['/angles/gam'][:],
            'phi': f['/angles/phi'][:],
            'psi': f['/angles/psi'][:],
            'e_s': f['/wing/e_s'][:],
            'e_r': f['/wing/e_r'][:],
            'e_c': f['/wing/e_c'][:],
        }
    return data


def get_phase(frame, phase_boundaries):
    """Return phase number (1, 2, or 3) and phase name."""
    if frame < phase_boundaries[1]:
        return 1, "Stroke plane (γ)"
    elif frame < phase_boundaries[2]:
        return 2, "Flapping (φ)"
    else:
        return 3, "Pitch (ψ)"


def animate_vectors(data, outfile):
    """Create animation of wing vectors."""
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')

    n_frames = data['total_frames']
    phase_boundaries = data['phase_boundaries']
    side = "left" if data['is_left'] else "right"

    # Arrow length
    arrow_len = 0.8

    def update(frame):
        ax.cla()

        # Get vectors for this frame
        e_s = data['e_s'][frame]
        e_r = data['e_r'][frame]
        e_c = data['e_c'][frame]

        # Draw coordinate axes (faint)
        ax.quiver(0, 0, 0, 1, 0, 0, color='gray', alpha=0.3, arrow_length_ratio=0.1)
        ax.quiver(0, 0, 0, 0, 1, 0, color='gray', alpha=0.3, arrow_length_ratio=0.1)
        ax.quiver(0, 0, 0, 0, 0, 1, color='gray', alpha=0.3, arrow_length_ratio=0.1)
        ax.text(1.1, 0, 0, 'X', color='gray', fontsize=10)
        ax.text(0, 1.1, 0, 'Y', color='gray', fontsize=10)
        ax.text(0, 0, 1.1, 'Z', color='gray', fontsize=10)

        # Draw wing vectors
        ax.quiver(0, 0, 0, *e_s * arrow_len, color='red', arrow_length_ratio=0.1, linewidth=2)
        ax.quiver(0, 0, 0, *e_r * arrow_len, color='green', arrow_length_ratio=0.1, linewidth=2)
        ax.quiver(0, 0, 0, *e_c * arrow_len, color='blue', arrow_length_ratio=0.1, linewidth=2)

        # Labels at arrow tips
        ax.text(*(e_s * (arrow_len + 0.15)), r'$\mathbf{e}_s$', color='red', fontsize=12, fontweight='bold')
        ax.text(*(e_r * (arrow_len + 0.15)), r'$\mathbf{e}_r$', color='green', fontsize=12, fontweight='bold')
        ax.text(*(e_c * (arrow_len + 0.15)), r'$\mathbf{e}_c$', color='blue', fontsize=12, fontweight='bold')

        # Get angles
        gam = data['gam'][frame]
        phi = data['phi'][frame]
        psi = data['psi'][frame]

        phase_num, phase_name = get_phase(frame, phase_boundaries)

        # Title with angles and wing side
        ax.set_title(f"{side.capitalize()} wing - Phase {phase_num}: {phase_name}\n"
                     f"γ = {gam:.1f}°, φ = {phi:.1f}°, ψ = {psi:.1f}°",
                     fontsize=12)

        # Set axis limits and labels
        ax.set_xlim([-1, 1])
        ax.set_ylim([-1, 1])
        ax.set_zlim([-1, 1])
        ax.set_xlabel('X (forward)')
        ax.set_ylabel('Y (right)')
        ax.set_zlabel('Z (up)')
        ax.set_box_aspect([1, 1, 1])

        # Viewing angle
        ax.view_init(elev=25, azim=-60)

    # Create animation
    anim = ani.FuncAnimation(fig, update, frames=n_frames, interval=50, blit=False)

    # Save
    if outfile.endswith('.mp4'):
        writer = ani.FFMpegWriter(fps=20, bitrate=2000)
        anim.save(outfile, writer=writer)
    elif outfile.endswith('.gif'):
        anim.save(outfile, writer='pillow', fps=20)

    plt.close()
    print(f"Saved: {outfile}")


def main():
    if len(sys.argv) < 3:
        print("Usage: python animate_wing_rotation.py <input.h5> <output.mp4|gif>")
        sys.exit(1)

    infile = sys.argv[1]
    outfile = sys.argv[2]

    print(f"Reading {infile}...")
    data = read_data(infile)

    side = "left" if data['is_left'] else "right"
    print(f"Wing: {side}")
    print(f"Total frames: {data['total_frames']}")
    print(f"Frames per phase: {data['frames_per_phase']}")
    print(f"Angle ranges:")
    print(f"  γ (stroke plane): {data['gam_range'][0]}° → {data['gam_range'][1]}°")
    print(f"  φ (flapping):     {data['phi_range'][0]}° → {data['phi_range'][1]}°")
    print(f"  ψ (pitch):        {data['psi_range'][0]}° → {data['psi_range'][1]}°")

    print(f"Creating animation...")
    animate_vectors(data, outfile)


if __name__ == "__main__":
    main()

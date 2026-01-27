#!/usr/bin/env python3
"""
Animate wing rotation test results.

Visualizes the wing basis vectors (e_s, e_r, e_c) as they rotate
through three successive phases with configurable rotation order.

Usage:
    python -m post.plot_wing_rotation <input.h5> <output.mp4|gif>
"""

import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as ani
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# Import package to set matplotlib config
import post
from post.io import read_wing_rotation


# Map angle keys to descriptive names
ANGLE_NAMES = {
    'gam': 'Stroke plane (gam)',
    'phi': 'Flapping (phi)',
    'psi': 'Pitch (psi)',
}


def _get_phase(frame, phase_boundaries, sequence):
    """Return phase number (1, 2, or 3) and phase name based on sequence."""
    if frame < phase_boundaries[1]:
        phase_idx = 0
    elif frame < phase_boundaries[2]:
        phase_idx = 1
    else:
        phase_idx = 2

    angle_key = sequence[phase_idx]
    return phase_idx + 1, ANGLE_NAMES.get(angle_key, angle_key)


def animate_vectors(data, outfile):
    """Create animation of wing vectors."""
    fig = plt.figure(figsize=(4, 4))
    ax = fig.add_subplot(111, projection='3d')

    n_frames = data['total_frames']

    # Arrow length
    arrow_len = 0.8

    def update(frame):
        ax.cla()

        # Get vectors for this frame
        e_s = data['e_s'][frame]
        e_r = data['e_r'][frame]
        e_c = data['e_c'][frame]
        e_n = np.cross(e_c, e_r)

        # Draw coordinate axes (faint)
        ax.quiver(0, 0, 0, 1, 0, 0, color='gray', alpha=0.3, arrow_length_ratio=0.1)
        ax.quiver(0, 0, 0, 0, 1, 0, color='gray', alpha=0.3, arrow_length_ratio=0.1)
        ax.quiver(0, 0, 0, 0, 0, 1, color='gray', alpha=0.3, arrow_length_ratio=0.1)
        ax.text(1.1, 0, 0, 'X', color='gray', fontsize=10)
        ax.text(0, 1.1, 0, 'Y', color='gray', fontsize=10)
        ax.text(0, 0, 1.1, 'Z', color='gray', fontsize=10)

        # Draw wing vectors
        ax.quiver(0, 0, 0, *e_s * arrow_len, color='orange', arrow_length_ratio=0.1, linewidth=2)
        ax.quiver(0, 0, 0, *e_r * arrow_len, color='green', arrow_length_ratio=0.1, linewidth=2)
        ax.quiver(0, 0, 0, *e_c * arrow_len, color='blue', arrow_length_ratio=0.1, linewidth=2)
        ax.quiver(0, 0, 0, *e_n * arrow_len, color='red', arrow_length_ratio=0.1, linewidth=2)

        # Labels at arrow tips
        ax.text(*(e_s * (arrow_len + 0.15)), r'$\mathbf{e}_s$', color='orange', fontsize=12, fontweight='bold')
        ax.text(*(e_r * (arrow_len + 0.15)), r'$\mathbf{e}_r$', color='green', fontsize=12, fontweight='bold')
        ax.text(*(e_c * (arrow_len + 0.15)), r'$\mathbf{e}_c$', color='blue', fontsize=12, fontweight='bold')
        ax.text(*(e_n * (arrow_len + 0.15) - e_r * 0.3), r'$\mathbf{e}_n$', color='red', fontsize=12, fontweight='bold')

        # Draw wing rectangle (span=0.8 along e_r, chord=0.2 along e_c)
        span, chord = 0.75, 0.2
        wing_verts = [
            -chord/2 * e_c,                    # root trailing edge
            chord/2 * e_c,                    # root leading edge
            span * e_r + chord/2 * e_c,       # tip leading edge
            span * e_r - chord/2 * e_c,                     # tip trailing edge
        ]
        wing_poly = Poly3DCollection([wing_verts], alpha=0.3, facecolor='k')
        ax.add_collection3d(wing_poly)

        # Get angles
        gam = data['gam'][frame]
        phi = data['phi'][frame]
        psi = data['psi'][frame]

        # Title with angles (Greek letters)
        ax.set_title(f"$\\gamma$ = {gam:.1f}°,  $\\phi$ = {phi:.1f}°,  $\\psi$ = {psi:.1f}°",
                     fontsize=12)

        # Set axis limits (centered on origin)
        lim = 0.5
        ax.set_xlim([-lim*0.5, lim*1.5])
        ax.set_ylim([-lim, lim])
        ax.set_zlim([-lim*0.1, lim*1.9])
        ax.set_box_aspect([1, 1, 1])

        # Hide axes
        ax.set_axis_off()

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
        print(__doc__)
        sys.exit(1)

    infile = sys.argv[1]
    outfile = sys.argv[2]

    print(f"Reading {infile}...")
    data = read_wing_rotation(infile)

    side = "left" if data['is_left'] else "right"
    seq = data['sequence']
    print(f"Wing: {side}")
    print(f"Rotation sequence: {seq[0]} -> {seq[1]} -> {seq[2]}")
    print(f"Total frames: {data['total_frames']}")
    print(f"Frames per phase: {data['frames_per_phase']}")
    print(f"Angle ranges:")
    print(f"  gam (stroke plane): {data['gam_range'][0]} deg -> {data['gam_range'][1]} deg")
    print(f"  phi (flapping):     {data['phi_range'][0]} deg -> {data['phi_range'][1]} deg")
    print(f"  psi (pitch):        {data['psi_range'][0]} deg -> {data['psi_range'][1]} deg")

    print(f"Creating animation...")
    animate_vectors(data, outfile)


if __name__ == "__main__":
    main()

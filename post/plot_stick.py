#!/usr/bin/env python3
"""
Visualize wing stroke motion projected onto a sphere.

Shows the wing chordline as a line with a hollow circle at the leading edge,
projected onto a sphere of radius 2/3*lb0, viewed from the side.

Usage:
    python -m post.plot_stick <input.h5> <wing_name> <output.mp4|gif> [--theme light|dark]

Example:
    python -m post.plot_stick output.h5 fore_left stroke.mp4 --theme dark
"""

import argparse
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as ani

from post.io import read_simulation
from post.style import apply_matplotlib_style, resolve_style


def cart_to_spherical(v):
    """Convert Cartesian unit vector to spherical (theta, phi) in radians.

    theta: azimuth angle in X-Y plane, measured from +X toward +Y
    phi: elevation angle from X-Y plane toward +Z
    """
    x, y, z = v[0], v[1], v[2]
    theta = np.arctan2(y, x)
    phi = np.arcsin(np.clip(z, -1, 1))
    return theta, phi


def vec_to_spherical(e_r, vec):
    """Project a 3D vector onto spherical tangent plane at e_r.

    Returns (dtheta, dphi) - the vector components in spherical coordinates.
    """
    x, y, z = e_r[0], e_r[1], e_r[2]
    r_xy = np.sqrt(x**2 + y**2)

    if r_xy < 1e-9:
        # At poles, use simple projection
        return vec[1], vec[0]

    # Tangent vectors on sphere at point e_r
    # e_theta: direction of increasing theta (azimuth)
    e_theta = np.array([-y, x, 0]) / r_xy
    # e_phi: direction of increasing phi (elevation)
    e_phi = np.array([-x * z, -y * z, r_xy]) / np.sqrt(r_xy**2 * z**2 + r_xy**2)

    # Project vector onto tangent plane
    dtheta = np.dot(vec, e_theta)
    dphi = np.dot(vec, e_phi)

    return dtheta, dphi


def animate_stroke(time, wings, wing_name, outfile, style=None):
    """Create animation of wing stroke on sphere, equirectangular projection."""
    style = resolve_style(style)
    apply_matplotlib_style(style)

    fig, ax = plt.subplots(figsize=(4, 4))

    n_frames = len(time)
    chord_len = 8  # visual chord length in degrees

    traj_theta, traj_phi = np.zeros(n_frames + 1), np.zeros(n_frames + 1)
    for i in range(n_frames):
        e_r_i = wings[i][wing_name]['e_r']
        traj_theta[i], traj_phi[i] = cart_to_spherical(e_r_i)
    traj_theta[-1], traj_phi[-1] = traj_theta[0], traj_phi[0]

    def update(frame):
        ax.cla()

        wing = wings[frame][wing_name]
        e_r = wing['e_r']  # radial direction (to wing tip)
        e_c = wing['e_c']  # chord direction
        lift = wing['lift']
        drag = wing['drag']

        # Convert to spherical coordinates
        theta, phi = cart_to_spherical(e_r)
        theta_deg, phi_deg = np.degrees(theta), np.degrees(phi)

        # Get chord direction in spherical coordinates
        dtheta, dphi = vec_to_spherical(e_r, e_c)
        chord_dir = np.array([dtheta, dphi])
        chord_norm = np.linalg.norm(chord_dir)
        if chord_norm > 1e-6:
            chord_dir = chord_dir / chord_norm

        # Leading and trailing edge positions
        le_theta = theta_deg + chord_len * chord_dir[0]
        le_phi = phi_deg + chord_len * chord_dir[1]
        te_theta = theta_deg - chord_len * chord_dir[0]
        te_phi = phi_deg - chord_len * chord_dir[1]

        # Draw chordline
        ax.plot([te_theta, le_theta], [te_phi, le_phi], '-', color=style.body_color, linewidth=2)

        # Draw hollow circle at leading edge
        ax.plot(
            le_theta, le_phi, 'o', markersize=4,
            markerfacecolor=style.axes_facecolor,
            markeredgecolor=style.body_color, markeredgewidth=2
        )

        # Draw lift and drag vectors (scaled, converted to degrees)
        force_scale = 0.05 * np.degrees(1)  # scale factor to degrees

        lift_dtheta, lift_dphi = vec_to_spherical(e_r, lift)
        drag_dtheta, drag_dphi = vec_to_spherical(e_r, drag)

        ax.arrow(
            theta_deg, phi_deg, lift_dtheta * force_scale, lift_dphi * force_scale,
            head_width=1, head_length=2, fc=style.lift_color, ec=style.lift_color, linewidth=1.5
        )
        ax.arrow(
            theta_deg, phi_deg, drag_dtheta * force_scale, drag_dphi * force_scale,
            head_width=1, head_length=2, fc=style.drag_color, ec=style.drag_color, linewidth=1.5
        )

        # Plot trajectory over wingbeat
        ax.plot(
            np.degrees(traj_theta), np.degrees(traj_phi), '-',
            color=style.trajectory_color, linewidth=0.7, zorder=-1
        )

        # Axis settings
        ax.set_xlim([45, 135])
        ax.set_ylim([-45, 45])
        ax.set_aspect('equal')
        ax.set_xlabel(r'Stroke Sphere Azimuth ($^\circ$)')
        ax.set_ylabel(r'Stroke Sphere Elevation ($^\circ$)')
        ax.grid(True, alpha=0.3)

        if frame == 0:
            fig.tight_layout()

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
    parser = argparse.ArgumentParser(description="Visualize wing stroke on sphere")
    parser.add_argument("input", help="Input HDF5 simulation file")
    parser.add_argument("wing", nargs="?", help="Wing name (e.g., fore_left)")
    parser.add_argument("output", nargs="?", help="Output animation (.mp4 or .gif)")
    parser.add_argument(
        "--theme",
        choices=["light", "dark"],
        default="light",
        help="Plot theme (default: light)",
    )
    args = parser.parse_args()

    print(f"Reading {args.input}...")
    params, time, states, wings = read_simulation(args.input)

    if not args.wing or not args.output:
        print("\nUsage: python -m post.plot_stick <input.h5> <wing_name> <output.mp4|gif> [--theme light|dark]")
        print("\nAvailable wings:")
        for name in wings[0].keys():
            print(f"  {name}")
        sys.exit(1)

    wing_name = args.wing
    outfile = args.output

    if wing_name not in wings[0]:
        print(f"Error: Wing '{wing_name}' not found.")
        print("Available wings:")
        for name in wings[0].keys():
            print(f"  {name}")
        sys.exit(1)

    lb0 = params['wing_lb0'][wing_name]
    print(f"Wing: {wing_name}")
    print(f"lb0: {lb0}")
    print(f"Sphere radius: {(2.0 / 3.0) * lb0:.4f}")
    print(f"Frames: {len(time)}")

    style = resolve_style(theme=args.theme)
    print(f"Theme: {style.theme}")

    print("Creating animation...")
    animate_stroke(time, wings, wing_name, outfile, style=style)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
Visualize terminal velocity simulation results.

Usage:
    python -m post.plot_terminal_velocity --psi 45 output.png   # Static plot
    python -m post.plot_terminal_velocity --psi 45 output.mp4   # Animation
    python -m post.plot_terminal_velocity --psi 45 output.gif   # Animation (GIF)

Options:
    --psi DEG    Wing pitch angle in degrees (default: 0)
    --dt VALUE   Time step (default: 0.01)
    --tmax VALUE Maximum simulation time (default: 50)
    --fps VALUE  Animation frames per second (default: 30)
    --skip N     Use every Nth frame for animation (default: 1)
    --theme MODE Plot theme: light or dark (default: light)
"""

import argparse
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as plticker
import matplotlib.animation as ani

from post.animation import save_animation
from post.hybrid_config import StyleConfig
from post.io import read_terminal_velocity, run_termvel_simulation
from post.style import apply_matplotlib_style, resolve_style


def plot_terminal_velocity(data, output_file=None, style: StyleConfig = None):
    """Plot terminal velocity time series."""
    style = resolve_style(style)
    apply_matplotlib_style(style)

    time = data['time']
    ux = data['ux']
    uz = data['uz']
    psi_deg = data['psi_deg']

    fig, ax = plt.subplots(figsize=(4, 3))

    ax.plot(time, ux, '-', color=style.trajectory_color, linewidth=1, label='Physics solver')
    ax.plot(time, uz, '-', color=style.trajectory_color, linewidth=1)
    ax.axhline(y=ux[-1], color=style.muted_text_color, linestyle=':', linewidth=1, label='Steady-state solution')
    ax.axhline(y=uz[-1], color=style.muted_text_color, linestyle=':', linewidth=1)

    ax.set_xlim(time[0], time[-1])

    # Labels at right edge of frame, offset above lines
    from matplotlib.transforms import offset_copy
    trans = offset_copy(ax.transData, fig=fig, x=-4, y=4, units='points')
    ax.text(time[-1], ux[-1], r'$\tilde{u}_x$', ha='right', va='bottom', transform=trans)
    ax.text(time[-1], uz[-1], r'$\tilde{u}_z$', ha='right', va='bottom', transform=trans)
    ax.set_xlabel(r'$\tilde{t}$')
    ax.set_ylabel(r'$\tilde{u}_i$')
    ax.set_title(f'Fixed orientation freefall $(\\psi$ = {psi_deg:.0f}°)')
    ax.margins(0.2)
    plt.legend()

    plt.tight_layout()

    if output_file:
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f'Saved to {output_file}')
    else:
        plt.show()


def animate_terminal_velocity(data, outfile, fps=30, skip=1, style: StyleConfig = None):
    """Create animation of falling wing, centered on wing with lift/drag vectors."""
    style = resolve_style(style)
    apply_matplotlib_style(style)

    time = data['time'][::skip]
    x = data['x'][::skip]
    z = data['z'][::skip]
    psi = data['psi']
    psi_deg = data['psi_deg']

    # Lift/drag from simulation
    lift_x = data['lift_x'][::skip]
    lift_z = data['lift_z'][::skip]
    drag_x = data['drag_x'][::skip]
    drag_z = data['drag_z'][::skip]

    n_frames = len(time)

    # Chord direction: e_c = (cos(psi), sin(psi)) for left wing with gam=90°
    chord_x = np.cos(psi)
    chord_z = np.sin(psi)

    # Visual sizes
    chord_len = 2.0
    view_size = 10.0  # half-width of view window
    force_scale = 5  # scale for force vectors

    fig, ax = plt.subplots(figsize=(4, 4))

    def update(frame):
        ax.cla()

        # Current position (wing center)
        px, pz = x[frame], z[frame]

        # Leading and trailing edge positions (absolute)
        le_x = px + chord_len * chord_x
        le_z = pz + chord_len * chord_z
        te_x = px - chord_len * chord_x
        te_z = pz - chord_len * chord_z

        # Get lift and drag from simulation data
        lift = np.array([lift_x[frame], lift_z[frame]])
        drag = np.array([drag_x[frame], drag_z[frame]])

        # Draw trajectory up to current frame
        ax.plot(
            x[:frame+1], z[:frame+1], '-',
            color=style.trajectory_color, linewidth=0.5, alpha=0.5
        )

        # Draw lift vector (blue)
        ax.arrow(px, pz, lift[0] * force_scale, lift[1] * force_scale,
                 head_width=1.2*0.11, head_length=2*0.11,
                 fc=style.lift_color, ec=style.lift_color, linewidth=1.5,
                 label='Lift')

        # Draw drag vector (red)
        ax.arrow(px, pz, drag[0] * force_scale, drag[1] * force_scale,
                 head_width=1.2*0.11, head_length=2*0.11,
                 fc=style.drag_color, ec=style.drag_color, linewidth=1.5,
                 label='Drag')

        # Draw chord line
        ax.plot([te_x, le_x], [te_z, le_z], '-', color=style.body_color, linewidth=2)

        # Draw hollow circle at leading edge
        ax.plot(
            le_x, le_z, 'o', markersize=4, markerfacecolor=style.axes_facecolor,
            markeredgecolor=style.body_color, markeredgewidth=2
        )

        # Axis settings (view centered on wing, but showing absolute coords)
        ax.set_xlim(px - view_size, px + view_size)
        ax.set_ylim(pz - view_size, pz + view_size)
        ax.set_aspect('equal')
        ax.set_xlabel(r'$\tilde{x}$')
        ax.set_ylabel(r'$\tilde{z}$')
        ax.set_title(f'Terminal Velocity ($\\psi = {psi_deg:.0f}$°)')
        ax.grid(True, alpha=0.3)

        ax.legend(loc='upper right')

        loc = plticker.MultipleLocator(base=5.0)
        ax.xaxis.set_major_locator(loc)
        loc = plticker.MultipleLocator(base=5.0)
        ax.yaxis.set_major_locator(loc)

        if frame == 0:
            fig.tight_layout()

    # Create animation
    anim = ani.FuncAnimation(fig, update, frames=n_frames, interval=1000/fps, blit=False)

    save_animation(anim, outfile, fps=fps)

    plt.close()
    print(f"Saved: {outfile}")


def main():
    parser = argparse.ArgumentParser(description='Terminal velocity simulation visualization')
    parser.add_argument('--psi', type=float, default=0.0, help='Pitch angle in degrees')
    parser.add_argument('--dt', type=float, default=0.01, help='Time step')
    parser.add_argument('--tmax', type=float, default=50.0, help='Max simulation time')
    parser.add_argument('--fps', type=int, default=30, help='Animation frames per second')
    parser.add_argument('--skip', type=int, default=1, help='Skip frames (use every Nth frame)')
    parser.add_argument(
        '--theme', choices=['light', 'dark'], default='light',
        help='Plot theme (default: light)'
    )
    parser.add_argument('output', nargs='?', help='Output file (png/pdf/svg for plot, mp4/gif for animation)')
    args = parser.parse_args()

    style = resolve_style(theme=args.theme)
    apply_matplotlib_style(style)

    print(f'Running simulation (psi = {args.psi} deg)...')
    data = run_termvel_simulation(args.psi, args.dt, args.tmax)
    print(f'  Final ux: {data["ux"][-1]:.4f}')
    print(f'  Final uz: {data["uz"][-1]:.4f}')

    # Determine output type from extension
    if args.output:
        ext = Path(args.output).suffix.lower()
        if ext in ('.mp4', '.gif'):
            print(f'Creating animation ({len(data["time"][::args.skip])} frames)...')
            animate_terminal_velocity(data, args.output, fps=args.fps, skip=args.skip, style=style)
        else:
            plot_terminal_velocity(data, args.output, style=style)
    else:
        plot_terminal_velocity(data, style=style)


if __name__ == '__main__':
    main()

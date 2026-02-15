#!/usr/bin/env python3
"""
Visualize right forewing/hindwing stick motion in the (X,Z) plane.

Usage:
    python -m post.plot_stick <input.h5> <output.mp4|gif> [--theme light|dark] [--station 0.6667]

Example:
    python -m post.plot_stick output.h5 stroke.mp4 --theme dark
"""

import argparse
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as ani

from post.animation import save_animation
from post.io import read_simulation
from post.style import apply_matplotlib_style, figure_size, resolve_style

STICK_LENGTH = 0.1
FORE_X_OFFSET = 0.1
HIND_X_OFFSET = -0.1


def _normalize_name(name):
    return str(name).strip().lower().replace("-", "_")


def _resolve_wing_name(available, primary_token):
    normalized = {_normalize_name(name): name for name in available}

    exact = f"{primary_token}_right"
    if exact in normalized:
        return normalized[exact]

    matches = [name for name in available if primary_token in _normalize_name(name) and "right" in _normalize_name(name)]
    if len(matches) == 1:
        return matches[0]
    if len(matches) > 1:
        return sorted(matches)[0]

    fallback = [name for name in available if primary_token in _normalize_name(name)]
    if len(fallback) == 1:
        return fallback[0]
    if len(fallback) > 1:
        return sorted(fallback)[0]

    return None


def resolve_right_wings(available):
    """Resolve fore/hind wing names, preferring right-side names when present."""
    fore = _resolve_wing_name(available, "fore")
    hind = _resolve_wing_name(available, "hind")
    if fore is None or hind is None:
        raise ValueError("Could not resolve fore/hind wing names from simulation output.")
    return fore, hind


def project_xz(vec):
    return np.array([vec[0], vec[2]], dtype=float)


def compute_stick_endpoints(wing_state, x_offset, stick_length, station, lambda0):
    """Compute stick center, leading edge, and trailing edge in nondimensional (X,Z)."""
    if not 0.0 <= float(station) <= 1.0:
        raise ValueError("station must be in [0, 1]")
    center = float(station) * float(lambda0) * project_xz(wing_state["e_r"]) + np.array([x_offset, 0.0], dtype=float)

    chord_dir = project_xz(wing_state["e_c"])
    chord_norm = np.linalg.norm(chord_dir)
    if chord_norm < 1e-9:
        chord_dir = np.array([1.0, 0.0], dtype=float)
    else:
        chord_dir = chord_dir / chord_norm

    half = 0.5 * float(stick_length)
    leading = center + half * chord_dir
    trailing = center - half * chord_dir
    return center, leading, trailing


def animate_stroke(
    time,
    wings,
    fore_wing_name,
    hind_wing_name,
    outfile,
    omega,
    style=None,
    station=2.0 / 3.0,
    fore_lambda0=1.0,
    hind_lambda0=1.0,
):
    """Create animation of right fore/hind stick motion in the nondimensional (X,Z) plane."""
    style = resolve_style(style)
    apply_matplotlib_style(style)
    omega = float(omega)
    time_scale = omega / (2.0 * np.pi)

    fig, ax = plt.subplots(figsize=figure_size(0.6))
    n_frames = len(time)
    fore_centers = np.zeros((n_frames, 2), dtype=float)
    hind_centers = np.zeros((n_frames, 2), dtype=float)
    fore_leading = np.zeros((n_frames, 2), dtype=float)
    fore_trailing = np.zeros((n_frames, 2), dtype=float)
    hind_leading = np.zeros((n_frames, 2), dtype=float)
    hind_trailing = np.zeros((n_frames, 2), dtype=float)

    for i in range(n_frames):
        fore_center, fore_le, fore_te = compute_stick_endpoints(
            wings[i][fore_wing_name],
            x_offset=FORE_X_OFFSET,
            stick_length=STICK_LENGTH,
            station=station,
            lambda0=fore_lambda0,
        )
        hind_center, hind_le, hind_te = compute_stick_endpoints(
            wings[i][hind_wing_name],
            x_offset=HIND_X_OFFSET,
            stick_length=STICK_LENGTH,
            station=station,
            lambda0=hind_lambda0,
        )
        fore_centers[i] = fore_center
        hind_centers[i] = hind_center
        fore_leading[i] = fore_le
        fore_trailing[i] = fore_te
        hind_leading[i] = hind_le
        hind_trailing[i] = hind_te

    all_centers = np.vstack([fore_centers, hind_centers])
    half_len = 0.5 * STICK_LENGTH
    pad = 0.08
    ax.set_xlim([
        float(np.min(all_centers[:, 0]) - half_len - pad),
        float(np.max(all_centers[:, 0]) + half_len + pad),
    ])
    ax.set_xlim([-0.75,0.75])
    ax.set_ylim([-0.75,0.75])
    ax.set_aspect("equal")
    ax.set_xlabel(r"$\tilde{X}$")
    ax.set_ylabel(r"$\tilde{Z}$")
    ax.grid(True, alpha=0.3)

    ax.plot(
        fore_centers[:, 0], fore_centers[:, 1], "-", color=style.muted_text_color, linewidth=0.5
    )
    ax.plot(
        hind_centers[:, 0], hind_centers[:, 1], "-", color=style.muted_text_color, linewidth=0.5
    )

    le_marker = dict(
        marker="o",
        markersize=4,
        markerfacecolor=style.axes_facecolor,
        markeredgewidth=1.8,
        linestyle="none",
    )

    fore_stick, = ax.plot([], [], "-", color=style.body_color, linewidth=2.2)
    hind_stick, = ax.plot([], [], "-", color=style.body_color, linewidth=2.2)
    fore_le_marker, = ax.plot([], [], markeredgecolor=style.body_color, **le_marker)
    hind_le_marker, = ax.plot([], [], markeredgecolor=style.body_color, **le_marker)
    time_text = ax.text(0.02, 0.98, "", transform=ax.transAxes, va="top", ha="left", color=style.text_color)

    fig.tight_layout()

    def update(frame):
        fore_stick.set_data(
            [fore_trailing[frame, 0], fore_leading[frame, 0]],
            [fore_trailing[frame, 1], fore_leading[frame, 1]],
        )
        hind_stick.set_data(
            [hind_trailing[frame, 0], hind_leading[frame, 0]],
            [hind_trailing[frame, 1], hind_leading[frame, 1]],
        )
        fore_le_marker.set_data([fore_leading[frame, 0]], [fore_leading[frame, 1]])
        hind_le_marker.set_data([hind_leading[frame, 0]], [hind_leading[frame, 1]])
        time_text.set_text(r"$t/T_{wb} = %.2f$" % (time[frame] * time_scale))
        return fore_stick, hind_stick, fore_le_marker, hind_le_marker, time_text

    # Create animation
    anim = ani.FuncAnimation(fig, update, frames=n_frames, interval=50, blit=False)

    save_animation(anim, outfile, fps=20)

    plt.close()
    print(f"Saved: {outfile}")


def main():
    parser = argparse.ArgumentParser(description="Visualize right fore/hind wing sticks in nondimensional (X,Z)")
    parser.add_argument("input", help="Input HDF5 simulation file")
    parser.add_argument("arg2", nargs="?", help="Output animation (.mp4/.gif), or legacy wing name")
    parser.add_argument("arg3", nargs="?", help="Legacy output animation path when arg2 is a wing name")
    parser.add_argument(
        "--theme",
        choices=["light", "dark"],
        default="light",
        help="Plot theme (default: light)",
    )
    parser.add_argument(
        "--station",
        type=float,
        default=2.0 / 3.0,
        help="Wing station along span in [0,1] (default: 2/3)",
    )
    args = parser.parse_args()

    print(f"Reading {args.input}...")
    params, time, _, wings = read_simulation(args.input)

    if not args.arg2:
        print("\nUsage: python -m post.plot_stick <input.h5> <output.mp4|gif> [--theme light|dark] [--station 0.6667]")
        print("Legacy usage (still supported):")
        print("  python -m post.plot_stick <input.h5> <wing_name> <output.mp4|gif> [--theme light|dark] [--station 0.6667]")
        print("\nAvailable wings:")
        for name in wings[0].keys():
            print(f"  {name}")
        sys.exit(1)

    if args.arg3:
        print(f"Note: ignoring explicit wing '{args.arg2}'. This utility now renders right fore/hind wings.")
        outfile = args.arg3
    else:
        outfile = args.arg2

    try:
        fore_wing_name, hind_wing_name = resolve_right_wings(wings[0].keys())
    except ValueError as exc:
        print(f"Error: {exc}")
        print("Available wings:")
        for name in wings[0].keys():
            print(f"  {name}")
        sys.exit(1)

    station = float(args.station)
    if station < 0.0 or station > 1.0:
        print(f"Error: --station must be in [0, 1], got {station}")
        sys.exit(1)

    wing_lb0 = params.get("wing_lb0", {})
    fore_lambda0 = float(wing_lb0.get(fore_wing_name, 1.0))
    hind_lambda0 = float(wing_lb0.get(hind_wing_name, 1.0))
    omega = float(params["omega"])

    print(f"Fore wing: {fore_wing_name} (X offset {FORE_X_OFFSET:+.3f})")
    print(f"Hind wing: {hind_wing_name} (X offset {HIND_X_OFFSET:+.3f})")
    print(f"Relative X offset: {FORE_X_OFFSET - HIND_X_OFFSET:.3f}")
    print(f"Fore lambda0 (lb0): {fore_lambda0:.4f}")
    print(f"Hind lambda0 (lb0): {hind_lambda0:.4f}")
    print(f"Wing station: {station:.4f}")
    print(f"Stick length (nondimensional): {STICK_LENGTH:.3f}")
    print(f"Frames: {len(time)}")

    style = resolve_style(theme=args.theme)
    print(f"Theme: {style.theme}")

    print("Creating animation...")
    animate_stroke(
        time,
        wings,
        fore_wing_name,
        hind_wing_name,
        outfile,
        style=style,
        station=station,
        fore_lambda0=fore_lambda0,
        hind_lambda0=hind_lambda0,
        omega=omega,
    )


if __name__ == "__main__":
    main()

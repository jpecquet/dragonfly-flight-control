#!/usr/bin/env python3
"""
Visualize right forewing/hindwing stick motion in the (X,Z) plane.

Usage:
    python -m post.plot_stick <input.h5> <output.mp4|gif> [--theme light|dark]
      [--station 0.6667] [--stations 0.25 0.5 0.75]

Example:
    python -m post.plot_stick output.h5 stroke.mp4 --theme dark --stations 0.25 0.5 0.75
"""

import argparse
import sys
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as ani

from post.animation import save_animation
from post.io import read_simulation
from post.style import apply_matplotlib_style, figure_size, resolve_style

STICK_LENGTH = 0.1
WING_ROOT_DISTANCE = 0.1
WING_ROOT_MIDPOINT_Z = 0.04
WING_ROOT_TILT_DEG = 15.0
PERIOD_TOL = 1e-9
STICK_FPS = 20
STICK_BITRATE = 4000
STICK_DPI = 200

SILHOUETTE_PATH = Path(__file__).resolve().parent.parent / "assets" / "dragonfly_silhouette.csv"


def wing_root_positions(distance=WING_ROOT_DISTANCE, midpoint_z=WING_ROOT_MIDPOINT_Z, tilt_deg=WING_ROOT_TILT_DEG):
    """Compute fore and hind wing root (x, z) from distance, midpoint z, and tilt angle."""
    half = 0.5 * float(distance)
    tilt = np.radians(float(tilt_deg))
    dx, dz = half * np.cos(tilt), half * np.sin(tilt)
    fore_root = np.array([dx, midpoint_z + dz], dtype=float)
    hind_root = np.array([-dx, midpoint_z - dz], dtype=float)
    return fore_root, hind_root


def load_silhouette():
    """Load the processed dragonfly silhouette as (N, 2) array of (x, z) coords."""
    return np.loadtxt(SILHOUETTE_PATH, delimiter=",")


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


def _to_float(value):
    if value is None:
        return None
    arr = np.asarray(value)
    if arr.shape == ():
        return float(arr)
    return None


def is_single_wingbeat_periodic(params, *, tol=PERIOD_TOL):
    """Return True when available harmonic-period metadata indicates 1-wingbeat periodicity."""
    periods = []

    global_period = _to_float(params.get("harmonic_period_wingbeats"))
    if global_period is not None:
        periods.append(global_period)

    wing_periods = params.get("wing_harmonic_period_wingbeats")
    if isinstance(wing_periods, dict):
        for value in wing_periods.values():
            period = _to_float(value)
            if period is not None:
                periods.append(period)

    if not periods:
        return False

    for period in periods:
        if not np.isfinite(period):
            return False
        if abs(period - 1.0) > float(tol):
            return False
    return True


def trim_to_wingbeats(time, wings, omega, n_wingbeats):
    """Trim time/wing arrays to the first n_wingbeats based on omega."""
    omega = float(omega)
    n_wingbeats = float(n_wingbeats)
    if omega <= 0.0:
        raise ValueError("omega must be > 0")
    if n_wingbeats <= 0.0:
        raise ValueError("n_wingbeats must be > 0")
    if len(time) == 0:
        return time, wings

    wingbeat_time = np.asarray(time, dtype=float) * omega / (2.0 * np.pi)
    cutoff = float(wingbeat_time[0]) + n_wingbeats + 1e-12
    keep = np.nonzero(wingbeat_time <= cutoff)[0]
    if keep.size == 0:
        return time, wings
    last = int(keep[-1] + 1)
    if last >= len(time):
        return time, wings

    trimmed_time = np.asarray(time)[:last]
    trimmed_wings = {
        wing_name: {key: np.asarray(values)[:last] for key, values in wing_data.items()}
        for wing_name, wing_data in wings.items()
    }
    return trimmed_time, trimmed_wings


def resolve_stations(single_station, multi_stations):
    """Resolve station list from CLI inputs."""
    if multi_stations is None or len(multi_stations) == 0:
        values = [float(single_station)]
    else:
        values = [float(v) for v in multi_stations]
    for station in values:
        if station < 0.0 or station > 1.0:
            raise ValueError(f"station must be in [0, 1], got {station}")
    return values


def _stroke_plane_normal_xz(gamma):
    """Return the stroke-plane normal projected into the (X, Z) stick-plot plane."""
    return np.array([np.sin(float(gamma)), np.cos(float(gamma))], dtype=float)


def _span_vector_xz_with_mean_beta(e_r, stroke_normal_xz, beta_mean):
    """Project span vector into XZ while replacing instantaneous beta with beta_mean."""
    p = project_xz(e_r)
    n = np.asarray(stroke_normal_xz, dtype=float)
    n_norm = float(np.linalg.norm(n))
    if n_norm < 1e-12:
        return p
    n = n / n_norm

    # In XZ, p decomposes into a stroke-normal offset term sin(beta)*n plus a
    # tangent term scaled by cos(beta). Replace beta with its mean while
    # preserving the tangent phase (the projected azimuthal factor).
    s_inst = float(np.clip(np.dot(p, n), -1.0, 1.0))
    t = p - s_inst * n
    c_inst = float(np.sqrt(max(0.0, 1.0 - s_inst * s_inst)))
    s_ref = float(np.sin(float(beta_mean)))
    c_ref = float(np.cos(float(beta_mean)))
    if c_inst < 1e-12:
        t_ref = np.zeros_like(t)
    else:
        t_ref = (c_ref / c_inst) * t
    return s_ref * n + t_ref


def _draw_pitch_glyph_2d(ax, center, chord_dir_2d, stroke_normal_2d, label_text,
                          *, color, alpha, line_width, fontsize,
                          ref_length=0.12, arc_radius=0.10):
    """Draw a pitch-angle arc glyph in 2D and return the created artists."""
    artists = []
    cn = float(np.linalg.norm(chord_dir_2d))
    sn = float(np.linalg.norm(stroke_normal_2d))
    if cn < 1e-12 or sn < 1e-12:
        return artists
    c_hat = np.asarray(chord_dir_2d, dtype=float) / cn
    s_hat = np.asarray(stroke_normal_2d, dtype=float) / sn

    # Reference lines from center.
    ref_end = center + ref_length * s_hat
    chord_end = center + ref_length * c_hat
    artists.append(ax.plot(
        [center[0], ref_end[0]], [center[1], ref_end[1]],
        "--", color=color, linewidth=line_width * 0.7, alpha=alpha,
    )[0])
    artists.append(ax.plot(
        [center[0], chord_end[0]], [center[1], chord_end[1]],
        "--", color=color, linewidth=line_width * 0.7, alpha=alpha,
    )[0])

    # Signed angle from stroke normal to chord direction.
    angle_ref = float(np.arctan2(s_hat[1], s_hat[0]))
    angle_chord = float(np.arctan2(c_hat[1], c_hat[0]))
    delta = angle_chord - angle_ref
    delta = (delta + np.pi) % (2.0 * np.pi) - np.pi

    if abs(delta) < 1e-6:
        # Angle ≈ 0 — just show the label near the reference end.
        artists.append(ax.text(
            float(ref_end[0]), float(ref_end[1]), label_text,
            color=color, fontsize=fontsize, ha="center", va="center",
        ))
        return artists

    sign = 1.0 if delta >= 0.0 else -1.0

    # Arrowhead sizing: fixed chord-length of 0.014, derive angular gap
    # so the triangle always bridges exactly from arc end to head tip.
    max_head_chord = 0.014
    head_angle = 2.0 * float(np.arcsin(np.clip(
        max_head_chord / (2.0 * arc_radius), 0.0, 1.0,
    )))
    head_angle = min(head_angle, 0.45 * abs(delta))
    arc_end_angle = delta - sign * head_angle
    if sign * arc_end_angle <= 0.0:
        arc_end_angle = 0.6 * delta

    theta = np.linspace(0.0, arc_end_angle, 48) + angle_ref
    arc_x = center[0] + arc_radius * np.cos(theta)
    arc_y = center[1] + arc_radius * np.sin(theta)
    artists.append(ax.plot(arc_x, arc_y, "-", color=color, linewidth=line_width, alpha=alpha)[0])

    # Arrowhead triangle — base is the arc end, tip is the angle endpoint.
    head_tip_angle = angle_ref + delta
    head_tip = center + arc_radius * np.array([np.cos(head_tip_angle), np.sin(head_tip_angle)])
    head_base = np.array([arc_x[-1], arc_y[-1]], dtype=float)
    head_vec = head_tip - head_base
    head_len = float(np.linalg.norm(head_vec))
    if head_len > 1e-12:
        t_hat = head_vec / head_len
        n_hat = np.array([-t_hat[1], t_hat[0]], dtype=float)
        head_w = 0.25 * min(head_len, max_head_chord)
        tri_x = [float(head_tip[0]), float(head_base[0] + head_w * n_hat[0]), float(head_base[0] - head_w * n_hat[0])]
        tri_y = [float(head_tip[1]), float(head_base[1] + head_w * n_hat[1]), float(head_base[1] - head_w * n_hat[1])]
        artists.append(ax.fill(tri_x, tri_y, color=color, alpha=alpha)[0])

    # Label at midpoint of arc, offset outward.
    mid_angle = angle_ref + 0.5 * delta
    label_r = arc_radius + 0.045
    label_pos = center + label_r * np.array([np.cos(mid_angle), np.sin(mid_angle)])
    artists.append(ax.text(
        float(label_pos[0]), float(label_pos[1]), label_text,
        color=color, fontsize=fontsize, ha="center", va="center",
    ))
    return artists


def project_xz(vec):
    return np.array([vec[0], vec[2]], dtype=float)


def compute_stick_endpoints(
    wing_state,
    root_offset,
    stick_length,
    station,
    lambda0,
    *,
    stroke_normal_xz=None,
    beta_mean=None,
):
    """Compute stick center, leading edge, and trailing edge in nondimensional (X,Z)."""
    if not 0.0 <= float(station) <= 1.0:
        raise ValueError("station must be in [0, 1]")
    span_xz = project_xz(wing_state["e_r"])
    if stroke_normal_xz is not None and beta_mean is not None:
        span_xz = _span_vector_xz_with_mean_beta(wing_state["e_r"], stroke_normal_xz, beta_mean)
    center = float(station) * float(lambda0) * span_xz + np.asarray(root_offset, dtype=float)

    chord_dir = project_xz(wing_state["e_c"])
    chord_norm = np.linalg.norm(chord_dir)
    if chord_norm < 1e-9:
        chord_dir = np.array([1.0, 0.0], dtype=float)
    else:
        chord_dir = chord_dir / chord_norm

    length = float(stick_length)
    leading = center + 0.25 * length * chord_dir
    trailing = center - 0.75 * length * chord_dir
    return center, leading, trailing


def animate_stroke(
    time,
    wings,
    fore_wing_name,
    hind_wing_name,
    outfile,
    omega,
    style=None,
    stations=(2.0 / 3.0,),
    fore_lambda0=1.0,
    hind_lambda0=1.0,
    show_axes=True,
    show_grid=True,
    show_timestamp=True,
    show_pitch_angle=False,
    params=None,
    stroke_plane_beta_mode="mean",
):
    """Create animation of right fore/hind stick motion in the nondimensional (X,Z) plane."""
    style = resolve_style(style)
    apply_matplotlib_style(style)
    omega = float(omega)
    time_scale = omega / (2.0 * np.pi)

    fig, ax = plt.subplots(figsize=figure_size(0.6 if show_axes else 0.4))
    stations = tuple(float(s) for s in stations)
    n_frames = len(time)
    n_stations = len(stations)
    fore_centers = np.zeros((n_stations, n_frames, 2), dtype=float)
    hind_centers = np.zeros((n_stations, n_frames, 2), dtype=float)
    fore_leading = np.zeros((n_stations, n_frames, 2), dtype=float)
    fore_trailing = np.zeros((n_stations, n_frames, 2), dtype=float)
    hind_leading = np.zeros((n_stations, n_frames, 2), dtype=float)
    hind_trailing = np.zeros((n_stations, n_frames, 2), dtype=float)

    fore_wing = wings[fore_wing_name]
    hind_wing = wings[hind_wing_name]
    fore_root, hind_root = wing_root_positions()
    fore_stroke_normal = None
    hind_stroke_normal = None
    fore_beta_mean = None
    hind_beta_mean = None
    beta_mode = str(stroke_plane_beta_mode).strip().lower()
    if beta_mode not in {"mean", "actual"}:
        raise ValueError(f"stroke_plane_beta_mode must be 'mean' or 'actual' (got {stroke_plane_beta_mode!r})")
    if params is not None and beta_mode == "mean":
        gamma_map = params.get("wing_gamma_mean", {})
        cone_mean_map = params.get("wing_cone_mean", {})
        cone_static_map = params.get("wing_cone_angle", {})
        if fore_wing_name in gamma_map:
            fore_stroke_normal = _stroke_plane_normal_xz(gamma_map[fore_wing_name])
        if hind_wing_name in gamma_map:
            hind_stroke_normal = _stroke_plane_normal_xz(gamma_map[hind_wing_name])
        if fore_wing_name in cone_mean_map:
            fore_beta_mean = float(cone_mean_map[fore_wing_name])
        elif fore_wing_name in cone_static_map:
            fore_beta_mean = float(cone_static_map[fore_wing_name])
        if hind_wing_name in cone_mean_map:
            hind_beta_mean = float(cone_mean_map[hind_wing_name])
        elif hind_wing_name in cone_static_map:
            hind_beta_mean = float(cone_static_map[hind_wing_name])

    for station_idx, station in enumerate(stations):
        for i in range(n_frames):
            fore_state = {k: v[i] for k, v in fore_wing.items() if not isinstance(v, dict)}
            hind_state = {k: v[i] for k, v in hind_wing.items() if not isinstance(v, dict)}
            fore_center, fore_le, fore_te = compute_stick_endpoints(
                fore_state,
                root_offset=fore_root,
                stick_length=STICK_LENGTH,
                station=station,
                lambda0=fore_lambda0,
                stroke_normal_xz=fore_stroke_normal,
                beta_mean=fore_beta_mean,
            )
            hind_center, hind_le, hind_te = compute_stick_endpoints(
                hind_state,
                root_offset=hind_root,
                stick_length=STICK_LENGTH,
                station=station,
                lambda0=hind_lambda0,
                stroke_normal_xz=hind_stroke_normal,
                beta_mean=hind_beta_mean,
            )
            fore_centers[station_idx, i] = fore_center
            hind_centers[station_idx, i] = hind_center
            fore_leading[station_idx, i] = fore_le
            fore_trailing[station_idx, i] = fore_te
            hind_leading[station_idx, i] = hind_le
            hind_trailing[station_idx, i] = hind_te

    all_centers = np.vstack([fore_centers.reshape(-1, 2), hind_centers.reshape(-1, 2)])
    half_len = 0.5 * STICK_LENGTH
    pad = 0.08
    ax.set_xlim([
        float(np.min(all_centers[:, 0]) - half_len - pad),
        float(np.max(all_centers[:, 0]) + half_len + pad),
    ])
    if show_axes:
        ax.set_xlim([-1, 0.6])
        ax.set_ylim([-0.8, 0.8])
    else:
        all_pts = np.vstack([
            fore_leading.reshape(-1, 2), fore_trailing.reshape(-1, 2),
            hind_leading.reshape(-1, 2), hind_trailing.reshape(-1, 2),
        ])
        if SILHOUETTE_PATH.exists():
            all_pts = np.vstack([all_pts, load_silhouette()])
        if show_pitch_angle:
            # Include pitch-glyph extent (label_r = arc_radius + 0.045).
            glyph_r = 0.145
            si = n_stations - 1
            glyph_centers = np.vstack([fore_centers[si], hind_centers[si]])
            offsets = np.array([[glyph_r, 0], [-glyph_r, 0], [0, glyph_r], [0, -glyph_r]], dtype=float)
            glyph_pts = glyph_centers[:, None, :] + offsets[None, :, :]
            all_pts = np.vstack([all_pts, glyph_pts.reshape(-1, 2)])
        ax.set_xlim([float(np.min(all_pts[:, 0]) - pad), float(np.max(all_pts[:, 0]) + pad)])
        ax.set_ylim([float(np.min(all_pts[:, 1]) - pad), float(np.max(all_pts[:, 1]) + pad)])
    ax.set_aspect("equal")
    if show_axes:
        ax.set_xlabel(r"$\tilde{X}$")
        ax.set_ylabel(r"$\tilde{Z}$")
    else:
        ax.set_axis_off()
    if show_grid:
        ax.grid(True, alpha=0.3)

    if SILHOUETTE_PATH.exists():
        silhouette = load_silhouette()
        ax.plot(
            silhouette[:, 0],
            silhouette[:, 1],
            "-",
            color=style.muted_text_color,
            linewidth=0.5,
        )

    for root in (fore_root, hind_root):
        ax.plot(root[0], root[1], ".", color=style.muted_text_color, markersize=4)

    for station_idx in range(n_stations):
        ax.plot(
            fore_centers[station_idx, :, 0],
            fore_centers[station_idx, :, 1],
            "-",
            color=style.muted_text_color,
            linewidth=0.5,
        )
        ax.plot(
            hind_centers[station_idx, :, 0],
            hind_centers[station_idx, :, 1],
            "-",
            color=style.muted_text_color,
            linewidth=0.5,
        )

    le_marker = dict(
        marker="o",
        markersize=4,
        markerfacecolor=style.axes_facecolor,
        markeredgewidth=1.8,
        linestyle="none",
    )

    fore_sticks = [ax.plot([], [], "-", color=style.body_color, linewidth=2.2)[0] for _ in range(n_stations)]
    hind_sticks = [ax.plot([], [], "-", color=style.body_color, linewidth=2.2)[0] for _ in range(n_stations)]
    fore_le_markers = [ax.plot([], [], markeredgecolor=style.body_color, **le_marker)[0] for _ in range(n_stations)]
    hind_le_markers = [ax.plot([], [], markeredgecolor=style.body_color, **le_marker)[0] for _ in range(n_stations)]
    time_text = ax.text(0.02, 0.98, "", transform=ax.transAxes, va="top", ha="left", color=style.text_color) if show_timestamp else None

    # Pitch-angle glyph state.
    pitch_glyph_cfg = None
    if show_pitch_angle and params is not None:
        gamma_map = params.get("wing_gamma_mean", {})
        fore_gamma = gamma_map.get(fore_wing_name)
        hind_gamma = gamma_map.get(hind_wing_name)
        if fore_gamma is not None and hind_gamma is not None:
            pitch_glyph_cfg = {
                "fore_normal": _stroke_plane_normal_xz(fore_gamma),
                "hind_normal": _stroke_plane_normal_xz(hind_gamma),
                "station_idx": n_stations - 1,
            }
    _pitch_artists = []

    fig.tight_layout()

    def update(frame):
        nonlocal _pitch_artists
        # Remove previous pitch-angle artists.
        for a in _pitch_artists:
            a.remove()
        _pitch_artists.clear()

        artists = []
        for station_idx in range(n_stations):
            fore_sticks[station_idx].set_data(
                [fore_trailing[station_idx, frame, 0], fore_leading[station_idx, frame, 0]],
                [fore_trailing[station_idx, frame, 1], fore_leading[station_idx, frame, 1]],
            )
            hind_sticks[station_idx].set_data(
                [hind_trailing[station_idx, frame, 0], hind_leading[station_idx, frame, 0]],
                [hind_trailing[station_idx, frame, 1], hind_leading[station_idx, frame, 1]],
            )
            fore_le_markers[station_idx].set_data(
                [fore_leading[station_idx, frame, 0]],
                [fore_leading[station_idx, frame, 1]],
            )
            hind_le_markers[station_idx].set_data(
                [hind_leading[station_idx, frame, 0]],
                [hind_leading[station_idx, frame, 1]],
            )
            artists.extend(
                [
                    fore_sticks[station_idx],
                    hind_sticks[station_idx],
                    fore_le_markers[station_idx],
                    hind_le_markers[station_idx],
                ]
            )
        if time_text is not None:
            time_text.set_text(r"$t/T_{wb} = %.2f$" % (time[frame] * time_scale))
            artists.append(time_text)

        if pitch_glyph_cfg is not None:
            si = pitch_glyph_cfg["station_idx"]
            fore_chord = fore_leading[si, frame] - fore_trailing[si, frame]
            _pitch_artists.extend(_draw_pitch_glyph_2d(
                ax, fore_centers[si, frame], fore_chord,
                pitch_glyph_cfg["fore_normal"],
                r"$\psi_f$",
                color=style.text_color, alpha=0.9, line_width=1.0,
                fontsize=style.font_size,
            ))
            hind_chord = hind_leading[si, frame] - hind_trailing[si, frame]
            _pitch_artists.extend(_draw_pitch_glyph_2d(
                ax, hind_centers[si, frame], hind_chord,
                pitch_glyph_cfg["hind_normal"],
                r"$\psi_h$",
                color=style.text_color, alpha=0.9, line_width=1.0,
                fontsize=style.font_size,
            ))
            artists.extend(_pitch_artists)

        return tuple(artists)

    # Create animation
    anim = ani.FuncAnimation(fig, update, frames=n_frames, interval=50, blit=False)

    save_animation(
        anim,
        outfile,
        fps=STICK_FPS,
        bitrate=STICK_BITRATE,
        dpi=STICK_DPI,
        progress_label="Matplotlib",
    )

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
        help="Single wing station along span in [0,1] (default: 2/3). Ignored if --stations is set.",
    )
    parser.add_argument(
        "--stations",
        type=float,
        nargs="+",
        default=None,
        help="One or more wing stations along span in [0,1] (example: --stations 0.25 0.5 0.75).",
    )
    parser.add_argument(
        "--stroke-plane-beta",
        choices=["mean", "actual"],
        default="mean",
        help="Stroke-plane projection mode for stick centers: use mean beta (default) or actual simulated beta.",
    )
    args = parser.parse_args()

    print(f"Reading {args.input}...")
    params, time, _, wings = read_simulation(args.input)

    if not args.arg2:
        print(
            "\nUsage: python -m post.plot_stick <input.h5> <output.mp4|gif> "
            "[--theme light|dark] [--station 0.6667] [--stations 0.25 0.5 0.75] "
            "[--stroke-plane-beta mean|actual]"
        )
        print("Legacy usage (still supported):")
        print(
            "  python -m post.plot_stick <input.h5> <wing_name> <output.mp4|gif> "
            "[--theme light|dark] [--station 0.6667] [--stations 0.25 0.5 0.75] "
            "[--stroke-plane-beta mean|actual]"
        )
        print("\nAvailable wings:")
        for name in wings.keys():
            print(f"  {name}")
        sys.exit(1)

    if args.arg3:
        print(f"Note: ignoring explicit wing '{args.arg2}'. This utility now renders right fore/hind wings.")
        outfile = args.arg3
    else:
        outfile = args.arg2

    try:
        fore_wing_name, hind_wing_name = resolve_right_wings(wings.keys())
    except ValueError as exc:
        print(f"Error: {exc}")
        print("Available wings:")
        for name in wings.keys():
            print(f"  {name}")
        sys.exit(1)

    try:
        stations = resolve_stations(args.station, args.stations)
    except ValueError as exc:
        print(f"Error: {exc}")
        sys.exit(1)

    wing_lb0 = params.get("wing_lb0", {})
    fore_lambda0 = float(wing_lb0.get(fore_wing_name, 1.0))
    hind_lambda0 = float(wing_lb0.get(hind_wing_name, 1.0))
    omega = float(params["omega"])

    if is_single_wingbeat_periodic(params):
        trimmed_time, trimmed_wings = trim_to_wingbeats(time, wings, omega, n_wingbeats=1.0)
        if len(trimmed_time) < len(time):
            print("Detected 1-wingbeat-periodic kinematics; limiting stick plot to first wingbeat.")
            time = trimmed_time
            wings = trimmed_wings

    fore_root, hind_root = wing_root_positions()
    print(f"Fore wing: {fore_wing_name} (root {fore_root[0]:+.4f}, {fore_root[1]:+.4f})")
    print(f"Hind wing: {hind_wing_name} (root {hind_root[0]:+.4f}, {hind_root[1]:+.4f})")
    print(f"Wing root distance: {WING_ROOT_DISTANCE:.3f}, midpoint Z: {WING_ROOT_MIDPOINT_Z:.3f}, tilt: {WING_ROOT_TILT_DEG:.1f} deg")
    print(f"Fore lambda0 (lb0): {fore_lambda0:.4f}")
    print(f"Hind lambda0 (lb0): {hind_lambda0:.4f}")
    print("Wing stations: " + ", ".join(f"{station:.4f}" for station in stations))
    print(f"Stick length (nondimensional): {STICK_LENGTH:.3f}")
    print(f"Frames: {len(time)}")
    print(f"Stroke-plane beta mode: {args.stroke_plane_beta}")

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
        stations=stations,
        fore_lambda0=fore_lambda0,
        hind_lambda0=hind_lambda0,
        omega=omega,
        params=params,
        stroke_plane_beta_mode=args.stroke_plane_beta,
    )


if __name__ == "__main__":
    main()

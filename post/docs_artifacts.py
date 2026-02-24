#!/usr/bin/env python3
"""Generic docs-media artifact helpers shared across cases."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any


def compute_body_flight_metrics(
    h5_path: Path,
    *,
    body_length_m: float,
    gravity_m_s2: float,
) -> dict[str, Any]:
    """Read HDF5 and compute dimensional body speed/direction over wingbeats."""
    import h5py
    import numpy as np

    with h5py.File(str(h5_path), "r") as f:
        time = np.asarray(f["/time"][:], dtype=float)
        states = np.asarray(f["/state"][:], dtype=float)
        omega_nondim = float(f["/parameters/omega"][()])

    speed_scale = math.sqrt(float(gravity_m_s2) * float(body_length_m))
    speed_m_s = np.linalg.norm(states[:, 3:6], axis=1) * speed_scale
    direction_deg = np.degrees(np.arctan2(states[:, 2], states[:, 0]))
    wingbeats = time * omega_nondim / (2.0 * np.pi)

    return {
        "wingbeats": wingbeats,
        "speed_m_s": speed_m_s,
        "direction_deg": direction_deg,
    }


def plot_body_flight_metrics_vs_reference(
    h5_path: Path,
    output_path: Path,
    *,
    body_length_m: float,
    gravity_m_s2: float,
    references: list[dict[str, Any]],
    theme: str | None = None,
) -> None:
    """Plot simulation body speed and direction against optional flight-condition refs."""
    import matplotlib.pyplot as plt
    import numpy as np

    from post.style import apply_matplotlib_style, figure_size, resolve_style

    metrics = compute_body_flight_metrics(
        h5_path,
        body_length_m=body_length_m,
        gravity_m_s2=gravity_m_s2,
    )
    t = np.asarray(metrics["wingbeats"])
    speed = np.asarray(metrics["speed_m_s"])
    direction = np.asarray(metrics["direction_deg"])

    speed_ref = None
    direction_ref = None
    for ref in references:
        if ref.get("kind") == "flight_condition":
            speed_ref = float(ref["speed"])
            direction_ref = float(ref["direction"])

    style = resolve_style(theme=theme)
    apply_matplotlib_style(style)

    fig, axes = plt.subplots(
        nrows=1,
        ncols=2,
        sharex=True,
        figsize=figure_size(height_over_width=0.4),
    )
    speed_ax, dir_ax = axes

    speed_ax.plot(t, speed, linewidth=1.5, color="C0", label="Simulation")
    if speed_ref is not None:
        speed_ax.axhline(speed_ref, linewidth=1.2, color="C1", linestyle="--", label="Reference")
    speed_ax.set_xlabel(r"$t/T_{wb}$")
    speed_ax.set_ylabel("Speed (m/s)")
    speed_ax.grid(True, alpha=0.25)
    speed_ax.legend(loc="best")

    dir_ax.plot(t, direction, linewidth=1.5, color="C0", label="Simulation")
    if direction_ref is not None:
        dir_ax.axhline(direction_ref, linewidth=1.2, color="C1", linestyle="--", label="Reference")
    dir_ax.set_xlabel(r"$t/T_{wb}$")
    dir_ax.set_ylabel("Direction (deg)")
    dir_ax.grid(True, alpha=0.25)
    dir_ax.legend(loc="best")

    speed_ax.set_xlim(float(t[0]), float(t[-1]))
    fig.tight_layout()

    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(str(output_path), dpi=300, bbox_inches="tight")
    plt.close(fig)


def _eval_fourier_degrees(t_wb, mean_deg: float, harmonics: list[list[float]] | list[tuple[float, float]]) -> Any:
    """Evaluate mean + cosine harmonic series in degrees."""
    import numpy as np

    result = np.full_like(t_wb, float(mean_deg))
    for k, (amp_deg, phase_deg) in enumerate(harmonics, start=1):
        result += float(amp_deg) * np.cos(k * 2.0 * np.pi * t_wb + np.radians(float(phase_deg)))
    return result


def plot_case_fore_hind_kinematics(
    case: dict[str, Any],
    output_path: Path,
    *,
    angle_keys: tuple[str, str] = ("phi", "psi"),
    wing_names: tuple[str, str] = ("fore", "hind"),
    ylabels: tuple[str, str] = (r"$\phi$ (deg)", r"$\psi$ (deg)"),
    n_points: int = 500,
    theme: str | None = None,
) -> None:
    """Plot fore/hind angle time series over one wingbeat from a case dict."""
    import numpy as np

    from post.time_series import plot_fore_hind_series

    if len(angle_keys) != 2 or len(ylabels) != 2:
        raise ValueError("angle_keys and ylabels must each contain exactly two entries")

    fore_name, hind_name = wing_names
    t_wb = np.linspace(0.0, 1.0, int(n_points))
    series: dict[str, Any] = {"t": t_wb}

    for wing_name in (fore_name, hind_name):
        kin = case["wings"][wing_name]["kinematics"]
        for angle in angle_keys:
            entry = kin[angle]
            series[f"{wing_name}_{angle}"] = _eval_fourier_degrees(
                t_wb,
                float(entry["mean"]),
                entry.get("harmonics", []),
            )

    rows = [
        (f"{fore_name}_{angle_keys[0]}", f"{hind_name}_{angle_keys[0]}", ylabels[0]),
        (f"{fore_name}_{angle_keys[1]}", f"{hind_name}_{angle_keys[1]}", ylabels[1]),
    ]
    plot_fore_hind_series(output_path, series, rows, theme=theme)


def render_simulation_video_from_h5(
    input_h5: Path,
    output_video: Path,
    *,
    render_config: Path,
    theme: str | None = None,
    no_blender: bool = False,
    frame_step: int = 1,
    annotation_overlay: dict | None = None,
) -> None:
    """Render a simulation video from an HDF5 file (Blender hybrid or mpl fallback)."""
    from post.composite import check_blender_available, render_hybrid, render_mpl_only
    from post.hybrid_config import HybridConfig
    from post.io import read_simulation
    from post.style import apply_theme_to_config

    params, time, states, wings = read_simulation(str(input_h5))
    config = HybridConfig.load(str(render_config))
    config = apply_theme_to_config(config, theme)

    if no_blender:
        print("Blender disabled via --no-blender; using matplotlib-only fallback")
        if annotation_overlay:
            print("Note: annotation_overlay is currently only applied in hybrid (Blender) rendering mode.")
        render_mpl_only(states, wings, params, str(output_video), config=config, frame_step=int(frame_step))
        return

    if check_blender_available():
        render_hybrid(
            states,
            wings,
            params,
            str(input_h5),
            str(output_video),
            time=time,
            config=config,
            frame_step=int(frame_step),
            annotation_overlay=annotation_overlay,
        )
        return

    print("Warning: Blender not available, using matplotlib-only fallback")
    if annotation_overlay:
        print("Note: annotation_overlay is currently only applied in hybrid (Blender) rendering mode.")
    render_mpl_only(states, wings, params, str(output_video), config=config, frame_step=int(frame_step))


def render_stick_video_from_h5(
    input_h5: Path,
    output_video: Path,
    *,
    theme: str | None = None,
    stations: list[float] | tuple[float, ...] | None = None,
    show_axes: bool = True,
    show_grid: bool = True,
    show_timestamp: bool = True,
    show_pitch_angle: bool = False,
) -> None:
    """Render a fore/hind stick video from an HDF5 file."""
    from post.io import read_simulation
    from post.plot_stick import (
        animate_stroke,
        is_single_wingbeat_periodic,
        resolve_right_wings,
        trim_to_wingbeats,
    )
    from post.style import resolve_style

    params, time, _, wings = read_simulation(str(input_h5))
    fore_wing_name, hind_wing_name = resolve_right_wings(wings.keys())

    omega = float(params["omega"])
    if is_single_wingbeat_periodic(params):
        time, wings = trim_to_wingbeats(time, wings, omega, n_wingbeats=1.0)

    wing_lb0 = params.get("wing_lb0", {})
    fore_lambda0 = float(wing_lb0.get(fore_wing_name, 1.0))
    hind_lambda0 = float(wing_lb0.get(hind_wing_name, 1.0))
    style = resolve_style(theme=theme)
    station_values = tuple(float(s) for s in (stations or [2.0 / 3.0]))

    animate_stroke(
        time,
        wings,
        fore_wing_name,
        hind_wing_name,
        str(output_video),
        omega=omega,
        style=style,
        stations=station_values,
        fore_lambda0=fore_lambda0,
        hind_lambda0=hind_lambda0,
        show_axes=show_axes,
        show_grid=show_grid,
        show_timestamp=show_timestamp,
        show_pitch_angle=show_pitch_angle,
        params=params,
    )

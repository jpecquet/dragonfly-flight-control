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


def plot_exp_kinematics_scatter(
    case: dict[str, Any],
    csv_path: Path,
    output_path: Path,
    *,
    theme: str | None = None,
) -> None:
    """Plot experimental phi and psi as wrapped scatter over one wingbeat."""
    import matplotlib.pyplot as plt
    import numpy as np
    import pandas as pd

    from post.style import apply_matplotlib_style, figure_size, resolve_style

    df = pd.read_csv(csv_path)
    t_s = df["t"].to_numpy(dtype=float)
    s_mm_fore = df["s_mm_fore"].to_numpy(dtype=float)
    s_mm_hind = df["s_mm_hind"].to_numpy(dtype=float)
    beta_deg_fore = df["beta_deg_fore"].to_numpy(dtype=float)
    beta_deg_hind = df["beta_deg_hind"].to_numpy(dtype=float)

    # Wing parameters from case.
    R_fore = float(case["wings"]["fore"]["span"]) * 1000.0  # m -> mm
    R_hind = float(case["wings"]["hind"]["span"]) * 1000.0
    cone_fore_rad = math.radians(float(case["wings"]["fore"]["cone"]))
    cone_hind_rad = math.radians(float(case["wings"]["hind"]["cone"]))

    # phi = arcsin(s_mm / (R * cos(cone)))
    phi_fore = np.degrees(np.arcsin(np.clip(s_mm_fore / (R_fore * math.cos(cone_fore_rad)), -1, 1)))
    phi_hind = np.degrees(np.arcsin(np.clip(s_mm_hind / (R_hind * math.cos(cone_hind_rad)), -1, 1)))

    # psi = 90 deg - beta
    psi_fore = 90 - beta_deg_fore
    psi_hind = 90 - beta_deg_hind

    # Time in CSV is already in wingbeat units (0 to n_wingbeats).
    t_wrapped = t_s % 1.0

    # --- Harmonic fitting ---------------------------------------------------
    def _fit_harmonics(t: np.ndarray, y: np.ndarray, n_harm: int) -> np.ndarray:
        """Least-squares fit: a0 + sum_k [a_k cos(k*2pi*t) + b_k sin(k*2pi*t)]."""
        cols = [np.ones_like(t)]
        for k in range(1, n_harm + 1):
            cols.append(np.cos(k * 2.0 * np.pi * t))
            cols.append(np.sin(k * 2.0 * np.pi * t))
        A = np.column_stack(cols)
        coeffs, *_ = np.linalg.lstsq(A, y, rcond=None)
        return coeffs

    def _eval_harmonics(t: np.ndarray, coeffs: np.ndarray) -> np.ndarray:
        n_harm = (len(coeffs) - 1) // 2
        result = np.full_like(t, coeffs[0])
        for k in range(1, n_harm + 1):
            result += coeffs[2 * k - 1] * np.cos(k * 2.0 * np.pi * t)
            result += coeffs[2 * k] * np.sin(k * 2.0 * np.pi * t)
        return result

    t_fit = np.linspace(0.0, 1.0, 500)

    phi_fore_coeffs = _fit_harmonics(t_wrapped, phi_fore, 2)
    phi_hind_coeffs = _fit_harmonics(t_wrapped, phi_hind, 2)
    psi_fore_coeffs = _fit_harmonics(t_wrapped, psi_fore, 4)
    psi_hind_coeffs = _fit_harmonics(t_wrapped, psi_hind, 4)

    # --- Plotting ------------------------------------------------------------
    style = resolve_style(theme=theme)
    apply_matplotlib_style(style)

    fig, (ax_phi, ax_psi) = plt.subplots(
        nrows=1,
        ncols=2,
        figsize=figure_size(height_over_width=0.45),
    )

    dot_kw = dict(s=10, alpha=0.3, edgecolors="none")

    ax_phi.scatter(t_wrapped, phi_fore, color="C0", **dot_kw)
    ax_phi.scatter(t_wrapped, phi_hind, color="C1", **dot_kw)
    ax_phi.plot(t_fit, _eval_harmonics(t_fit, phi_fore_coeffs), color="C0", linewidth=1.5, label="Forewing")
    ax_phi.plot(t_fit, _eval_harmonics(t_fit, phi_hind_coeffs), color="C1", linewidth=1.5, label="Hindwing")
    ax_phi.set_xlabel(r"$t/T_{wb}$")
    ax_phi.set_ylabel(r"$\phi$ (deg)")
    ax_phi.set_xlim(0.0, 1.0)
    ax_phi.grid(True, alpha=0.25)

    ax_psi.scatter(t_wrapped, psi_fore, color="C0", **dot_kw)
    ax_psi.scatter(t_wrapped, psi_hind, color="C1", **dot_kw)
    ax_psi.plot(t_fit, _eval_harmonics(t_fit, psi_fore_coeffs), color="C0", linewidth=1.5, label="Forewing")
    ax_psi.plot(t_fit, _eval_harmonics(t_fit, psi_hind_coeffs), color="C1", linewidth=1.5, label="Hindwing")
    ax_psi.set_xlabel(r"$t/T_{wb}$")
    ax_psi.set_ylabel(r"$\psi$ (deg)")
    ax_psi.set_xlim(0.0, 1.0)
    ax_psi.grid(True, alpha=0.25)

    fig.tight_layout()
    handles, labels = ax_phi.get_legend_handles_labels()
    fig.legend(
        handles, labels,
        loc="lower center", bbox_to_anchor=(0.5, 1.01), ncol=2, fontsize=10.0,
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(str(output_path), dpi=300, bbox_inches="tight")
    plt.close(fig)


def plot_mass_regression(
    case: dict,
    body_csv: Path,
    forewing_csv: Path,
    output_path: Path,
    *,
    extra_specimens: list[dict] | None = None,
    theme: str | None = None,
) -> None:
    """Allometric mass vs. body-length regression plot with case specimen highlighted."""
    import matplotlib.pyplot as plt
    import numpy as np
    import pandas as pd

    from post.style import apply_matplotlib_style, figure_size, resolve_style

    # Load and filter Wakeling data (dragonflies only, no damselflies).
    body_df = pd.read_csv(body_csv)
    body_df = body_df[~body_df["species"].str.startswith("Calopteryx")]
    merged = body_df[["ID", "L", "m"]].dropna(subset=["L", "m"])

    # Append extra specimens (e.g. from other case studies).
    if extra_specimens:
        merged = pd.concat(
            [merged, pd.DataFrame(extra_specimens)], ignore_index=True
        )

    L = merged["L"].values.astype(float)
    m = merged["m"].values.astype(float)

    # Log-log regression.
    log_L = np.log(L)
    log_m = np.log(m)
    A = np.column_stack([np.ones_like(log_L), log_L])
    c, *_ = np.linalg.lstsq(A, log_m, rcond=None)

    # Extend fit line across the full padded x range.
    x_lo, x_hi = L.min() * 0.8, L.max() * 1.25
    L_fit = np.geomspace(x_lo, x_hi, 300)
    m_fit = np.exp(c[0] + c[1] * np.log(L_fit))

    # Target specimen from case.
    L_target = float(case["specimen"]["body_length"]) * 1000.0  # m -> mm
    m_target = np.exp(c[0] + c[1] * np.log(L_target))

    style = resolve_style(theme=theme)
    apply_matplotlib_style(style)

    fig, ax = plt.subplots(figsize=figure_size(height_over_width=0.45))

    ax.scatter(L, m, s=18, color="#9b6dff", alpha=0.7, zorder=2, label="Wakeling (1997)")
    ax.plot(L_fit, m_fit, linewidth=1.5, color="#f0a030", zorder=3,
            label=rf"$m \propto L^{{{c[1]:.2f}}}$")
    ax.scatter([L_target], [m_target], s=18, color="#f0a030", marker="o", alpha=0.7,
               zorder=4, label=f"Case study estimate")

    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_xlim(30, 90)
    ax.set_ylim(50, 2000)
    ax.set_xticks([30, 40, 50, 60, 70, 80, 90])
    ax.set_yticks([100, 200, 500, 1000])
    ax.xaxis.set_major_formatter(plt.matplotlib.ticker.ScalarFormatter())
    ax.yaxis.set_major_formatter(plt.matplotlib.ticker.ScalarFormatter())
    ax.set_xlabel(r"$L$ (mm)")
    ax.set_ylabel(r"$m$ (mg)")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="lower right", fontsize=10.0)

    # Center a narrowed axes with a fixed aspect ratio (~1.6) in the figure.
    fig_w, fig_h = fig.get_size_inches()
    bot, top = 0.2, 0.93
    ax_w_frac = (top - bot) * (fig_h / fig_w) * 1.6
    left = (1.0 - ax_w_frac) / 2.0
    fig.subplots_adjust(left=left, right=left + ax_w_frac, bottom=bot, top=top)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(str(output_path), dpi=300)
    plt.close(fig)


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

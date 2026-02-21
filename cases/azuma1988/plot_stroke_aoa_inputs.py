#!/usr/bin/env python3
"""
Standalone Azuma 1988 AoA contour plot from case inputs (no simulator HDF5).

This script rebuilds stroke-plane AoA contours directly from:
- experiment kinematics in data/case_studies/azuma1988/case.json
- experiment flight speed/direction (m/s, deg)
- wing span and stroke-plane/coning geometry

It does not read simulator state vectors and does not use nondimensionalized
velocity inputs.

AoA is computed by `angle_of_attack_azuma1988(...)` using raw paper angles:
stroke plane angle `gamma`, flapping angle `psi`, and pitching angle `theta`.

Usage:
    python -m cases.azuma1988.plot_stroke_aoa_inputs <output.png>
        [--experiment 4] [--wing fore_right]
        [--n-time 200] [--n-eta 50]
        [--theme light|dark]
"""

from __future__ import annotations

import argparse
import importlib
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import FancyArrowPatch

from cases.azuma1988.plot_kinematics import SCRIPTS_DIR, load_adapter
from post.style import apply_matplotlib_style, figure_size, resolve_style


def angle_of_attack_azuma1988(r, body_velocity, stroke_plane_angle, coning_angle, flapping_angle, pitching_angle, flapping_angle_velocity):
    """
    Compute angle of attack at given span r from body velocity and wing kinematics.
    All angles and angular velocities defined using (Azuma, 1988) conventions.
    """
    V = body_velocity
    gamma = stroke_plane_angle
    beta = coning_angle
    psi = flapping_angle
    theta = pitching_angle
    psi_dot = flapping_angle_velocity

    U_T = V * np.cos(gamma) * np.cos(psi) + r * psi_dot * np.cos(beta)
    U_P = V * (np.sin(gamma) * np.cos(beta) + np.cos(gamma) * np.sin(psi) * np.sin(beta))
    # Use atan2 for correct quadrant, then map AoA to principal aerodynamic branch.
    phi = np.atan2(U_P, U_T)
    alpha = theta - phi
    return (alpha + 0.5 * np.pi) % np.pi - 0.5 * np.pi


@dataclass(frozen=True)
class WingChoice:
    wing_key: str
    is_left: bool
    label: str


WING_CHOICES: dict[str, WingChoice] = {
    "fore_right": WingChoice(wing_key="fore", is_left=False, label="fore_right"),
    "fore_left": WingChoice(wing_key="fore", is_left=True, label="fore_left"),
    "hind_right": WingChoice(wing_key="hind", is_left=False, label="hind_right"),
    "hind_left": WingChoice(wing_key="hind", is_left=True, label="hind_left"),
    # Common aliases used in other parts of the project.
    "forewing_right": WingChoice(wing_key="fore", is_left=False, label="fore_right"),
    "forewing_left": WingChoice(wing_key="fore", is_left=True, label="fore_left"),
    "hindwing_right": WingChoice(wing_key="hind", is_left=False, label="hind_right"),
    "hindwing_left": WingChoice(wing_key="hind", is_left=True, label="hind_left"),
}


def _load_case_helpers():
    if str(SCRIPTS_DIR) not in sys.path:
        sys.path.insert(0, str(SCRIPTS_DIR))
    mod = importlib.import_module("case_data")
    return mod.load_case_data, mod.select_experiment, mod.find_output_reference


def _load_case_experiment(experiment: str | int) -> dict:
    load_case_data, select_experiment, _ = _load_case_helpers()
    case = load_case_data("azuma1988")
    return select_experiment(case, experiment_id=experiment)


def _wing_config(exp_case: dict, wing_key: str) -> tuple[float, float]:
    sim_defaults = exp_case["simulation_defaults"]
    specimen = exp_case["specimen"]
    if wing_key == "fore":
        span_m = float(specimen["fore_span_m"])
        cone_deg = float(sim_defaults["coning_angles_deg"]["fore"])
    else:
        span_m = float(specimen["hind_span_m"])
        cone_deg = float(sim_defaults["coning_angles_deg"]["hind"])
    return span_m, math.radians(cone_deg)


def _flight_speed(exp_case: dict, speed_m_s: float | None) -> float:
    _, _, find_output_reference = _load_case_helpers()
    ref = find_output_reference(
        exp_case,
        kind="flight_condition",
        name="body_speed_and_direction",
    )
    return float(ref["speed_m_s"]) if speed_m_s is None else float(speed_m_s)


def _eval_series_and_rate(
    paper_series,
    omega: float,
    time: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    def eval_one(angle_series):
        mean = math.radians(float(angle_series.mean_deg))
        val = np.full_like(time, mean, dtype=float)
        rate = np.zeros_like(time, dtype=float)
        for term in angle_series.terms:
            k = float(term.harmonic)
            amp = math.radians(float(term.amplitude_deg))
            phase = math.radians(float(term.phase_deg))
            arg = k * omega * time + phase
            val += amp * np.cos(arg)
            rate += -amp * k * omega * np.sin(arg)
        return val, rate

    stroke_plane, stroke_plane_dot = eval_one(paper_series.gamma)
    flapping_psi, flapping_psi_dot = eval_one(paper_series.phi)
    pitching_theta, _ = eval_one(paper_series.psi)
    return stroke_plane, stroke_plane_dot, flapping_psi, flapping_psi_dot, pitching_theta


def _first_harmonic_component(
    angle_series,
    omega: float,
    time: np.ndarray,
) -> tuple[np.ndarray, float]:
    """Return first-harmonic signal at reference span station and its coefficient magnitude."""
    c1 = 0.0
    s1 = 0.0
    for term in angle_series.terms:
        if int(term.harmonic) != 1:
            continue
        amp = math.radians(float(term.amplitude_deg))
        phase = math.radians(float(term.phase_deg))
        c1 += amp * math.cos(phase)
        s1 += -amp * math.sin(phase)
    h1 = c1 * np.cos(omega * time) + s1 * np.sin(omega * time)
    return h1, float(math.hypot(c1, s1))


def _fan_grid_from_phi(phi_sel: np.ndarray, phi_mean: float, eta: np.ndarray, span_m: float, side: str):
    theta = np.asarray(phi_sel, dtype=float) - float(phi_mean)
    r = eta * span_m
    x = np.cos(theta)
    # Azuma paper convention: negative psi = wings raised, positive psi = wings lowered.
    # Plot this with raised at top (positive y) by flipping the vertical sign.
    y = -np.sin(theta)
    if side == "left":
        x = -x
    x_grid = np.outer(x, r)
    y_grid = np.outer(y, r)
    return x_grid, y_grid


def _smooth_along_axis(arr: np.ndarray, window: int, axis: int) -> np.ndarray:
    if window <= 1:
        return arr
    pad = window // 2
    pads = [(0, 0)] * arr.ndim
    pads[axis] = (pad, pad)
    padded = np.pad(arr, pads, mode="edge")
    kernel = np.ones(window, dtype=float) / float(window)
    return np.apply_along_axis(lambda x: np.convolve(x, kernel, mode="valid"), axis, padded)


def _resample_aoa(phi_sel: np.ndarray, aoa: np.ndarray, n_phi: int = 72, smooth_phi: int = 13, smooth_eta: int = 9):
    order = np.argsort(phi_sel)
    phi_sorted = phi_sel[order]
    aoa_sorted = aoa[order]
    phi_unique, unique_idx = np.unique(phi_sorted, return_index=True)
    aoa_unique = aoa_sorted[unique_idx]
    if len(phi_unique) < 2:
        return phi_unique, aoa_unique

    n_phi = max(16, int(n_phi))
    phi_target = np.linspace(phi_unique[0], phi_unique[-1], n_phi)
    aoa_target = np.empty((n_phi, aoa_unique.shape[1]), dtype=float)
    for j in range(aoa_unique.shape[1]):
        col = aoa_unique[:, j]
        valid = np.isfinite(col)
        if np.count_nonzero(valid) >= 2:
            aoa_target[:, j] = np.interp(phi_target, phi_unique[valid], col[valid])
        elif np.count_nonzero(valid) == 1:
            aoa_target[:, j] = col[valid][0]
        else:
            aoa_target[:, j] = 0.0

    aoa_target = _smooth_along_axis(aoa_target, window=smooth_phi, axis=0)
    aoa_target = _smooth_along_axis(aoa_target, window=smooth_eta, axis=1)
    return phi_target, aoa_target


def _select_half(phi: np.ndarray, phi_dot: np.ndarray, side: str) -> np.ndarray:
    # Azuma (1988) convention:
    #   downstroke -> psi_dot > 0
    #   upstroke   -> psi_dot < 0
    # Here `phi_dot` stores paper flapping-rate psi_dot (see compute_aoa_from_inputs).
    if side == "right":
        mask = phi_dot > 0.0
    else:
        mask = phi_dot < 0.0
    idx = np.where(mask)[0]
    return idx[np.argsort(phi[idx])]


def _plot_combined(output_path: Path, phi: np.ndarray, phi_dot: np.ndarray, aoa: np.ndarray, phi_mean: float, eta: np.ndarray, span_m: float, style) -> None:
    idx_down = _select_half(phi, phi_dot, "right")
    idx_up = _select_half(phi, phi_dot, "left")
    if len(idx_down) == 0 or len(idx_up) == 0:
        raise ValueError("Both downstroke and upstroke timesteps are required for combined plot output.")

    phi_d, aoa_down = _resample_aoa(phi[idx_down], aoa[idx_down])
    phi_u, aoa_up = _resample_aoa(phi[idx_up], aoa[idx_up])
    x_down, y_down = _fan_grid_from_phi(phi_d, phi_mean, eta, span_m, side="right")
    x_up, y_up = _fan_grid_from_phi(phi_u, phi_mean, eta, span_m, side="left")

    all_aoa = np.concatenate([aoa_down.ravel(), aoa_up.ravel()])
    finite = all_aoa[np.isfinite(all_aoa)]
    if finite.size == 0:
        raise ValueError("Computed AoA contains no finite values.")
    step = 5.0
    abs_lim = np.nanpercentile(np.abs(finite), 90.0)
    hi = step * np.ceil(abs_lim / step)
    lo = -hi
    if hi <= 0.0:
        hi = step
        lo = -step
    levels = np.arange(lo, hi + 0.5 * step, step)
    label_levels = levels[::2]

    fig, ax = plt.subplots(figsize=figure_size(height_over_width=0.45))
    fig.patch.set_facecolor("white")
    ax.set_facecolor("white")

    lc = style.text_color
    ct_down = ax.contour(x_down, y_down, aoa_down, levels=levels, colors=lc, linewidths=1.6, linestyles="solid")
    ct_up = ax.contour(x_up, y_up, aoa_up, levels=levels, colors=lc, linewidths=1.6, linestyles="solid")
    ax.clabel(ct_down, levels=label_levels, fmt="%.0f", fontsize=9, inline=True, rightside_up=True, use_clabeltext=True)
    ax.clabel(ct_up, levels=label_levels, fmt="%.0f", fontsize=9, inline=True, rightside_up=True, use_clabeltext=True)

    for x_grid, y_grid in ((x_down, y_down), (x_up, y_up)):
        ax.plot(x_grid[0, :], y_grid[0, :], color=lc, lw=1.0)
        ax.plot(x_grid[-1, :], y_grid[-1, :], color=lc, lw=1.0)
        ax.plot(x_grid[:, -1], y_grid[:, -1], color=lc, lw=1.0)

    ax.set_aspect("equal")
    ax.set_axis_off()
    x_max = np.nanmax(np.abs(np.concatenate([x_down.ravel(), x_up.ravel()])))
    y_max = np.nanmax(np.abs(np.concatenate([y_down.ravel(), y_up.ravel()])))
    ax.set_xlim(-x_max - 0.35 * x_max, x_max + 0.35 * x_max)
    ax.set_ylim(-y_max - 0.15 * y_max, y_max + 0.15 * y_max)

    r_wing = float(span_m * eta[-1])
    r_arrow = 1.06 * r_wing
    theta_span = np.degrees(np.arcsin(np.clip(0.78 * y_max / max(r_arrow, 1e-12), 0.0, 1.0))) * 0.5

    def _draw_circular_arrow(theta_start_deg: float, theta_end_deg: float) -> None:
        theta = np.deg2rad(np.linspace(theta_start_deg, theta_end_deg, 120))
        x = r_arrow * np.cos(theta)
        y = r_arrow * np.sin(theta)
        body_end = -5
        ax.plot(x[:body_end], y[:body_end], color=lc, lw=1.0, solid_capstyle="butt")
        head = FancyArrowPatch(
            (x[body_end], y[body_end]),
            (x[-1], y[-1]),
            arrowstyle="Simple,head_length=0.45,head_width=0.225,tail_width=0.05",
            mutation_scale=14,
            fc=lc,
            ec=lc,
            lw=0.0,
        )
        ax.add_patch(head)

    _draw_circular_arrow(180.0 + theta_span, 180.0 - theta_span)
    _draw_circular_arrow(theta_span, -theta_span)
    ax.text(-1.1 * r_wing, 0.02 * y_max, "Up", color=lc, ha="right", va="center")
    ax.text(1.1 * r_wing, 0.02 * y_max, "Down", color=lc, ha="left", va="center")

    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(str(output_path), dpi=300, bbox_inches="tight")
    plt.close(fig)


def compute_aoa_from_inputs(
    experiment: str | int,
    wing: str,
    n_time: int,
    n_eta: int,
    speed_m_s: float | None,
    frequency_hz: float | None,
) -> dict[str, np.ndarray | float]:
    if wing not in WING_CHOICES:
        raise ValueError(f"Unsupported wing '{wing}'. Available: {sorted(WING_CHOICES)}")

    choice = WING_CHOICES[wing]
    exp_case = _load_case_experiment(experiment)
    adapter = load_adapter(experiment=experiment)
    paper_series = adapter.source_series[choice.wing_key]

    freq = (
        float(exp_case["simulation_defaults"]["frequency_hz"])
        if frequency_hz is None
        else float(frequency_hz)
    )
    omega = 2.0 * math.pi * freq
    period = 1.0 / freq
    time = np.linspace(0.0, period, int(n_time), endpoint=False, dtype=float)
    span_m, cone_rad = _wing_config(exp_case, choice.wing_key)
    body_speed = _flight_speed(exp_case, speed_m_s=speed_m_s)

    stroke_plane, _, flapping_psi, flapping_psi_dot, pitching_theta = _eval_series_and_rate(
        paper_series,
        omega=omega,
        time=time,
    )
    theta_h1_ref, theta_h1_ref_coeff = _first_harmonic_component(
        paper_series.psi,
        omega=omega,
        time=time,
    )

    # Linear spanwise twist on the first theta harmonic, with reference at 0.75R.
    theta_root_deg = 0.0 if choice.wing_key == "fore" else 12.0
    theta_root_coeff = math.radians(theta_root_deg)
    twist_ref_eta = 0.75

    eta = np.linspace(0.0, 1.0, int(n_eta) + 1, dtype=float)

    if theta_h1_ref_coeff > 1e-12:
        theta_coeff_eta = ((theta_h1_ref_coeff - theta_root_coeff) * (eta / twist_ref_eta)) + theta_root_coeff
        theta_h1_scale_eta = theta_coeff_eta / theta_h1_ref_coeff
    else:
        theta_h1_scale_eta = np.ones_like(eta)

    aoa = np.zeros((len(time), len(eta)), dtype=float)
    for i in range(len(time)):
        for j, eta_j in enumerate(eta):
            theta_eta = pitching_theta[i] + (theta_h1_scale_eta[j] - 1.0) * theta_h1_ref[i]
            alpha = angle_of_attack_azuma1988(
                r=float(eta_j * span_m),
                body_velocity=body_speed,
                stroke_plane_angle=float(stroke_plane[i]),
                coning_angle=cone_rad,
                flapping_angle=float(flapping_psi[i]),
                pitching_angle=float(theta_eta),
                flapping_angle_velocity=float(flapping_psi_dot[i]),
            )
            aoa[i, j] = math.degrees(alpha)

    return {
        "time": time,
        # Use paper flapping angle psi (not simulator-mapped phi) as the stroke coordinate.
        "phi": flapping_psi,
        "phi_dot": flapping_psi_dot,
        "aoa": aoa,
        "eta": eta,
        "span_m": span_m,
        "phi_mean": float(np.mean(flapping_psi)),
        "speed_m_s": body_speed,
        "frequency_hz": freq,
    }


def plot_aoa_from_inputs(
    output_path: Path,
    *,
    experiment: str | int,
    wing: str,
    n_time: int,
    n_eta: int,
    speed_m_s: float | None,
    frequency_hz: float | None,
    theme: str | None,
) -> dict[str, float]:
    result = compute_aoa_from_inputs(
        experiment=experiment,
        wing=wing,
        n_time=n_time,
        n_eta=n_eta,
        speed_m_s=speed_m_s,
        frequency_hz=frequency_hz,
    )

    style = resolve_style(theme=theme)
    apply_matplotlib_style(style)

    phi = np.asarray(result["phi"], dtype=float)
    phi_dot = np.asarray(result["phi_dot"], dtype=float)
    aoa = np.asarray(result["aoa"], dtype=float)
    eta = np.asarray(result["eta"], dtype=float)
    span_m = float(result["span_m"])
    phi_mean = float(result["phi_mean"])

    _plot_combined(
        output_path=output_path,
        phi=phi,
        phi_dot=phi_dot,
        aoa=aoa,
        phi_mean=phi_mean,
        eta=eta,
        span_m=span_m,
        style=style,
    )

    finite = aoa[np.isfinite(aoa)]
    return {
        "aoa_min_deg": float(np.min(finite)) if finite.size else float("nan"),
        "aoa_max_deg": float(np.max(finite)) if finite.size else float("nan"),
        "speed_m_s": float(result["speed_m_s"]),
        "frequency_hz": float(result["frequency_hz"]),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", help="Output image path")
    parser.add_argument("--experiment", default="4", help="Azuma 1988 experiment id (default: 4)")
    parser.add_argument("--wing", default="fore_right", help="Wing name (e.g. fore_right, hind_right)")
    parser.add_argument("--n-time", type=int, default=200, help="Time samples over one wingbeat")
    parser.add_argument("--n-eta", type=int, default=50, help="Span stations for AoA grid")
    parser.add_argument("--speed-m-s", type=float, default=None, help="Optional speed override")
    parser.add_argument("--frequency-hz", type=float, default=None, help="Optional flapping frequency override")
    parser.add_argument("--theme", choices=["light", "dark"], default="light")
    args = parser.parse_args()

    output = Path(args.output)
    summary = plot_aoa_from_inputs(
        output,
        experiment=args.experiment,
        wing=args.wing,
        n_time=args.n_time,
        n_eta=args.n_eta,
        speed_m_s=args.speed_m_s,
        frequency_hz=args.frequency_hz,
        theme=args.theme,
    )
    print(
        "Saved: "
        f"{output} | wing={args.wing} exp={args.experiment} "
        f"| speed={summary['speed_m_s']:.3f} m/s freq={summary['frequency_hz']:.3f} Hz "
        f"| AoA range [{summary['aoa_min_deg']:.2f}, {summary['aoa_max_deg']:.2f}] deg"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

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
    last_n_wingbeats: float | None = None,
) -> dict[str, Any]:
    """Read HDF5 and compute dimensional body speed/direction over wingbeats."""
    import h5py
    import numpy as np

    with h5py.File(str(h5_path), "r") as f:
        time = np.asarray(f["/time"][:], dtype=float)
        states = np.asarray(f["/state"][:], dtype=float)
        omega_nondim = float(f["/parameters/omega"][()])

    if last_n_wingbeats is not None:
        t_wb = 2.0 * math.pi / omega_nondim
        t_start = time[-1] - float(last_n_wingbeats) * t_wb
        mask = time >= t_start - 1e-9 * t_wb
        time = time[mask]
        states = states[mask]

    speed_scale = math.sqrt(float(gravity_m_s2) * float(body_length_m))
    speed_m_s = np.linalg.norm(states[:, 3:6], axis=1) * speed_scale
    direction_deg = np.degrees(np.arctan2(states[:, 5], states[:, 3]))
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
    last_n_wingbeats: float | None = None,
) -> None:
    """Plot simulation body speed and direction against optional flight-condition refs."""
    import matplotlib.pyplot as plt
    import numpy as np

    from post.style import apply_matplotlib_style, figure_size, resolve_style

    metrics = compute_body_flight_metrics(
        h5_path,
        body_length_m=body_length_m,
        gravity_m_s2=gravity_m_s2,
        last_n_wingbeats=last_n_wingbeats,
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


def plot_wing_aoa_timeseries(
    h5_path: Path,
    output_path: Path,
    *,
    wing_names: tuple[str, str] = ("fore_right", "hind_right"),
    eta: float = 2.0 / 3.0,
    aoa_csv_path: Path | None = None,
    source_case: dict[str, Any] | None = None,
    simplified_speed_m_s: float | None = None,
    theme: str | None = None,
    last_n_wingbeats: float | None = None,
    curve_variant: str = "model",
) -> None:
    """Plot AoA time traces for a fore/hind wing pair at a fixed span station."""
    import matplotlib.pyplot as plt
    import numpy as np

    from post.io import read_simulation
    from post.style import apply_matplotlib_style, figure_size, resolve_style

    if not (0.0 <= float(eta) <= 1.0):
        raise ValueError("eta must be in [0, 1]")
    if curve_variant not in {"model", "simplified"}:
        raise ValueError("curve_variant must be one of: model, simplified")

    params, time, states, wings = read_simulation(str(h5_path))
    omega_nondim = float(params["omega"])

    if last_n_wingbeats is not None:
        t_wb = 2.0 * math.pi / omega_nondim
        t_start = time[-1] - float(last_n_wingbeats) * t_wb
        mask = time >= t_start - 1e-9 * t_wb
        time = time[mask]
        states = states[mask]
        wings = {
            name: {
                k: (np.asarray(v[mask]) if not isinstance(v, dict) else v)
                for k, v in data.items()
            }
            for name, data in wings.items()
        }

    has_wing_harmonics = all(
        key in params
        for key in (
            "wing_phi_mean",
            "wing_phi_amp",
            "wing_phi_phase",
            "wing_gamma_mean",
            "wing_gamma_amp",
            "wing_gamma_phase",
        )
    )
    if has_wing_harmonics:
        from post.plot_stroke_aoa import _aoa_grid, _compute_angles

        angle_params = {
            "phi_mean": params["wing_phi_mean"],
            "phi_amp": params["wing_phi_amp"],
            "phi_phase": params["wing_phi_phase"],
            "gamma_mean": params["wing_gamma_mean"],
            "gamma_amp": params["wing_gamma_amp"],
            "gamma_phase": params["wing_gamma_phase"],
        }

    def _aoa_fd_single_eta(
        e_r: np.ndarray,
        e_c: np.ndarray,
        states_arr: np.ndarray,
        time_arr: np.ndarray,
        *,
        lb0: float,
        eta_val: float,
    ) -> np.ndarray:
        """Fallback AoA from finite-difference span-point velocity r * d(e_r)/dt."""
        de_r_dt = np.gradient(e_r, time_arr, axis=0, edge_order=2 if len(time_arr) >= 3 else 1)
        r = float(eta_val) * float(lb0)
        aoa = np.zeros(len(time_arr), dtype=float)
        for i in range(len(time_arr)):
            er = e_r[i]
            ec = e_c[i]
            ub = states_arr[i, 3:6]
            uw = ub + r * de_r_dt[i]
            u = uw - np.dot(uw, er) * er
            U_sq = np.dot(u, u)
            if U_sq < 1e-20:
                continue
            U_inv = 1.0 / np.sqrt(U_sq)
            c_alpha = np.dot(u, ec) * U_inv
            s_alpha = np.dot(np.cross(u, ec), er) * U_inv
            aoa[i] = np.degrees(np.arctan2(s_alpha, c_alpha))
        return aoa

    def _aoa_from_blade_alpha_single_eta(wing_data: dict[str, Any], *, eta_val: float) -> np.ndarray | None:
        """Interpolate per-blade AoA (radians) to a target span station and return degrees."""
        blade = wing_data.get("blade")
        if not isinstance(blade, dict):
            return None
        if "eta" not in blade or "alpha" not in blade:
            return None

        eta_grid = np.asarray(blade["eta"], dtype=float)
        alpha_blade = np.asarray(blade["alpha"], dtype=float)
        if eta_grid.ndim != 1 or alpha_blade.ndim != 2:
            return None
        if alpha_blade.shape[0] != len(time) or alpha_blade.shape[1] != eta_grid.size:
            return None
        if eta_grid.size == 0:
            return None

        order = np.argsort(eta_grid)
        eta_sorted = eta_grid[order]
        alpha_sorted = alpha_blade[:, order]

        # Avoid extrapolating outside the blade quadrature range; exact method fallback is better there.
        tol = 1e-12
        if float(eta_val) < float(eta_sorted[0]) - tol or float(eta_val) > float(eta_sorted[-1]) + tol:
            return None

        if eta_sorted.size == 1:
            if abs(float(eta_val) - float(eta_sorted[0])) > tol:
                return None
            return np.degrees(np.asarray(alpha_sorted[:, 0], dtype=float))

        sin_vals = np.sin(alpha_sorted)
        cos_vals = np.cos(alpha_sorted)
        sin_interp = np.asarray([np.interp(float(eta_val), eta_sorted, row) for row in sin_vals], dtype=float)
        cos_interp = np.asarray([np.interp(float(eta_val), eta_sorted, row) for row in cos_vals], dtype=float)
        return np.degrees(np.arctan2(sin_interp, cos_interp))

    aoa_series: dict[str, np.ndarray] = {}
    for wing_name in wing_names:
        if wing_name not in wings:
            raise ValueError(f"Wing '{wing_name}' not found in {h5_path}")
        if wing_name not in params["wing_lb0"]:
            raise ValueError(f"Wing parameters missing for '{wing_name}' in {h5_path}")

        aoa_from_blade = _aoa_from_blade_alpha_single_eta(wings[wing_name], eta_val=float(eta))
        if aoa_from_blade is not None:
            aoa_series[wing_name] = np.asarray(aoa_from_blade, dtype=float)
            continue

        e_r = np.asarray(wings[wing_name]["e_r"], dtype=float)
        e_s = np.asarray(wings[wing_name]["e_s"], dtype=float)
        e_c = np.asarray(wings[wing_name]["e_c"], dtype=float)
        if has_wing_harmonics:
            _, phi_dot, _, gam_raw_dot = _compute_angles(time, wing_name, params, angle_params)
            aoa = _aoa_grid(
                e_r,
                e_s,
                e_c,
                np.asarray(states, dtype=float),
                np.asarray(phi_dot, dtype=float),
                np.asarray(gam_raw_dot, dtype=float),
                float(params["wing_lb0"][wing_name]),
                bool(wing_name.endswith("_left")),
                np.asarray([float(eta)], dtype=float),
            )
            aoa_series[wing_name] = np.asarray(aoa[:, 0], dtype=float)
        else:
            aoa_series[wing_name] = _aoa_fd_single_eta(
                e_r,
                e_c,
                np.asarray(states, dtype=float),
                np.asarray(time, dtype=float),
                lb0=float(params["wing_lb0"][wing_name]),
                eta_val=float(eta),
            )

    t = np.asarray(time, dtype=float) * omega_nondim / (2.0 * np.pi)
    if t.size:
        t = t - t[0]

    exp_aoa: dict[str, np.ndarray] | None = None
    if aoa_csv_path is not None:
        import pandas as pd

        df = pd.read_csv(aoa_csv_path)
        if "t_fore" in df.columns:
            # Per-wing time columns (e.g. wang2007): t_fore, alpha_deg_fore, t_hind, alpha_deg_hind
            fore_mask = df["t_fore"].notna()
            hind_mask = df["t_hind"].notna()
            exp_aoa = {
                "fore_t": np.mod(df.loc[fore_mask, "t_fore"].to_numpy(dtype=float), 1.0),
                "fore_aoa": df.loc[fore_mask, "alpha_deg_fore"].to_numpy(dtype=float),
                "hind_t": np.mod(df.loc[hind_mask, "t_hind"].to_numpy(dtype=float), 1.0),
                "hind_aoa": df.loc[hind_mask, "alpha_deg_hind"].to_numpy(dtype=float),
            }
        else:
            # Shared time column (e.g. azuma1985): t, alpha_fore, alpha_hind
            t_col = np.mod(df["t"].to_numpy(dtype=float), 1.0)
            exp_aoa = {
                "fore_t": t_col,
                "fore_aoa": df["alpha_fore"].to_numpy(dtype=float),
                "hind_t": t_col,
                "hind_aoa": df["alpha_hind"].to_numpy(dtype=float),
            }

    simplified_aoa: dict[str, np.ndarray] | None = None
    if source_case is not None:
        import numpy as np

        spec = source_case.get("specimen", {})
        frequency_hz = float(spec["frequency"])
        refs = source_case.get("references", [])
        speed_ref_m_s = float(simplified_speed_m_s) if simplified_speed_m_s is not None else None
        if speed_ref_m_s is None:
            for ref in refs:
                if isinstance(ref, dict) and ref.get("kind") == "flight_condition" and "speed" in ref:
                    speed_ref_m_s = float(ref["speed"])
                    break
        if speed_ref_m_s is None:
            raise ValueError(
                "source_case for wing_aoa_timeseries must include references.kind=flight_condition speed "
                "or provide simplified_speed_m_s"
            )

        def _angle_rad(case_wing: dict[str, Any], key: str, t_wb_arr: np.ndarray) -> np.ndarray:
            entry = case_wing["kinematics"][key]
            out = np.full_like(t_wb_arr, np.radians(float(entry["mean"])), dtype=float)
            for k, (amp_deg, phase_deg) in enumerate(entry.get("harmonics", []), start=1):
                out += np.radians(float(amp_deg)) * np.cos(
                    k * 2.0 * np.pi * t_wb_arr + np.radians(float(phase_deg))
                )
            return out

        def _angle_rate_rad_s(case_wing: dict[str, Any], key: str, t_wb_arr: np.ndarray) -> np.ndarray:
            entry = case_wing["kinematics"][key]
            rate = np.zeros_like(t_wb_arr, dtype=float)
            for k, (amp_deg, phase_deg) in enumerate(entry.get("harmonics", []), start=1):
                amp_rad = np.radians(float(amp_deg))
                phase_rad = np.radians(float(phase_deg))
                rate += -k * (2.0 * np.pi * frequency_hz) * amp_rad * np.sin(
                    k * 2.0 * np.pi * t_wb_arr + phase_rad
                )
            return rate

        simplified_aoa = {}
        t_wb_line = np.asarray(t, dtype=float)
        for wing_name in wing_names:
            wing_key = str(wing_name).split("_", 1)[0]
            if wing_key not in source_case.get("wings", {}):
                raise ValueError(f"source_case is missing wing '{wing_key}' for '{wing_name}'")
            case_wing = source_case["wings"][wing_key]
            span_m = float(case_wing["span"])
            psi_rad = _angle_rad(case_wing, "psi", t_wb_line)
            # Simplified estimate uses the tangential speed at the selected span station eta*R.
            denom = float(eta) * span_m * _angle_rate_rad_s(case_wing, "phi", t_wb_line)
            with np.errstate(divide="ignore", invalid="ignore"):
                alpha_simplified = psi_rad + (0.5 * np.pi) - np.arctan2(speed_ref_m_s, -denom)
            simplified_aoa[wing_name] = np.degrees(alpha_simplified)
    if curve_variant == "simplified" and simplified_aoa is None:
        raise ValueError("curve_variant='simplified' requires source_case to compute the simplified expression")

    style = resolve_style(theme=theme)
    apply_matplotlib_style(style)

    def _mask_discontinuities(y: np.ndarray, threshold: float = 90.0) -> np.ndarray:
        """Return a copy of y with NaN inserted before large jumps."""
        y_out = np.array(y, dtype=float)
        jumps = np.abs(np.diff(y_out)) > threshold
        y_out[1:][jumps] = np.nan
        return y_out

    fig, ax = plt.subplots(figsize=figure_size(height_over_width=0.38))
    colors = ("C0", "C1")
    labels = ("Forewing", "Hindwing")
    if exp_aoa is not None:
        dot_kw = dict(s=12, alpha=0.35, linewidths=0.5, zorder=1)
        ax.scatter(exp_aoa["fore_t"], exp_aoa["fore_aoa"], color=colors[0], **dot_kw)
        ax.scatter(exp_aoa["hind_t"], exp_aoa["hind_aoa"], color=colors[1], **dot_kw)
    for wing_name, color, label in zip(wing_names, colors, labels):
        if curve_variant == "model":
            y_vals = aoa_series[wing_name]
        else:
            assert simplified_aoa is not None
            y_vals = simplified_aoa[wing_name]
        ax.plot(
            t,
            _mask_discontinuities(y_vals),
            linewidth=1.6,
            color=color,
            linestyle="-",
            label=label,
            zorder=3,
        )

    ax.set_xlabel(r"$t/T_{wb}$")
    ax.set_ylabel(r"$\alpha$ (deg)")
    y_arrays: list[np.ndarray] = []
    if exp_aoa is not None:
        y_arrays.extend(
            [
                np.asarray(exp_aoa["fore_aoa"], dtype=float),
                np.asarray(exp_aoa["hind_aoa"], dtype=float),
            ]
        )
    y_arrays.extend(np.asarray(aoa_series[name], dtype=float) for name in wing_names if name in aoa_series)
    if simplified_aoa is not None:
        y_arrays.extend(np.asarray(simplified_aoa[name], dtype=float) for name in wing_names if name in simplified_aoa)
    if y_arrays:
        y_concat = np.concatenate([arr[np.isfinite(arr)] for arr in y_arrays if arr.size])
        if y_concat.size:
            y_min = float(np.min(y_concat))
            y_max = float(np.max(y_concat))
            if y_max <= y_min:
                y_pad = max(5.0, 0.05 * max(abs(y_min), 1.0))
            else:
                y_pad = max(5.0, 0.05 * (y_max - y_min))
            ax.set_ylim(y_min - y_pad, y_max + y_pad)
    ax.grid(True, alpha=0.25)
    if t.size:
        ax.set_xlim(float(t[0]), float(t[-1]))

    fig.tight_layout()
    handles, labels = ax.get_legend_handles_labels()
    if handles:
        label_to_handle = {label: handle for handle, label in zip(handles, labels)}
        legend_order = ["Forewing", "Hindwing"]
        ordered_labels = [label for label in legend_order if label in label_to_handle]
        ordered_handles = [label_to_handle[label] for label in ordered_labels]
        fig.legend(
            ordered_handles,
            ordered_labels,
            loc="lower center",
            bbox_to_anchor=(0.5, 1.01),
            ncol=2,
            fontsize=10.0,
            columnspacing=1.2,
            handlelength=2.2,
            handletextpad=0.5,
        )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(str(output_path), dpi=300, bbox_inches="tight")
    plt.close(fig)


def plot_wing_force_components_timeseries(
    h5_path: Path,
    output_path: Path,
    *,
    force_csv_path: Path | None = None,
    body_weight_mN: float | None = None,
    theme: str | None = None,
    last_n_wingbeats: float | None = None,
) -> None:
    """Plot fore/hind/total horizontal and vertical aerodynamic force components."""
    import matplotlib.pyplot as plt
    import numpy as np

    from post.io import read_simulation
    from post.style import apply_matplotlib_style, figure_size, resolve_style

    params, time, _, wings = read_simulation(str(h5_path))
    omega_nondim = float(params["omega"])

    if last_n_wingbeats is not None:
        t_wb = 2.0 * math.pi / omega_nondim
        t_start = time[-1] - float(last_n_wingbeats) * t_wb
        mask = time >= t_start - 1e-9 * t_wb
        time = time[mask]
        wings = {
            name: {
                k: (np.asarray(v[mask]) if not isinstance(v, dict) else v)
                for k, v in data.items()
            }
            for name, data in wings.items()
        }

    if len(time) == 0:
        raise ValueError(f"No time samples found in {h5_path}")

    pair_forces: dict[str, np.ndarray] = {}
    for wing_name, data in wings.items():
        prefix = str(wing_name).split("_", 1)[0].lower()
        if prefix not in {"fore", "hind"}:
            continue
        force = np.asarray(data["lift"], dtype=float) + np.asarray(data["drag"], dtype=float)
        if prefix in pair_forces:
            pair_forces[prefix] = pair_forces[prefix] + force
        else:
            pair_forces[prefix] = np.array(force, copy=True)

    if "fore" not in pair_forces or "hind" not in pair_forces:
        found = sorted(pair_forces.keys())
        raise ValueError(f"Expected fore/hind wings in {h5_path}, found groups: {found}")

    total_force = pair_forces["fore"] + pair_forces["hind"]
    t = np.asarray(time, dtype=float) * omega_nondim / (2.0 * np.pi)
    t = t - float(t[0])

    exp_force: dict[str, np.ndarray] | None = None
    if force_csv_path is not None:
        import pandas as pd

        if body_weight_mN is None:
            raise ValueError("force_csv_path requires body_weight_mN for nondimensionalization")
        df = pd.read_csv(force_csv_path)
        exp_t = np.mod(df["t"].to_numpy(dtype=float), 1.0)
        exp_force = {
            "t": exp_t,
            "fore_Fx": df["Fx_fore"].to_numpy(dtype=float) / body_weight_mN,
            "fore_Fz": df["Fz_fore"].to_numpy(dtype=float) / body_weight_mN,
            "hind_Fx": df["Fx_hind"].to_numpy(dtype=float) / body_weight_mN,
            "hind_Fz": df["Fz_hind"].to_numpy(dtype=float) / body_weight_mN,
        }
        exp_force["total_Fx"] = exp_force["fore_Fx"] + exp_force["hind_Fx"]
        exp_force["total_Fz"] = exp_force["fore_Fz"] + exp_force["hind_Fz"]

    style = resolve_style(theme=theme)
    apply_matplotlib_style(style)

    fig, axes = plt.subplots(
        nrows=1,
        ncols=2,
        sharex=True,
        figsize=figure_size(height_over_width=0.45),
    )

    total_color = "#f0a030"  # Match Wang 2007 force-comparison total-force orange.
    panels = (
        (0, r"$\tilde{F}_x$"),
        (2, r"$\tilde{F}_z$"),
    )
    y_min = math.inf
    y_max = -math.inf

    for panel_idx, (ax, (comp_idx, ylabel)) in enumerate(zip(axes, panels)):
        fore_vals = pair_forces["fore"][:, comp_idx]
        hind_vals = pair_forces["hind"][:, comp_idx]
        total_vals = total_force[:, comp_idx]
        model_mean_total = float(np.mean(total_vals))
        paper_mean_total: float | None = None
        all_vals = [fore_vals, hind_vals, total_vals, np.array([model_mean_total])]
        if exp_force is not None:
            comp_key = "Fx" if comp_idx == 0 else "Fz"
            paper_mean_total = float(np.mean(exp_force[f"total_{comp_key}"]))
            all_vals.extend([exp_force[f"fore_{comp_key}"], exp_force[f"hind_{comp_key}"], exp_force[f"total_{comp_key}"]])
        y_min = min(y_min, *(float(np.min(v)) for v in all_vals))
        y_max = max(y_max, *(float(np.max(v)) for v in all_vals))

        ax.plot(
            t,
            fore_vals,
            linewidth=1.5,
            color="C0",
            label="Forewing" if panel_idx == 0 else None,
        )
        ax.plot(
            t,
            hind_vals,
            linewidth=1.5,
            color="C1",
            label="Hindwing" if panel_idx == 0 else None,
        )
        ax.plot(
            t,
            total_vals,
            linewidth=1.7,
            color=total_color,
            label="Total" if panel_idx == 0 else None,
            zorder=3,
        )
        ax.axhline(
            model_mean_total,
            linewidth=1.2,
            linestyle="--",
            color=total_color,
            alpha=0.95,
            zorder=2,
        )

        if exp_force is not None:
            comp_key = "Fx" if comp_idx == 0 else "Fz"
            dot_kw = dict(s=12, alpha=0.35, linewidths=0.5, zorder=1)
            ax.scatter(exp_force["t"], exp_force[f"fore_{comp_key}"], color="C0", **dot_kw)
            ax.scatter(exp_force["t"], exp_force[f"hind_{comp_key}"], color="C1", **dot_kw)
            ax.scatter(exp_force["t"], exp_force[f"total_{comp_key}"], color=total_color, **dot_kw)
            if paper_mean_total is not None:
                ax.axhline(
                    paper_mean_total,
                    linewidth=1.2,
                    linestyle=":",
                    color=total_color,
                    alpha=0.95,
                    zorder=2,
                )

        mean_lines = [f"Model mean: {model_mean_total:.2f}"]
        if paper_mean_total is not None:
            mean_lines.append(f"Paper mean: {paper_mean_total:.2f}")
        text_x = 0.03 if panel_idx == 0 else 0.97
        text_ha = "left" if panel_idx == 0 else "right"
        ax.text(
            text_x, 0.95, "\n".join(mean_lines),
            transform=ax.transAxes,
            ha=text_ha, va="top",
            fontsize=9,
            linespacing=1.6,
        )

        ax.set_ylabel(ylabel)
        ax.set_xlim(float(t[0]), float(t[-1]))
        ax.grid(True, alpha=0.25)

    if not math.isfinite(y_min) or not math.isfinite(y_max):
        raise ValueError(f"Invalid force values in {h5_path}")
    if y_max <= y_min:
        y_pad = 1.0 if y_max == y_min else 0.05 * max(abs(y_min), abs(y_max), 1.0)
        y_lo, y_hi = y_min - y_pad, y_max + y_pad
    else:
        y_pad = 0.05 * (y_max - y_min)
        y_lo, y_hi = y_min - y_pad, y_max + y_pad
    for ax in axes:
        ax.set_ylim(y_lo, y_hi)

    for ax in axes:
        ax.set_xlabel(r"$t/T_{wb}$")

    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(
        handles,
        labels,
        loc="lower center",
        bbox_to_anchor=(0.5, 1.01),
        ncol=max(1, len(labels)),
        fontsize=10.0,
    )
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
    layout: str = "vertical",
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
    plot_fore_hind_series(output_path, series, rows, theme=theme, layout=layout)


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

    from matplotlib.gridspec import GridSpec

    def _wing_cone_deg(wing_payload: dict[str, Any]) -> float:
        if "cone" in wing_payload:
            return float(wing_payload["cone"])
        kin = wing_payload.get("kinematics", {})
        beta = kin.get("beta", {}) if isinstance(kin, dict) else {}
        if "mean" in beta:
            return float(beta["mean"])
        return 0.0

    df = pd.read_csv(csv_path)
    t_s = df["t"].to_numpy(dtype=float)
    s_mm_fore = df["s_mm_fore"].to_numpy(dtype=float)
    s_mm_hind = df["s_mm_hind"].to_numpy(dtype=float)
    beta_deg_fore = df["beta_deg_fore"].to_numpy(dtype=float)
    beta_deg_hind = df["beta_deg_hind"].to_numpy(dtype=float)
    d_mm_fore = df["d_mm_fore"].to_numpy(dtype=float)
    d_mm_hind = df["d_mm_hind"].to_numpy(dtype=float)

    # Wing parameters from case.
    R_fore = float(case["wings"]["fore"]["span"]) * 1000.0  # m -> mm
    R_hind = float(case["wings"]["hind"]["span"]) * 1000.0
    cone_fore_rad = math.radians(_wing_cone_deg(case["wings"]["fore"]))
    cone_hind_rad = math.radians(_wing_cone_deg(case["wings"]["hind"]))

    # s is measured at the 2/3-span station, so use (2R/3) in the inversion.
    phi_fore = np.degrees(np.arcsin(np.clip(s_mm_fore / ((2.0 * R_fore / 3.0) * math.cos(cone_fore_rad)), -1, 1)))
    phi_hind = np.degrees(np.arcsin(np.clip(s_mm_hind / ((2.0 * R_hind / 3.0) * math.cos(cone_hind_rad)), -1, 1)))

    # psi = 90 deg - beta
    psi_fore = 90 - beta_deg_fore
    psi_hind = 90 - beta_deg_hind

    # beta - beta_mean = arcsin(3*(d - d_bar) / (2*R))
    dbeta_fore = np.degrees(np.arcsin(np.clip((d_mm_fore - np.mean(d_mm_fore)) / (2.0 * R_fore / 3.0), -1, 1)))
    dbeta_hind = np.degrees(np.arcsin(np.clip((d_mm_hind - np.mean(d_mm_hind)) / (2.0 * R_hind / 3.0), -1, 1)))

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
    dbeta_fore_coeffs = _fit_harmonics(t_wrapped, dbeta_fore, 5)
    dbeta_hind_coeffs = _fit_harmonics(t_wrapped, dbeta_hind, 5)

    # --- Plotting ------------------------------------------------------------
    style = resolve_style(theme=theme)
    apply_matplotlib_style(style)

    # 2-row, 4-column grid: phi and psi span 2 cols each on top;
    # beta-beta_mean spans the middle 2 cols on the bottom (same width, centered).
    fig = plt.figure(figsize=figure_size(height_over_width=0.90))
    gs = GridSpec(2, 4, figure=fig)
    ax_phi  = fig.add_subplot(gs[0, 0:2])
    ax_psi  = fig.add_subplot(gs[0, 2:4])
    ax_beta = fig.add_subplot(gs[1, 1:3])

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

    ax_beta.scatter(t_wrapped, dbeta_fore, color="C0", **dot_kw)
    ax_beta.scatter(t_wrapped, dbeta_hind, color="C1", **dot_kw)
    ax_beta.plot(t_fit, _eval_harmonics(t_fit, dbeta_fore_coeffs), color="C0", linewidth=1.5)
    ax_beta.plot(t_fit, _eval_harmonics(t_fit, dbeta_hind_coeffs), color="C1", linewidth=1.5)
    ax_beta.set_xlabel(r"$t/T_{wb}$")
    ax_beta.set_ylabel(r"$\beta - \bar{\beta}$ (deg)")
    ax_beta.set_xlim(0.0, 1.0)
    ax_beta.grid(True, alpha=0.25)

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
    """Allometric mass vs. forewing-length regression plot with case specimen highlighted."""
    import matplotlib.pyplot as plt
    import numpy as np
    import pandas as pd

    from post.style import apply_matplotlib_style, figure_size, resolve_style

    # Load and filter Wakeling data (dragonflies only, no damselflies).
    body_df = pd.read_csv(body_csv)
    body_df = body_df[~body_df["species"].str.startswith("Calopteryx")]
    forewing_df = pd.read_csv(forewing_csv)
    forewing_df = forewing_df[~forewing_df["species"].str.startswith("Calopteryx")]
    merged = body_df[["ID", "m"]].merge(forewing_df[["ID", "R"]], on="ID", how="inner")
    merged = merged.dropna(subset=["R", "m"])

    # Append extra specimens (e.g. from other case studies).
    if extra_specimens:
        extra_df = pd.DataFrame(extra_specimens)
        if {"R", "m"}.issubset(extra_df.columns):
            merged = pd.concat([merged, extra_df[["ID", "R", "m"]]], ignore_index=True)

    L = merged["R"].values.astype(float)
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

    # Target specimen from case (forewing span).
    L_target = float(case["wings"]["fore"]["span"]) * 1000.0  # m -> mm
    m_target = np.exp(c[0] + c[1] * np.log(L_target))

    style = resolve_style(theme=theme)
    apply_matplotlib_style(style)

    fig, ax = plt.subplots(figsize=figure_size(height_over_width=0.45))

    ax.scatter(L, m, s=18, color="#9b6dff", alpha=0.7, zorder=2, label="Wakeling (1997)")
    ax.plot(L_fit, m_fit, linewidth=1.5, color="#f0a030", zorder=3,
            label=rf"$m \propto R_f^{{{c[1]:.2f}}}$")
    ax.scatter([L_target], [m_target], s=18, color="#f0a030", marker="o", alpha=0.7,
               zorder=4, label=f"Case study estimate")

    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_xlim(20, 60)
    ax.set_ylim(50, 2000)
    ax.set_xticks([20, 30, 40, 50, 60])
    ax.set_yticks([100, 200, 500, 1000])
    ax.xaxis.set_major_formatter(plt.matplotlib.ticker.ScalarFormatter())
    ax.yaxis.set_major_formatter(plt.matplotlib.ticker.ScalarFormatter())
    ax.set_xlabel(r"$R_f$ (mm)")
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
    last_n_wingbeats: float | None = None,
) -> None:
    """Render a simulation video from an HDF5 file (Blender hybrid or mpl fallback)."""
    from post.composite import check_blender_available, render_hybrid, render_mpl_only
    from post.hybrid_config import HybridConfig
    from post.io import read_simulation
    from post.style import apply_theme_to_config

    params, time, states, wings = read_simulation(str(input_h5))

    if last_n_wingbeats is not None:
        omega = float(params["omega"])
        t_wb = 2.0 * math.pi / omega
        t_start = float(time[-1]) - float(last_n_wingbeats) * t_wb
        mask = time >= t_start - 1e-9 * t_wb
        time = time[mask]
        states = states[mask]
        wings = {
            wn: {vn: (arr[mask] if not isinstance(arr, dict) else arr) for vn, arr in wd.items()}
            for wn, wd in wings.items()
        }

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
    stroke_plane_beta_mode: str = "mean",
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
        stroke_plane_beta_mode=stroke_plane_beta_mode,
    )

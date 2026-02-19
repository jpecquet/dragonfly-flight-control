#!/usr/bin/env python3
"""
Plot Azuma 1988 exp1 vertical aerodynamic force over one wingbeat.

Compares blade-element counts (spanwise wing segments) with and without
first-harmonic pitch twist, at fixed body velocity from the experiment's
reported speed and direction.

Usage:
    python -m post.plot_azuma1988_exp1_force_segments <out.png> [--theme light|dark]
"""

from __future__ import annotations

import argparse
import math
import subprocess
import tempfile
from dataclasses import dataclass
from pathlib import Path

import h5py
import matplotlib.pyplot as plt
import numpy as np

from post.plot_force_comparison import read_aero_force_z
from post.style import apply_matplotlib_style, figure_size, resolve_style

try:
    from case_data import find_output_reference, load_case_data, select_experiment
except ModuleNotFoundError:
    from scripts.case_data import find_output_reference, load_case_data, select_experiment


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_BINARY = REPO_ROOT / "build" / "bin" / "dragonfly"
DEFAULT_BASE_CONFIG = REPO_ROOT / "docs" / "case_studies" / "azuma1988" / "artifacts" / "exp1" / "sim" / "sim_azuma1988.cfg"
DEFAULT_SEGMENTS = (1, 5, 10, 20, 50)
DEFAULT_SEGMENTS_CSV = ",".join(str(n) for n in DEFAULT_SEGMENTS)
PLOT_HEIGHT_OVER_WIDTH = 0.72


@dataclass(frozen=True)
class VelocitySpec:
    speed_m_s: float
    direction_deg: float
    ux0: float
    uy0: float
    uz0: float


def parse_segments(value: str) -> tuple[int, ...]:
    items = [chunk.strip() for chunk in str(value).split(",") if chunk.strip()]
    if not items:
        raise ValueError("segments list is empty")

    segments: list[int] = []
    for item in items:
        n = int(item)
        if n <= 0:
            raise ValueError(f"segment count must be > 0, got {n}")
        if n not in segments:
            segments.append(n)
    return tuple(segments)


def experimental_velocity_spec(experiment_id: str | int | None = "1") -> VelocitySpec:
    case = load_case_data("azuma1988")
    selected = select_experiment(case, experiment_id=experiment_id)
    flight_ref = find_output_reference(
        selected,
        kind="flight_condition",
        name="body_speed_and_direction",
    )

    speed_m_s = float(flight_ref["speed_m_s"])
    direction_deg = float(flight_ref["direction_deg"])
    body_length_m = float(selected["specimen"]["body_length_m"])
    gravity_m_s2 = float(selected["simulation_defaults"]["gravity_m_s2"])
    speed_scale = math.sqrt(gravity_m_s2 * body_length_m)
    speed_nd = speed_m_s / speed_scale
    direction_rad = math.radians(direction_deg)

    return VelocitySpec(
        speed_m_s=speed_m_s,
        direction_deg=direction_deg,
        ux0=speed_nd * math.cos(direction_rad),
        uy0=0.0,
        uz0=speed_nd * math.sin(direction_rad),
    )


def _fmt(x: float) -> str:
    return f"{float(x):.12f}"


def prepare_sim_config(
    base_cfg_text: str,
    *,
    n_blade_elements: int,
    enable_twist: bool,
    output_h5_path: str,
    velocity: VelocitySpec,
) -> str:
    if n_blade_elements <= 0:
        raise ValueError("n_blade_elements must be > 0")

    required_keys = {
        "tether": False,
        "n_blade_elements": False,
        "n_wingbeats": False,
        "output": False,
        "ux0": False,
        "uy0": False,
        "uz0": False,
    }
    removed_twist_lines = 0
    rewritten: list[str] = []

    for line in base_cfg_text.splitlines():
        stripped = line.strip()
        if (not enable_twist) and (
            stripped.startswith("psi_twist_h1_root_deg =")
            or stripped.startswith("psi_twist_ref_eta =")
        ):
            removed_twist_lines += 1
            continue
        if stripped.startswith("tether ="):
            rewritten.append("tether = true")
            required_keys["tether"] = True
            continue
        if stripped.startswith("n_blade_elements ="):
            rewritten.append(f"n_blade_elements = {n_blade_elements}")
            required_keys["n_blade_elements"] = True
            continue
        if stripped.startswith("n_wingbeats ="):
            rewritten.append("n_wingbeats = 1")
            required_keys["n_wingbeats"] = True
            continue
        if stripped.startswith("output ="):
            rewritten.append(f"output = {output_h5_path}")
            required_keys["output"] = True
            continue
        if stripped.startswith("ux0 ="):
            rewritten.append(f"ux0 = {_fmt(velocity.ux0)}")
            required_keys["ux0"] = True
            continue
        if stripped.startswith("uy0 ="):
            rewritten.append(f"uy0 = {_fmt(velocity.uy0)}")
            required_keys["uy0"] = True
            continue
        if stripped.startswith("uz0 ="):
            rewritten.append(f"uz0 = {_fmt(velocity.uz0)}")
            required_keys["uz0"] = True
            continue
        rewritten.append(line)

    missing = [key for key, seen in required_keys.items() if not seen]
    if missing:
        raise ValueError(f"base config missing required keys: {missing}")
    if not enable_twist and removed_twist_lines == 0:
        raise ValueError("expected twist-enabled base config; found no twist lines to remove")

    return "\n".join(rewritten) + "\n"


def _read_omega(h5_path: Path) -> float:
    with h5py.File(str(h5_path), "r") as f:
        return float(f["/parameters/omega"][()])


def _validate_constant_velocity(h5_path: Path, velocity: VelocitySpec, tol: float = 1e-12) -> None:
    with h5py.File(str(h5_path), "r") as f:
        state = np.asarray(f["/state"][:], dtype=float)
    vel = state[:, 3:6]
    target = np.array([velocity.ux0, velocity.uy0, velocity.uz0], dtype=float)
    if not np.allclose(vel, target[None, :], atol=tol, rtol=0.0):
        raise RuntimeError(f"velocity is not constant at requested target in {h5_path}")


def run_force_sweep(
    *,
    binary_path: Path,
    base_config_path: Path,
    segments: tuple[int, ...],
    velocity: VelocitySpec,
) -> dict[tuple[bool, int], tuple[np.ndarray, np.ndarray]]:
    if not binary_path.exists():
        raise FileNotFoundError(f"dragonfly binary not found: {binary_path}")
    if not base_config_path.exists():
        raise FileNotFoundError(f"base config not found: {base_config_path}")

    base_cfg_text = base_config_path.read_text(encoding="utf-8")
    results: dict[tuple[bool, int], tuple[np.ndarray, np.ndarray]] = {}

    with tempfile.TemporaryDirectory() as td:
        tmpdir = Path(td)
        for twist_enabled in (True, False):
            twist_tag = "twist" if twist_enabled else "no_twist"
            for nseg in segments:
                run_tag = f"{twist_tag}_{nseg:02d}"
                cfg_path = tmpdir / f"sim_{run_tag}.cfg"
                out_h5 = tmpdir / f"output_{run_tag}.h5"
                cfg_text = prepare_sim_config(
                    base_cfg_text,
                    n_blade_elements=int(nseg),
                    enable_twist=twist_enabled,
                    output_h5_path=str(out_h5),
                    velocity=velocity,
                )
                cfg_path.write_text(cfg_text, encoding="utf-8")

                cmd = [str(binary_path), "sim", "-c", str(cfg_path)]
                print(f"[cmd] {' '.join(cmd)}")
                subprocess.run(cmd, cwd=str(REPO_ROOT), check=True, capture_output=True, text=True)
                if not out_h5.exists():
                    raise FileNotFoundError(f"Expected output not found: {out_h5}")

                _validate_constant_velocity(out_h5, velocity)
                time, fz = read_aero_force_z(out_h5)
                omega = _read_omega(out_h5)
                wingbeats = time * omega / (2.0 * np.pi)
                results[(twist_enabled, int(nseg))] = (wingbeats, fz)

    return results


def plot_force_sweep(
    results: dict[tuple[bool, int], tuple[np.ndarray, np.ndarray]],
    output_path: Path,
    *,
    segments: tuple[int, ...],
    velocity: VelocitySpec,
    experiment_id: str,
    theme: str | None = None,
) -> None:
    style = resolve_style(theme=theme)
    apply_matplotlib_style(style)

    fig, axes = plt.subplots(
        nrows=2,
        ncols=1,
        sharex=True,
        figsize=figure_size(height_over_width=PLOT_HEIGHT_OVER_WIDTH),
    )

    panel_specs = (
        (True, "With Wing Twist"),
        (False, "Without Wing Twist"),
    )

    for ax, (twist_enabled, title) in zip(axes, panel_specs):
        for idx, nseg in enumerate(segments):
            t_wb, fz = results[(twist_enabled, int(nseg))]
            ax.plot(t_wb, fz, linewidth=1.4, color=f"C{idx}", label=f"{nseg}")
        ax.grid(True, alpha=0.25)
        ax.set_ylabel(r"$\tilde{F}_z$")
        ax.set_title(title)

    axes[-1].set_xlabel(r"$t/T_{wb}$")
    axes[-1].set_xlim(0.0, 1.0)

    axes[0].legend(
        title="Blade elements",
        loc="lower center",
        bbox_to_anchor=(0.5, 1.02),
        ncol=max(1, len(segments)),
        fontsize=9.5,
    )

    subtitle = (
        f"Exp {experiment_id} fixed velocity: {velocity.speed_m_s:.2f} m/s @ "
        f"{velocity.direction_deg:.1f} deg"
    )
    fig.suptitle(subtitle, y=0.995, fontsize=10.5)
    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.97))

    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(str(output_path), dpi=300, bbox_inches="tight")
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", help="Output image path (e.g. force_segments_twist.png).")
    parser.add_argument(
        "--base-config",
        default=str(DEFAULT_BASE_CONFIG),
        help="Base simulation config path (default: exp1 docs artifact config).",
    )
    parser.add_argument(
        "--binary",
        default=str(DEFAULT_BINARY),
        help="Path to dragonfly binary.",
    )
    parser.add_argument(
        "--experiment",
        default="1",
        help="Azuma 1988 experiment id used to resolve experimental velocity (default: 1).",
    )
    parser.add_argument(
        "--segments",
        default=DEFAULT_SEGMENTS_CSV,
        help=f"Comma-separated blade-element counts (default: {DEFAULT_SEGMENTS_CSV}).",
    )
    parser.add_argument("--theme", choices=["light", "dark"], default="light")
    args = parser.parse_args()

    output = Path(args.output)
    base_config = Path(args.base_config)
    if not base_config.is_absolute():
        base_config = REPO_ROOT / base_config

    binary_path = Path(args.binary)
    if not binary_path.is_absolute():
        binary_path = REPO_ROOT / binary_path

    segments = parse_segments(args.segments)
    velocity = experimental_velocity_spec(args.experiment)

    print(f"Running force sweep for segments={list(segments)} using {base_config}")
    results = run_force_sweep(
        binary_path=binary_path,
        base_config_path=base_config,
        segments=segments,
        velocity=velocity,
    )
    print(f"Writing Azuma 1988 exp{args.experiment} force-segment plot: {output}")
    plot_force_sweep(
        results,
        output,
        segments=segments,
        velocity=velocity,
        experiment_id=str(args.experiment),
        theme=args.theme,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Shared helpers for pipeline scripts."""

from __future__ import annotations

import os
import subprocess
from datetime import datetime
from pathlib import Path
from typing import Any


def ensure_dir(path: Path) -> Path:
    path.mkdir(parents=True, exist_ok=True)
    return path


def run_cmd(cmd: list[str], cwd: Path | None = None, env: dict[str, str] | None = None) -> None:
    printable = " ".join(str(x) for x in cmd)
    print(f"[cmd] {printable}")
    cmd_env = os.environ.copy()
    if env:
        cmd_env.update(env)
    subprocess.run(cmd, cwd=str(cwd) if cwd else None, check=True, env=cmd_env)


def resolve_run_dir(run_dir_arg: str | None, *, repo_root: Path, runs_root: Path) -> Path:
    if run_dir_arg:
        run_dir = Path(run_dir_arg).expanduser()
        if not run_dir.is_absolute():
            run_dir = repo_root / run_dir
        return run_dir
    run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
    return runs_root / run_id


def build_plot_env(run_dir: Path) -> dict[str, str]:
    cache_root = ensure_dir(run_dir / ".cache")
    mpl_cache = ensure_dir(cache_root / "matplotlib")
    return {
        "MPLCONFIGDIR": str(mpl_cache.resolve()),
        "XDG_CACHE_HOME": str(cache_root.resolve()),
    }


def fmt(x: float) -> str:
    """Format a float with 12 decimal places for simulator config files."""
    return f"{x:.12f}"


def fmt_list(values: list[float]) -> str:
    """Format a list of floats as comma-separated 12-decimal-place values."""
    return ", ".join(fmt(float(x)) for x in values)


def _series_amp_phase_from_motion(motion: dict[str, Any], prefix: str) -> tuple[list[float], list[float]]:
    amp_key = f"{prefix}_amp"
    phase_key = f"{prefix}_phase"

    amp_values = [float(x) for x in motion.get(amp_key, [])]
    phase_values = [float(x) for x in motion.get(phase_key, [])]
    if not amp_values and phase_values:
        amp_values = [0.0] * len(phase_values)
    if not phase_values and amp_values:
        phase_values = [0.0] * len(amp_values)
    if len(amp_values) != len(phase_values):
        raise ValueError(f"{amp_key} and {phase_key} must have matching lengths")
    if amp_values or phase_values:
        return amp_values, phase_values

    raise ValueError(
        f"motion is missing harmonic keys for '{prefix}' (expected {amp_key}/{phase_key})"
    )


def build_wing_block(
    name: str,
    side: str,
    wing_mu0: float,
    wing_lb0: float,
    phase: float,
    wing_cd0: float | None = None,
    wing_cl0: float | None = None,
    motion: dict[str, Any] | None = None,
    cone: float | None = None,
    psi_twist_h1_root_deg: float | None = None,
    psi_twist_ref_eta: float | None = None,
    drag_model: str | None = None,
    drag_coeff_set: str | None = None,
    lift_model: str | None = None,
    lift_coeff_set: str | None = None,
) -> str:
    """Build a [[wing]] config block for the simulator."""
    lines = [
        "[[wing]]",
        f"name = {name}",
        f"side = {side}",
        f"mu0 = {fmt(wing_mu0)}",
        f"lb0 = {fmt(wing_lb0)}",
        f"phase = {fmt(phase)}",
    ]
    if drag_model is not None:
        lines.append(f"drag_model = {drag_model}")
    if drag_coeff_set is not None:
        lines.append(f"drag_coeff_set = {drag_coeff_set}")
    elif wing_cd0 is not None:
        lines.append(f"Cd_min = {fmt(wing_cd0)}")
    else:
        raise ValueError("build_wing_block requires wing_cd0 unless drag_coeff_set is provided")

    if lift_model is not None:
        lines.append(f"lift_model = {lift_model}")
    if lift_coeff_set is not None:
        lines.append(f"lift_coeff_set = {lift_coeff_set}")
    elif wing_cl0 is not None:
        lines.append(f"Cl0 = {fmt(wing_cl0)}")
    else:
        raise ValueError("build_wing_block requires wing_cl0 unless lift_coeff_set is provided")

    if cone is not None and cone != 0.0:
        lines.append(f"cone = {fmt(cone)}")
    if psi_twist_h1_root_deg is not None:
        lines.append(f"psi_twist_h1_root_deg = {fmt(float(psi_twist_h1_root_deg))}")
    if psi_twist_ref_eta is not None:
        lines.append(f"psi_twist_ref_eta = {fmt(float(psi_twist_ref_eta))}")

    if motion is not None:
        for prefix in ("gamma", "phi", "psi"):
            amp, phase_vals = _series_amp_phase_from_motion(motion, prefix)
            lines.extend(
                [
                    f"{prefix}_mean = {fmt(float(motion[f'{prefix}_mean']))}",
                    f"{prefix}_amp = {fmt_list(amp)}",
                    f"{prefix}_phase = {fmt_list(phase_vals)}",
                ]
            )

        if any(k in motion for k in ("cone_mean", "cone_amp", "cone_phase")):
            cone_amp, cone_phase = _series_amp_phase_from_motion(motion, "cone")
            lines.extend(
                [
                    f"cone_mean = {fmt(float(motion.get('cone_mean', 0.0)))}",
                    f"cone_amp = {fmt_list(cone_amp)}",
                    f"cone_phase = {fmt_list(cone_phase)}",
                ]
            )

    return "\n".join(lines)

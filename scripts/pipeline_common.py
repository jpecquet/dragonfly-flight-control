#!/usr/bin/env python3
"""Shared helpers for pipeline scripts."""

from __future__ import annotations

import json
import os
import subprocess
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


def now_utc_iso() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


def ensure_dir(path: Path) -> Path:
    path.mkdir(parents=True, exist_ok=True)
    return path


def write_json(path: Path, data: dict[str, Any]) -> None:
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def read_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


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


def get_git_commit(repo_root: Path) -> str:
    try:
        out = subprocess.check_output(
            ["git", "rev-parse", "HEAD"],
            cwd=str(repo_root),
            text=True,
            stderr=subprocess.DEVNULL,
        )
        return out.strip()
    except Exception:
        return "unknown"


def fmt(x: float) -> str:
    """Format a float with 12 decimal places for simulator config files."""
    return f"{x:.12f}"


def fmt_list(values: list[float]) -> str:
    """Format a list of floats as comma-separated 12-decimal-place values."""
    return ", ".join(fmt(float(x)) for x in values)


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
        lines.extend(
            [
                f"gamma_mean = {fmt(float(motion['gamma_mean']))}",
                f"gamma_cos = {fmt_list([float(x) for x in motion['gamma_cos']])}",
                f"gamma_sin = {fmt_list([float(x) for x in motion['gamma_sin']])}",
                f"phi_mean = {fmt(float(motion['phi_mean']))}",
                f"phi_cos = {fmt_list([float(x) for x in motion['phi_cos']])}",
                f"phi_sin = {fmt_list([float(x) for x in motion['phi_sin']])}",
                f"psi_mean = {fmt(float(motion['psi_mean']))}",
                f"psi_cos = {fmt_list([float(x) for x in motion['psi_cos']])}",
                f"psi_sin = {fmt_list([float(x) for x in motion['psi_sin']])}",
            ]
        )

    return "\n".join(lines)


def update_manifest(
    run_dir: Path,
    stage: str,
    artifacts: list[Path],
    metadata: dict[str, Any],
    *,
    repo_root: Path,
    deterministic: bool = False,
) -> None:
    repo_root = repo_root.resolve()

    def normalize_path(path: Path) -> str:
        resolved = path.resolve()
        if not deterministic:
            return str(resolved)
        try:
            return str(resolved.relative_to(repo_root))
        except ValueError:
            return str(resolved)

    def normalize_value(value: Any) -> Any:
        if isinstance(value, str):
            path = Path(value)
            if path.is_absolute():
                return normalize_path(path)
            return value
        if isinstance(value, list):
            return [normalize_value(v) for v in value]
        if isinstance(value, dict):
            return {k: normalize_value(v) for k, v in value.items()}
        return value

    manifest_path = run_dir / "manifest.json"
    if manifest_path.exists():
        manifest = read_json(manifest_path)
    else:
        manifest = {
            "git_commit": get_git_commit(repo_root),
            "stages": {},
        }
    manifest["repo_root"] = "." if deterministic else str(repo_root)

    stage_payload = {
        "artifacts": [normalize_path(p) for p in artifacts],
        "metadata": normalize_value(metadata),
    }
    if not deterministic:
        now = now_utc_iso()
        manifest["created_at_utc"] = manifest.get("created_at_utc", now)
        manifest["updated_at_utc"] = now
        stage_payload["completed_at_utc"] = now
    else:
        manifest.pop("created_at_utc", None)
        manifest.pop("updated_at_utc", None)
        # Normalize legacy nondeterministic stage payloads when switching modes.
        for existing_stage, payload in list(manifest.get("stages", {}).items()):
            if not isinstance(payload, dict):
                continue
            payload = normalize_value(payload)
            payload.pop("completed_at_utc", None)
            manifest["stages"][existing_stage] = payload

    manifest["stages"][stage] = stage_payload
    write_json(manifest_path, manifest)

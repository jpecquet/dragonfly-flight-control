#!/usr/bin/env python3
"""Convert a YAML case definition to a simulator .cfg and optionally run the sim.

Usage:
    python -m scripts.case_runner <case.yaml> [--run-dir DIR] [--binary PATH]
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Any

import yaml

from scripts.pipeline_common import (
    build_wing_block,
    ensure_dir,
    fmt,
    resolve_run_dir,
    run_cmd,
)

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_BINARY = REPO_ROOT / "build" / "bin" / "dragonfly"
RUNS_ROOT = REPO_ROOT / "runs"

# Defaults for optional YAML fields.
DEFAULT_ENV = {"rho_air": 1.225, "gravity": 9.81}
DEFAULT_INITIAL_STATE = {"x": 0.0, "y": 0.0, "z": 0.0, "ux": 0.0, "uy": 0.0, "uz": 0.0}
DEFAULT_N_BLADE_ELEMENTS = 5


def load_case_yaml(path: Path) -> dict[str, Any]:
    """Read a case YAML file, validate required fields, and apply defaults."""
    raw = yaml.safe_load(path.read_text(encoding="utf-8"))

    for key in ("case_id", "specimen", "wings", "simulation"):
        if key not in raw:
            raise ValueError(f"Missing required top-level key: {key}")

    spec = raw["specimen"]
    for key in ("body_length", "body_mass", "frequency"):
        if key not in spec:
            raise ValueError(f"Missing specimen.{key}")

    for wing_name, wing in raw["wings"].items():
        for key in ("span", "area", "kinematics"):
            if key not in wing:
                raise ValueError(f"Missing wings.{wing_name}.{key}")
        kin = wing["kinematics"]
        for angle in ("gamma", "phi", "psi"):
            if angle not in kin:
                raise ValueError(f"Missing wings.{wing_name}.kinematics.{angle}")
            if "mean" not in kin[angle]:
                raise ValueError(f"Missing wings.{wing_name}.kinematics.{angle}.mean")

    # Apply defaults.
    raw.setdefault("environment", {})
    for k, v in DEFAULT_ENV.items():
        raw["environment"].setdefault(k, v)
    raw.setdefault("initial_state", dict(DEFAULT_INITIAL_STATE))
    raw.setdefault("tether", False)
    raw["simulation"].setdefault("n_blade_elements", DEFAULT_N_BLADE_ELEMENTS)

    return raw


def _harmonics_to_motion(
    kinematics: dict[str, Any],
    n_harmonics: int,
) -> dict[str, Any]:
    """Convert YAML kinematics block to the motion dict expected by build_wing_block.

    Angles in the YAML are in degrees; the motion dict uses radians.
    Negative amplitudes are normalised to positive with a pi phase offset.
    Harmonics arrays are padded to n_harmonics.
    """
    motion: dict[str, Any] = {}
    for angle in ("gamma", "phi", "psi"):
        entry = kinematics[angle]
        mean_rad = math.radians(float(entry["mean"]))
        motion[f"{angle}_mean"] = mean_rad

        pairs = entry.get("harmonics", [])
        amp_rad: list[float] = []
        phase_rad: list[float] = []
        for amp_deg, phase_deg in pairs:
            a = float(amp_deg)
            p = float(phase_deg)
            if a < 0:
                a = -a
                p += 180.0
            amp_rad.append(math.radians(a))
            p_rad = math.radians(p)
            # Normalise phase to [-pi, pi].
            phase_rad.append(math.atan2(math.sin(p_rad), math.cos(p_rad)))

        # Pad to n_harmonics.
        while len(amp_rad) < n_harmonics:
            amp_rad.append(0.0)
            phase_rad.append(0.0)

        motion[f"{angle}_amp"] = amp_rad
        motion[f"{angle}_phase"] = phase_rad

    return motion


def _max_harmonics(case: dict[str, Any]) -> int:
    """Determine n_harmonics as the max harmonic count across all wings/angles."""
    n = 0
    for wing in case["wings"].values():
        for angle in ("gamma", "phi", "psi"):
            terms = wing["kinematics"][angle].get("harmonics", [])
            n = max(n, len(terms))
    return max(n, 1)


def yaml_to_cfg(case: dict[str, Any], source_path: Path) -> str:
    """Generate a simulator .cfg string from a loaded YAML case dict."""
    spec = case["specimen"]
    env = case["environment"]
    sim = case["simulation"]

    body_length = float(spec["body_length"])
    body_mass = float(spec["body_mass"])
    frequency = float(spec["frequency"])
    rho_air = float(env["rho_air"])
    gravity = float(env["gravity"])

    omega = 2.0 * math.pi * frequency * math.sqrt(body_length / gravity)
    n_harmonics = _max_harmonics(case)
    harmonic_period_wingbeats = float(sim.get("harmonic_period_wingbeats", 1.0))
    tether = case.get("tether", False)
    n_blade_elements = sim.get("n_blade_elements", DEFAULT_N_BLADE_ELEMENTS)
    initial = case.get("initial_state", DEFAULT_INITIAL_STATE)

    # Build wing blocks (left + right per wing group).
    wing_blocks: list[str] = []
    wing_motions: dict[str, dict[str, Any]] = {}
    for wing_name, wing in case["wings"].items():
        span = float(wing["span"])
        area = float(wing["area"])
        lb0 = span / body_length
        mu0 = rho_air * area * span / body_mass
        cone_rad = math.radians(float(wing.get("cone", 0.0)))
        motion = _harmonics_to_motion(wing["kinematics"], n_harmonics)
        wing_motions[wing_name] = motion

        block_kwargs: dict[str, Any] = {
            "wing_mu0": mu0,
            "wing_lb0": lb0,
            "phase": 0.0,
            "motion": motion,
        }
        if cone_rad != 0.0:
            block_kwargs["cone"] = cone_rad

        # Aerodynamic model: prefer named coefficient sets, fall back to scalar.
        if "drag_coeff_set" in wing:
            block_kwargs["drag_coeff_set"] = wing["drag_coeff_set"]
        elif "drag_coeff" in wing:
            block_kwargs["wing_cd0"] = float(wing["drag_coeff"])
        if "drag_model" in wing:
            block_kwargs["drag_model"] = wing["drag_model"]
        if "lift_coeff_set" in wing:
            block_kwargs["lift_coeff_set"] = wing["lift_coeff_set"]
        elif "lift_coeff" in wing:
            block_kwargs["wing_cl0"] = float(wing["lift_coeff"])
        if "lift_model" in wing:
            block_kwargs["lift_model"] = wing["lift_model"]

        # Pitch twist (linear first harmonic along span).
        pitch_twist = wing.get("pitch_twist")
        if isinstance(pitch_twist, dict):
            block_kwargs["psi_twist_h1_root_deg"] = float(pitch_twist["root_coeff_deg"])
            block_kwargs["psi_twist_ref_eta"] = float(pitch_twist["ref_eta"])

        for side in ("left", "right"):
            wing_blocks.append(
                build_wing_block(name=wing_name, side=side, **block_kwargs)
            )

    # Global means as average across wing groups.
    names = list(wing_motions.keys())
    gamma_mean_global = sum(wing_motions[n]["gamma_mean"] for n in names) / len(names)
    phi_mean_global = sum(wing_motions[n]["phi_mean"] for n in names) / len(names)
    psi_mean_global = sum(wing_motions[n]["psi_mean"] for n in names) / len(names)

    tether_str = "true" if tether else "false"
    wings_text = "\n\n".join(wing_blocks)

    harmonic_period_line = (
        f"\nharmonic_period_wingbeats = {fmt(harmonic_period_wingbeats)}"
        if harmonic_period_wingbeats != 1.0
        else ""
    )

    return f"""# Auto-generated by scripts/case_runner.py
# Source: {source_path}

# Kinematic parameters
omega = {fmt(omega)}
n_harmonics = {n_harmonics}{harmonic_period_line}
gamma_mean = {fmt(gamma_mean_global)}
phi_mean = {fmt(phi_mean_global)}
psi_mean = {fmt(psi_mean_global)}

# Integration/control
tether = {tether_str}
n_blade_elements = {n_blade_elements}
n_wingbeats = {sim["n_wingbeats"]}
steps_per_wingbeat = {sim["steps_per_wingbeat"]}

# Initial conditions
x0 = {float(initial.get("x", 0.0))}
y0 = {float(initial.get("y", 0.0))}
z0 = {float(initial.get("z", 0.0))}
ux0 = {float(initial.get("ux", 0.0))}
uy0 = {float(initial.get("uy", 0.0))}
uz0 = {float(initial.get("uz", 0.0))}

# Output
output = output.h5

{wings_text}
"""


def run_case(
    case_yaml: Path,
    *,
    run_dir: Path | None = None,
    binary: Path | None = None,
) -> Path:
    """Load YAML, write .cfg, and run the simulator. Returns the HDF5 output path."""
    case = load_case_yaml(case_yaml)
    case_id = case["case_id"]

    if run_dir is None:
        runs_root = RUNS_ROOT / case_id
        run_dir = resolve_run_dir(None, repo_root=REPO_ROOT, runs_root=runs_root)
    run_dir = ensure_dir(run_dir)

    if binary is None:
        binary = DEFAULT_BINARY
    binary = Path(binary).expanduser()
    if not binary.is_absolute():
        binary = REPO_ROOT / binary
    if not binary.exists():
        raise FileNotFoundError(f"dragonfly binary not found: {binary}")

    cfg_text = yaml_to_cfg(case, case_yaml)
    cfg_path = run_dir / f"sim_{case_id}.cfg"
    cfg_path.write_text(cfg_text, encoding="utf-8")

    output_h5 = run_dir / "output.h5"
    run_cmd([str(binary), "sim", "-c", str(cfg_path)], cwd=run_dir)
    if not output_h5.exists():
        raise FileNotFoundError(f"Simulation output not found: {output_h5}")

    print(f"[done] output: {output_h5}")
    return output_h5


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("case_yaml", help="Path to case YAML file.")
    parser.add_argument(
        "--run-dir",
        default=None,
        help="Run directory (default: runs/<case_id>/<timestamp>).",
    )
    parser.add_argument(
        "--binary",
        default=str(DEFAULT_BINARY),
        help="Path to dragonfly binary.",
    )
    args = parser.parse_args()

    case_yaml = Path(args.case_yaml)
    if not case_yaml.exists():
        print(f"Error: {case_yaml} not found")
        return 1

    run_dir_arg = args.run_dir
    run_dir: Path | None = None
    if run_dir_arg:
        run_dir = Path(run_dir_arg).expanduser()
        if not run_dir.is_absolute():
            run_dir = REPO_ROOT / run_dir

    run_case(case_yaml, run_dir=run_dir, binary=Path(args.binary))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

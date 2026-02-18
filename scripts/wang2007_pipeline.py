#!/usr/bin/env python3
"""Wang 2007 case study pipeline: sim -> post.

Runs two tethered simulations:
- high-fidelity Fourier wing motion (7 harmonics/wingbeat = 35 components)
- single-harmonic wing motion
and produces three visualizations (3D animation, stick plot, force comparison).

Stages:
  sim   Run both simulations.
  post  Run postprocessing from simulation outputs.
  all   Run sim -> post.
"""

from __future__ import annotations

import argparse
import importlib
import math
import sys
from pathlib import Path
from typing import Any

import numpy as np

try:
    from pipeline_common import (
        build_plot_env,
        build_wing_block,
        ensure_dir,
        fmt,
        resolve_run_dir,
        run_cmd,
    )
except ModuleNotFoundError:
    from scripts.pipeline_common import (
        build_plot_env,
        build_wing_block,
        ensure_dir,
        fmt,
        resolve_run_dir,
        run_cmd,
    )

try:
    from case_data import load_case_data
except ModuleNotFoundError:
    from scripts.case_data import load_case_data


REPO_ROOT = Path(__file__).resolve().parents[1]
WANG_DIR = REPO_ROOT / "data" / "kinematics" / "wang2007"
RUNS_ROOT = REPO_ROOT / "runs" / "wang2007"
DEFAULT_BINARY = REPO_ROOT / "build" / "bin" / "dragonfly"
WANG2007_CASE = load_case_data("wang2007")
WANG2007_SPECIMEN = WANG2007_CASE["specimen"]
WANG2007_SIM_DEFAULTS = WANG2007_CASE["simulation_defaults"]

DEFAULT_BODY_LENGTH_MM = float(WANG2007_SPECIMEN["body_length_mm"])
DEFAULT_WING_LENGTH_MM = float(WANG2007_SPECIMEN["wing_length_mm"])
DEFAULT_WING_AREA_MM2 = float(WANG2007_SPECIMEN["wing_area_mm2"])
DEFAULT_FREQUENCY_HZ = float(WANG2007_SIM_DEFAULTS["frequency_hz"])
DEFAULT_BODY_MASS_MG = float(WANG2007_SPECIMEN["body_mass_mg"])
DEFAULT_RHO_AIR = float(WANG2007_SIM_DEFAULTS["rho_air_kg_m3"])
DEFAULT_GRAVITY = float(WANG2007_SIM_DEFAULTS["gravity_m_s2"])

N_WINGBEATS = int(WANG2007_SIM_DEFAULTS["n_wingbeats"])
STEPS_PER_WINGBEAT = int(WANG2007_SIM_DEFAULTS["steps_per_wingbeat"])
HARMONICS_PER_WINGBEAT = int(WANG2007_SIM_DEFAULTS["harmonics_per_wingbeat"])
N_FOURIER_COMPONENTS = HARMONICS_PER_WINGBEAT * N_WINGBEATS


def load_common_module():
    if str(WANG_DIR) not in sys.path:
        sys.path.insert(0, str(WANG_DIR))
    return importlib.import_module("common")


# ---------------------------------------------------------------------------
# Nondimensionalization
# ---------------------------------------------------------------------------

def compute_nondim_scales(
    body_length_mm: float,
    wing_length_mm: float,
    wing_area_mm2: float,
    frequency_hz: float,
    body_mass_mg: float,
    rho_air: float,
    gravity: float,
) -> dict[str, float]:
    if body_length_mm <= 0.0:
        raise ValueError("body_length_mm must be > 0")
    if wing_length_mm <= 0.0:
        raise ValueError("wing_length_mm must be > 0")
    if wing_area_mm2 <= 0.0:
        raise ValueError("wing_area_mm2 must be > 0")
    if frequency_hz <= 0.0:
        raise ValueError("frequency_hz must be > 0")
    if body_mass_mg <= 0.0:
        raise ValueError("body_mass_mg must be > 0")
    if rho_air <= 0.0:
        raise ValueError("rho_air must be > 0")
    if gravity <= 0.0:
        raise ValueError("gravity must be > 0")

    body_length_m = body_length_mm * 1e-3
    wing_length_m = wing_length_mm * 1e-3
    wing_area_m2 = wing_area_mm2 * 1e-6
    body_mass_kg = body_mass_mg * 1e-6

    tau = math.sqrt(body_length_m / gravity)  # time scale
    omega_nd = 2.0 * math.pi * frequency_hz * tau
    lambda_nd = wing_length_m / body_length_m
    mu_nd = rho_air * wing_area_m2 * wing_length_m / body_mass_kg

    return {
        "body_length_m": body_length_m,
        "wing_length_m": wing_length_m,
        "wing_area_m2": wing_area_m2,
        "body_mass_kg": body_mass_kg,
        "time_scale_s": tau,
        "omega_nondim": omega_nd,
        "lambda_nondim": lambda_nd,
        "mu_nondim": mu_nd,
        "force_prefactor_per_wing": 0.5 * mu_nd / lambda_nd,
    }


# ---------------------------------------------------------------------------
# Harmonic fitting
# ---------------------------------------------------------------------------

def fit_harmonic_series(
    common: Any,
    t: Any,
    y: Any,
    n_harmonics: int,
    harmonic_period_wingbeats: float = 1.0,
) -> tuple[float, list[float], list[float]]:
    if n_harmonics <= 0:
        raise ValueError("n_harmonics must be >= 1")
    if harmonic_period_wingbeats <= 0.0:
        raise ValueError("harmonic_period_wingbeats must be > 0")

    np = common.np
    omega = float(common.omega) / float(harmonic_period_wingbeats)
    cols = [np.ones_like(t)]
    for k in range(1, n_harmonics + 1):
        cols.append(np.cos(k * omega * t))
        cols.append(np.sin(k * omega * t))
    mat = np.column_stack(cols)
    coeff = np.linalg.lstsq(mat, y, rcond=None)[0]

    mean = float(coeff[0])
    cos_coeff = [float(coeff[1 + 2 * (k - 1)]) for k in range(1, n_harmonics + 1)]
    sin_coeff = [float(coeff[2 + 2 * (k - 1)]) for k in range(1, n_harmonics + 1)]
    return mean, cos_coeff, sin_coeff


def compute_smoothed_motion_mapping(
    common: Any,
    omega_nondim: float,
    n_harmonics: int,
    wing_length_mm: float = DEFAULT_WING_LENGTH_MM,
    harmonic_period_wingbeats: float = 1.0,
) -> dict[str, Any]:
    if n_harmonics <= 0:
        raise ValueError("smoothed_n_harmonics must be >= 1")
    if harmonic_period_wingbeats <= 0.0:
        raise ValueError("harmonic_period_wingbeats must be > 0")
    if wing_length_mm <= 0.0:
        raise ValueError("wing_length_mm must be > 0")

    np = common.np
    # common.fourier_smooth expects harmonics per wingbeat.
    smooth_harmonics_per_wingbeat_float = n_harmonics / harmonic_period_wingbeats
    smooth_harmonics_per_wingbeat = int(round(smooth_harmonics_per_wingbeat_float))
    if abs(smooth_harmonics_per_wingbeat_float - smooth_harmonics_per_wingbeat) > 1e-9:
        raise ValueError(
            "n_harmonics / harmonic_period_wingbeats must be an integer "
            f"(got {n_harmonics} / {harmonic_period_wingbeats})"
        )
    if smooth_harmonics_per_wingbeat <= 0:
        raise ValueError("Derived smooth_harmonics_per_wingbeat must be >= 1")

    smooth_kwargs = {"n_harmonics": smooth_harmonics_per_wingbeat, "n_points": 2000}
    t_sf, sf = common.fourier_smooth(*common.sorted_xy("s_fore"), **smooth_kwargs)
    t_sh, sh = common.fourier_smooth(*common.sorted_xy("s_hind"), **smooth_kwargs)
    t_bf, bf = common.fourier_smooth(*common.sorted_xy("beta_fore"), **smooth_kwargs)
    t_bh, bh = common.fourier_smooth(*common.sorted_xy("beta_hind"), **smooth_kwargs)

    r_mm = (2.0 / 3.0) * wing_length_mm  # reference span station
    phi_fore = np.arcsin(np.clip(sf / r_mm, -1.0, 1.0))
    phi_hind = np.arcsin(np.clip(sh / r_mm, -1.0, 1.0))
    psi_fore = np.pi / 2.0 - np.radians(bf)
    psi_hind = np.pi / 2.0 - np.radians(bh)

    phi_fore_mean, phi_fore_cos, phi_fore_sin = fit_harmonic_series(
        common, t_sf, phi_fore, n_harmonics, harmonic_period_wingbeats
    )
    phi_hind_mean, phi_hind_cos, phi_hind_sin = fit_harmonic_series(
        common, t_sh, phi_hind, n_harmonics, harmonic_period_wingbeats
    )
    psi_fore_mean, psi_fore_cos, psi_fore_sin = fit_harmonic_series(
        common, t_bf, psi_fore, n_harmonics, harmonic_period_wingbeats
    )
    psi_hind_mean, psi_hind_cos, psi_hind_sin = fit_harmonic_series(
        common, t_bh, psi_hind, n_harmonics, harmonic_period_wingbeats
    )

    gamma_fore = float(common.gamma_fore)
    gamma_hind = float(common.gamma_hind)
    zero = [0.0 for _ in range(n_harmonics)]
    global_gamma_mean = 0.5 * (gamma_fore + gamma_hind)
    global_phi_mean = 0.5 * (phi_fore_mean + phi_hind_mean)
    global_psi_mean = 0.5 * (psi_fore_mean + psi_hind_mean)

    return {
        "source": "smoothed_experimental_data",
        "method": {
            "smoothing": "fourier_truncation",
            "series_basis": "least_squares in cos(k*2*pi*t) + sin(k*2*pi*t)",
            "n_harmonics": n_harmonics,
            "harmonics_per_wingbeat_for_smoothing": smooth_harmonics_per_wingbeat,
        },
        "parameters": {
            "omega": float(omega_nondim),
            "n_harmonics": int(n_harmonics),
            "harmonic_period_wingbeats": float(harmonic_period_wingbeats),
            "gamma_mean": global_gamma_mean,
            "phi_mean": global_phi_mean,
            "psi_mean": global_psi_mean,
        },
        "wing_parameters": {},
        "wing_phase_offsets": {
            "fore": 0.0,
            "hind": 0.0,
        },
        "wing_motion_overrides": {
            "fore": {
                "gamma_mean": gamma_fore,
                "gamma_cos": zero.copy(),
                "gamma_sin": zero.copy(),
                "phi_mean": phi_fore_mean,
                "phi_cos": phi_fore_cos,
                "phi_sin": phi_fore_sin,
                "psi_mean": psi_fore_mean,
                "psi_cos": psi_fore_cos,
                "psi_sin": psi_fore_sin,
            },
            "hind": {
                "gamma_mean": gamma_hind,
                "gamma_cos": zero.copy(),
                "gamma_sin": zero.copy(),
                "phi_mean": phi_hind_mean,
                "phi_cos": phi_hind_cos,
                "phi_sin": phi_hind_sin,
                "psi_mean": psi_hind_mean,
                "psi_cos": psi_hind_cos,
                "psi_sin": psi_hind_sin,
            },
        },
        "notes": [
            "Fore/hind kinematics are fit independently from Fourier-smoothed experimental s and beta traces.",
            "Wing phase is encoded directly in per-wing harmonic coefficients, so wing phase offsets are zero.",
        ],
    }


# ---------------------------------------------------------------------------
# Config generation
# ---------------------------------------------------------------------------

def build_sim_cfg(
    mapping: dict[str, Any],
    n_wingbeats: int,
    steps_per_wingbeat: int,
    output_name: str,
    wing_mu0: float,
    wing_lb0: float,
    wing_cd0: float,
    wing_cl0: float,
) -> str:
    params = mapping["parameters"]
    phase = mapping["wing_phase_offsets"]
    overrides = mapping["wing_motion_overrides"]
    n_harm = mapping["method"]["n_harmonics"]
    harmonic_period_wingbeats = float(params.get("harmonic_period_wingbeats", 1.0))
    motion_comment = (
        f"Fourier-smoothed experimental data ({n_harm} components "
        f"over {harmonic_period_wingbeats:g} wingbeats)"
    )

    fore_left = build_wing_block(
        name="fore", side="left", wing_mu0=wing_mu0, wing_lb0=wing_lb0,
        wing_cd0=wing_cd0, wing_cl0=wing_cl0,
        phase=float(phase["fore"]), motion=overrides["fore"],
    )
    fore_right = build_wing_block(
        name="fore", side="right", wing_mu0=wing_mu0, wing_lb0=wing_lb0,
        wing_cd0=wing_cd0, wing_cl0=wing_cl0,
        phase=float(phase["fore"]), motion=overrides["fore"],
    )
    hind_left = build_wing_block(
        name="hind", side="left", wing_mu0=wing_mu0, wing_lb0=wing_lb0,
        wing_cd0=wing_cd0, wing_cl0=wing_cl0,
        phase=float(phase["hind"]), motion=overrides["hind"],
    )
    hind_right = build_wing_block(
        name="hind", side="right", wing_mu0=wing_mu0, wing_lb0=wing_lb0,
        wing_cd0=wing_cd0, wing_cl0=wing_cl0,
        phase=float(phase["hind"]), motion=overrides["hind"],
    )

    return f"""# Auto-generated by scripts/wang2007_pipeline.py
# Source: data/case_studies/wang2007/case.json
# Wing motion source: {motion_comment}

# Kinematic parameters
omega = {fmt(float(params["omega"]))}
n_harmonics = {int(params["n_harmonics"])}
harmonic_period_wingbeats = {fmt(harmonic_period_wingbeats)}
gamma_mean = {fmt(float(params["gamma_mean"]))}
phi_mean = {fmt(float(params["phi_mean"]))}
psi_mean = {fmt(float(params["psi_mean"]))}

# Integration/control
tether = true
n_wingbeats = {n_wingbeats}
steps_per_wingbeat = {steps_per_wingbeat}

# Initial conditions
x0 = 0.0
y0 = 0.0
z0 = 0.0
ux0 = 0.0
uy0 = 0.0
uz0 = 0.0

# Output
output = {output_name}

{fore_left}

{fore_right}

{hind_left}

{hind_right}
"""


# ---------------------------------------------------------------------------
# Pipeline stages
# ---------------------------------------------------------------------------

def stage_sim(run_dir: Path, binary: str) -> tuple[Path, Path]:
    """Compute motion mappings and run high-fidelity + 1-harmonic simulations."""
    common = load_common_module()
    scales = compute_nondim_scales(
        body_length_mm=DEFAULT_BODY_LENGTH_MM,
        wing_length_mm=DEFAULT_WING_LENGTH_MM,
        wing_area_mm2=DEFAULT_WING_AREA_MM2,
        frequency_hz=DEFAULT_FREQUENCY_HZ,
        body_mass_mg=DEFAULT_BODY_MASS_MG,
        rho_air=DEFAULT_RHO_AIR,
        gravity=DEFAULT_GRAVITY,
    )
    omega_nd = scales["omega_nondim"]
    wing_lb0 = scales["lambda_nondim"]
    wing_mu0 = scales["mu_nondim"]

    binary_path = Path(binary).expanduser()
    if not binary_path.is_absolute():
        binary_path = REPO_ROOT / binary_path
    if not binary_path.exists():
        raise FileNotFoundError(f"dragonfly binary not found: {binary_path}")

    sim_dir = ensure_dir(run_dir / "sim")

    configs = []
    configs_spec = [
        (N_FOURIER_COMPONENTS, "7harm", float(N_WINGBEATS)),
        (1, "1harm", 1.0),
    ]
    for n_harm, tag, harmonic_period_wingbeats in configs_spec:
        mapping = compute_smoothed_motion_mapping(
            common,
            omega_nd,
            n_harm,
            wing_length_mm=DEFAULT_WING_LENGTH_MM,
            harmonic_period_wingbeats=harmonic_period_wingbeats,
        )
        cfg_text = build_sim_cfg(
            mapping,
            n_wingbeats=N_WINGBEATS,
            steps_per_wingbeat=STEPS_PER_WINGBEAT,
            output_name=f"output_{tag}.h5",
            wing_mu0=wing_mu0,
            wing_lb0=wing_lb0,
            wing_cd0=0.4,
            wing_cl0=1.2,
        )
        cfg_path = sim_dir / f"sim_wang2007_{tag}.cfg"
        cfg_path.write_text(cfg_text, encoding="utf-8")
        configs.append((cfg_path, sim_dir / f"output_{tag}.h5"))

    for cfg_path, output_h5 in configs:
        run_cmd([str(binary_path), "sim", "-c", str(cfg_path)], cwd=sim_dir)
        if not output_h5.exists():
            raise FileNotFoundError(f"Simulation output not found: {output_h5}")

    h5_7harm = configs[0][1]
    h5_1harm = configs[1][1]
    return h5_7harm, h5_1harm


def stage_post(
    run_dir: Path,
    h5_7harm: Path,
    h5_1harm: Path,
    no_blender: bool,
    frame_step: int,
    skip_stick: bool,
) -> list[Path]:
    """Run postprocessing: 3D animation, stick plot, and force comparison."""
    if frame_step < 1:
        raise ValueError(f"frame_step must be >= 1, got {frame_step}")

    plot_env = build_plot_env(run_dir)
    post_dir = ensure_dir(run_dir / "post")
    artifacts: list[Path] = []

    # 3D animation from high-fidelity sim
    sim_mp4 = post_dir / "simulation.mp4"
    sim_cmd = [sys.executable, "-m", "post.plot_simulation", str(h5_7harm), str(sim_mp4)]
    if no_blender:
        sim_cmd.append("--no-blender")
    if frame_step > 1:
        sim_cmd.extend(["--frame-step", str(frame_step)])
    run_cmd(sim_cmd, cwd=REPO_ROOT, env=plot_env)
    artifacts.append(sim_mp4)

    # Stick plot from high-fidelity sim
    if not skip_stick:
        stick_mp4 = post_dir / "stroke_fore_left.mp4"
        run_cmd(
            [sys.executable, "-m", "post.plot_stick", str(h5_7harm), str(stick_mp4)],
            cwd=REPO_ROOT,
            env=plot_env,
        )
        artifacts.append(stick_mp4)

    # Force comparison
    scales = compute_nondim_scales(
        body_length_mm=DEFAULT_BODY_LENGTH_MM,
        wing_length_mm=DEFAULT_WING_LENGTH_MM,
        wing_area_mm2=DEFAULT_WING_AREA_MM2,
        frequency_hz=DEFAULT_FREQUENCY_HZ,
        body_mass_mg=DEFAULT_BODY_MASS_MG,
        rho_air=DEFAULT_RHO_AIR,
        gravity=DEFAULT_GRAVITY,
    )
    force_png = post_dir / "force_comparison.png"
    force_cmd = [
        sys.executable,
        "-m",
        "post.plot_force_comparison",
        str(force_png),
        "--omega-nondim",
        f"{float(scales['omega_nondim']):.12f}",
        "--series",
        str(h5_7harm),
        "35-comp",
        "--series",
        str(h5_1harm),
        "1-comp",
    ]
    cfd_csv = WANG_DIR / "cfd_data.csv"
    if cfd_csv.exists():
        force_cmd.extend(["--series-csv", str(cfd_csv), "t", "Fz", "CFD"])
    run_cmd(force_cmd, cwd=REPO_ROOT, env=plot_env)
    artifacts.append(force_png)

    return artifacts


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="stage", required=True)

    sim_parser = subparsers.add_parser("sim", help="Run both simulations.")
    sim_parser.add_argument("--run-dir", default=None)
    sim_parser.add_argument("--binary", default=str(DEFAULT_BINARY))

    post_parser = subparsers.add_parser("post", help="Run postprocessing from simulation outputs.")
    post_parser.add_argument("--run-dir", default=None)
    post_parser.add_argument("--no-blender", action="store_true", help="Force matplotlib-only rendering.")
    post_parser.add_argument("--skip-stick", action="store_true", help="Skip stroke sphere animation.")
    post_parser.add_argument("--frame-step", type=int, default=1, help="Render every Nth frame (default: 1).")

    all_parser = subparsers.add_parser("all", help="Run sim -> post.")
    all_parser.add_argument("--run-dir", default=None)
    all_parser.add_argument("--binary", default=str(DEFAULT_BINARY))
    all_parser.add_argument("--no-blender", action="store_true", help="Force matplotlib-only rendering.")
    all_parser.add_argument("--skip-stick", action="store_true", help="Skip stroke sphere animation.")
    all_parser.add_argument("--frame-step", type=int, default=1, help="Render every Nth frame (default: 1).")

    args = parser.parse_args()
    run_dir = ensure_dir(resolve_run_dir(args.run_dir, repo_root=REPO_ROOT, runs_root=RUNS_ROOT))

    if args.stage == "sim":
        h5_7harm, h5_1harm = stage_sim(run_dir=run_dir, binary=args.binary)
        print(
            f"[done] high-fidelity "
            f"({HARMONICS_PER_WINGBEAT} harmonics/wingbeat, {N_FOURIER_COMPONENTS} components): {h5_7harm}"
        )
        print(f"[done] 1-harmonic: {h5_1harm}")
        print(f"[done] run_dir: {run_dir}")
        return 0

    if args.stage == "post":
        sim_dir = run_dir / "sim"
        h5_7harm = sim_dir / "output_7harm.h5"
        h5_1harm = sim_dir / "output_1harm.h5"
        if not h5_7harm.exists():
            raise FileNotFoundError(
                f"Missing high-fidelity output ({N_FOURIER_COMPONENTS} components): {h5_7harm}"
            )
        if not h5_1harm.exists():
            raise FileNotFoundError(f"Missing 1-harmonic output: {h5_1harm}")
        artifacts = stage_post(
            run_dir=run_dir,
            h5_7harm=h5_7harm,
            h5_1harm=h5_1harm,
            no_blender=args.no_blender,
            frame_step=args.frame_step,
            skip_stick=args.skip_stick,
        )
        print("[done] post artifacts:")
        for p in artifacts:
            print(f"  - {p}")
        print(f"[done] run_dir: {run_dir}")
        return 0

    if args.stage == "all":
        h5_7harm, h5_1harm = stage_sim(run_dir=run_dir, binary=args.binary)
        artifacts = stage_post(
            run_dir=run_dir,
            h5_7harm=h5_7harm,
            h5_1harm=h5_1harm,
            no_blender=args.no_blender,
            frame_step=args.frame_step,
            skip_stick=args.skip_stick,
        )
        print("[done] pipeline artifacts:")
        for p in artifacts:
            print(f"  - {p}")
        print(f"[done] run_dir: {run_dir}")
        return 0

    raise RuntimeError(f"Unhandled stage: {args.stage}")


if __name__ == "__main__":
    raise SystemExit(main())

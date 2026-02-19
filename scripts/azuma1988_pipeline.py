#!/usr/bin/env python3
"""Convenience pipeline: Azuma 1988 parameters -> sim -> post artifacts.

Stages:
  translate Generate a simulator config from Azuma (1988) selected experiment data.
  sim       Run dragonfly simulation for generated config.
  post      Run postprocessing (3D animation + flight metrics) from simulation output.
  all       Run translate -> sim -> post.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path
from typing import Any

try:
    from experimental_conventions import azuma1988_adapter, build_sim_wing_motion
except ModuleNotFoundError:
    from scripts.experimental_conventions import azuma1988_adapter, build_sim_wing_motion

try:
    from pipeline_common import (
        build_plot_env,
        build_wing_block,
        ensure_dir,
        fmt,
        now_utc_iso,
        resolve_run_dir,
        run_cmd,
        update_manifest,
        write_json,
    )
except ModuleNotFoundError:
    from scripts.pipeline_common import (
        build_plot_env,
        build_wing_block,
        ensure_dir,
        fmt,
        now_utc_iso,
        resolve_run_dir,
        run_cmd,
        update_manifest,
        write_json,
    )

try:
    from case_data import find_output_reference, load_case_data, select_experiment
except ModuleNotFoundError:
    from scripts.case_data import find_output_reference, load_case_data, select_experiment

# Reuse shared helpers from the azuma1985 pipeline.
try:
    from azuma1985_pipeline import (
        PipelineParams,
        artifact_ref,
        omega_nondim,
        run_dir_uses_docs_artifacts,
        wing_scales,
    )
except ModuleNotFoundError:
    from scripts.azuma1985_pipeline import (
        PipelineParams,
        artifact_ref,
        omega_nondim,
        run_dir_uses_docs_artifacts,
        wing_scales,
    )


REPO_ROOT = Path(__file__).resolve().parents[1]
RUNS_ROOT = REPO_ROOT / "runs" / "azuma1988"
DEFAULT_BINARY = REPO_ROOT / "build" / "bin" / "dragonfly"
AZUMA1988_CASE = load_case_data("azuma1988")
DEFAULT_EXPERIMENT_ID = str(AZUMA1988_CASE.get("default_experiment_id", "1"))
AZUMA1988_DEFAULT_CASE = select_experiment(AZUMA1988_CASE, experiment_id=DEFAULT_EXPERIMENT_ID)
AZUMA1988_SPECIMEN = AZUMA1988_DEFAULT_CASE["specimen"]
AZUMA1988_SIM_DEFAULTS = AZUMA1988_DEFAULT_CASE["simulation_defaults"]

# Azuma (1988) defaults loaded from selected default experiment in case data.
DEFAULT_BODY_LENGTH_M = float(AZUMA1988_SPECIMEN["body_length_m"])
DEFAULT_BODY_MASS_KG = float(AZUMA1988_SPECIMEN["body_mass_kg"])
DEFAULT_R_FW_M = float(AZUMA1988_SPECIMEN["fore_span_m"])
DEFAULT_S_FW_M2 = float(AZUMA1988_SPECIMEN["fore_area_m2"])
DEFAULT_R_HW_M = float(AZUMA1988_SPECIMEN["hind_span_m"])
DEFAULT_S_HW_M2 = float(AZUMA1988_SPECIMEN["hind_area_m2"])
DEFAULT_FREQUENCY_HZ = float(AZUMA1988_SIM_DEFAULTS["frequency_hz"])
DEFAULT_RHO_AIR = float(AZUMA1988_SIM_DEFAULTS["rho_air_kg_m3"])
DEFAULT_GRAVITY = float(AZUMA1988_SIM_DEFAULTS["gravity_m_s2"])
DEFAULT_CD0 = float(AZUMA1988_SIM_DEFAULTS["wing_cd0"])
DEFAULT_CL0 = float(AZUMA1988_SIM_DEFAULTS["wing_cl0"])
DEFAULT_N_HARMONICS = int(AZUMA1988_SIM_DEFAULTS["n_harmonics"])
DEFAULT_N_BLADE_ELEMENTS = 5

# Coning angles (radians), loaded from case data.
FORE_CONE_RAD = math.radians(float(AZUMA1988_SIM_DEFAULTS["coning_angles_deg"]["fore"]))
HIND_CONE_RAD = math.radians(float(AZUMA1988_SIM_DEFAULTS["coning_angles_deg"]["hind"]))
TWIST_EXP1_ROOT_COEFF_DEG = 9.0
TWIST_REF_ETA = 0.75


def resolve_experiment_case(experiment_id: str | int | None) -> dict[str, Any]:
    exp_id = DEFAULT_EXPERIMENT_ID if experiment_id is None else str(experiment_id)
    return select_experiment(AZUMA1988_CASE, experiment_id=exp_id)


def azuma1988_wing_motion(
    experiment_id: str | int | None,
    n_harmonics: int,
    stroke_plane_deg: float | None,
    fore_stroke_plane_deg: float | None,
    hind_stroke_plane_deg: float | None,
) -> tuple[dict[str, dict[str, Any]], dict[str, Any]]:
    adapter = azuma1988_adapter(experiment=experiment_id)
    gamma_overrides: dict[str, float] = {}
    if stroke_plane_deg is not None:
        gamma_overrides["fore"] = float(stroke_plane_deg)
        gamma_overrides["hind"] = float(stroke_plane_deg)
    if fore_stroke_plane_deg is not None:
        gamma_overrides["fore"] = float(fore_stroke_plane_deg)
    if hind_stroke_plane_deg is not None:
        gamma_overrides["hind"] = float(hind_stroke_plane_deg)

    resolved_adapter = adapter.with_gamma_overrides(gamma_overrides if gamma_overrides else None)
    sim_motion = build_sim_wing_motion(resolved_adapter, n_harmonics=n_harmonics)

    wing_motion: dict[str, dict[str, Any]] = {}
    for wing_name, motion in sim_motion.items():
        wing_motion[wing_name] = {
            "gamma_mean": motion.gamma_mean,
            "gamma_cos": list(motion.gamma_cos),
            "gamma_sin": list(motion.gamma_sin),
            "phi_mean": motion.phi_mean,
            "phi_cos": list(motion.phi_cos),
            "phi_sin": list(motion.phi_sin),
            "psi_mean": motion.psi_mean,
            "psi_cos": list(motion.psi_cos),
            "psi_sin": list(motion.psi_sin),
        }

    mapping_summary = resolved_adapter.summary()
    mapping_summary["resolved_stroke_plane_deg"] = {
        wing: resolved_adapter.source_series[wing].gamma.mean_deg
        for wing in ("fore", "hind")
    }
    mapping_summary["override_inputs_deg"] = {
        "stroke_plane_deg": stroke_plane_deg,
        "fore_stroke_plane_deg": fore_stroke_plane_deg,
        "hind_stroke_plane_deg": hind_stroke_plane_deg,
    }
    mapping_summary["coning_angles_deg"] = {
        "fore": math.degrees(FORE_CONE_RAD),
        "hind": math.degrees(HIND_CONE_RAD),
    }
    if str(experiment_id) == "1":
        mapping_summary["pitch_twist_model"] = {
            "kind": "linear_first_harmonic_along_span",
            "enabled": True,
            "ref_eta": TWIST_REF_ETA,
            "root_coeff_deg": {
                "fore": TWIST_EXP1_ROOT_COEFF_DEG,
                "hind": TWIST_EXP1_ROOT_COEFF_DEG,
            },
            "formula": "coeff(eta) = (coeff(ref_eta)-coeff(0))*(eta/ref_eta) + coeff(0)",
            "applied_to": "first harmonic coefficient magnitude of pitch series",
        }
    else:
        mapping_summary["pitch_twist_model"] = {
            "enabled": False,
        }
    return wing_motion, mapping_summary


def initial_velocity_from_experiment(
    experiment_case: dict[str, Any],
    *,
    body_length_m: float,
    gravity_m_s2: float,
) -> tuple[dict[str, float], dict[str, float]]:
    flight_ref = find_output_reference(
        experiment_case,
        kind="flight_condition",
        name="body_speed_and_direction",
    )
    speed_m_s = float(flight_ref["speed_m_s"])
    direction_deg = float(flight_ref["direction_deg"])
    speed_scale_m_s = math.sqrt(gravity_m_s2 * body_length_m)
    speed_nondim = speed_m_s / speed_scale_m_s
    direction_rad = math.radians(direction_deg)

    velocity_nd = {
        "ux0": speed_nondim * math.cos(direction_rad),
        "uy0": 0.0,
        "uz0": speed_nondim * math.sin(direction_rad),
    }
    velocity_meta = {
        "speed_m_s": speed_m_s,
        "direction_deg": direction_deg,
        "direction_definition": str(flight_ref.get("direction_definition", "")),
        "speed_scale_m_s": speed_scale_m_s,
        "speed_nondim": speed_nondim,
    }
    return velocity_nd, velocity_meta


def build_sim_cfg(
    experiment_id: str,
    n_harmonics: int,
    omega: float,
    n_wingbeats: int,
    steps_per_wingbeat: int,
    tether: bool,
    output_name: str,
    fore_lb0: float,
    fore_mu0: float,
    hind_lb0: float,
    hind_mu0: float,
    wing_cd0: float,
    wing_cl0: float,
    wing_motion: dict[str, dict[str, Any]],
    ux0: float,
    uy0: float,
    uz0: float,
) -> str:
    tether_str = "true" if tether else "false"

    fore = wing_motion["fore"]
    hind = wing_motion["hind"]
    enable_twist_model = str(experiment_id) == "1"
    twist_root_deg = TWIST_EXP1_ROOT_COEFF_DEG if enable_twist_model else None
    twist_ref_eta = TWIST_REF_ETA if enable_twist_model else None
    gamma_mean_global = 0.5 * (float(fore["gamma_mean"]) + float(hind["gamma_mean"]))
    phi_mean_global = 0.5 * (float(fore["phi_mean"]) + float(hind["phi_mean"]))
    psi_mean_global = 0.5 * (float(fore["psi_mean"]) + float(hind["psi_mean"]))

    fore_left = build_wing_block(
        name="fore", side="left", wing_mu0=fore_mu0, wing_lb0=fore_lb0,
        wing_cd0=wing_cd0, wing_cl0=wing_cl0, phase=0.0, motion=fore,
        cone=FORE_CONE_RAD, psi_twist_h1_root_deg=twist_root_deg, psi_twist_ref_eta=twist_ref_eta,
    )
    fore_right = build_wing_block(
        name="fore", side="right", wing_mu0=fore_mu0, wing_lb0=fore_lb0,
        wing_cd0=wing_cd0, wing_cl0=wing_cl0, phase=0.0, motion=fore,
        cone=FORE_CONE_RAD, psi_twist_h1_root_deg=twist_root_deg, psi_twist_ref_eta=twist_ref_eta,
    )
    hind_left = build_wing_block(
        name="hind", side="left", wing_mu0=hind_mu0, wing_lb0=hind_lb0,
        wing_cd0=wing_cd0, wing_cl0=wing_cl0, phase=0.0, motion=hind,
        cone=HIND_CONE_RAD, psi_twist_h1_root_deg=twist_root_deg, psi_twist_ref_eta=twist_ref_eta,
    )
    hind_right = build_wing_block(
        name="hind", side="right", wing_mu0=hind_mu0, wing_lb0=hind_lb0,
        wing_cd0=wing_cd0, wing_cl0=wing_cl0, phase=0.0, motion=hind,
        cone=HIND_CONE_RAD, psi_twist_h1_root_deg=twist_root_deg, psi_twist_ref_eta=twist_ref_eta,
    )

    return f"""# Auto-generated by scripts/azuma1988_pipeline.py
# Source: data/case_studies/azuma1988/case.json
# Selected experiment: {experiment_id}

# Kinematic parameters
omega = {fmt(omega)}
n_harmonics = {n_harmonics}
gamma_mean = {fmt(gamma_mean_global)}
phi_mean = {fmt(phi_mean_global)}
psi_mean = {fmt(psi_mean_global)}

# Integration/control
tether = {tether_str}
n_blade_elements = {DEFAULT_N_BLADE_ELEMENTS}
n_wingbeats = {n_wingbeats}
steps_per_wingbeat = {steps_per_wingbeat}

# Initial conditions
x0 = 0.0
y0 = 0.0
z0 = 0.0
ux0 = {fmt(ux0)}
uy0 = {fmt(uy0)}
uz0 = {fmt(uz0)}

# Output
output = {output_name}

{fore_left}

{fore_right}

{hind_left}

{hind_right}
"""


def stage_translate(run_dir: Path, params: PipelineParams, *, experiment_id: str) -> tuple[Path, Path]:
    deterministic_manifest = run_dir_uses_docs_artifacts(run_dir)
    experiment_case = resolve_experiment_case(experiment_id)

    n_harmonics = int(experiment_case["simulation_defaults"]["n_harmonics"])
    omega = omega_nondim(
        frequency_hz=params.frequency_hz,
        body_length_m=params.body_length_m,
        gravity=params.gravity,
    )
    fore_lb0, fore_mu0 = wing_scales(
        wing_span_m=params.fore_span_m,
        wing_area_m2=params.fore_area_m2,
        body_length_m=params.body_length_m,
        body_mass_kg=params.body_mass_kg,
        rho_air=params.rho_air,
    )
    hind_lb0, hind_mu0 = wing_scales(
        wing_span_m=params.hind_span_m,
        wing_area_m2=params.hind_area_m2,
        body_length_m=params.body_length_m,
        body_mass_kg=params.body_mass_kg,
        rho_air=params.rho_air,
    )
    wing_motion, mapping_summary = azuma1988_wing_motion(
        experiment_id=experiment_id,
        n_harmonics=n_harmonics,
        stroke_plane_deg=params.stroke_plane_deg,
        fore_stroke_plane_deg=params.fore_stroke_plane_deg,
        hind_stroke_plane_deg=params.hind_stroke_plane_deg,
    )
    initial_velocity_nd, initial_velocity_meta = initial_velocity_from_experiment(
        experiment_case,
        body_length_m=params.body_length_m,
        gravity_m_s2=params.gravity,
    )

    sim_dir = ensure_dir(run_dir / "sim")
    cfg_path = sim_dir / "sim_azuma1988.cfg"
    output_h5 = sim_dir / params.output_name
    cfg_text = build_sim_cfg(
        experiment_id=experiment_id,
        n_harmonics=n_harmonics,
        omega=omega,
        n_wingbeats=params.n_wingbeats,
        steps_per_wingbeat=params.steps_per_wingbeat,
        tether=params.tether,
        output_name=params.output_name,
        fore_lb0=fore_lb0,
        fore_mu0=fore_mu0,
        hind_lb0=hind_lb0,
        hind_mu0=hind_mu0,
        wing_cd0=params.wing_cd0,
        wing_cl0=params.wing_cl0,
        wing_motion=wing_motion,
        ux0=initial_velocity_nd["ux0"],
        uy0=initial_velocity_nd["uy0"],
        uz0=initial_velocity_nd["uz0"],
    )
    cfg_path.write_text(cfg_text, encoding="utf-8")

    summary_path = sim_dir / "translate_summary.json"
    summary_payload: dict[str, Any] = {
        "sim_config_path": artifact_ref(cfg_path, deterministic=deterministic_manifest),
        "sim_output_path": artifact_ref(output_h5, deterministic=deterministic_manifest),
        "n_harmonics": n_harmonics,
        "n_wingbeats": params.n_wingbeats,
        "steps_per_wingbeat": params.steps_per_wingbeat,
        "tether": params.tether,
        "nondimensional_parameters": {
            "omega": omega,
            "fore_lb0_lambda": fore_lb0,
            "fore_mu0_mu": fore_mu0,
            "hind_lb0_lambda": hind_lb0,
            "hind_mu0_mu": hind_mu0,
        },
        "physical_inputs": {
            "body_length_m": params.body_length_m,
            "body_mass_kg": params.body_mass_kg,
            "fore_span_m": params.fore_span_m,
            "fore_area_m2": params.fore_area_m2,
            "hind_span_m": params.hind_span_m,
            "hind_area_m2": params.hind_area_m2,
            "frequency_hz": params.frequency_hz,
            "stroke_plane_deg": params.stroke_plane_deg,
            "fore_stroke_plane_deg": params.fore_stroke_plane_deg,
            "hind_stroke_plane_deg": params.hind_stroke_plane_deg,
            "rho_air_kg_m3": params.rho_air,
            "gravity_m_s2": params.gravity,
        },
        "initial_velocity": {
            "nondimensional": initial_velocity_nd,
            "experiment_reference": {
                "speed_m_s": initial_velocity_meta["speed_m_s"],
                "direction_deg": initial_velocity_meta["direction_deg"],
                "direction_definition": initial_velocity_meta["direction_definition"],
            },
            "speed_scale_m_s": initial_velocity_meta["speed_scale_m_s"],
            "speed_nondim": initial_velocity_meta["speed_nondim"],
        },
        "selected_experiment": experiment_case["selected_experiment"],
        "convention_mapping": mapping_summary,
    }
    if not deterministic_manifest:
        summary_payload["generated_at_utc"] = now_utc_iso()

    write_json(summary_path, summary_payload)

    update_manifest(
        run_dir,
        "translate",
        [cfg_path, summary_path],
        {
            "sim_config_path": str(cfg_path.resolve()),
            "sim_output_path": str(output_h5.resolve()),
            "experiment_id": str(experiment_id),
        },
        repo_root=REPO_ROOT,
        deterministic=deterministic_manifest,
    )
    return cfg_path, output_h5


def stage_sim(run_dir: Path, binary: str, params: PipelineParams, *, experiment_id: str) -> Path:
    deterministic_manifest = run_dir_uses_docs_artifacts(run_dir)

    cfg_path, output_h5 = stage_translate(run_dir=run_dir, params=params, experiment_id=experiment_id)

    binary_path = Path(binary).expanduser()
    if not binary_path.is_absolute():
        binary_path = REPO_ROOT / binary_path
    if not binary_path.exists():
        raise FileNotFoundError(f"dragonfly binary not found: {binary_path}")

    run_cmd([str(binary_path), "sim", "-c", str(cfg_path)], cwd=cfg_path.parent)
    if not output_h5.exists():
        raise FileNotFoundError(f"Simulation output not found: {output_h5}")

    update_manifest(
        run_dir,
        "sim",
        [cfg_path, output_h5],
        {
            "binary": str(binary_path.resolve()),
            "config": str(cfg_path.resolve()),
            "output_h5": str(output_h5.resolve()),
            "experiment_id": str(experiment_id),
        },
        repo_root=REPO_ROOT,
        deterministic=deterministic_manifest,
    )
    return output_h5


def stage_post(
    run_dir: Path,
    params: PipelineParams,
    binary: str,
    input_h5: str | None,
    no_blender: bool,
    frame_step: int,
    *,
    experiment_id: str,
) -> list[Path]:
    if frame_step < 1:
        raise ValueError(f"frame_step must be >= 1, got {frame_step}")

    deterministic_manifest = run_dir_uses_docs_artifacts(run_dir)

    if input_h5 is not None:
        h5_path = Path(input_h5).expanduser()
        if not h5_path.is_absolute():
            h5_path = REPO_ROOT / h5_path
    else:
        h5_path = run_dir / "sim" / params.output_name
        if not h5_path.exists():
            h5_path = stage_sim(run_dir=run_dir, binary=binary, params=params, experiment_id=experiment_id)

    if not h5_path.exists():
        raise FileNotFoundError(f"Input HDF5 not found: {h5_path}")

    plot_env = build_plot_env(run_dir)
    post_dir = ensure_dir(run_dir / "post")
    sim_mp4 = post_dir / "simulation.mp4"
    flight_metrics_png = post_dir / "flight_metrics.png"

    cmd = [sys.executable, "-m", "post.plot_simulation", str(h5_path), str(sim_mp4)]
    if no_blender:
        cmd.append("--no-blender")
    if frame_step > 1:
        cmd.extend(["--frame-step", str(frame_step)])
    run_cmd(cmd, cwd=REPO_ROOT, env=plot_env)
    run_cmd(
        [
            sys.executable,
            "-m",
            "post.plot_azuma1988_flight_metrics",
            str(h5_path),
            str(flight_metrics_png),
            "--experiment",
            str(experiment_id),
        ],
        cwd=REPO_ROOT,
        env=plot_env,
    )
    artifacts = [sim_mp4, flight_metrics_png]

    update_manifest(
        run_dir,
        "post",
        artifacts,
        {
            "input_h5": str(h5_path.resolve()),
            "no_blender": no_blender,
            "frame_step": frame_step,
            "experiment_id": str(experiment_id),
        },
        repo_root=REPO_ROOT,
        deterministic=deterministic_manifest,
    )
    return artifacts


def add_shared_args(p: argparse.ArgumentParser) -> None:
    experiments = AZUMA1988_CASE.get("experiments")
    experiment_choices = None
    if isinstance(experiments, dict) and experiments:
        experiment_choices = sorted(experiments.keys(), key=lambda s: int(s) if str(s).isdigit() else str(s))

    p.add_argument("--run-dir", default=None, help="Run directory (default: runs/azuma1988/<timestamp>).")
    p.add_argument("--binary", default=str(DEFAULT_BINARY), help="Path to dragonfly binary.")
    p.add_argument(
        "--experiment",
        default=DEFAULT_EXPERIMENT_ID,
        choices=experiment_choices,
        help="Azuma (1988) experiment identifier from case.json experiments map.",
    )
    p.add_argument("--n-wingbeats", type=int, default=5)
    p.add_argument("--steps-per-wingbeat", type=int, default=200)
    p.add_argument("--tether", action="store_true")
    p.add_argument("--output-name", default="output.h5")
    p.add_argument("--wing-cd0", type=float, default=DEFAULT_CD0)
    p.add_argument("--wing-cl0", type=float, default=DEFAULT_CL0)
    p.add_argument("--body-length-m", type=float, default=DEFAULT_BODY_LENGTH_M)
    p.add_argument("--body-mass-kg", type=float, default=DEFAULT_BODY_MASS_KG)
    p.add_argument("--fore-span-m", type=float, default=DEFAULT_R_FW_M)
    p.add_argument("--fore-area-m2", type=float, default=DEFAULT_S_FW_M2)
    p.add_argument("--hind-span-m", type=float, default=DEFAULT_R_HW_M)
    p.add_argument("--hind-area-m2", type=float, default=DEFAULT_S_HW_M2)
    p.add_argument(
        "--frequency-hz",
        type=float,
        default=None,
        help="Flapping frequency in Hz (default: selected experiment frequency).",
    )
    p.add_argument("--stroke-plane-deg", type=float, default=None)
    p.add_argument("--fore-stroke-plane-deg", type=float, default=None)
    p.add_argument("--hind-stroke-plane-deg", type=float, default=None)
    p.add_argument("--rho-air", type=float, default=DEFAULT_RHO_AIR)
    p.add_argument("--gravity", type=float, default=DEFAULT_GRAVITY)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="stage", required=True)

    translate_parser = subparsers.add_parser("translate", help="Generate sim config from Azuma 1988 parameters.")
    add_shared_args(translate_parser)

    sim_parser = subparsers.add_parser("sim", help="Run simulation from generated config.")
    add_shared_args(sim_parser)

    post_parser = subparsers.add_parser("post", help="Run postprocessing from simulation output.")
    add_shared_args(post_parser)
    post_parser.add_argument("--input-h5", default=None)
    post_parser.add_argument("--no-blender", action="store_true")
    post_parser.add_argument("--frame-step", type=int, default=1)

    all_parser = subparsers.add_parser("all", help="Run translate -> sim -> post.")
    add_shared_args(all_parser)
    all_parser.add_argument("--no-blender", action="store_true")
    all_parser.add_argument("--frame-step", type=int, default=1)

    args = parser.parse_args()
    experiment_case = resolve_experiment_case(args.experiment)
    sim_defaults = experiment_case["simulation_defaults"]
    run_dir = ensure_dir(resolve_run_dir(args.run_dir, repo_root=REPO_ROOT, runs_root=RUNS_ROOT))

    # Resolve experiment-scoped defaults after parsing, so --experiment can drive defaults.
    args.frequency_hz = (
        float(sim_defaults["frequency_hz"]) if args.frequency_hz is None else float(args.frequency_hz)
    )
    params = PipelineParams.from_args(args)
    experiment_id = str(experiment_case["selected_experiment"]["id"])

    if args.stage == "translate":
        cfg_path, output_h5 = stage_translate(run_dir=run_dir, params=params, experiment_id=experiment_id)
        print(f"[done] cfg: {cfg_path}")
        print(f"[done] sim_output: {output_h5}")
        print(f"[done] run_dir: {run_dir}")
        return 0

    if args.stage == "sim":
        output_h5 = stage_sim(run_dir=run_dir, binary=args.binary, params=params, experiment_id=experiment_id)
        print(f"[done] sim_output: {output_h5}")
        print(f"[done] run_dir: {run_dir}")
        return 0

    if args.stage == "post":
        artifacts = stage_post(
            run_dir=run_dir,
            params=params,
            binary=args.binary,
            input_h5=args.input_h5,
            no_blender=args.no_blender,
            frame_step=args.frame_step,
            experiment_id=experiment_id,
        )
        print("[done] post artifacts:")
        for artifact in artifacts:
            print(f"  - {artifact}")
        print(f"[done] run_dir: {run_dir}")
        return 0

    if args.stage == "all":
        stage_sim(run_dir=run_dir, binary=args.binary, params=params, experiment_id=experiment_id)
        artifacts = stage_post(
            run_dir=run_dir,
            params=params,
            binary=args.binary,
            input_h5=None,
            no_blender=args.no_blender,
            frame_step=args.frame_step,
            experiment_id=experiment_id,
        )
        print("[done] post artifacts:")
        for artifact in artifacts:
            print(f"  - {artifact}")
        print(f"[done] run_dir: {run_dir}")
        return 0

    raise RuntimeError(f"Unhandled stage: {args.stage}")


if __name__ == "__main__":
    raise SystemExit(main())

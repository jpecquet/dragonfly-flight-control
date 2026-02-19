#!/usr/bin/env python3
"""Convenience pipeline: Azuma 1985 parameters -> sim -> post artifacts.

Stages:
  translate Generate a simulator config from Azuma (1985) parameters.
  sim       Run dragonfly simulation for generated config.
  post      Run postprocessing (3D animation + flight metrics) from simulation output.
  all       Run translate -> sim -> post.
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

try:
    from experimental_conventions import azuma1985_adapter, build_sim_wing_motion
except ModuleNotFoundError:
    from scripts.experimental_conventions import azuma1985_adapter, build_sim_wing_motion

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
    from case_data import load_case_data
except ModuleNotFoundError:
    from scripts.case_data import load_case_data


REPO_ROOT = Path(__file__).resolve().parents[1]
RUNS_ROOT = REPO_ROOT / "runs" / "azuma1985"
DEFAULT_BINARY = REPO_ROOT / "build" / "bin" / "dragonfly"
AZUMA1985_CASE = load_case_data("azuma1985")
AZUMA1985_SPECIMEN = AZUMA1985_CASE["specimen"]
AZUMA1985_SIM_DEFAULTS = AZUMA1985_CASE["simulation_defaults"]

# Azuma (1985) defaults loaded from data/case_studies/azuma1985/case.json
DEFAULT_BODY_LENGTH_M = float(AZUMA1985_SPECIMEN["body_length_m"])
DEFAULT_BODY_MASS_KG = float(AZUMA1985_SPECIMEN["body_mass_kg"])
DEFAULT_R_FW_M = float(AZUMA1985_SPECIMEN["fore_span_m"])
DEFAULT_S_FW_M2 = float(AZUMA1985_SPECIMEN["fore_area_m2"])
DEFAULT_R_HW_M = float(AZUMA1985_SPECIMEN["hind_span_m"])
DEFAULT_S_HW_M2 = float(AZUMA1985_SPECIMEN["hind_area_m2"])
DEFAULT_FREQUENCY_HZ = float(AZUMA1985_SIM_DEFAULTS["frequency_hz"])
DEFAULT_RHO_AIR = float(AZUMA1985_SIM_DEFAULTS["rho_air_kg_m3"])
DEFAULT_GRAVITY = float(AZUMA1985_SIM_DEFAULTS["gravity_m_s2"])
DEFAULT_CD0 = float(AZUMA1985_SIM_DEFAULTS["wing_cd0"])
DEFAULT_CL0 = float(AZUMA1985_SIM_DEFAULTS["wing_cl0"])
DEFAULT_N_HARMONICS = int(AZUMA1985_SIM_DEFAULTS["n_harmonics"])
DEFAULT_N_BLADE_ELEMENTS = 5

# Coning angles (radians), loaded from case data.
FORE_CONE_RAD = math.radians(float(AZUMA1985_SIM_DEFAULTS["coning_angles_deg"]["fore"]))
HIND_CONE_RAD = math.radians(float(AZUMA1985_SIM_DEFAULTS["coning_angles_deg"]["hind"]))


def run_dir_uses_docs_artifacts(run_dir: Path) -> bool:
    docs_root = (REPO_ROOT / "docs" / "case_studies").resolve()
    run_dir = run_dir.resolve()
    try:
        run_dir.relative_to(docs_root)
        return True
    except ValueError:
        return False


def artifact_ref(path: Path, *, deterministic: bool) -> str:
    resolved = path.resolve()
    if not deterministic:
        return str(resolved)
    try:
        return str(resolved.relative_to(REPO_ROOT.resolve()))
    except ValueError:
        return str(resolved)


@dataclass(frozen=True)
class PipelineParams:
    n_wingbeats: int
    steps_per_wingbeat: int
    tether: bool
    output_name: str
    wing_cd0: float
    wing_cl0: float
    body_length_m: float
    body_mass_kg: float
    fore_span_m: float
    fore_area_m2: float
    hind_span_m: float
    hind_area_m2: float
    frequency_hz: float
    stroke_plane_deg: float | None
    fore_stroke_plane_deg: float | None
    hind_stroke_plane_deg: float | None
    rho_air: float
    gravity: float

    @classmethod
    def from_args(cls, args: argparse.Namespace) -> "PipelineParams":
        return cls(
            n_wingbeats=args.n_wingbeats,
            steps_per_wingbeat=args.steps_per_wingbeat,
            tether=args.tether,
            output_name=args.output_name,
            wing_cd0=args.wing_cd0,
            wing_cl0=args.wing_cl0,
            body_length_m=args.body_length_m,
            body_mass_kg=args.body_mass_kg,
            fore_span_m=args.fore_span_m,
            fore_area_m2=args.fore_area_m2,
            hind_span_m=args.hind_span_m,
            hind_area_m2=args.hind_area_m2,
            frequency_hz=args.frequency_hz,
            stroke_plane_deg=args.stroke_plane_deg,
            fore_stroke_plane_deg=args.fore_stroke_plane_deg,
            hind_stroke_plane_deg=args.hind_stroke_plane_deg,
            rho_air=args.rho_air,
            gravity=args.gravity,
        )


def omega_nondim(frequency_hz: float, body_length_m: float, gravity: float) -> float:
    if frequency_hz <= 0.0:
        raise ValueError("frequency_hz must be > 0")
    if body_length_m <= 0.0:
        raise ValueError("body_length_m must be > 0")
    if gravity <= 0.0:
        raise ValueError("gravity must be > 0")
    return 2.0 * math.pi * frequency_hz * math.sqrt(body_length_m / gravity)


def wing_scales(
    wing_span_m: float,
    wing_area_m2: float,
    body_length_m: float,
    body_mass_kg: float,
    rho_air: float,
) -> tuple[float, float]:
    if wing_span_m <= 0.0:
        raise ValueError("wing_span_m must be > 0")
    if wing_area_m2 <= 0.0:
        raise ValueError("wing_area_m2 must be > 0")
    if body_length_m <= 0.0:
        raise ValueError("body_length_m must be > 0")
    if body_mass_kg <= 0.0:
        raise ValueError("body_mass_kg must be > 0")
    if rho_air <= 0.0:
        raise ValueError("rho_air must be > 0")
    lb0 = wing_span_m / body_length_m
    mu0 = rho_air * wing_area_m2 * wing_span_m / body_mass_kg
    return lb0, mu0


def azuma_wing_motion(
    n_harmonics: int,
    stroke_plane_deg: float | None,
    fore_stroke_plane_deg: float | None,
    hind_stroke_plane_deg: float | None,
) -> tuple[dict[str, dict[str, Any]], dict[str, Any]]:
    adapter = azuma1985_adapter()
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
    return wing_motion, mapping_summary


def build_sim_cfg(
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
) -> str:
    tether_str = "true" if tether else "false"

    fore = wing_motion["fore"]
    hind = wing_motion["hind"]
    gamma_mean_global = 0.5 * (float(fore["gamma_mean"]) + float(hind["gamma_mean"]))
    phi_mean_global = 0.5 * (float(fore["phi_mean"]) + float(hind["phi_mean"]))
    psi_mean_global = 0.5 * (float(fore["psi_mean"]) + float(hind["psi_mean"]))

    fore_left = build_wing_block(
        name="fore",
        side="left",
        wing_mu0=fore_mu0,
        wing_lb0=fore_lb0,
        wing_cd0=wing_cd0,
        wing_cl0=wing_cl0,
        phase=0.0,
        motion=fore,
        cone=FORE_CONE_RAD,
    )
    fore_right = build_wing_block(
        name="fore",
        side="right",
        wing_mu0=fore_mu0,
        wing_lb0=fore_lb0,
        wing_cd0=wing_cd0,
        wing_cl0=wing_cl0,
        phase=0.0,
        motion=fore,
        cone=FORE_CONE_RAD,
    )
    hind_left = build_wing_block(
        name="hind",
        side="left",
        wing_mu0=hind_mu0,
        wing_lb0=hind_lb0,
        wing_cd0=wing_cd0,
        wing_cl0=wing_cl0,
        phase=0.0,
        motion=hind,
        cone=HIND_CONE_RAD,
    )
    hind_right = build_wing_block(
        name="hind",
        side="right",
        wing_mu0=hind_mu0,
        wing_lb0=hind_lb0,
        wing_cd0=wing_cd0,
        wing_cl0=wing_cl0,
        phase=0.0,
        motion=hind,
        cone=HIND_CONE_RAD,
    )

    return f"""# Auto-generated by scripts/azuma1985_pipeline.py
# Source: data/case_studies/azuma1985/case.json

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


def stage_translate(run_dir: Path, params: PipelineParams) -> tuple[Path, Path]:
    deterministic_manifest = run_dir_uses_docs_artifacts(run_dir)

    n_harmonics = DEFAULT_N_HARMONICS
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
    wing_motion, mapping_summary = azuma_wing_motion(
        n_harmonics=n_harmonics,
        stroke_plane_deg=params.stroke_plane_deg,
        fore_stroke_plane_deg=params.fore_stroke_plane_deg,
        hind_stroke_plane_deg=params.hind_stroke_plane_deg,
    )

    sim_dir = ensure_dir(run_dir / "sim")
    cfg_path = sim_dir / "sim_azuma1985.cfg"
    output_h5 = sim_dir / params.output_name
    cfg_text = build_sim_cfg(
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
        "convention_mapping": mapping_summary,
    }
    if not deterministic_manifest:
        summary_payload["generated_at_utc"] = now_utc_iso()

    write_json(
        summary_path,
        summary_payload,
    )

    update_manifest(
        run_dir,
        "translate",
        [cfg_path, summary_path],
        {
            "sim_config_path": str(cfg_path.resolve()),
            "sim_output_path": str(output_h5.resolve()),
        },
        repo_root=REPO_ROOT,
        deterministic=deterministic_manifest,
    )
    return cfg_path, output_h5


def stage_sim(run_dir: Path, binary: str, params: PipelineParams) -> Path:
    deterministic_manifest = run_dir_uses_docs_artifacts(run_dir)

    # Always refresh translated config so simulation stays aligned with current
    # paper->sim convention mapping and CLI parameter overrides.
    cfg_path, output_h5 = stage_translate(run_dir=run_dir, params=params)

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
            h5_path = stage_sim(run_dir=run_dir, binary=binary, params=params)

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
            "post.plot_azuma1985_flight_metrics",
            str(h5_path),
            str(flight_metrics_png),
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
        },
        repo_root=REPO_ROOT,
        deterministic=deterministic_manifest,
    )
    return artifacts


def add_shared_args(p: argparse.ArgumentParser) -> None:
    p.add_argument(
        "--run-dir",
        default=None,
        help="Run directory (default: runs/azuma1985/<timestamp>). Relative paths are repo-relative.",
    )
    p.add_argument(
        "--binary",
        default=str(DEFAULT_BINARY),
        help="Path to dragonfly binary.",
    )
    p.add_argument(
        "--n-wingbeats",
        type=int,
        default=5,
        help="Number of wingbeats for generated simulation config (default: 5).",
    )
    p.add_argument(
        "--steps-per-wingbeat",
        type=int,
        default=200,
        help="Integration steps per wingbeat.",
    )
    p.add_argument(
        "--tether",
        action="store_true",
        help="Generate tethered simulation config.",
    )
    p.add_argument(
        "--output-name",
        default="output.h5",
        help="Simulation output filename inside run_dir/sim (default: output.h5).",
    )
    p.add_argument(
        "--wing-cd0",
        type=float,
        default=DEFAULT_CD0,
        help="Wing Cd0 used in generated config.",
    )
    p.add_argument(
        "--wing-cl0",
        type=float,
        default=DEFAULT_CL0,
        help="Wing Cl0 used in generated config.",
    )
    p.add_argument(
        "--body-length-m",
        type=float,
        default=DEFAULT_BODY_LENGTH_M,
        help="Body length L (meters).",
    )
    p.add_argument(
        "--body-mass-kg",
        type=float,
        default=DEFAULT_BODY_MASS_KG,
        help="Body mass (kg).",
    )
    p.add_argument(
        "--fore-span-m",
        type=float,
        default=DEFAULT_R_FW_M,
        help="Fore wing span/length R_fw (meters).",
    )
    p.add_argument(
        "--fore-area-m2",
        type=float,
        default=DEFAULT_S_FW_M2,
        help="Fore wing area S_fw (m^2).",
    )
    p.add_argument(
        "--hind-span-m",
        type=float,
        default=DEFAULT_R_HW_M,
        help="Hind wing span/length R_hw (meters).",
    )
    p.add_argument(
        "--hind-area-m2",
        type=float,
        default=DEFAULT_S_HW_M2,
        help="Hind wing area S_hw (m^2).",
    )
    p.add_argument(
        "--frequency-hz",
        type=float,
        default=DEFAULT_FREQUENCY_HZ,
        help="Wingbeat frequency in Hz.",
    )
    p.add_argument(
        "--stroke-plane-deg",
        type=float,
        default=None,
        help="Override stroke plane angle gamma (degrees) for both fore and hind wings.",
    )
    p.add_argument(
        "--fore-stroke-plane-deg",
        type=float,
        default=None,
        help="Optional fore-wing-specific override of stroke plane angle gamma (degrees).",
    )
    p.add_argument(
        "--hind-stroke-plane-deg",
        type=float,
        default=None,
        help="Optional hind-wing-specific override of stroke plane angle gamma (degrees).",
    )
    p.add_argument(
        "--rho-air",
        type=float,
        default=DEFAULT_RHO_AIR,
        help="Air density in kg/m^3 for mu = rho*S*L_wing/m.",
    )
    p.add_argument(
        "--gravity",
        type=float,
        default=DEFAULT_GRAVITY,
        help="Gravity in m/s^2 for omega = 2*pi*f*sqrt(L/g).",
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="stage", required=True)

    translate_parser = subparsers.add_parser("translate", help="Generate sim config from Azuma parameters.")
    add_shared_args(translate_parser)

    sim_parser = subparsers.add_parser("sim", help="Run simulation from generated config.")
    add_shared_args(sim_parser)

    post_parser = subparsers.add_parser("post", help="Run postprocessing from simulation output.")
    add_shared_args(post_parser)
    post_parser.add_argument(
        "--input-h5",
        default=None,
        help="Optional explicit HDF5 input. If omitted, uses run_dir/sim/<output-name>.",
    )
    post_parser.add_argument("--no-blender", action="store_true", help="Force matplotlib-only rendering.")
    post_parser.add_argument(
        "--frame-step",
        type=int,
        default=1,
        help="Render every Nth frame in 3D animation (default: 1).",
    )

    all_parser = subparsers.add_parser("all", help="Run translate -> sim -> post.")
    add_shared_args(all_parser)
    all_parser.add_argument("--no-blender", action="store_true", help="Force matplotlib-only rendering.")
    all_parser.add_argument(
        "--frame-step",
        type=int,
        default=1,
        help="Render every Nth frame in 3D animation (default: 1).",
    )

    args = parser.parse_args()
    run_dir = ensure_dir(resolve_run_dir(args.run_dir, repo_root=REPO_ROOT, runs_root=RUNS_ROOT))
    params = PipelineParams.from_args(args)

    if args.stage == "translate":
        cfg_path, output_h5 = stage_translate(run_dir=run_dir, params=params)
        print(f"[done] cfg: {cfg_path}")
        print(f"[done] sim_output: {output_h5}")
        print(f"[done] run_dir: {run_dir}")
        return 0

    if args.stage == "sim":
        output_h5 = stage_sim(run_dir=run_dir, binary=args.binary, params=params)
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
        )
        print("[done] post artifacts:")
        for artifact in artifacts:
            print(f"  - {artifact}")
        print(f"[done] run_dir: {run_dir}")
        return 0

    if args.stage == "all":
        stage_sim(run_dir=run_dir, binary=args.binary, params=params)
        artifacts = stage_post(
            run_dir=run_dir,
            params=params,
            binary=args.binary,
            input_h5=None,
            no_blender=args.no_blender,
            frame_step=args.frame_step,
        )
        print("[done] post artifacts:")
        for artifact in artifacts:
            print(f"  - {artifact}")
        print(f"[done] run_dir: {run_dir}")
        return 0

    raise RuntimeError(f"Unhandled stage: {args.stage}")


if __name__ == "__main__":
    raise SystemExit(main())

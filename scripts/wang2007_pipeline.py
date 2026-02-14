#!/usr/bin/env python3
"""Convenience pipeline: Wang 2007 kinematics -> sim -> postprocessing.

Stages:
  fit       Fit sinusoidal kinematics and export diagnostics/artifacts.
  translate Generate a simulator config from fit outputs.
  sim       Run dragonfly simulation for generated config.
  post      Run postprocessing animation(s) from simulation output.
  all       Run fit -> translate -> sim -> post.
"""

from __future__ import annotations

import argparse
import importlib
import json
import math
import os
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[1]
WANG_DIR = REPO_ROOT / "data" / "kinematics" / "wang2007"
RUNS_ROOT = REPO_ROOT / "runs" / "wang2007"
DEFAULT_BINARY = REPO_ROOT / "build" / "bin" / "dragonfly"
DEFAULT_BODY_LENGTH_MM = 50.0
DEFAULT_WING_LENGTH_MM = 40.0
DEFAULT_WING_AREA_MM2 = 400.0
DEFAULT_FREQUENCY_HZ = 33.4
DEFAULT_BODY_MASS_MG = 300.0
DEFAULT_RHO_AIR = 1.225
DEFAULT_GRAVITY = 9.81
DEFAULT_SMOOTHED_N_HARMONICS = 7
MOTION_SOURCE_FIT = "fit"
MOTION_SOURCE_SMOOTHED = "smoothed"
MOTION_SOURCES = (MOTION_SOURCE_FIT, MOTION_SOURCE_SMOOTHED)


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


def resolve_run_dir(run_dir_arg: str | None) -> Path:
    if run_dir_arg:
        p = Path(run_dir_arg).expanduser()
        if not p.is_absolute():
            p = REPO_ROOT / p
        return p
    run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
    return RUNS_ROOT / run_id


def get_git_commit() -> str:
    try:
        out = subprocess.check_output(
            ["git", "rev-parse", "HEAD"],
            cwd=str(REPO_ROOT),
            text=True,
            stderr=subprocess.DEVNULL,
        )
        return out.strip()
    except Exception:
        return "unknown"


def update_manifest(run_dir: Path, stage: str, artifacts: list[Path], metadata: dict[str, Any]) -> None:
    manifest_path = run_dir / "manifest.json"
    if manifest_path.exists():
        manifest = read_json(manifest_path)
    else:
        manifest = {
            "created_at_utc": now_utc_iso(),
            "repo_root": str(REPO_ROOT),
            "git_commit": get_git_commit(),
            "stages": {},
        }

    manifest["updated_at_utc"] = now_utc_iso()
    manifest["stages"][stage] = {
        "completed_at_utc": now_utc_iso(),
        "artifacts": [str(p.resolve()) for p in artifacts],
        "metadata": metadata,
    }
    write_json(manifest_path, manifest)


def load_common_module():
    if str(WANG_DIR) not in sys.path:
        sys.path.insert(0, str(WANG_DIR))
    return importlib.import_module("common")


def build_plot_env(run_dir: Path) -> dict[str, str]:
    cache_root = ensure_dir(run_dir / ".cache")
    mpl_cache = ensure_dir(cache_root / "matplotlib")
    return {
        "MPLCONFIGDIR": str(mpl_cache.resolve()),
        "XDG_CACHE_HOME": str(cache_root.resolve()),
    }


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


def fit_harmonic_series(
    common: Any,
    t: Any,
    y: Any,
    n_harmonics: int,
) -> tuple[float, list[float], list[float]]:
    if n_harmonics <= 0:
        raise ValueError("n_harmonics must be >= 1")

    np = common.np
    omega = float(common.omega)
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


def first_harmonic_from_cosine_fit(
    phi_amp: float,
    phi_phase: float,
    phi_mean_wang: float,
    beta_amp_deg: float,
    beta_phase: float,
    beta_mean_deg: float,
) -> dict[str, Any]:
    # phi_sim = pi/2 - phi_wang
    phi_mean = float(math.pi / 2.0 - phi_mean_wang)
    phi_cos = float(-phi_amp * math.cos(phi_phase))
    phi_sin = float(phi_amp * math.sin(phi_phase))

    # psi_sim = pi/2 - beta_wang
    beta_amp = float(math.radians(beta_amp_deg))
    beta_mean = float(math.radians(beta_mean_deg))
    psi_mean = float(math.pi / 2.0 - beta_mean)
    psi_cos = float(-beta_amp * math.cos(beta_phase))
    psi_sin = float(beta_amp * math.sin(beta_phase))

    return {
        "phi_mean": phi_mean,
        "phi_cos": [phi_cos],
        "phi_sin": [phi_sin],
        "psi_mean": psi_mean,
        "psi_cos": [psi_cos],
        "psi_sin": [psi_sin],
    }


def compute_smoothed_motion_mapping(
    common: Any,
    omega_nondim: float,
    n_harmonics: int,
) -> dict[str, Any]:
    if n_harmonics <= 0:
        raise ValueError("smoothed_n_harmonics must be >= 1")

    np = common.np
    smooth_kwargs = {"n_harmonics": n_harmonics, "n_points": 2000}
    t_sf, sf = common.fourier_smooth(*common.sorted_xy("s_fore"), **smooth_kwargs)
    t_sh, sh = common.fourier_smooth(*common.sorted_xy("s_hind"), **smooth_kwargs)
    t_bf, bf = common.fourier_smooth(*common.sorted_xy("beta_fore"), **smooth_kwargs)
    t_bh, bh = common.fourier_smooth(*common.sorted_xy("beta_hind"), **smooth_kwargs)

    phi_fore = np.pi / 2.0 - np.arccos(np.clip(sf / float(common.r), -1.0, 1.0))
    phi_hind = np.pi / 2.0 - np.arccos(np.clip(sh / float(common.r), -1.0, 1.0))
    psi_fore = np.pi / 2.0 - np.radians(bf)
    psi_hind = np.pi / 2.0 - np.radians(bh)

    phi_fore_mean, phi_fore_cos, phi_fore_sin = fit_harmonic_series(common, t_sf, phi_fore, n_harmonics)
    phi_hind_mean, phi_hind_cos, phi_hind_sin = fit_harmonic_series(common, t_sh, phi_hind, n_harmonics)
    psi_fore_mean, psi_fore_cos, psi_fore_sin = fit_harmonic_series(common, t_bf, psi_fore, n_harmonics)
    psi_hind_mean, psi_hind_cos, psi_hind_sin = fit_harmonic_series(common, t_bh, psi_hind, n_harmonics)

    gamma_fore = float(common.gam_fore)
    gamma_hind = float(common.gam_hind)
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
        },
        "parameters": {
            "omega": float(omega_nondim),
            "n_harmonics": int(n_harmonics),
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


def compute_fit_payload(
    body_length_mm: float,
    wing_length_mm: float,
    wing_area_mm2: float,
    frequency_hz: float,
    body_mass_mg: float,
    rho_air: float,
    gravity: float,
    smoothed_n_harmonics: int,
) -> dict[str, Any]:
    common = load_common_module()
    np = common.np

    t_sf, sf = common.sorted_xy("s_fore")
    t_sh, sh = common.sorted_xy("s_hind")
    t_bf, bf = common.sorted_xy("beta_fore")
    t_bh, bh = common.sorted_xy("beta_hind")

    phi_fore_exp = np.arccos(np.clip(sf / float(common.r), -1.0, 1.0))
    phi_hind_exp = np.arccos(np.clip(sh / float(common.r), -1.0, 1.0))

    A_phi_fore, psi_phi_fore, C_phi_fore = common.fit_cosine(t_sf, phi_fore_exp)
    A_phi_hind, psi_phi_hind, C_phi_hind = common.fit_cosine(t_sh, phi_hind_exp)
    A_beta_fore_deg, psi_beta_fore, C_beta_fore_deg = common.fit_cosine(t_bf, bf)
    A_beta_hind_deg, psi_beta_hind, C_beta_hind_deg = common.fit_cosine(t_bh, bh)

    fit_fore = first_harmonic_from_cosine_fit(
        phi_amp=float(A_phi_fore),
        phi_phase=float(psi_phi_fore),
        phi_mean_wang=float(C_phi_fore),
        beta_amp_deg=float(A_beta_fore_deg),
        beta_phase=float(psi_beta_fore),
        beta_mean_deg=float(C_beta_fore_deg),
    )
    fit_hind = first_harmonic_from_cosine_fit(
        phi_amp=float(A_phi_hind),
        phi_phase=float(psi_phi_hind),
        phi_mean_wang=float(C_phi_hind),
        beta_amp_deg=float(A_beta_hind_deg),
        beta_phase=float(psi_beta_hind),
        beta_mean_deg=float(C_beta_hind_deg),
    )

    gamma_fore = float(common.gam_fore)
    gamma_hind = float(common.gam_hind)
    gamma_mean = 0.5 * (gamma_fore + gamma_hind)
    zero = [0.0]

    scales = compute_nondim_scales(
        body_length_mm=body_length_mm,
        wing_length_mm=wing_length_mm,
        wing_area_mm2=wing_area_mm2,
        frequency_hz=frequency_hz,
        body_mass_mg=body_mass_mg,
        rho_air=rho_air,
        gravity=gravity,
    )
    smoothed_mapping = compute_smoothed_motion_mapping(
        common,
        omega_nondim=float(scales["omega_nondim"]),
        n_harmonics=smoothed_n_harmonics,
    )
    smoothed_mapping["wing_parameters"] = {
        "lb0_lambda": float(scales["lambda_nondim"]),
        "mu0_mu": float(scales["mu_nondim"]),
    }
    fit_mapping = {
        "source": "per_wing_sinusoid_fit",
        "method": {
            "fit_model": "C + A*cos(2*pi*t + psi)",
            "fore_hind_fitted_independently": True,
            "n_harmonics": 1,
        },
        "parameters": {
            "omega": float(scales["omega_nondim"]),
            "n_harmonics": 1,
            "gamma_mean": gamma_mean,
            "phi_mean": 0.5 * (float(fit_fore["phi_mean"]) + float(fit_hind["phi_mean"])),
            "psi_mean": 0.5 * (float(fit_fore["psi_mean"]) + float(fit_hind["psi_mean"])),
        },
        "wing_parameters": {
            "lb0_lambda": float(scales["lambda_nondim"]),
            "mu0_mu": float(scales["mu_nondim"]),
        },
        "wing_phase_offsets": {
            "fore": 0.0,
            "hind": 0.0,
        },
        "wing_motion_overrides": {
            "fore": {
                "gamma_mean": gamma_fore,
                "gamma_cos": zero.copy(),
                "gamma_sin": zero.copy(),
                **fit_fore,
            },
            "hind": {
                "gamma_mean": gamma_hind,
                "gamma_cos": zero.copy(),
                "gamma_sin": zero.copy(),
                **fit_hind,
            },
        },
        "notes": [
            "Fore and hind wing kinematics are fitted independently from raw experimental traces.",
            "Stroke-plane angle uses per-wing values (fore and hind are not averaged in wing overrides).",
            "Wing phase offsets are zero; phase is represented directly in per-wing harmonic coefficients.",
        ],
    }

    return {
        "source": {
            "exp_data_csv": str((WANG_DIR / "exp_data.csv").resolve()),
            "computed_at_utc": now_utc_iso(),
        },
        "nondimensionalization": {
            "physical_inputs": {
                "body_length_mm": body_length_mm,
                "wing_length_mm": wing_length_mm,
                "wing_area_mm2": wing_area_mm2,
                "frequency_hz": frequency_hz,
                "body_mass_mg": body_mass_mg,
                "rho_air_kg_m3": rho_air,
                "gravity_m_s2": gravity,
            },
            "derived": scales,
            "definitions": {
                "time_scale": "sqrt(L/g)",
                "omega": "2*pi*f*sqrt(L/g)",
                "lambda": "L_wing / L",
                "mu": "rho_air * wing_surface_area * L_wing / m",
            },
        },
        "fit_raw": {
            "A_phi_rad": float(A_phi_fore),
            "psi_phi_rad": float(psi_phi_fore),
            "C_phi_rad": float(C_phi_fore),
            "A_beta_deg": float(A_beta_fore_deg),
            "psi_beta_rad": float(psi_beta_fore),
            "C_beta_deg": float(C_beta_fore_deg),
            "fore": {
                "A_phi_rad": float(A_phi_fore),
                "psi_phi_rad": float(psi_phi_fore),
                "C_phi_rad": float(C_phi_fore),
                "A_beta_deg": float(A_beta_fore_deg),
                "psi_beta_rad": float(psi_beta_fore),
                "C_beta_deg": float(C_beta_fore_deg),
            },
            "hind": {
                "A_phi_rad": float(A_phi_hind),
                "psi_phi_rad": float(psi_phi_hind),
                "C_phi_rad": float(C_phi_hind),
                "A_beta_deg": float(A_beta_hind_deg),
                "psi_beta_rad": float(psi_beta_hind),
                "C_beta_deg": float(C_beta_hind_deg),
            },
        },
        "simulator_mapping": fit_mapping,
        "simulator_mapping_smoothed": smoothed_mapping,
        "physical_reference": {
            "f_phys_hz": float(common.f_phys),
            "omega_phys_rad_s": float(common.omega_phys),
            "mass_body_kg": float(common.m_body),
            "mg_newton": float(common.mg),
        },
    }


def stage_fit(
    run_dir: Path,
    make_plots: bool,
    body_length_mm: float,
    wing_length_mm: float,
    wing_area_mm2: float,
    frequency_hz: float,
    body_mass_mg: float,
    rho_air: float,
    gravity: float,
    smoothed_n_harmonics: int,
) -> Path:
    fit_dir = ensure_dir(run_dir / "fit")
    fit_params_path = fit_dir / "fit_params.json"
    plot_env = build_plot_env(run_dir)
    os.environ.update(plot_env)

    payload = compute_fit_payload(
        body_length_mm=body_length_mm,
        wing_length_mm=wing_length_mm,
        wing_area_mm2=wing_area_mm2,
        frequency_hz=frequency_hz,
        body_mass_mg=body_mass_mg,
        rho_air=rho_air,
        gravity=gravity,
        smoothed_n_harmonics=smoothed_n_harmonics,
    )
    write_json(fit_params_path, payload)

    artifacts = [fit_params_path]
    if make_plots:
        scripts = [
            "plot_data.py",
            "fit_sinusoids.py",
            "compare_alpha.py",
            "plot_force.py",
        ]
        for script in scripts:
            run_cmd([sys.executable, str(WANG_DIR / script)], cwd=fit_dir, env=plot_env)
        artifacts.extend(
            [
                fit_dir / "kinematics.pdf",
                fit_dir / "fit_sinusoids.pdf",
                fit_dir / "compare_alpha.pdf",
                fit_dir / "force.pdf",
            ]
        )

    update_manifest(
        run_dir,
        "fit",
        artifacts,
        {
            "fit_params_path": str(fit_params_path.resolve()),
            "plots_generated": make_plots,
            "smoothed_n_harmonics": smoothed_n_harmonics,
        },
    )
    return fit_params_path


def fmt(x: float) -> str:
    return f"{x:.12f}"


def fmt_list(values: list[float]) -> str:
    return ", ".join(fmt(float(x)) for x in values)


def build_wing_block(
    name: str,
    side: str,
    wing_mu0: float,
    wing_lb0: float,
    wing_cd0: float,
    wing_cl0: float,
    phase: float,
    motion: dict[str, Any] | None = None,
) -> str:
    lines = [
        "[[wing]]",
        f"name = {name}",
        f"side = {side}",
        f"mu0 = {fmt(wing_mu0)}",
        f"lb0 = {fmt(wing_lb0)}",
        f"Cd0 = {fmt(wing_cd0)}",
        f"Cl0 = {fmt(wing_cl0)}",
        f"phase = {fmt(phase)}",
    ]

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


def build_sim_cfg(
    mapping: dict[str, Any],
    n_wingbeats: int,
    steps_per_wingbeat: int,
    tether: bool,
    output_name: str,
    wing_mu0: float,
    wing_lb0: float,
    wing_cd0: float,
    wing_cl0: float,
    motion_source_comment: str,
) -> str:
    params = mapping["parameters"]
    phase = mapping["wing_phase_offsets"]
    overrides = mapping["wing_motion_overrides"]
    tether_str = "true" if tether else "false"

    fore_left = build_wing_block(
        name="fore",
        side="left",
        wing_mu0=wing_mu0,
        wing_lb0=wing_lb0,
        wing_cd0=wing_cd0,
        wing_cl0=wing_cl0,
        phase=float(phase["fore"]),
        motion=overrides["fore"],
    )
    fore_right = build_wing_block(
        name="fore",
        side="right",
        wing_mu0=wing_mu0,
        wing_lb0=wing_lb0,
        wing_cd0=wing_cd0,
        wing_cl0=wing_cl0,
        phase=float(phase["fore"]),
        motion=overrides["fore"],
    )
    hind_left = build_wing_block(
        name="hind",
        side="left",
        wing_mu0=wing_mu0,
        wing_lb0=wing_lb0,
        wing_cd0=wing_cd0,
        wing_cl0=wing_cl0,
        phase=float(phase["hind"]),
        motion=overrides["hind"],
    )
    hind_right = build_wing_block(
        name="hind",
        side="right",
        wing_mu0=wing_mu0,
        wing_lb0=wing_lb0,
        wing_cd0=wing_cd0,
        wing_cl0=wing_cl0,
        phase=float(phase["hind"]),
        motion=overrides["hind"],
    )

    return f"""# Auto-generated by scripts/wang2007_pipeline.py
# Source: data/kinematics/wang2007/exp_data.csv
# Wing motion source: {motion_source_comment}

# Kinematic parameters
omega = {fmt(float(params["omega"]))}
n_harmonics = {int(params["n_harmonics"])}
gamma_mean = {fmt(float(params["gamma_mean"]))}
phi_mean = {fmt(float(params["phi_mean"]))}
psi_mean = {fmt(float(params["psi_mean"]))}

# Integration/control
tether = {tether_str}
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


def stage_translate(
    run_dir: Path,
    n_wingbeats: int,
    steps_per_wingbeat: int,
    tether: bool,
    output_name: str,
    wing_mu0: float | None,
    wing_lb0: float | None,
    wing_cd0: float,
    wing_cl0: float,
    auto_fit: bool,
    fit_plots: bool,
    body_length_mm: float,
    wing_length_mm: float,
    wing_area_mm2: float,
    frequency_hz: float,
    body_mass_mg: float,
    rho_air: float,
    gravity: float,
    motion_source: str,
    smoothed_n_harmonics: int,
) -> tuple[Path, Path]:
    if motion_source not in MOTION_SOURCES:
        raise ValueError(f"motion_source must be one of {MOTION_SOURCES}, got {motion_source!r}")
    if smoothed_n_harmonics <= 0:
        raise ValueError("smoothed_n_harmonics must be >= 1")

    mapping_key = "simulator_mapping" if motion_source == MOTION_SOURCE_FIT else "simulator_mapping_smoothed"

    def regenerate_fit_payload() -> Path:
        return stage_fit(
            run_dir=run_dir,
            make_plots=fit_plots,
            body_length_mm=body_length_mm,
            wing_length_mm=wing_length_mm,
            wing_area_mm2=wing_area_mm2,
            frequency_hz=frequency_hz,
            body_mass_mg=body_mass_mg,
            rho_air=rho_air,
            gravity=gravity,
            smoothed_n_harmonics=smoothed_n_harmonics,
        )

    fit_params_path = run_dir / "fit" / "fit_params.json"
    if auto_fit and not fit_params_path.exists():
        fit_params_path = regenerate_fit_payload()
    if not fit_params_path.exists():
        raise FileNotFoundError(f"Missing fit parameters: {fit_params_path}")

    payload = read_json(fit_params_path)

    # Existing runs may have fit_params.json generated before this option existed
    # or with a different smoothed harmonic count. Regenerate when needed.
    if motion_source == MOTION_SOURCE_SMOOTHED:
        existing_harmonics = int(
            payload.get(mapping_key, {}).get("parameters", {}).get("n_harmonics", -1)
        )
        if auto_fit and existing_harmonics != smoothed_n_harmonics:
            fit_params_path = regenerate_fit_payload()
            payload = read_json(fit_params_path)

    if mapping_key not in payload and auto_fit:
        fit_params_path = regenerate_fit_payload()
        payload = read_json(fit_params_path)
    if mapping_key not in payload:
        raise KeyError(f"Missing mapping '{mapping_key}' in {fit_params_path}")

    mapping = payload[mapping_key]
    if motion_source == MOTION_SOURCE_FIT:
        def has_per_wing_fit_fields(value: dict[str, Any]) -> bool:
            return (
                isinstance(value.get("parameters"), dict)
                and "n_harmonics" in value["parameters"]
                and isinstance(value.get("wing_motion_overrides"), dict)
                and "fore" in value["wing_motion_overrides"]
                and "hind" in value["wing_motion_overrides"]
            )

        fit_mapping_ready = has_per_wing_fit_fields(mapping)
        if auto_fit and not fit_mapping_ready:
            fit_params_path = regenerate_fit_payload()
            payload = read_json(fit_params_path)
            mapping = payload[mapping_key]
            fit_mapping_ready = has_per_wing_fit_fields(mapping)
        if not fit_mapping_ready:
            raise KeyError(
                f"Mapping '{mapping_key}' in {fit_params_path} is missing per-wing fit fields; rerun fit stage."
            )

    derived_lb0 = float(mapping["wing_parameters"]["lb0_lambda"])
    derived_mu0 = float(mapping["wing_parameters"]["mu0_mu"])
    final_lb0 = derived_lb0 if wing_lb0 is None else wing_lb0
    final_mu0 = derived_mu0 if wing_mu0 is None else wing_mu0

    sim_dir = ensure_dir(run_dir / "sim")
    cfg_path = sim_dir / "sim_wang2007.cfg"
    output_h5 = sim_dir / output_name
    motion_source_comment = (
        "separate sinusoid fit per wing"
        if motion_source == MOTION_SOURCE_FIT
        else "Fourier-smoothed experimental data"
    )
    cfg_text = build_sim_cfg(
        mapping,
        n_wingbeats,
        steps_per_wingbeat,
        tether,
        output_name,
        final_mu0,
        final_lb0,
        wing_cd0,
        wing_cl0,
        motion_source_comment,
    )
    cfg_path.write_text(cfg_text, encoding="utf-8")

    summary_path = sim_dir / "translate_summary.json"
    write_json(
        summary_path,
        {
            "fit_params_path": str(fit_params_path.resolve()),
            "sim_config_path": str(cfg_path.resolve()),
            "sim_output_path": str(output_h5.resolve()),
            "generated_at_utc": now_utc_iso(),
            "n_wingbeats": n_wingbeats,
            "steps_per_wingbeat": steps_per_wingbeat,
            "tether": tether,
            "omega": float(mapping["parameters"]["omega"]),
            "wing_motion_source": motion_source,
            "motion_n_harmonics": int(mapping["parameters"].get("n_harmonics", 1)),
            "lb0_lambda": final_lb0,
            "mu0_mu": final_mu0,
            "lb0_derived_from_physical": derived_lb0,
            "mu0_derived_from_physical": derived_mu0,
            "mu0_lb0_overridden": (wing_mu0 is not None or wing_lb0 is not None),
        },
    )

    update_manifest(
        run_dir,
        "translate",
        [cfg_path, summary_path],
        {
            "sim_config_path": str(cfg_path.resolve()),
            "sim_output_path": str(output_h5.resolve()),
            "wing_motion_source": motion_source,
        },
    )
    return cfg_path, output_h5


def stage_sim(
    run_dir: Path,
    binary: str,
    n_wingbeats: int,
    steps_per_wingbeat: int,
    tether: bool,
    output_name: str,
    wing_mu0: float | None,
    wing_lb0: float | None,
    wing_cd0: float,
    wing_cl0: float,
    auto_fit: bool,
    fit_plots: bool,
    body_length_mm: float,
    wing_length_mm: float,
    wing_area_mm2: float,
    frequency_hz: float,
    body_mass_mg: float,
    rho_air: float,
    gravity: float,
    motion_source: str,
    smoothed_n_harmonics: int,
) -> Path:
    cfg_path = run_dir / "sim" / "sim_wang2007.cfg"
    output_h5 = run_dir / "sim" / output_name
    if not cfg_path.exists():
        cfg_path, output_h5 = stage_translate(
            run_dir=run_dir,
            n_wingbeats=n_wingbeats,
            steps_per_wingbeat=steps_per_wingbeat,
            tether=tether,
            output_name=output_name,
            wing_mu0=wing_mu0,
            wing_lb0=wing_lb0,
            wing_cd0=wing_cd0,
            wing_cl0=wing_cl0,
            auto_fit=auto_fit,
            fit_plots=fit_plots,
            body_length_mm=body_length_mm,
            wing_length_mm=wing_length_mm,
            wing_area_mm2=wing_area_mm2,
            frequency_hz=frequency_hz,
            body_mass_mg=body_mass_mg,
            rho_air=rho_air,
            gravity=gravity,
            motion_source=motion_source,
            smoothed_n_harmonics=smoothed_n_harmonics,
        )

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
    )
    return output_h5


def stage_post(
    run_dir: Path,
    input_h5: str | None,
    no_blender: bool,
    frame_step: int,
    binary: str,
    n_wingbeats: int,
    steps_per_wingbeat: int,
    tether: bool,
    output_name: str,
    wing_mu0: float | None,
    wing_lb0: float | None,
    wing_cd0: float,
    wing_cl0: float,
    auto_fit: bool,
    fit_plots: bool,
    skip_stick: bool,
    body_length_mm: float,
    wing_length_mm: float,
    wing_area_mm2: float,
    frequency_hz: float,
    body_mass_mg: float,
    rho_air: float,
    gravity: float,
    motion_source: str,
    smoothed_n_harmonics: int,
) -> list[Path]:
    if frame_step < 1:
        raise ValueError(f"frame_step must be >= 1, got {frame_step}")

    plot_env = build_plot_env(run_dir)
    if input_h5 is not None:
        h5_path = Path(input_h5).expanduser()
        if not h5_path.is_absolute():
            h5_path = REPO_ROOT / h5_path
    else:
        h5_path = run_dir / "sim" / output_name
        if not h5_path.exists():
            h5_path = stage_sim(
                run_dir=run_dir,
                binary=binary,
                n_wingbeats=n_wingbeats,
                steps_per_wingbeat=steps_per_wingbeat,
                tether=tether,
                output_name=output_name,
                wing_mu0=wing_mu0,
                wing_lb0=wing_lb0,
                wing_cd0=wing_cd0,
                wing_cl0=wing_cl0,
                auto_fit=auto_fit,
                fit_plots=fit_plots,
                body_length_mm=body_length_mm,
                wing_length_mm=wing_length_mm,
                wing_area_mm2=wing_area_mm2,
                frequency_hz=frequency_hz,
                body_mass_mg=body_mass_mg,
                rho_air=rho_air,
                gravity=gravity,
                motion_source=motion_source,
                smoothed_n_harmonics=smoothed_n_harmonics,
            )

    if not h5_path.exists():
        raise FileNotFoundError(f"Input HDF5 not found: {h5_path}")

    post_dir = ensure_dir(run_dir / "post")
    sim_mp4 = post_dir / "simulation.mp4"
    sim_cmd = [sys.executable, "-m", "post.plot_simulation", str(h5_path), str(sim_mp4)]
    if no_blender:
        sim_cmd.append("--no-blender")
    if frame_step > 1:
        sim_cmd.extend(["--frame-step", str(frame_step)])
    run_cmd(sim_cmd, cwd=REPO_ROOT, env=plot_env)

    artifacts = [sim_mp4]
    if not skip_stick:
        stick_mp4 = post_dir / "stroke_fore_left.mp4"
        run_cmd(
            [
                sys.executable,
                "-m",
                "post.plot_stick",
                str(h5_path),
                "fore_left",
                str(stick_mp4),
            ],
            cwd=REPO_ROOT,
            env=plot_env,
        )
        artifacts.append(stick_mp4)

    update_manifest(
        run_dir,
        "post",
        artifacts,
        {
            "input_h5": str(h5_path.resolve()),
            "no_blender": no_blender,
            "skip_stick": skip_stick,
            "frame_step": frame_step,
        },
    )
    return artifacts


def add_shared_args(p: argparse.ArgumentParser) -> None:
    p.add_argument(
        "--run-dir",
        default=None,
        help="Run directory (default: runs/wang2007/<timestamp>). Relative paths are repo-relative.",
    )
    p.add_argument(
        "--n-wingbeats",
        type=int,
        default=5,
        help="Number of wingbeats for generated simulation config.",
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
        "--wing-mu0",
        type=float,
        default=None,
        help="Override wing mu0. Default is derived from physical values.",
    )
    p.add_argument(
        "--wing-lb0",
        type=float,
        default=None,
        help="Override wing lb0. Default is lambda = L_wing/L from physical values.",
    )
    p.add_argument(
        "--wing-cd0",
        type=float,
        default=0.4,
        help="Wing Cd0 used in generated config.",
    )
    p.add_argument(
        "--wing-cl0",
        type=float,
        default=1.2,
        help="Wing Cl0 used in generated config.",
    )
    p.add_argument(
        "--binary",
        default=str(DEFAULT_BINARY),
        help="Path to dragonfly binary.",
    )
    p.add_argument(
        "--skip-fit-plots",
        action="store_true",
        help="Skip diagnostic plot generation in the fit stage.",
    )
    p.add_argument(
        "--wing-motion-source",
        choices=MOTION_SOURCES,
        default=MOTION_SOURCE_FIT,
        help="Wing motion input for generated config: 'fit' (sinusoid fit) or 'smoothed' (Fourier-smoothed experimental data).",
    )
    p.add_argument(
        "--smoothed-n-harmonics",
        type=int,
        default=DEFAULT_SMOOTHED_N_HARMONICS,
        help="Number of harmonics used when --wing-motion-source=smoothed.",
    )
    p.add_argument(
        "--body-length-mm",
        type=float,
        default=DEFAULT_BODY_LENGTH_MM,
        help="Body length L in mm (default: 50).",
    )
    p.add_argument(
        "--wing-length-mm",
        type=float,
        default=DEFAULT_WING_LENGTH_MM,
        help="Wing length L_wing in mm for lambda = L_wing/L.",
    )
    p.add_argument(
        "--wing-area-mm2",
        type=float,
        default=DEFAULT_WING_AREA_MM2,
        help="Wing surface area in mm^2 for mu = rho*S*L_wing/m.",
    )
    p.add_argument(
        "--frequency-hz",
        type=float,
        default=DEFAULT_FREQUENCY_HZ,
        help="Wingbeat frequency in Hz for omega = 2*pi*f*sqrt(L/g).",
    )
    p.add_argument(
        "--body-mass-mg",
        type=float,
        default=DEFAULT_BODY_MASS_MG,
        help="Body mass in mg for mu = rho*S*L_wing/m.",
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
        help="Gravity in m/s^2 for time scale sqrt(L/g).",
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="stage", required=True)

    fit_parser = subparsers.add_parser("fit", help="Run fit stage only.")
    fit_parser.add_argument("--run-dir", default=None)
    fit_parser.add_argument("--skip-fit-plots", action="store_true")
    fit_parser.add_argument(
        "--smoothed-n-harmonics",
        type=int,
        default=DEFAULT_SMOOTHED_N_HARMONICS,
        help="Number of harmonics stored for smoothed experimental wing-motion mapping.",
    )
    fit_parser.add_argument(
        "--body-length-mm",
        type=float,
        default=DEFAULT_BODY_LENGTH_MM,
        help="Body length L in mm (default: 50).",
    )
    fit_parser.add_argument(
        "--wing-length-mm",
        type=float,
        default=DEFAULT_WING_LENGTH_MM,
        help="Wing length L_wing in mm for lambda = L_wing/L.",
    )
    fit_parser.add_argument(
        "--wing-area-mm2",
        type=float,
        default=DEFAULT_WING_AREA_MM2,
        help="Wing surface area in mm^2 for mu = rho*S*L_wing/m.",
    )
    fit_parser.add_argument(
        "--frequency-hz",
        type=float,
        default=DEFAULT_FREQUENCY_HZ,
        help="Wingbeat frequency in Hz for omega = 2*pi*f*sqrt(L/g).",
    )
    fit_parser.add_argument(
        "--body-mass-mg",
        type=float,
        default=DEFAULT_BODY_MASS_MG,
        help="Body mass in mg for mu = rho*S*L_wing/m.",
    )
    fit_parser.add_argument(
        "--rho-air",
        type=float,
        default=DEFAULT_RHO_AIR,
        help="Air density in kg/m^3 for mu = rho*S*L_wing/m.",
    )
    fit_parser.add_argument(
        "--gravity",
        type=float,
        default=DEFAULT_GRAVITY,
        help="Gravity in m/s^2 for time scale sqrt(L/g).",
    )

    translate_parser = subparsers.add_parser("translate", help="Generate sim config from fit outputs.")
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
    post_parser.add_argument("--skip-stick", action="store_true", help="Skip stroke sphere animation.")
    post_parser.add_argument(
        "--frame-step",
        type=int,
        default=1,
        help="Render every Nth frame in 3D animation (default: 1).",
    )

    all_parser = subparsers.add_parser("all", help="Run fit -> translate -> sim -> post.")
    add_shared_args(all_parser)
    all_parser.add_argument("--no-blender", action="store_true", help="Force matplotlib-only rendering.")
    all_parser.add_argument("--skip-stick", action="store_true", help="Skip stroke sphere animation.")
    all_parser.add_argument(
        "--frame-step",
        type=int,
        default=1,
        help="Render every Nth frame in 3D animation (default: 1).",
    )

    args = parser.parse_args()
    run_dir = ensure_dir(resolve_run_dir(args.run_dir))
    make_fit_plots = not args.skip_fit_plots

    if args.stage == "fit":
        fit_params = stage_fit(
            run_dir=run_dir,
            make_plots=make_fit_plots,
            body_length_mm=args.body_length_mm,
            wing_length_mm=args.wing_length_mm,
            wing_area_mm2=args.wing_area_mm2,
            frequency_hz=args.frequency_hz,
            body_mass_mg=args.body_mass_mg,
            rho_air=args.rho_air,
            gravity=args.gravity,
            smoothed_n_harmonics=args.smoothed_n_harmonics,
        )
        print(f"[done] fit_params: {fit_params}")
        print(f"[done] run_dir: {run_dir}")
        return 0

    if args.stage == "translate":
        cfg_path, output_h5 = stage_translate(
            run_dir=run_dir,
            n_wingbeats=args.n_wingbeats,
            steps_per_wingbeat=args.steps_per_wingbeat,
            tether=args.tether,
            output_name=args.output_name,
            wing_mu0=args.wing_mu0,
            wing_lb0=args.wing_lb0,
            wing_cd0=args.wing_cd0,
            wing_cl0=args.wing_cl0,
            auto_fit=True,
            fit_plots=make_fit_plots,
            body_length_mm=args.body_length_mm,
            wing_length_mm=args.wing_length_mm,
            wing_area_mm2=args.wing_area_mm2,
            frequency_hz=args.frequency_hz,
            body_mass_mg=args.body_mass_mg,
            rho_air=args.rho_air,
            gravity=args.gravity,
            motion_source=args.wing_motion_source,
            smoothed_n_harmonics=args.smoothed_n_harmonics,
        )
        print(f"[done] cfg: {cfg_path}")
        print(f"[done] sim_output: {output_h5}")
        print(f"[done] run_dir: {run_dir}")
        return 0

    if args.stage == "sim":
        output_h5 = stage_sim(
            run_dir=run_dir,
            binary=args.binary,
            n_wingbeats=args.n_wingbeats,
            steps_per_wingbeat=args.steps_per_wingbeat,
            tether=args.tether,
            output_name=args.output_name,
            wing_mu0=args.wing_mu0,
            wing_lb0=args.wing_lb0,
            wing_cd0=args.wing_cd0,
            wing_cl0=args.wing_cl0,
            auto_fit=True,
            fit_plots=make_fit_plots,
            body_length_mm=args.body_length_mm,
            wing_length_mm=args.wing_length_mm,
            wing_area_mm2=args.wing_area_mm2,
            frequency_hz=args.frequency_hz,
            body_mass_mg=args.body_mass_mg,
            rho_air=args.rho_air,
            gravity=args.gravity,
            motion_source=args.wing_motion_source,
            smoothed_n_harmonics=args.smoothed_n_harmonics,
        )
        print(f"[done] sim_output: {output_h5}")
        print(f"[done] run_dir: {run_dir}")
        return 0

    if args.stage == "post":
        artifacts = stage_post(
            run_dir=run_dir,
            input_h5=args.input_h5,
            no_blender=args.no_blender,
            frame_step=args.frame_step,
            binary=args.binary,
            n_wingbeats=args.n_wingbeats,
            steps_per_wingbeat=args.steps_per_wingbeat,
            tether=args.tether,
            output_name=args.output_name,
            wing_mu0=args.wing_mu0,
            wing_lb0=args.wing_lb0,
            wing_cd0=args.wing_cd0,
            wing_cl0=args.wing_cl0,
            auto_fit=True,
            fit_plots=make_fit_plots,
            skip_stick=True,
            body_length_mm=args.body_length_mm,
            wing_length_mm=args.wing_length_mm,
            wing_area_mm2=args.wing_area_mm2,
            frequency_hz=args.frequency_hz,
            body_mass_mg=args.body_mass_mg,
            rho_air=args.rho_air,
            gravity=args.gravity,
            motion_source=args.wing_motion_source,
            smoothed_n_harmonics=args.smoothed_n_harmonics,
        )
        print("[done] post artifacts:")
        for p in artifacts:
            print(f"  - {p}")
        print(f"[done] run_dir: {run_dir}")
        return 0

    if args.stage == "all":
        stage_fit(
            run_dir=run_dir,
            make_plots=make_fit_plots,
            body_length_mm=args.body_length_mm,
            wing_length_mm=args.wing_length_mm,
            wing_area_mm2=args.wing_area_mm2,
            frequency_hz=args.frequency_hz,
            body_mass_mg=args.body_mass_mg,
            rho_air=args.rho_air,
            gravity=args.gravity,
            smoothed_n_harmonics=args.smoothed_n_harmonics,
        )
        stage_translate(
            run_dir=run_dir,
            n_wingbeats=args.n_wingbeats,
            steps_per_wingbeat=args.steps_per_wingbeat,
            tether=args.tether,
            output_name=args.output_name,
            wing_mu0=args.wing_mu0,
            wing_lb0=args.wing_lb0,
            wing_cd0=args.wing_cd0,
            wing_cl0=args.wing_cl0,
            auto_fit=True,
            fit_plots=make_fit_plots,
            body_length_mm=args.body_length_mm,
            wing_length_mm=args.wing_length_mm,
            wing_area_mm2=args.wing_area_mm2,
            frequency_hz=args.frequency_hz,
            body_mass_mg=args.body_mass_mg,
            rho_air=args.rho_air,
            gravity=args.gravity,
            motion_source=args.wing_motion_source,
            smoothed_n_harmonics=args.smoothed_n_harmonics,
        )
        stage_sim(
            run_dir=run_dir,
            binary=args.binary,
            n_wingbeats=args.n_wingbeats,
            steps_per_wingbeat=args.steps_per_wingbeat,
            tether=args.tether,
            output_name=args.output_name,
            wing_mu0=args.wing_mu0,
            wing_lb0=args.wing_lb0,
            wing_cd0=args.wing_cd0,
            wing_cl0=args.wing_cl0,
            auto_fit=True,
            fit_plots=make_fit_plots,
            body_length_mm=args.body_length_mm,
            wing_length_mm=args.wing_length_mm,
            wing_area_mm2=args.wing_area_mm2,
            frequency_hz=args.frequency_hz,
            body_mass_mg=args.body_mass_mg,
            rho_air=args.rho_air,
            gravity=args.gravity,
            motion_source=args.wing_motion_source,
            smoothed_n_harmonics=args.smoothed_n_harmonics,
        )
        artifacts = stage_post(
            run_dir=run_dir,
            input_h5=None,
            no_blender=args.no_blender,
            frame_step=args.frame_step,
            binary=args.binary,
            n_wingbeats=args.n_wingbeats,
            steps_per_wingbeat=args.steps_per_wingbeat,
            tether=args.tether,
            output_name=args.output_name,
            wing_mu0=args.wing_mu0,
            wing_lb0=args.wing_lb0,
            wing_cd0=args.wing_cd0,
            wing_cl0=args.wing_cl0,
            auto_fit=True,
            fit_plots=make_fit_plots,
            skip_stick=True,
            body_length_mm=args.body_length_mm,
            wing_length_mm=args.wing_length_mm,
            wing_area_mm2=args.wing_area_mm2,
            frequency_hz=args.frequency_hz,
            body_mass_mg=args.body_mass_mg,
            rho_air=args.rho_air,
            gravity=args.gravity,
            motion_source=args.wing_motion_source,
            smoothed_n_harmonics=args.smoothed_n_harmonics,
        )
        print("[done] pipeline artifacts:")
        for p in artifacts:
            print(f"  - {p}")
        print(f"[done] run_dir: {run_dir}")
        return 0

    raise RuntimeError(f"Unhandled stage: {args.stage}")


if __name__ == "__main__":
    raise SystemExit(main())

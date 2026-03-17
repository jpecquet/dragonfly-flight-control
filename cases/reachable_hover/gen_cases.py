#!/usr/bin/env python3
"""Select equilibrium solutions at true hover (ux = uz = 0) and write case YAMLs + post.yaml.

Run once manually:
    python cases/reachable_hover/gen_cases.py

Produces:
    cases/reachable_hover/case_0_sync.yaml ... case_N_sync.yaml
    cases/reachable_hover/post.yaml
"""

from __future__ import annotations

import math
from pathlib import Path

import h5py
import numpy as np
import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
H5_PATH = REPO_ROOT / "docs/research/reachable/artifacts/reachable_hover_zero.h5"
OUT_DIR = Path(__file__).resolve().parent

# Physical morphology — identical to reachable_boundary cases.
BODY_LENGTH_M = 0.04
GRAVITY = 9.81
MU0 = 0.025
LB0 = 0.75
RHO = 1.225

WING_SPAN_M = LB0 * BODY_LENGTH_M
WING_AREA_M2 = 2.21e-4
BODY_MASS_KG = RHO * WING_AREA_M2 * WING_SPAN_M / MU0
FREQUENCY_HZ = (8 * math.pi / (2 * math.pi)) * math.sqrt(GRAVITY / BODY_LENGTH_M)

PARAM_TOL = 0.05


def _find_hover_point(
    ux: np.ndarray,
    uz: np.ndarray,
) -> tuple[int, int]:
    """Return the grid indices of the point closest to (ux=0, uz=0)."""
    ix = int(np.argmin(np.abs(ux)))
    iz = int(np.argmin(np.abs(uz)))
    return ix, iz




def _deduplicate_params(branch_params: list[np.ndarray]) -> list[np.ndarray]:
    unique: list[np.ndarray] = []
    for p in branch_params:
        if not any(np.max(np.abs(p - q)) < PARAM_TOL for q in unique):
            unique.append(p)
    return unique


def _write_case_yaml(branch_idx: int, params: np.ndarray, ux_val: float, uz_val: float) -> Path:
    """Write a sync case YAML (all wings same phase) for one hover branch."""
    gamma_mean_deg = math.degrees(params[0])
    phi_amp_deg = math.degrees(params[1])
    psi_amp_deg = math.degrees(params[2])
    psi_phase_deg = math.degrees(params[3])

    def _wing_block(phi_phase_offset: float) -> dict:
        return {
            "span": round(WING_SPAN_M, 6),
            "area": WING_AREA_M2,
            "drag_coeff_set": "wang2004",
            "lift_coeff_set": "wang2004",
            "kinematics": {
                "gamma": {"mean": round(gamma_mean_deg, 4)},
                "phi": {
                    "mean": 0.0,
                    "harmonics": [[round(phi_amp_deg, 4), round(phi_phase_offset, 4)]],
                },
                "psi": {
                    "mean": 0.0,
                    "harmonics": [
                        [round(psi_amp_deg, 4), round(psi_phase_deg + phi_phase_offset, 4)]
                    ],
                },
            },
        }

    case = {
        "case_id": f"reachable_hover_{branch_idx}_sync",
        "description": f"Reachable hover equilibrium branch {branch_idx} (sync)",
        "specimen": {
            "body_length": round(BODY_LENGTH_M, 6),
            "body_mass": round(BODY_MASS_KG, 7),
            "frequency": round(FREQUENCY_HZ, 4),
        },
        "environment": {
            "rho_air": RHO,
            "gravity": GRAVITY,
        },
        "simulation": {
            "n_wingbeats": 3,
            "steps_per_wingbeat": 200,
            "n_blade_elements": 1,
        },
        "tether": True,
        "initial_state": {
            "x": 0.0, "y": 0.0, "z": 0.0,
            "ux": round(ux_val, 6),
            "uy": 0.0,
            "uz": round(uz_val, 6),
        },
        "wings": {
            "fore": _wing_block(phi_phase_offset=0.0),
            "hind": _wing_block(phi_phase_offset=0.0),
        },
    }

    out_path = OUT_DIR / f"case_{branch_idx}_sync.yaml"
    out_path.write_text(yaml.dump(case, default_flow_style=False, sort_keys=False), encoding="utf-8")
    print(f"  Wrote {out_path.relative_to(REPO_ROOT)}")
    return out_path


def _write_post_yaml(n_branches: int) -> None:
    sync_cases = [f"cases/reachable_hover/case_{i}_sync.yaml" for i in range(n_branches)]

    artifacts = []
    for i in range(n_branches):
        artifacts.append({
            "kind": "boundary_stroke_and_forces",
            "input_h5": f"{{run_dir}}/branch_{i}/output.h5",
            "output": f"reachable_hover_{i}_panel.{{theme}}.png",
            "n_samples": 10,
            "stroke_lim": 0.5,
            "force_scale": 0.06,
        })

    config = {
        "version": 1,
        "simulation": {
            "driver": "multi_yaml_case",
            "cases": sync_cases,
            "run_dir": "docs/research/reachable/artifacts/hover_sync",
            "binary": "build/bin/dragonfly",
        },
        "docs_media": {
            "media_dir": "docs/_static/media/reachable",
            "themes": ["light", "dark"],
        },
        "artifacts": artifacts,
    }

    out_path = OUT_DIR / "post.yaml"
    out_path.write_text(yaml.dump(config, default_flow_style=False, sort_keys=False), encoding="utf-8")
    print(f"  Wrote {out_path.relative_to(REPO_ROOT)}")


def main() -> None:
    print(f"Loading {H5_PATH.relative_to(REPO_ROOT)} ...")
    with h5py.File(H5_PATH) as f:
        ux = f["grid/ux"][:]
        uz = f["grid/uz"][:]
        min_res = f["grid/min_residual"][:]
        tol = float(f["metadata/equilibrium_tol"][()])
        param_names = [s.decode() for s in f["metadata/param_names"][:]]

        ix, iz = _find_hover_point(ux, uz)
        ux_val, uz_val = float(ux[ix]), float(uz[iz])
        print(
            f"Hover grid point: ix={ix}, iz={iz}, "
            f"ux={ux_val:.4f}, uz={uz_val:.4f} (nondim), "
            f"min_res={min_res[ix,iz]:.2e}"
        )
        print(f"  params: {param_names}")

        raw_branches: list[np.ndarray] = []
        for k in range(len(f["branches"])):
            res = float(f[f"branches/{k}/residual"][ix, iz])
            if res < tol:
                p = f[f"branches/{k}/params"][ix, iz, :]
                raw_branches.append(np.array(p))
                print(f"  branch {k}: residual={res:.2e}  params={p}")

    unique_branches = _deduplicate_params(raw_branches)
    unique_branches.sort(key=lambda p: p[0])  # ascending stroke plane angle
    print(f"Distinct branches after deduplication: {len(unique_branches)}")

    for i, params in enumerate(unique_branches):
        _write_case_yaml(i, params, ux_val, uz_val)

    _write_post_yaml(len(unique_branches))

    print("\nDocs parameter lines:")
    for i, params in enumerate(unique_branches):
        gamma_deg = math.degrees(params[0])
        phi_amp_deg = math.degrees(params[1])
        psi_amp_deg = math.degrees(params[2])
        psi_phase_deg = math.degrees(params[3])
        print(
            f"  {i}: $\\gamma_0 = {gamma_deg:.1f}^\\circ$, "
            f"$\\phi_1 = {phi_amp_deg:.1f}^\\circ$, "
            f"$\\psi_1 = {psi_amp_deg:.1f}^\\circ$, "
            f"$\\delta_\\psi = {psi_phase_deg:.1f}^\\circ$"
        )

    print("\nDone. Run:")
    print("  python -m scripts.docs_media_runner cases/reachable_hover/post.yaml")


if __name__ == "__main__":
    main()

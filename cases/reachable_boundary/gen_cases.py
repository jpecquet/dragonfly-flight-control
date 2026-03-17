#!/usr/bin/env python3
"""Select four boundary grid points from the reachable set and write case YAMLs + post.yaml.

Picks four representative points along the reachable boundary (ux > 0), one per region:
  0: fast descent,  small forward component  (~-78° in velocity space)
  1: slow descent,  large forward component  (~-21°)
  2: slow climb,    large forward component  (~+21°)
  3: fast climb,    small forward component  (~+77°)

For each point, all valid optimizer branches are simulated and the one with minimum
peak Fx²+Fz² (maximum instantaneous aerodynamic load over one wingbeat) is selected.

Run once manually:
    python cases/reachable_boundary/gen_cases.py

Produces:
    cases/reachable_boundary/case_0.yaml ... case_3.yaml
    cases/reachable_boundary/case_0_sync.yaml ... case_3_sync.yaml
    cases/reachable_boundary/post.yaml
"""

from __future__ import annotations

import math
import sys
import tempfile
from pathlib import Path

import h5py
import numpy as np
import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

from scripts.case_runner import run_case  # noqa: E402

H5_PATH = REPO_ROOT / "docs/research/reachable/artifacts/reachable_4param_psiamp.h5"
OUT_DIR = Path(__file__).resolve().parent
BINARY = REPO_ROOT / "build/bin/dragonfly"

# Physical morphology consistent with the nondimensional reachable set config.
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

# Four target directions in (ux, uz) velocity space; only ux>0 boundary points considered.
TARGET_ANGLES_DEG = [
    -78.0,   # 0: fast descent,  small forward component
    -21.0,   # 1: slow descent,  large forward component
    +21.0,   # 2: slow climb,    large forward component
    +77.0,   # 3: fast climb,    small forward component
]
TARGET_LABELS = [
    "fast descent, small forward component",
    "slow descent, large forward component",
    "slow climb, large forward component",
    "fast climb, small forward component",
]


def _find_boundary_points(
    ux: np.ndarray, uz: np.ndarray, min_res: np.ndarray, tol: float
) -> list[tuple[int, int]]:
    """Return (ix, iz) indices for all ux>0 boundary points."""
    reachable = min_res < tol
    n_ux, n_uz = len(ux), len(uz)
    boundary = []
    for ix in range(n_ux):
        if ux[ix] <= 0:
            continue
        for iz in range(n_uz):
            if not reachable[ix, iz]:
                continue
            at_boundary = False
            for dix, diz in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                nix, niz = ix + dix, iz + diz
                if 0 <= nix < n_ux and 0 <= niz < n_uz:
                    if not reachable[nix, niz]:
                        at_boundary = True
                        break
                else:
                    at_boundary = True
                    break
            if at_boundary:
                boundary.append((ix, iz))
    return boundary


def _closest_to_angle(
    boundary: list[tuple[int, int]],
    ux: np.ndarray,
    uz: np.ndarray,
    target_deg: float,
) -> tuple[int, int]:
    """Return the boundary point whose velocity angle is closest to target_deg."""
    target_rad = math.radians(target_deg)
    best, best_diff = boundary[0], math.inf
    for ix, iz in boundary:
        angle = math.atan2(float(uz[iz]), float(ux[ix]))
        diff = abs(math.atan2(math.sin(angle - target_rad), math.cos(angle - target_rad)))
        if diff < best_diff:
            best_diff = diff
            best = (ix, iz)
    return best


def _deduplicate_params(branch_params: list[np.ndarray]) -> list[np.ndarray]:
    unique: list[np.ndarray] = []
    for p in branch_params:
        if not any(np.max(np.abs(p - q)) < PARAM_TOL for q in unique):
            unique.append(p)
    return unique


def _build_case_dict(
    case_id: str, params: np.ndarray, ux_val: float, uz_val: float, *, sync: bool = False
) -> dict:
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

    return {
        "case_id": case_id,
        "description": case_id,
        "specimen": {
            "body_length": round(BODY_LENGTH_M, 6),
            "body_mass": round(BODY_MASS_KG, 7),
            "frequency": round(FREQUENCY_HZ, 4),
        },
        "environment": {"rho_air": RHO, "gravity": GRAVITY},
        "simulation": {
            "n_wingbeats": 3,
            "steps_per_wingbeat": 200,
            "n_blade_elements": 1,
        },
        "tether": True,
        "initial_state": {
            "x": 0.0, "y": 0.0, "z": 0.0,
            "ux": round(ux_val, 6), "uy": 0.0, "uz": round(uz_val, 6),
        },
        "wings": {
            "fore": _wing_block(phi_phase_offset=0.0),
            "hind": _wing_block(phi_phase_offset=0.0 if sync else 180.0),
        },
    }


def _compute_peak_fx2_fz2(h5_path: Path) -> float:
    """Return peak (Fx²+Fz²) over the last wingbeat from a simulation output H5."""
    with h5py.File(str(h5_path)) as f:
        time = f["/time"][:]
        omega = float(f["/parameters/omega"][()])
        T_wb = 2.0 * math.pi / omega
        mask = time >= time[-1] - T_wb
        wing_names = sorted(k for k in f["/wings"].keys() if k != "num_wings")
        n = int(mask.sum())
        total_fx = np.zeros(n)
        total_fz = np.zeros(n)
        for wname in wing_names:
            lift = f[f"/wings/{wname}/lift"][mask]
            drag = f[f"/wings/{wname}/drag"][mask]
            total_fx += lift[:, 0] + drag[:, 0]
            total_fz += lift[:, 2] + drag[:, 2]
    return float(np.max(total_fx**2 + total_fz**2))


def _select_best_branch(
    branches: list[np.ndarray], ux_val: float, uz_val: float
) -> tuple[np.ndarray, float]:
    """Run each branch in a temp dir; return the params with minimum peak Fx²+Fz²."""
    best_peak = math.inf
    best_params = branches[0]
    with tempfile.TemporaryDirectory() as tmpdir:
        tmp = Path(tmpdir)
        for k, params in enumerate(branches):
            case_id = f"tmp_branch_{k}"
            case = _build_case_dict(case_id, params, ux_val, uz_val, sync=True)
            yaml_path = tmp / f"{case_id}.yaml"
            yaml_path.write_text(
                yaml.dump(case, default_flow_style=False, sort_keys=False), encoding="utf-8"
            )
            run_dir = tmp / f"run_{k}"
            run_dir.mkdir()
            try:
                h5_path = run_case(yaml_path, run_dir=run_dir, binary=BINARY)
                peak = _compute_peak_fx2_fz2(h5_path)
                print(f"    branch {k}: peak Fx²+Fz² = {peak:.4f}")
                if peak < best_peak:
                    best_peak = peak
                    best_params = params
            except Exception as e:
                print(f"    branch {k}: FAILED ({e})")
    return best_params, best_peak


def _write_case_yaml(
    idx: int, params: np.ndarray, ux_val: float, uz_val: float, *, sync: bool = False
) -> Path:
    suffix = "_sync" if sync else ""
    case_id = f"reachable_boundary_{idx}{suffix}"
    case = _build_case_dict(case_id, params, ux_val, uz_val, sync=sync)
    case["description"] = (
        f"Reachable boundary: {TARGET_LABELS[idx]}{' (sync)' if sync else ''}"
    )
    out_path = OUT_DIR / f"case_{idx}{suffix}.yaml"
    out_path.write_text(
        yaml.dump(case, default_flow_style=False, sort_keys=False), encoding="utf-8"
    )
    print(f"  Wrote {out_path.relative_to(REPO_ROOT)}")
    return out_path


def _write_post_yaml(n_cases: int) -> None:
    sync_cases = [f"cases/reachable_boundary/case_{i}_sync.yaml" for i in range(n_cases)]
    artifacts = [
        {
            "kind": "boundary_stroke_and_forces",
            "input_h5": f"{{run_dir}}/branch_{i}/output.h5",
            "output": f"reachable_boundary_{i}_panel.{{theme}}.png",
        }
        for i in range(n_cases)
    ]
    config = {
        "version": 1,
        "simulation": {
            "driver": "multi_yaml_case",
            "cases": sync_cases,
            "run_dir": "docs/research/reachable/artifacts/boundary_sync",
            "binary": "build/bin/dragonfly",
        },
        "docs_media": {
            "media_dir": "docs/_static/media/reachable",
            "themes": ["light", "dark"],
        },
        "artifacts": artifacts,
    }
    out_path = OUT_DIR / "post.yaml"
    out_path.write_text(
        yaml.dump(config, default_flow_style=False, sort_keys=False), encoding="utf-8"
    )
    print(f"  Wrote {out_path.relative_to(REPO_ROOT)}")


def main() -> None:
    print(f"Loading {H5_PATH.relative_to(REPO_ROOT)} ...")
    with h5py.File(H5_PATH) as f:
        ux = f["grid/ux"][:]
        uz = f["grid/uz"][:]
        min_res = f["grid/min_residual"][:]
        tol = float(f["metadata/equilibrium_tol"][()])

        boundary = _find_boundary_points(ux, uz, min_res, tol)

        all_point_data: list[tuple[float, float, list[np.ndarray], str]] = []
        for target_angle, label in zip(TARGET_ANGLES_DEG, TARGET_LABELS):
            ix, iz = _closest_to_angle(boundary, ux, uz, target_angle)
            ux_val, uz_val = float(ux[ix]), float(uz[iz])
            angle_actual = math.degrees(math.atan2(uz_val, ux_val))
            print(
                f"\nTarget '{label}' ({target_angle:+.0f}°): "
                f"grid point ux={ux_val:.3f}, uz={uz_val:.3f} ({angle_actual:.1f}°)"
            )

            raw: list[np.ndarray] = []
            for k in range(len(f["branches"])):
                res = float(f[f"branches/{k}/residual"][ix, iz])
                if res < tol:
                    raw.append(np.array(f[f"branches/{k}/params"][ix, iz, :]))

            unique = _deduplicate_params(raw)
            print(f"  {len(unique)} unique branches")
            all_point_data.append((ux_val, uz_val, unique, label))

    results: list[tuple[float, float, np.ndarray, float]] = []
    for i, (ux_val, uz_val, branches, label) in enumerate(all_point_data):
        print(f"\nSelecting best branch for point {i} ({label}) ...")
        best_params, best_peak = _select_best_branch(branches, ux_val, uz_val)
        results.append((ux_val, uz_val, best_params, best_peak))

    print("\nWriting case YAMLs ...")
    for i, (ux_val, uz_val, params, _) in enumerate(results):
        _write_case_yaml(i, params, ux_val, uz_val)
        _write_case_yaml(i, params, ux_val, uz_val, sync=True)

    _write_post_yaml(len(results))

    print("\nDocs parameter lines:")
    for i, (ux_val, uz_val, params, peak) in enumerate(results):
        gamma_deg = math.degrees(params[0])
        phi_amp_deg = math.degrees(params[1])
        psi_amp_deg = math.degrees(params[2])
        psi_phase_deg = math.degrees(params[3])
        print(f"\n  {i}: {TARGET_LABELS[i]}")
        print(f"     ux={ux_val:.3f}, uz={uz_val:.3f} (nondim)")
        print(
            f"     $\\gamma_0 = {gamma_deg:.1f}^\\circ$, "
            f"$\\phi_1 = {phi_amp_deg:.1f}^\\circ$, "
            f"$\\psi_1 = {psi_amp_deg:.1f}^\\circ$, "
            f"$\\delta_\\psi = {psi_phase_deg:.1f}^\\circ$"
        )

    print("\nDone. Review YAMLs, then run:")
    print("  python -m scripts.docs_media_runner cases/reachable_boundary/post.yaml")


if __name__ == "__main__":
    main()

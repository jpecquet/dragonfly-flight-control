#!/usr/bin/env python3
"""
Plot Azuma 1988 paper-convention kinematic angles (psi, theta) over one wingbeat.

Usage:
    python -m cases.azuma1988.plot_kinematics <out.png> [--experiment 1] [--theme light|dark]
"""

from __future__ import annotations

import argparse
import importlib
import sys
from pathlib import Path

import numpy as np

from post.time_series import plot_fore_hind_series

REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPTS_DIR = REPO_ROOT / "scripts"
DEFAULT_N_POINTS = 500
PLOT_HEIGHT_OVER_WIDTH = 0.45


def load_adapter(experiment: str | int | None = None):
    if str(SCRIPTS_DIR) not in sys.path:
        sys.path.insert(0, str(SCRIPTS_DIR))
    mod = importlib.import_module("experimental_conventions")
    return mod.azuma1988_adapter(experiment=experiment)


def compute_paper_kinematics(
    n_points: int = DEFAULT_N_POINTS,
    experiment: str | int | None = None,
) -> dict[str, np.ndarray]:
    adapter = load_adapter(experiment=experiment)
    t = np.linspace(0.0, 1.0, n_points)
    return {
        "t": t,
        "psi_fore_deg": adapter.source_series["fore"].phi.eval_deg(t),
        "psi_hind_deg": adapter.source_series["hind"].phi.eval_deg(t),
        "theta_fore_deg": adapter.source_series["fore"].psi.eval_deg(t),
        "theta_hind_deg": adapter.source_series["hind"].psi.eval_deg(t),
    }


def plot_paper_kinematics(
    output_path: Path,
    *,
    n_points: int = DEFAULT_N_POINTS,
    experiment: str | int | None = None,
    theme: str | None = None,
) -> None:
    series = compute_paper_kinematics(n_points=n_points, experiment=experiment)
    plot_fore_hind_series(
        output_path, series,
        rows=[
            ("psi_fore_deg", "psi_hind_deg", r"$\psi$ (deg)"),
            ("theta_fore_deg", "theta_hind_deg", r"$\theta$ (deg)"),
        ],
        height_over_width=PLOT_HEIGHT_OVER_WIDTH, theme=theme,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", help="Output image path (e.g. kinematics_inputs.png)")
    parser.add_argument("--n-points", type=int, default=DEFAULT_N_POINTS)
    parser.add_argument("--experiment", default=None, help="Azuma 1988 experiment id (e.g. 1-4).")
    parser.add_argument("--theme", choices=["light", "dark"], default="light")
    args = parser.parse_args()

    output = Path(args.output)
    print(f"Writing Azuma 1988 paper-convention kinematics plot: {output} (experiment={args.experiment})")
    plot_paper_kinematics(output, n_points=args.n_points, experiment=args.experiment, theme=args.theme)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

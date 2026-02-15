#!/usr/bin/env python3
"""
Plot Azuma 1985 paper-convention kinematic angles (psi, theta) over one wingbeat.

Usage:
    python -m post.plot_azuma1985_kinematics <out.png> [--theme light|dark]
"""

from __future__ import annotations

import argparse
import importlib
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from post.style import apply_matplotlib_style, figure_size, resolve_style

REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = REPO_ROOT / "scripts"
DEFAULT_N_POINTS = 500
PLOT_HEIGHT_OVER_WIDTH = 0.45


def load_adapter():
    if str(SCRIPTS_DIR) not in sys.path:
        sys.path.insert(0, str(SCRIPTS_DIR))
    mod = importlib.import_module("experimental_conventions")
    return mod.azuma1985_adapter()


def compute_paper_kinematics(n_points: int = DEFAULT_N_POINTS) -> dict[str, np.ndarray]:
    adapter = load_adapter()
    t = np.linspace(0.0, 1.0, n_points)
    # adapter.source_series[wing].phi stores flapping angle (paper calls it psi)
    # adapter.source_series[wing].psi stores pitch angle (paper calls it theta)
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
    theme: str | None = None,
) -> None:
    series = compute_paper_kinematics(n_points=n_points)

    style = resolve_style(theme=theme)
    apply_matplotlib_style(style)

    fig, axes = plt.subplots(
        nrows=2,
        ncols=1,
        sharex=True,
        figsize=figure_size(height_over_width=PLOT_HEIGHT_OVER_WIDTH),
    )
    t = series["t"]

    rows = [
        ("psi_fore_deg", "psi_hind_deg", r"$\psi$ (deg)"),
        ("theta_fore_deg", "theta_hind_deg", r"$\theta$ (deg)"),
    ]

    for ax, (fore_key, hind_key, ylabel) in zip(axes, rows):
        ax.plot(t, series[fore_key], linewidth=1.5, label="Forewing", color="C0")
        ax.plot(t, series[hind_key], linewidth=1.5, label="Hindwing", color="C1")
        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.25)

    axes[0].legend(
        loc="lower center",
        bbox_to_anchor=(0.5, 1.03),
        ncol=2,
        fontsize=10.0,
    )
    axes[-1].set_xlabel(r"$t/T_{wb}$")
    axes[-1].set_xlim(float(t[0]), float(t[-1]))
    fig.tight_layout()

    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(str(output_path), dpi=300, bbox_inches="tight")
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", help="Output image path (e.g. kinematics_inputs.png)")
    parser.add_argument(
        "--n-points",
        type=int,
        default=DEFAULT_N_POINTS,
        help="Samples per trace (default: 500).",
    )
    parser.add_argument(
        "--theme",
        choices=["light", "dark"],
        default="light",
        help="Plot theme (default: light).",
    )
    args = parser.parse_args()

    output = Path(args.output)
    print(f"Writing Azuma 1985 paper-convention kinematics plot: {output}")
    plot_paper_kinematics(output, n_points=args.n_points, theme=args.theme)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

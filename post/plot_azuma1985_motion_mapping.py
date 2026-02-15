#!/usr/bin/env python3
"""
Plot Azuma 1985 simulator-convention angles (phi, psi) after convention mapping.

Usage:
    python -m post.plot_azuma1985_motion_mapping <out.png> [--theme light|dark]
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from post.plot_azuma1985_kinematics import DEFAULT_N_POINTS, load_adapter
from post.style import apply_matplotlib_style, figure_size, resolve_style

PLOT_HEIGHT_OVER_WIDTH = 0.45


def compute_motion_mapping_series(
    n_points: int = DEFAULT_N_POINTS,
) -> dict[str, np.ndarray]:
    adapter = load_adapter()
    sim = adapter.sim_series()
    t = np.linspace(0.0, 1.0, n_points)
    return {
        "t": t,
        "phi_fore_deg": sim["fore"].phi.eval_deg(t),
        "phi_hind_deg": sim["hind"].phi.eval_deg(t),
        "psi_fore_deg": sim["fore"].psi.eval_deg(t),
        "psi_hind_deg": sim["hind"].psi.eval_deg(t),
    }


def plot_motion_mapping(
    output_path: Path,
    *,
    n_points: int = DEFAULT_N_POINTS,
    theme: str | None = None,
) -> None:
    series = compute_motion_mapping_series(n_points=n_points)

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
        ("phi_fore_deg", "phi_hind_deg", r"$\phi$ (deg)"),
        ("psi_fore_deg", "psi_hind_deg", r"$\psi$ (deg)"),
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
    parser.add_argument("output", help="Output image path (e.g. motion_mapping.png)")
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
    print(f"Writing Azuma 1985 motion mapping plot: {output}")
    plot_motion_mapping(output, n_points=args.n_points, theme=args.theme)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

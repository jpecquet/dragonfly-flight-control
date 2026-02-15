#!/usr/bin/env python3
"""
Plot Fourier-smoothed Wang 2007 kinematic inputs (s, d, beta).

Usage:
    python -m post.plot_wang2007_kinematics <out.png> [--theme light|dark]
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
WANG_DIR = REPO_ROOT / "data" / "kinematics" / "wang2007"
DEFAULT_N_COMPONENTS = 35
DEFAULT_N_WINGBEATS = 5
DEFAULT_N_POINTS = 2000
PLOT_HEIGHT_OVER_WIDTH = 0.6


def load_common_module():
    if str(WANG_DIR) not in sys.path:
        sys.path.insert(0, str(WANG_DIR))
    return importlib.import_module("common")


def harmonics_per_wingbeat(n_components: int, n_wingbeats: int) -> int:
    if n_components <= 0:
        raise ValueError("n_components must be > 0")
    if n_wingbeats <= 0:
        raise ValueError("n_wingbeats must be > 0")
    if n_components % n_wingbeats != 0:
        raise ValueError(
            "n_components must be divisible by n_wingbeats "
            f"(got {n_components} and {n_wingbeats})"
        )
    return n_components // n_wingbeats


def _smooth_pair(common, prefix: str, n_harmonics_per_wingbeat: int, n_points: int) -> dict[str, np.ndarray]:
    t_fore, y_fore = common.fourier_smooth(
        *common.sorted_xy(f"{prefix}_fore"),
        n_harmonics=n_harmonics_per_wingbeat,
        n_points=n_points,
    )
    t_hind, y_hind = common.fourier_smooth(
        *common.sorted_xy(f"{prefix}_hind"),
        n_harmonics=n_harmonics_per_wingbeat,
        n_points=n_points,
    )
    return {
        "t_fore": np.asarray(t_fore),
        "y_fore": np.asarray(y_fore),
        "t_hind": np.asarray(t_hind),
        "y_hind": np.asarray(y_hind),
    }


def compute_smoothed_kinematics(
    *,
    n_components: int = DEFAULT_N_COMPONENTS,
    n_wingbeats: int = DEFAULT_N_WINGBEATS,
    n_points: int = DEFAULT_N_POINTS,
) -> dict[str, dict[str, np.ndarray]]:
    if n_points <= 0:
        raise ValueError("n_points must be > 0")
    n_harm = harmonics_per_wingbeat(n_components, n_wingbeats)
    common = load_common_module()
    return {
        "s": _smooth_pair(common, "s", n_harm, n_points),
        "d": _smooth_pair(common, "d", n_harm, n_points),
        "beta": _smooth_pair(common, "beta", n_harm, n_points),
    }


def plot_smoothed_kinematics(
    output_path: Path,
    *,
    n_components: int = DEFAULT_N_COMPONENTS,
    n_wingbeats: int = DEFAULT_N_WINGBEATS,
    n_points: int = DEFAULT_N_POINTS,
    theme: str | None = None,
) -> None:
    series = compute_smoothed_kinematics(
        n_components=n_components,
        n_wingbeats=n_wingbeats,
        n_points=n_points,
    )
    t_start = min(
        float(series[key][time_key][0])
        for key in ("s", "d", "beta")
        for time_key in ("t_fore", "t_hind")
    )
    t_end = max(
        float(series[key][time_key][-1])
        for key in ("s", "d", "beta")
        for time_key in ("t_fore", "t_hind")
    )

    style = resolve_style(theme=theme)
    apply_matplotlib_style(style)

    fig, axes = plt.subplots(
        nrows=3,
        ncols=1,
        sharex=True,
        figsize=figure_size(height_over_width=PLOT_HEIGHT_OVER_WIDTH),
    )

    rows = [
        ("s", r"$s$ (mm)"),
        ("d", r"$d$ (mm)"),
        ("beta", r"$\beta$ (deg)"),
    ]

    for ax, (key, ylabel) in zip(axes, rows):
        values = series[key]
        ax.plot(values["t_fore"], values["y_fore"], linewidth=1.5, label="Forewing", color="C0")
        ax.plot(values["t_hind"], values["y_hind"], linewidth=1.5, label="Hindwing", color="C1")
        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.25)

    axes[0].legend(
        loc="lower center",
        bbox_to_anchor=(0.5, 1.03),
        ncol=2,
        fontsize=10.0,
    )
    axes[-1].set_xlabel(r"$t/T_{wb}$")
    axes[-1].set_xlim(t_start, t_end)
    fig.tight_layout()

    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(str(output_path), dpi=300, bbox_inches="tight")
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", help="Output image path (e.g. kinematics_inputs.png)")
    parser.add_argument(
        "--n-components",
        type=int,
        default=DEFAULT_N_COMPONENTS,
        help="Total Fourier components over the full window (default: 35).",
    )
    parser.add_argument(
        "--n-wingbeats",
        type=int,
        default=DEFAULT_N_WINGBEATS,
        help="Wingbeats represented in the source data window (default: 5).",
    )
    parser.add_argument(
        "--n-points",
        type=int,
        default=DEFAULT_N_POINTS,
        help="Samples used for each smoothed trace (default: 2000).",
    )
    parser.add_argument(
        "--theme",
        choices=["light", "dark"],
        default="light",
        help="Plot theme (default: light).",
    )
    args = parser.parse_args()

    output = Path(args.output)
    print(
        "Writing Wang 2007 smoothed kinematics plot: "
        f"{output} ({args.n_components} components over {args.n_wingbeats} wingbeats)"
    )
    plot_smoothed_kinematics(
        output,
        n_components=args.n_components,
        n_wingbeats=args.n_wingbeats,
        n_points=args.n_points,
        theme=args.theme,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

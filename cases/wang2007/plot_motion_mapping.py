#!/usr/bin/env python3
"""
Plot Wang 2007 simulator-angle mapping from smoothed experimental inputs.

Usage:
    python -m cases.wang2007.plot_motion_mapping <out.png> [--theme light|dark]
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from cases.wang2007.plot_kinematics import (
    DEFAULT_N_COMPONENTS,
    DEFAULT_N_POINTS,
    DEFAULT_N_WINGBEATS,
    compute_smoothed_kinematics,
    load_common_module,
)
from post.time_series import plot_fore_hind_series

from scripts.case_data import load_case_data


WANG2007_CASE = load_case_data("wang2007")
DEFAULT_WING_LENGTH_MM = float(WANG2007_CASE["specimen"]["wing_length_mm"])
PLOT_HEIGHT_OVER_WIDTH = 0.45


def compute_motion_mapping_series(
    *,
    n_components: int = DEFAULT_N_COMPONENTS,
    n_wingbeats: int = DEFAULT_N_WINGBEATS,
    n_points: int = DEFAULT_N_POINTS,
    wing_length_mm: float = DEFAULT_WING_LENGTH_MM,
) -> dict[str, np.ndarray]:
    common = load_common_module()
    smooth = compute_smoothed_kinematics(
        n_components=n_components,
        n_wingbeats=n_wingbeats,
        n_points=n_points,
    )

    t = smooth["s"]["t_fore"]
    sf = smooth["s"]["y_fore"]
    sh = smooth["s"]["y_hind"]
    bf = smooth["beta"]["y_fore"]
    bh = smooth["beta"]["y_hind"]

    r_ref = (2.0 / 3.0) * wing_length_mm
    phi_fore = np.arcsin(np.clip(sf / r_ref, -1.0, 1.0))
    phi_hind = np.arcsin(np.clip(sh / r_ref, -1.0, 1.0))
    psi_fore = np.pi / 2.0 - np.radians(bf)
    psi_hind = np.pi / 2.0 - np.radians(bh)

    gamma_fore = np.full_like(t, float(common.gamma_fore))
    gamma_hind = np.full_like(t, float(common.gamma_hind))

    return {
        "t": np.asarray(t),
        "phi_fore_deg": np.degrees(phi_fore),
        "phi_hind_deg": np.degrees(phi_hind),
        "psi_fore_deg": np.degrees(psi_fore),
        "psi_hind_deg": np.degrees(psi_hind),
        "gamma_fore_deg": np.degrees(gamma_fore),
        "gamma_hind_deg": np.degrees(gamma_hind),
    }


def plot_motion_mapping(
    output_path: Path,
    *,
    n_components: int = DEFAULT_N_COMPONENTS,
    n_wingbeats: int = DEFAULT_N_WINGBEATS,
    n_points: int = DEFAULT_N_POINTS,
    wing_length_mm: float = DEFAULT_WING_LENGTH_MM,
    theme: str | None = None,
) -> None:
    series = compute_motion_mapping_series(
        n_components=n_components,
        n_wingbeats=n_wingbeats,
        n_points=n_points,
        wing_length_mm=wing_length_mm,
    )
    plot_fore_hind_series(
        output_path, series,
        rows=[
            ("phi_fore_deg", "phi_hind_deg", r"$\phi$ (deg)"),
            ("psi_fore_deg", "psi_hind_deg", r"$\psi$ (deg)"),
        ],
        height_over_width=PLOT_HEIGHT_OVER_WIDTH, theme=theme,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", help="Output image path (e.g. motion_mapping.png)")
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
        "--wing-length-mm",
        type=float,
        default=DEFAULT_WING_LENGTH_MM,
        help=f"Wing length in mm (default: {DEFAULT_WING_LENGTH_MM}).",
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
        "Writing Wang 2007 motion mapping plot: "
        f"{output} ({args.n_components} components over {args.n_wingbeats} wingbeats)"
    )
    plot_motion_mapping(
        output,
        n_components=args.n_components,
        n_wingbeats=args.n_wingbeats,
        n_points=args.n_points,
        wing_length_mm=args.wing_length_mm,
        theme=args.theme,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

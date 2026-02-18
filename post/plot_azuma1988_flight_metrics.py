#!/usr/bin/env python3
"""
Plot Azuma 1988 body-flight metrics.

Reuses compute_flight_metrics from the azuma1985 module with Azuma 1988 defaults.

Usage:
    python -m post.plot_azuma1988_flight_metrics <input.h5> <out.png> [--experiment 1] [--theme light|dark]
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from post.plot_azuma1985_flight_metrics import compute_flight_metrics
from post.style import apply_matplotlib_style, figure_size, resolve_style

try:
    from case_data import find_output_reference, load_case_data, select_experiment
except ModuleNotFoundError:
    from scripts.case_data import find_output_reference, load_case_data, select_experiment


AZUMA1988_CASE = load_case_data("azuma1988")
AZUMA1988_DEFAULT_CASE = select_experiment(AZUMA1988_CASE, experiment_id=None)
AZUMA1988_SPECIMEN = AZUMA1988_DEFAULT_CASE["specimen"]
AZUMA1988_SIM_DEFAULTS = AZUMA1988_DEFAULT_CASE["simulation_defaults"]

DEFAULT_BODY_LENGTH_M = float(AZUMA1988_SPECIMEN["body_length_m"])
DEFAULT_GRAVITY_M_S2 = float(AZUMA1988_SIM_DEFAULTS["gravity_m_s2"])
PLOT_HEIGHT_OVER_WIDTH = 0.4


def plot_flight_metrics(
    h5_path: Path,
    output_path: Path,
    *,
    body_length_m: float | None = None,
    gravity_m_s2: float | None = None,
    speed_ref_m_s: float,
    direction_ref_deg: float,
    theme: str | None = None,
) -> None:
    series = compute_flight_metrics(
        h5_path,
        body_length_m=body_length_m,
        gravity_m_s2=gravity_m_s2,
    )
    t = np.asarray(series["wingbeats"])
    speed = np.asarray(series["speed_m_s"])
    direction = np.asarray(series["direction_deg"])

    style = resolve_style(theme=theme)
    apply_matplotlib_style(style)

    fig, axes = plt.subplots(
        nrows=1,
        ncols=2,
        sharex=True,
        figsize=figure_size(height_over_width=PLOT_HEIGHT_OVER_WIDTH),
    )

    speed_ax, dir_ax = axes
    speed_ax.plot(t, speed, linewidth=1.5, color="C0", label="Simulation")
    speed_ax.axhline(speed_ref_m_s, linewidth=1.2, color="C1", linestyle="--", label="Experiment")
    speed_ax.set_xlabel(r"$t/T_{wb}$")
    speed_ax.set_ylabel("Speed (m/s)")
    speed_ax.grid(True, alpha=0.25)
    speed_ax.legend(loc="best")

    dir_ax.plot(t, direction, linewidth=1.5, color="C0", label="Simulation")
    dir_ax.axhline(direction_ref_deg, linewidth=1.2, color="C1", linestyle="--", label="Experiment")
    dir_ax.set_xlabel(r"$t/T_{wb}$")
    dir_ax.set_ylabel("Direction (deg)")
    dir_ax.grid(True, alpha=0.25)
    dir_ax.legend(loc="best")

    speed_ax.set_xlim(float(t[0]), float(t[-1]))
    fig.tight_layout()

    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(str(output_path), dpi=300, bbox_inches="tight")
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_h5", help="Simulation output HDF5 path.")
    parser.add_argument("output", help="Output image path (e.g. flight_metrics.png).")
    parser.add_argument("--experiment", default=None, help="Azuma 1988 experiment id (e.g. 1-4).")
    parser.add_argument("--body-length-m", type=float, default=None)
    parser.add_argument("--gravity-m-s2", type=float, default=None)
    parser.add_argument("--speed-ref-m-s", type=float, default=None)
    parser.add_argument("--direction-ref-deg", type=float, default=None)
    parser.add_argument("--theme", choices=["light", "dark"], default="light")
    args = parser.parse_args()

    case = select_experiment(AZUMA1988_CASE, experiment_id=args.experiment)
    flight_ref = find_output_reference(
        case,
        kind="flight_condition",
        name="body_speed_and_direction",
    )

    input_h5 = Path(args.input_h5)
    output = Path(args.output)
    print(f"Reading Azuma 1988 simulation: {input_h5} (experiment={case['selected_experiment']['id']})")
    print(f"Writing Azuma 1988 flight metrics plot: {output}")
    plot_flight_metrics(
        input_h5,
        output,
        body_length_m=args.body_length_m,
        gravity_m_s2=args.gravity_m_s2,
        speed_ref_m_s=(
            float(flight_ref["speed_m_s"]) if args.speed_ref_m_s is None else float(args.speed_ref_m_s)
        ),
        direction_ref_deg=(
            float(flight_ref["direction_deg"])
            if args.direction_ref_deg is None
            else float(args.direction_ref_deg)
        ),
        theme=args.theme,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""
Plot Azuma 1988 body-flight metrics.

Reuses compute_flight_metrics from the azuma1985 module with Azuma 1988 defaults.

Usage:
    python -m cases.azuma1988.plot_flight_metrics <input.h5> <out.png> [--experiment 1] [--theme light|dark]
"""

from __future__ import annotations

import argparse
from pathlib import Path

from cases.azuma1985.plot_flight_metrics import plot_flight_metrics

from scripts.case_data import find_output_reference, load_case_data, select_experiment


AZUMA1988_CASE = load_case_data("azuma1988")

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

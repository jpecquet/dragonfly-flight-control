#!/usr/bin/env python3
"""
Plot Azuma 1985 body-flight metrics against experimental references.

Usage:
    python -m post.plot_azuma1985_flight_metrics <input.h5> <out.png> [--theme light|dark]
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import h5py
import matplotlib.pyplot as plt
import numpy as np

from post.style import apply_matplotlib_style, figure_size, resolve_style

DEFAULT_BODY_LENGTH_M = 4.0e-2
DEFAULT_GRAVITY_M_S2 = 9.81
DEFAULT_SPEED_REF_MPS = 0.54
DEFAULT_DIRECTION_REF_DEG = 60.0
PLOT_HEIGHT_OVER_WIDTH = 0.4


def _load_scales_from_translate_summary(h5_path: Path) -> tuple[float, float] | None:
    summary_path = h5_path.parent / "translate_summary.json"
    if not summary_path.exists():
        return None
    try:
        payload = json.loads(summary_path.read_text(encoding="utf-8"))
        physical = payload.get("physical_inputs", {})
        body_length_m = float(physical["body_length_m"])
        gravity_m_s2 = float(physical["gravity_m_s2"])
    except (OSError, json.JSONDecodeError, KeyError, TypeError, ValueError):
        return None
    if body_length_m <= 0.0 or gravity_m_s2 <= 0.0:
        return None
    return body_length_m, gravity_m_s2


def compute_flight_metrics(
    h5_path: Path,
    *,
    body_length_m: float | None = None,
    gravity_m_s2: float | None = None,
) -> dict[str, np.ndarray | float]:
    with h5py.File(str(h5_path), "r") as f:
        time = np.asarray(f["/time"][:], dtype=float)
        states = np.asarray(f["/state"][:], dtype=float)
        omega_nondim = float(f["/parameters/omega"][()])

    if states.ndim != 2 or states.shape[1] < 6:
        raise ValueError(f"Expected /state shape [N,6+], got {states.shape}")

    if body_length_m is None or gravity_m_s2 is None:
        summary_scales = _load_scales_from_translate_summary(h5_path)
    else:
        summary_scales = None
    if body_length_m is None:
        body_length_m = summary_scales[0] if summary_scales is not None else DEFAULT_BODY_LENGTH_M
    if gravity_m_s2 is None:
        gravity_m_s2 = summary_scales[1] if summary_scales is not None else DEFAULT_GRAVITY_M_S2
    if body_length_m <= 0.0:
        raise ValueError(f"body_length_m must be > 0, got {body_length_m}")
    if gravity_m_s2 <= 0.0:
        raise ValueError(f"gravity_m_s2 must be > 0, got {gravity_m_s2}")

    vel_nondim = np.linalg.norm(states[:, 3:6], axis=1)
    speed_scale_m_s = math.sqrt(float(gravity_m_s2) * float(body_length_m))
    speed_m_s = vel_nondim * speed_scale_m_s

    x = states[:, 0]
    z = states[:, 2]
    direction_deg = np.degrees(np.arctan2(z, x))

    wingbeats = time * omega_nondim / (2.0 * np.pi)

    return {
        "wingbeats": wingbeats,
        "speed_m_s": speed_m_s,
        "direction_deg": direction_deg,
        "body_length_m": float(body_length_m),
        "gravity_m_s2": float(gravity_m_s2),
        "omega_nondim": omega_nondim,
    }


def plot_flight_metrics(
    h5_path: Path,
    output_path: Path,
    *,
    body_length_m: float | None = None,
    gravity_m_s2: float | None = None,
    speed_ref_m_s: float = DEFAULT_SPEED_REF_MPS,
    direction_ref_deg: float = DEFAULT_DIRECTION_REF_DEG,
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
    parser.add_argument(
        "--body-length-m",
        type=float,
        default=None,
        help="Optional body length for dimensionalization. Defaults to translate_summary.json value.",
    )
    parser.add_argument(
        "--gravity-m-s2",
        type=float,
        default=None,
        help="Optional gravity for dimensionalization. Defaults to translate_summary.json value.",
    )
    parser.add_argument(
        "--speed-ref-m-s",
        type=float,
        default=DEFAULT_SPEED_REF_MPS,
        help=f"Experimental speed reference in m/s (default: {DEFAULT_SPEED_REF_MPS}).",
    )
    parser.add_argument(
        "--direction-ref-deg",
        type=float,
        default=DEFAULT_DIRECTION_REF_DEG,
        help=f"Experimental direction reference in degrees (default: {DEFAULT_DIRECTION_REF_DEG}).",
    )
    parser.add_argument(
        "--theme",
        choices=["light", "dark"],
        default="light",
        help="Plot theme (default: light).",
    )
    args = parser.parse_args()

    input_h5 = Path(args.input_h5)
    output = Path(args.output)
    print(f"Reading Azuma 1985 simulation: {input_h5}")
    print(f"Writing Azuma 1985 flight metrics plot: {output}")
    plot_flight_metrics(
        input_h5,
        output,
        body_length_m=args.body_length_m,
        gravity_m_s2=args.gravity_m_s2,
        speed_ref_m_s=args.speed_ref_m_s,
        direction_ref_deg=args.direction_ref_deg,
        theme=args.theme,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

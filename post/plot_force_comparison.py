#!/usr/bin/env python3
"""
Plot and compare vertical aerodynamic force time series.

Usage:
    python -m post.plot_force_comparison <out.png> --omega-nondim <value> \
        --series <run_a.h5> "Run A" --series <run_b.h5> "Run B"
    python -m post.plot_force_comparison <out.png> --omega-nondim <value> \
        --series <run_a.h5> "Run A" --series-csv <ref.csv> t Fz "Reference CFD"
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import h5py
import matplotlib.pyplot as plt
import numpy as np

from post.time_series import SeriesSpec, plot_time_series


@dataclass(frozen=True)
class ExternalForceSeries:
    """External force time series, already represented as (x, Fz)."""

    x: np.ndarray
    fz: np.ndarray
    label: str
    style: str = "line"  # "line" or "scatter"
    color: str | None = None
    wrap_period: float | None = None  # wrap x into [0, period]


def read_aero_force_z(h5_path: Path) -> tuple[np.ndarray, np.ndarray, float]:
    """Read total aerodynamic z-force and omega from a simulation output HDF5 file."""
    with h5py.File(str(h5_path), "r") as f:
        time = f["/time"][:]
        omega = float(f["/parameters/omega"][()])
        wing_names = sorted(k for k in f["/wings"].keys() if k != "num_wings")
        fz = np.zeros_like(time)
        for wname in wing_names:
            fz += f[f"/wings/{wname}/lift"][:, 2]
            fz += f[f"/wings/{wname}/drag"][:, 2]
    return time, fz, omega


def _resolve_csv_column_index(header: list[str], key: str) -> int:
    stripped_header = [h.strip() for h in header]
    key_stripped = key.strip()
    try:
        idx = int(key_stripped)
    except ValueError:
        if key_stripped in stripped_header:
            return stripped_header.index(key_stripped)
        raise KeyError(f"Column {key!r} not found in CSV header: {stripped_header}") from None

    if idx < 0 or idx >= len(header):
        raise IndexError(f"CSV column index {idx} out of range for {len(header)} columns")
    return idx


def read_force_series_csv(csv_path: Path, x_col: str, y_col: str, label: str) -> ExternalForceSeries:
    """Read an external force series from CSV columns."""
    xs: list[float] = []
    ys: list[float] = []
    with csv_path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.reader(f)
        header = next(reader, None)
        if header is None:
            raise ValueError(f"CSV is empty: {csv_path}")
        x_idx = _resolve_csv_column_index(header, x_col)
        y_idx = _resolve_csv_column_index(header, y_col)
        for line_no, row in enumerate(reader, start=2):
            if not row:
                continue
            if max(x_idx, y_idx) >= len(row):
                raise ValueError(
                    f"Row {line_no} in {csv_path} has {len(row)} columns, "
                    f"needs at least {max(x_idx, y_idx) + 1}"
                )
            try:
                x_val = float(row[x_idx].strip())
                y_val = float(row[y_idx].strip())
            except ValueError as exc:
                raise ValueError(f"Failed parsing numeric values at {csv_path}:{line_no}") from exc
            xs.append(x_val)
            ys.append(y_val)

    if not xs:
        raise ValueError(f"No data rows read from {csv_path}")
    return ExternalForceSeries(x=np.asarray(xs), fz=np.asarray(ys), label=label)


def plot_force_comparison(
    h5_inputs: Sequence[Path] | None,
    output_path: Path,
    omega_nondim: float | None = None,
    *,
    labels: Sequence[str] | None = None,
    external_series: Sequence[ExternalForceSeries] | None = None,
    theme: str | None = None,
    include_mean_in_label: bool = False,
) -> None:
    """Plot vertical aerodynamic force for an arbitrary number of simulation outputs.

    omega_nondim is unused; each H5 file's omega is read directly from /parameters/omega.
    The parameter is retained for backward compatibility.
    """
    resolved_h5_inputs = list(h5_inputs or [])
    resolved_external = list(external_series or [])
    if not resolved_h5_inputs and not resolved_external:
        raise ValueError("Provide at least one simulation series or external series")
    if labels is not None and len(labels) != len(resolved_h5_inputs):
        raise ValueError(
            f"labels length ({len(labels)}) must match h5_inputs length ({len(resolved_h5_inputs)})"
        )

    resolved_labels = list(labels) if labels is not None else [Path(p).stem for p in resolved_h5_inputs]
    series: list[SeriesSpec] = []
    series_data: list[tuple[np.ndarray, np.ndarray, str]] = []

    for h5_path, label in zip(resolved_h5_inputs, resolved_labels):
        time, fz, omega = read_aero_force_z(Path(h5_path))
        wingbeats = time * omega / (2.0 * np.pi)
        series_data.append((wingbeats, fz, label))

    # Separate scatter externals from line externals.
    scatter_externals: list[ExternalForceSeries] = []
    for ext in resolved_external:
        x = ext.x
        if ext.wrap_period is not None:
            x = x % ext.wrap_period
        if ext.style == "scatter":
            scatter_externals.append(ExternalForceSeries(
                x=x, fz=ext.fz, label=ext.label,
                style=ext.style, color=ext.color, wrap_period=ext.wrap_period,
            ))
        else:
            series_data.append((ext.x, ext.fz, ext.label))

    # Compute xlim from simulation (H5) series only so external reference data
    # that extends slightly beyond the simulation range doesn't shorten the plot.
    n_sim = len(resolved_h5_inputs)
    if n_sim > 0:
        sim_xlim: tuple[float, float] | None = (
            float(min(series_data[i][0][0] for i in range(n_sim))),
            float(max(series_data[i][0][-1] for i in range(n_sim))),
        )
    else:
        sim_xlim = None

    _sim_colors = ["#f0a030", "C1", "C2", "C3"]  # yellow-orange for first sim series
    n_labeled = len(series_data)  # for legend ncol (envelope lines are unlabeled)
    for i, (x_values, fz, label) in enumerate(series_data):
        mean_force = float(np.mean(fz))
        legend_label = f"{label} (mean {mean_force:.3f})" if include_mean_in_label else label
        color = _sim_colors[i] if i < len(_sim_colors) else f"C{i}"
        series.append(SeriesSpec(x_values, fz, label=legend_label, color=color))
        if i < n_sim:
            series.append(SeriesSpec(x_values, 0.5 * fz, label=None, color=color, linewidth=0.8, linestyle="--", alpha=0.5))
            series.append(SeriesSpec(x_values, 1.5 * fz, label=None, color=color, linewidth=0.8, linestyle="--", alpha=0.5))

    # Build scatter specs to pass through to the plotting call.
    scatter_specs: list[tuple[np.ndarray, np.ndarray, str, str]] = []
    for ext in scatter_externals:
        mean_force = float(np.mean(ext.fz))
        legend_label = f"{ext.label} (mean {mean_force:.3f})" if include_mean_in_label else ext.label
        scolor = ext.color or "green"
        scatter_specs.append((ext.x, ext.fz, legend_label, scolor))

    fig, ax = plot_time_series(
        series=series,
        output_path=None,  # defer saving — we add scatter overlay first
        xlabel="$t/T_{wb}$",
        ylabel=r"$\tilde{F}_z$",
        theme=theme,
        height_over_width=1.0 / 2.0,
        xlim=sim_xlim,
        show_grid=True,
        legend_loc="lower center",
        legend_bbox_to_anchor=(0.5, 1.03),
        legend_ncol=max(1, min(3, n_labeled + len(scatter_specs))),
        legend_fontsize=10.0,
    )
    for sx, sy, slabel, scolor in scatter_specs:
        ax.scatter(sx, sy, s=12, alpha=0.35, color=scolor, label=slabel, zorder=1)

    # Re-draw legend to include scatter entries.
    if scatter_specs:
        ax.legend(
            loc="lower center",
            bbox_to_anchor=(0.5, 1.03),
            ncol=max(1, min(3, n_labeled + len(scatter_specs))),
            fontsize=10.0,
        )

    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(str(output_path), bbox_inches="tight", dpi=300)
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", help="Output image path (e.g. force_comparison.png).")
    parser.add_argument(
        "--omega-nondim",
        type=float,
        default=None,
        help="Unused; omega is read from each HDF5 file. Retained for backward compatibility.",
    )
    parser.add_argument(
        "--series",
        nargs=2,
        action="append",
        metavar=("INPUT_H5", "LABEL"),
        default=[],
        help="One time-series entry (repeat). Format: --series <input.h5> <legend label>.",
    )
    parser.add_argument(
        "--series-csv",
        nargs=4,
        action="append",
        metavar=("CSV_PATH", "X_COL", "Y_COL", "LABEL"),
        default=[],
        help=(
            "One external CSV series (repeat). X_COL/Y_COL may be column names "
            "or 0-based indices."
        ),
    )
    parser.add_argument(
        "--hide-mean",
        action="store_true",
        help="Do not append '(mean ...)' to legend labels.",
    )
    parser.add_argument(
        "--theme",
        choices=["light", "dark"],
        default="light",
        help="Plot theme (default: light).",
    )
    args = parser.parse_args()

    if not args.series and not args.series_csv:
        parser.error("Provide at least one --series or --series-csv entry.")

    inputs = [Path(item[0]) for item in args.series]
    labels = [item[1] for item in args.series]
    external_series = [
        read_force_series_csv(Path(item[0]), item[1], item[2], item[3])
        for item in args.series_csv
    ]
    output = Path(args.output)

    for h5_path, label in zip(inputs, labels):
        print(f"Reading series: {h5_path} (label: {label})")
    for item in args.series_csv:
        print(
            f"Reading external series: {item[0]} "
            f"(x={item[1]}, y={item[2]}, label: {item[3]})"
        )
    print(f"Writing force comparison: {output}")

    plot_force_comparison(
        h5_inputs=inputs,
        output_path=output,
        omega_nondim=args.omega_nondim,
        labels=labels,
        external_series=external_series,
        theme=args.theme,
        include_mean_in_label=not args.hide_mean,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

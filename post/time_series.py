#!/usr/bin/env python3
"""
Shared helpers for static time-series plots.

This module provides a small reusable plotting template that keeps styling,
figure sizing, and labeling consistent across time-series visualizations.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import matplotlib.pyplot as plt
import numpy as np

from post.hybrid_config import StyleConfig
from post.style import apply_matplotlib_style, figure_size, resolve_style


@dataclass(frozen=True)
class SeriesSpec:
    """Line-series specification for time-series plots."""

    x: np.ndarray
    y: np.ndarray
    label: str | None = None
    color: str | None = None
    linewidth: float = 1.2
    linestyle: str = "-"
    alpha: float = 1.0


@dataclass(frozen=True)
class HorizontalLineSpec:
    """Horizontal reference line specification."""

    y: float
    label: str | None = None
    color: str | None = None
    linewidth: float = 0.8
    linestyle: str = "--"
    alpha: float = 0.5


def _as_1d_array(values: np.ndarray) -> np.ndarray:
    arr = np.asarray(values)
    if arr.ndim != 1:
        raise ValueError(f"Expected 1D array, got shape {arr.shape}")
    return arr


def plot_time_series(
    series: Sequence[SeriesSpec],
    output_path: Path | str | None = None,
    *,
    xlabel: str = "Time",
    ylabel: str = "Value",
    title: str | None = None,
    style: StyleConfig | None = None,
    theme: str | None = None,
    height_over_width: float = 1.0 / 2.0,
    xlim: tuple[float, float] | None = None,
    ylim: tuple[float, float] | None = None,
    hlines: Sequence[HorizontalLineSpec] | None = None,
    show_grid: bool = False,
    legend_loc: str = "best",
    legend_bbox_to_anchor: tuple[float, float] | None = None,
    legend_ncol: int = 1,
    legend_fontsize: float = 10.0,
) -> tuple[plt.Figure, plt.Axes]:
    """Plot one or more time-series lines and optional horizontal references."""
    if not series:
        raise ValueError("series must contain at least one entry")

    resolved_style = resolve_style(style, theme=theme)
    apply_matplotlib_style(resolved_style)

    fig, ax = plt.subplots(figsize=figure_size(height_over_width))

    has_labels = False
    x_starts: list[float] = []
    x_ends: list[float] = []
    for spec in series:
        x = _as_1d_array(spec.x)
        y = _as_1d_array(spec.y)
        if x.shape != y.shape:
            raise ValueError(f"x/y length mismatch: {x.shape} vs {y.shape}")
        if x.size == 0:
            raise ValueError("x/y series must be non-empty")
        x_starts.append(float(x[0]))
        x_ends.append(float(x[-1]))
        ax.plot(
            x,
            y,
            label=spec.label,
            color=spec.color,
            linewidth=spec.linewidth,
            linestyle=spec.linestyle,
            alpha=spec.alpha,
        )
        has_labels = has_labels or bool(spec.label)

    for line in hlines or []:
        ax.axhline(
            y=float(line.y),
            label=line.label,
            color=line.color,
            linewidth=line.linewidth,
            linestyle=line.linestyle,
            alpha=line.alpha,
        )
        has_labels = has_labels or bool(line.label)

    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    if title:
        ax.set_title(title)
    if xlim is not None:
        ax.set_xlim(*xlim)
    else:
        ax.set_xlim(min(x_starts), max(x_ends))
    if ylim is not None:
        ax.set_ylim(*ylim)
    if show_grid:
        ax.grid(True, alpha=0.3)
    if has_labels:
        legend_kwargs: dict[str, object] = {
            "loc": legend_loc,
            "fontsize": legend_fontsize,
            "ncol": max(1, int(legend_ncol)),
        }
        if legend_bbox_to_anchor is not None:
            legend_kwargs["bbox_to_anchor"] = legend_bbox_to_anchor
        ax.legend(**legend_kwargs)

    fig.tight_layout()

    if output_path is not None:
        output = Path(output_path)
        output.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(str(output), bbox_inches="tight", dpi=300)
        plt.close(fig)

    return fig, ax

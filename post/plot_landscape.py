#!/usr/bin/env python3
"""
Visualize objective function landscape from optimizer.

Usage:
    python -m post.plot_landscape <input.h5> [output.png] [--theme light|dark]
"""

import argparse
import numpy as np
import matplotlib.pyplot as plt

from post.hybrid_config import StyleConfig
from post.io import read_landscape
from post.style import apply_matplotlib_style, resolve_style


def plot_1d_landscape(data, style: StyleConfig, output_file=None):
    """Plot 1D landscape (single variable parameter)."""
    fig, ax = plt.subplots(figsize=(8, 5))

    # Convert to degrees for display
    param_deg = np.rad2deg(data['param1_values'])
    obj = data['objective'][:, 0]

    ax.plot(param_deg, obj, color=style.trajectory_color, linewidth=2)
    ax.axhline(y=0, color=style.muted_text_color, linestyle='--', alpha=0.4)

    # Mark minimum
    min_idx = np.argmin(obj)
    ax.plot(
        param_deg[min_idx], obj[min_idx], 'o',
        color=style.landscape_min_marker_color, markersize=10,
        label=f'min: {param_deg[min_idx]:.1f} deg, obj={obj[min_idx]:.2e}',
    )

    ax.set_xlabel(f'{data["param_names"][0]} (deg)')
    ax.set_ylabel('Objective (acceleration magnitude)')
    ax.set_title(f'Objective Landscape at ux = {data["ux"]:.2f}')
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()

    if output_file:
        plt.savefig(output_file, dpi=300)
        print(f'Saved to {output_file}')
    else:
        plt.show()


def plot_2d_landscape(data, style: StyleConfig, output_file=None):
    """Plot 2D landscape (two variable parameters)."""
    fig, ax = plt.subplots(figsize=(10, 8))

    # Convert to degrees for display
    p1_deg = np.rad2deg(data['param1_values'])
    p2_deg = np.rad2deg(data['param2_values'])
    obj = data['objective']

    # Create mesh grid
    P1, P2 = np.meshgrid(p1_deg, p2_deg, indexing='ij')

    # Use log scale for better visibility (add small epsilon to avoid log(0))
    obj_log = np.log10(obj + 1e-10)

    # Contour plot
    levels = 30
    cf = ax.contourf(P1, P2, obj_log, levels=levels, cmap=style.landscape_colormap)
    ax.contour(
        P1, P2, obj_log, levels=10,
        colors=style.landscape_contour_color, alpha=0.35, linewidths=0.5,
    )

    # Colorbar
    cbar = plt.colorbar(cf, ax=ax)
    cbar.set_label('log10(acceleration magnitude)')

    # Mark global minimum
    min_idx = np.unravel_index(np.argmin(obj), obj.shape)
    ax.plot(
        p1_deg[min_idx[0]], p2_deg[min_idx[1]], '*',
        markersize=15, color=style.landscape_min_marker_color,
        markeredgecolor=style.figure_facecolor, markeredgewidth=1.5,
        label=f'min: ({p1_deg[min_idx[0]]:.1f}, {p2_deg[min_idx[1]]:.1f}) deg',
    )

    ax.set_xlabel(f'{data["param_names"][0]} (deg)')
    ax.set_ylabel(f'{data["param_names"][1]} (deg)')
    ax.set_title(f'Objective Landscape at ux = {data["ux"]:.2f}')
    ax.legend(loc='upper right')

    plt.tight_layout()

    if output_file:
        plt.savefig(output_file, dpi=300)
        print(f'Saved to {output_file}')
    else:
        plt.show()


def main():
    parser = argparse.ArgumentParser(description="Visualize objective function landscape")
    parser.add_argument("input", help="Input HDF5 file")
    parser.add_argument("output", nargs="?", help="Output image file (optional)")
    parser.add_argument(
        "--theme",
        choices=["light", "dark"],
        default="light",
        help="Plot theme (default: light)",
    )
    args = parser.parse_args()

    style = resolve_style(theme=args.theme)
    apply_matplotlib_style(style)

    print(f'Loading {args.input}...')
    data = read_landscape(args.input)

    print(f'Forward velocity: ux = {data["ux"]:.2f}')
    print(f'Parameters: {", ".join(data["param_names"])}')
    print(f'Theme: {style.theme}')

    if 'param2_values' in data and len(data['param2_values']) > 0:
        print('Generating 2D contour plot...')
        plot_2d_landscape(data, style, args.output)
    else:
        print('Generating 1D plot...')
        plot_1d_landscape(data, style, args.output)


if __name__ == '__main__':
    main()

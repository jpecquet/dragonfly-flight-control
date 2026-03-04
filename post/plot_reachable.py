"""Visualize reachable set analysis results.

Usage:
    python -m post.plot_reachable results.h5 [--plot TYPE] [--branch N]

Plot types:
    reachable      Heatmap of log10(min_residual) with equilibrium boundary
    branches       Integer heatmap of number of equilibrium branches found
    manipulability Heatmap of minimum singular value of the Jacobian
    params         Parameter values across the grid for a given branch
"""

import argparse
import sys

import h5py
import matplotlib.pyplot as plt
import numpy as np


def load_data(path):
    """Load HDF5 reachable set data."""
    data = {}
    with h5py.File(path, "r") as f:
        data["ux"] = f["grid/ux"][:]
        data["uz"] = f["grid/uz"][:]
        data["min_residual"] = f["grid/min_residual"][:]
        data["n_branches"] = f["grid/n_branches"][:]
        data["param_names"] = [s.decode() if isinstance(s, bytes) else s
                               for s in f["metadata/param_names"][:]]
        data["equilibrium_tol"] = float(f["metadata/equilibrium_tol"][()])

        data["branches"] = []
        bg = f["branches"]
        for key in sorted(bg.keys(), key=int):
            branch = {
                "params": bg[key]["params"][:],
                "residual": bg[key]["residual"][:],
                "jacobian": bg[key]["jacobian"][:],
                "sigma_min": bg[key]["sigma_min"][:],
            }
            data["branches"].append(branch)

    return data


def plot_reachable(data):
    """Heatmap of log10(min_residual) with equilibrium contour."""
    ux, uz = data["ux"], data["uz"]
    res = data["min_residual"]
    tol = data["equilibrium_tol"]

    fig, ax = plt.subplots(figsize=(8, 6))
    log_res = np.log10(np.where(np.isnan(res), np.nan, np.maximum(res, 1e-20)))

    im = ax.pcolormesh(ux, uz, log_res.T, shading="auto", cmap="viridis_r")
    cb = fig.colorbar(im, ax=ax, label="log10(min residual)")

    # Contour at equilibrium tolerance
    ax.contour(ux, uz, res.T, levels=[tol], colors="red", linewidths=2)

    ax.set_xlabel("ux (forward velocity)")
    ax.set_ylabel("uz (vertical velocity)")
    ax.set_title("Reachable Set: Minimum Residual")
    plt.tight_layout()
    return fig


def plot_branches(data):
    """Integer heatmap of number of equilibrium branches."""
    ux, uz = data["ux"], data["uz"]
    nb = data["n_branches"]

    fig, ax = plt.subplots(figsize=(8, 6))
    max_b = max(nb.max(), 1)
    cmap = plt.cm.get_cmap("Set1", max_b + 1)
    im = ax.pcolormesh(ux, uz, nb.T, shading="auto", cmap=cmap, vmin=-0.5, vmax=max_b + 0.5)
    cb = fig.colorbar(im, ax=ax, label="Number of branches", ticks=range(max_b + 1))

    ax.set_xlabel("ux (forward velocity)")
    ax.set_ylabel("uz (vertical velocity)")
    ax.set_title("Equilibrium Branch Count")
    plt.tight_layout()
    return fig


def plot_manipulability(data, branch=0):
    """Heatmap of minimum singular value of Jacobian."""
    ux, uz = data["ux"], data["uz"]
    sigma = data["branches"][branch]["sigma_min"]

    fig, ax = plt.subplots(figsize=(8, 6))
    im = ax.pcolormesh(ux, uz, sigma.T, shading="auto", cmap="inferno")
    fig.colorbar(im, ax=ax, label="Min singular value (sigma_min)")

    ax.set_xlabel("ux (forward velocity)")
    ax.set_ylabel("uz (vertical velocity)")
    ax.set_title(f"Control Manipulability (branch {branch})")
    plt.tight_layout()
    return fig


def plot_params(data, branch=0):
    """Parameter values across the grid for a given branch."""
    ux, uz = data["ux"], data["uz"]
    params = data["branches"][branch]["params"]  # (n_ux, n_uz, n_var)
    names = data["param_names"]
    n_var = len(names)

    n_cols = 3
    n_rows = (n_var + n_cols - 1) // n_cols
    fig, axes = plt.subplots(n_rows, n_cols, figsize=(5 * n_cols, 4 * n_rows))
    axes = np.atleast_2d(axes)

    for k, name in enumerate(names):
        ax = axes[k // n_cols, k % n_cols]
        vals = params[:, :, k]
        im = ax.pcolormesh(ux, uz, vals.T, shading="auto", cmap="coolwarm")
        fig.colorbar(im, ax=ax)
        ax.set_title(name)
        ax.set_xlabel("ux")
        ax.set_ylabel("uz")

    # Hide unused axes
    for k in range(n_var, n_rows * n_cols):
        axes[k // n_cols, k % n_cols].set_visible(False)

    fig.suptitle(f"Parameter Values (branch {branch})", fontsize=14)
    plt.tight_layout()
    return fig


def main():
    parser = argparse.ArgumentParser(description="Visualize reachable set analysis")
    parser.add_argument("file", help="HDF5 results file")
    parser.add_argument("--plot", default="reachable",
                        choices=["reachable", "branches", "manipulability", "params"],
                        help="Plot type (default: reachable)")
    parser.add_argument("--branch", type=int, default=0,
                        help="Branch index for manipulability/params plots (default: 0)")
    args = parser.parse_args()

    data = load_data(args.file)

    plot_funcs = {
        "reachable": lambda: plot_reachable(data),
        "branches": lambda: plot_branches(data),
        "manipulability": lambda: plot_manipulability(data, args.branch),
        "params": lambda: plot_params(data, args.branch),
    }

    plot_funcs[args.plot]()
    plt.show()


if __name__ == "__main__":
    main()

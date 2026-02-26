#!/usr/bin/env python3
"""Generate aerodynamic coefficient comparison plot for docs.

Usage:
  python -m post.plot_aero_coefficients
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
OUTPUT_DIR = REPO_ROOT / "docs/_static/media/modeling/blade_elements"


def plot_aero_coefficient_comparison(output_path: Path, *, theme: str | None = None) -> None:
    """Side-by-side Cl/Cd vs angle of attack for Wang 2004, Azuma 1985, and Azuma 1988 models."""
    import matplotlib.pyplot as plt

    from post.lcm.aerodynamics import (
        cd_azuma1985,
        cd_azuma1988,
        cd_sinusoidal,
        cl_azuma1985,
        cl_azuma1988,
        cl_sinusoidal,
    )
    from post.style import apply_matplotlib_style, figure_size, resolve_style

    alpha_deg = np.linspace(-90.0, 90.0, 600)
    alpha_rad = np.radians(alpha_deg)

    cl_wang = cl_sinusoidal(alpha_rad)
    cd_wang = cd_sinusoidal(alpha_rad)
    cl_az85 = cl_azuma1985(alpha_rad)
    cd_az85 = cd_azuma1985(alpha_rad)
    cl_az88 = cl_azuma1988(alpha_rad)
    cd_az88 = cd_azuma1988(alpha_rad)

    style = resolve_style(theme=theme)
    apply_matplotlib_style(style)

    fig, (ax_cl, ax_cd) = plt.subplots(1, 2, figsize=figure_size(height_over_width=0.45))

    lw = 1.6
    line_styles = ["-", "--", ":"]
    datasets = [
        ("Wang 2004", cl_wang, cd_wang),
        ("Azuma 1985", cl_az85, cd_az85),
        ("Azuma 1988", cl_az88, cd_az88),
    ]
    for (label, cl, cd), ls in zip(datasets, line_styles):
        ax_cl.plot(alpha_deg, cl, linewidth=lw, color=style.text_color, linestyle=ls, label=label)
        ax_cd.plot(alpha_deg, cd, linewidth=lw, color=style.text_color, linestyle=ls)
    ax_cl.axhline(0.0, linewidth=0.75, color=style.text_color, alpha=0.25)
    ax_cl.axvline(0.0, linewidth=0.75, color=style.text_color, alpha=0.25)
    ax_cl.set_xlabel(r"$\alpha$ (deg)")
    ax_cl.set_ylabel(r"$C_L$")
    ax_cl.set_xlim(-90.0, 90.0)
    ax_cl.grid(True, alpha=0.25)
    ax_cd.axvline(0.0, linewidth=0.75, color=style.text_color, alpha=0.25)
    ax_cd.set_xlabel(r"$\alpha$ (deg)")
    ax_cd.set_ylabel(r"$C_D$")
    ax_cd.set_xlim(-90.0, 90.0)
    ax_cd.set_ylim(bottom=0.0)
    ax_cd.grid(True, alpha=0.25)

    handles, labels = ax_cl.get_legend_handles_labels()
    fig.legend(
        handles,
        labels,
        loc="lower center",
        bbox_to_anchor=(0.5, 1.01),
        ncol=3,
        fontsize=10.0,
        columnspacing=1.2,
        handlelength=2.2,
        handletextpad=0.5,
    )
    fig.tight_layout()

    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(str(output_path), dpi=300, bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    for theme in ("light", "dark"):
        out = OUTPUT_DIR / f"aero_coefficients.{theme}.png"
        print(f"Generating {out.relative_to(REPO_ROOT)} ...")
        plot_aero_coefficient_comparison(out, theme=theme)
    print("Done.")

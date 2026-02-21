#!/usr/bin/env python3
"""Two-wing stick animation for Azuma (1985) kinematics.

Styled and structured to match data/kinematics/wang2007/stick_animation.py.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from post.style import figure_size

REPO_ROOT = Path(__file__).resolve().parents[2]

from scripts.experimental_conventions import azuma1985_adapter, build_sim_wing_motion

plt.rcParams.update({
    "font.family": "serif",
    "font.serif": ["STIXGeneral"],
    "mathtext.fontset": "stix",
    "font.size": 12,
})


# Azuma (1985) parameters from docs/validation/azuma1985.md
AZUMA_ADAPTER = azuma1985_adapter()
AZUMA_SIM_MOTION = build_sim_wing_motion(AZUMA_ADAPTER, n_harmonics=3)
GAMMA_FW_DEG = AZUMA_ADAPTER.source_series["fore"].gamma.mean_deg
GAMMA_HW_DEG = AZUMA_ADAPTER.source_series["hind"].gamma.mean_deg
R_FW_MM = 33.5
R_HW_MM = 32.5
CHORD_FW_MM = 6.60
CHORD_HW_MM = 8.40


def eval_sim_harmonics(
    t_nondim: np.ndarray,
    mean_rad: float,
    amp_coeff: tuple[float, ...],
    phase_coeff: tuple[float, ...],
) -> np.ndarray:
    """Evaluate simulator harmonic series mean + sum(A_k cos(k*w*t + B_k))."""
    w_t = 2.0 * np.pi * t_nondim
    out = np.full_like(t_nondim, float(mean_rad), dtype=float)
    for k, (a_k, b_k) in enumerate(zip(amp_coeff, phase_coeff), start=1):
        out += float(a_k) * np.cos(k * w_t + float(b_k))
    return out


def experimental_kinematics(t_nondim: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Azuma kinematics interpreted as experimental traces.

    Returns:
      s_fw, s_hw in mm
      beta_fw, beta_hw in radians
    """
    src = AZUMA_ADAPTER.source_series
    phi_fw_deg = src["fore"].phi.eval_deg(t_nondim)
    phi_hw_deg = src["hind"].phi.eval_deg(t_nondim)
    psi_fw_deg = src["fore"].psi.eval_deg(t_nondim)
    psi_hw_deg = src["hind"].psi.eval_deg(t_nondim)
    s_fw = R_FW_MM * np.sin(np.radians(phi_fw_deg))
    s_hw = R_HW_MM * np.sin(np.radians(phi_hw_deg))
    beta_fw = np.radians(psi_fw_deg)
    beta_hw = np.radians(psi_hw_deg)
    return s_fw, s_hw, beta_fw, beta_hw


def model_kinematics(t_nondim: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Model traces generated from the same paper->sim mapping used by the pipeline."""
    fore = AZUMA_SIM_MOTION["fore"]
    hind = AZUMA_SIM_MOTION["hind"]
    phi_fw_sim = eval_sim_harmonics(t_nondim, fore.phi_mean, fore.phi_amp, fore.phi_phase)
    phi_hw_sim = eval_sim_harmonics(t_nondim, hind.phi_mean, hind.phi_amp, hind.phi_phase)
    psi_fw_sim = eval_sim_harmonics(t_nondim, fore.psi_mean, fore.psi_amp, fore.psi_phase)
    psi_hw_sim = eval_sim_harmonics(t_nondim, hind.psi_mean, hind.psi_amp, hind.psi_phase)

    # Convert back to paper notation so the overlay remains in the experimental plotting convention.
    phi_transform = AZUMA_ADAPTER.sim_transforms["phi"]
    psi_transform = AZUMA_ADAPTER.sim_transforms["psi"]
    phi_fw_deg = phi_transform.inverse_value_deg(np.degrees(phi_fw_sim))
    phi_hw_deg = phi_transform.inverse_value_deg(np.degrees(phi_hw_sim))
    psi_fw_deg = psi_transform.inverse_value_deg(np.degrees(psi_fw_sim))
    psi_hw_deg = psi_transform.inverse_value_deg(np.degrees(psi_hw_sim))

    s_fw = R_FW_MM * np.sin(np.radians(phi_fw_deg))
    s_hw = R_HW_MM * np.sin(np.radians(phi_hw_deg))
    beta_fw = np.radians(psi_fw_deg)
    beta_hw = np.radians(psi_hw_deg)
    return s_fw, s_hw, beta_fw, beta_hw


def wing_endpoints(
    s: float,
    d: float,
    beta: float,
    gamma: float,
    chord_mm: float,
    offset: tuple[float, float],
) -> tuple[np.ndarray, np.ndarray]:
    """Compute leading/trailing edge in lab frame (x,z), Wang-style."""
    e_stroke = np.array([-np.cos(gamma), np.sin(gamma)])
    e_normal = np.array([np.sin(gamma), np.cos(gamma)])
    leading = s * e_stroke + d * e_normal + np.array(offset)
    chord_dir = np.cos(beta) * e_stroke + np.sin(beta) * e_normal
    trailing = leading - chord_mm * chord_dir
    return leading, trailing


def theme_colors(theme: str) -> dict[str, object]:
    if theme == "dark":
        return {
            "figure_face": "#111111",
            "axes_face": "#111111",
            "text": "#f0f0f0",
            "exp": "#9a9a9a",
            "model": "#f2f2f2",
            "stroke": "#b7b7b7",
        }
    return {
        "figure_face": "#ffffff",
        "axes_face": "#ffffff",
        "text": "#111111",
        "exp": "grey",
        "model": "black",
        "stroke": "#666666",
    }


def build_animation(
    output: Path,
    theme: str,
    n_frames: int,
    n_wingbeats: int,
    fps: int,
    dpi: int,
) -> None:
    colors = theme_colors(theme)
    gamma_fore = np.radians(GAMMA_FW_DEG)
    gamma_hind = np.radians(GAMMA_HW_DEG)
    t = np.linspace(0.0, float(n_wingbeats), n_frames, endpoint=False)
    sf_exp, sh_exp, bf_exp, bh_exp = experimental_kinematics(t)
    sf_mod, sh_mod, bf_mod, bh_mod = model_kinematics(t)

    chord_ref = max(CHORD_FW_MM, CHORD_HW_MM)
    fore_offset = (1.5 * chord_ref, 0.0)
    hind_offset = (-1.5 * chord_ref, 0.0)

    fig, ax = plt.subplots(figsize=figure_size(3.8 / 4.0))
    fig.patch.set_facecolor(colors["figure_face"])
    ax.set_facecolor(colors["axes_face"])
    ax.set_aspect("equal")
    ax.set_xlabel("x (mm)", color=colors["text"])
    ax.set_ylabel("z (mm)", color=colors["text"])
    ax.tick_params(colors=colors["text"])
    for spine in ax.spines.values():
        spine.set_color(colors["text"])

    sp_len = 45.0
    for gamma, off in ((gamma_fore, fore_offset), (gamma_hind, hind_offset)):
        xs = np.array([-sp_len, sp_len]) * (-np.cos(gamma)) + off[0]
        zs = np.array([-sp_len, sp_len]) * np.sin(gamma) + off[1]
        ax.plot(xs, zs, "-", color=colors["stroke"], linewidth=0.6, alpha=0.4)

    le_style_exp = dict(
        marker="o",
        markersize=4,
        markerfacecolor=colors["axes_face"],
        markeredgecolor=colors["exp"],
        markeredgewidth=2,
        linestyle="none",
    )
    le_style_mod = dict(
        marker="o",
        markersize=4,
        markerfacecolor=colors["axes_face"],
        markeredgecolor=colors["model"],
        markeredgewidth=2,
        linestyle="none",
    )

    fore_line, = ax.plot([], [], ":", color=colors["exp"], linewidth=2, label="Experiment")
    hind_line, = ax.plot([], [], ":", color=colors["exp"], linewidth=2)
    fore_dot_exp, = ax.plot([], [], **le_style_exp)
    hind_dot_exp, = ax.plot([], [], **le_style_exp)
    fore_line_m, = ax.plot([], [], "-", color=colors["model"], linewidth=2, label="Model")
    hind_line_m, = ax.plot([], [], "-", color=colors["model"], linewidth=2)
    fore_dot_m, = ax.plot([], [], **le_style_mod)
    hind_dot_m, = ax.plot([], [], **le_style_mod)

    ax.legend(loc="upper right", fontsize=10, facecolor=colors["axes_face"], edgecolor=colors["text"])
    ax.set_xlim(-40.0, 40.0)
    ax.set_ylim(-40.0, 40.0)
    time_text = ax.text(0.02, 0.98, "", transform=ax.transAxes, va="top", ha="left", color=colors["text"])

    def update(i: int):
        # Experimental traces
        le_f, te_f = wing_endpoints(sf_exp[i], 0.0, bf_exp[i], gamma_fore, CHORD_FW_MM, fore_offset)
        le_h, te_h = wing_endpoints(sh_exp[i], 0.0, bh_exp[i], gamma_hind, CHORD_HW_MM, hind_offset)
        # Model traces
        le_fm, te_fm = wing_endpoints(sf_mod[i], 0.0, bf_mod[i], gamma_fore, CHORD_FW_MM, fore_offset)
        le_hm, te_hm = wing_endpoints(sh_mod[i], 0.0, bh_mod[i], gamma_hind, CHORD_HW_MM, hind_offset)

        fore_line.set_data([le_f[0], te_f[0]], [le_f[1], te_f[1]])
        hind_line.set_data([le_h[0], te_h[0]], [le_h[1], te_h[1]])
        fore_dot_exp.set_data([le_f[0]], [le_f[1]])
        hind_dot_exp.set_data([le_h[0]], [le_h[1]])

        fore_line_m.set_data([le_fm[0], te_fm[0]], [le_fm[1], te_fm[1]])
        hind_line_m.set_data([le_hm[0], te_hm[0]], [le_hm[1], te_hm[1]])
        fore_dot_m.set_data([le_fm[0]], [le_fm[1]])
        hind_dot_m.set_data([le_hm[0]], [le_hm[1]])

        time_text.set_text(f"$t/T_{{wb}}$ = {t[i] % 1.0:.2f}")
        return (
            fore_line,
            hind_line,
            fore_dot_exp,
            hind_dot_exp,
            fore_line_m,
            hind_line_m,
            fore_dot_m,
            hind_dot_m,
            time_text,
        )

    fig.tight_layout()
    anim = FuncAnimation(fig, update, frames=n_frames, interval=1000.0 / float(fps), blit=False)
    output.parent.mkdir(parents=True, exist_ok=True)
    anim.save(str(output), writer="ffmpeg", fps=fps, dpi=dpi)
    plt.close(fig)
    print(f"Saved {output} ({n_frames} frames)")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", default="stick_two_wing.mp4", help="Output animation path.")
    parser.add_argument("--theme", choices=["light", "dark"], default="light", help="Theme.")
    parser.add_argument("--frames", type=int, default=500, help="Number of animation frames.")
    parser.add_argument("--wingbeats", type=int, default=1, help="Wingbeats to animate.")
    parser.add_argument("--fps", type=int, default=30, help="Video framerate.")
    parser.add_argument("--dpi", type=int, default=300, help="Video DPI.")
    args = parser.parse_args()

    if args.frames < 2:
        raise ValueError("--frames must be >= 2")
    if args.wingbeats < 1:
        raise ValueError("--wingbeats must be >= 1")
    if args.fps < 1:
        raise ValueError("--fps must be >= 1")
    if args.dpi < 50:
        raise ValueError("--dpi must be >= 50")

    build_animation(
        output=Path(args.output),
        theme=args.theme,
        n_frames=args.frames,
        n_wingbeats=args.wingbeats,
        fps=args.fps,
        dpi=args.dpi,
    )


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
Plot pitch-angle contours in the stroke plane over one half-wingbeat.

View is normal to the stroke plane. The swept area of the wing is drawn as a
fan-shaped region and contoured by pitch angle, showing how pitch varies along
the span and over the stroke. Expects a simulation file exactly one wingbeat
long.

Default mode ('both') shows upstroke and downstroke side by side as a split
fan: the downstroke occupies the right half and the upstroke occupies the left
half, meeting at the stroke center.

Usage:
    python -m post.plot_stroke_pitch <input.h5> <output.png>
        --wing fore_right [--stroke both|downstroke|upstroke]
        [--n-eta N] [--theme light|dark]
"""

from __future__ import annotations

import argparse
from pathlib import Path

import h5py
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import FancyArrowPatch

from post.io import decode_string_array, read_simulation
from post.kinematics import eval_series, eval_series_dot, read_angle_params
from post.stroke_common import fan_grid_from_phi, resample_field, select_half
from post.style import apply_matplotlib_style, figure_size, resolve_style


# ---------------------------------------------------------------------------
# Kinematic angle computation
# ---------------------------------------------------------------------------

def _compute_angles(time, wing_name, params, angle_params):
    """
    Compute phi, phi_dot, psi, phase arrays for a wing over time.
    """
    omega     = float(params['wing_omega'][wing_name])
    period    = float(params.get('wing_harmonic_period_wingbeats', {}).get(wing_name, 1.0))
    phase_off = float(params.get('wing_phase_offset', {}).get(wing_name, 0.0))
    basis_omega = omega / period
    phase = basis_omega * time + phase_off

    phi = float(angle_params['phi_mean'][wing_name]) + eval_series(
        angle_params['phi_amp'][wing_name], angle_params['phi_phase'][wing_name], phase)
    phi_dot = eval_series_dot(
        angle_params['phi_amp'][wing_name], angle_params['phi_phase'][wing_name], basis_omega, phase)

    psi = float(angle_params['psi_mean'][wing_name]) + eval_series(
        angle_params['psi_amp'][wing_name], angle_params['psi_phase'][wing_name], phase)

    return phi, phi_dot, psi, phase


# ---------------------------------------------------------------------------
# Pitch computation
# ---------------------------------------------------------------------------

def _pitch_grid(psi, phase, psi_amp, psi_phase, has_twist, twist_root, twist_ref_eta, eta):
    """
    Compute pitch angle (degrees) over (time, span), including optional twist.

    Matches wing.cpp twist logic:
      psi_eta = psi + (scale(eta) - 1) * psi_h1_value
    """
    psi = np.asarray(psi, dtype=float)
    phase = np.asarray(phase, dtype=float)
    eta = np.asarray(eta, dtype=float)

    pitch = np.repeat(np.degrees(psi)[:, None], len(eta), axis=1)
    if not has_twist:
        return pitch
    if len(psi_amp) == 0 or len(psi_phase) == 0:
        return pitch
    if not np.isfinite(twist_ref_eta) or twist_ref_eta <= 0.0:
        return pitch

    amp1 = float(psi_amp[0])
    phase1 = float(psi_phase[0])
    ref_coeff = abs(amp1)
    if ref_coeff <= 1e-12:
        return pitch

    # First-harmonic pitch component at reference station.
    psi_h1 = amp1 * np.cos(phase + phase1)
    # Spanwise scale from wing.cpp buildTwistH1Scales.
    coeff_eta = ((ref_coeff - twist_root) * (eta / twist_ref_eta)) + twist_root
    scale_eta = coeff_eta / ref_coeff
    psi_eta = psi[:, None] + (scale_eta[None, :] - 1.0) * psi_h1[:, None]
    return np.degrees(psi_eta)


def plot_stroke_pitch(
    output_path: Path,
    filename: str,
    wing_name: str,
    stroke: str = 'both',
    n_eta: int = 50,
    theme: str | None = None,
) -> None:
    """
    Create and save the stroke-plane pitch contour plot.

    stroke='both'      : split fan — downstroke right, upstroke left (default)
    stroke='downstroke': single fan showing only the downstroke
    stroke='upstroke'  : single fan showing only the upstroke
    """
    params, time, _, wings = read_simulation(filename)

    with h5py.File(filename, 'r') as f:
        wing_names_param = decode_string_array(f['/parameters/wings/names'][:])

    if wing_name not in wings:
        raise ValueError(f"Wing '{wing_name}' not found. Available: {sorted(wings.keys())}")

    angle_params = read_angle_params(filename, wing_names_param, ('phi', 'psi'))
    lb0 = float(params['wing_lb0'][wing_name])
    phi, phi_dot, psi, phase = _compute_angles(time, wing_name, params, angle_params)
    phi_mean = float(angle_params['phi_mean'][wing_name])
    has_twist = bool(params.get('wing_has_psi_twist_h1', {}).get(wing_name, 0))
    twist_root = float(params.get('wing_psi_twist_h1_root', {}).get(wing_name, 0.0))
    twist_ref_eta = float(params.get('wing_psi_twist_ref_eta', {}).get(wing_name, 0.75))
    psi_amp = np.asarray(angle_params['psi_amp'][wing_name], dtype=float)
    psi_phase = np.asarray(angle_params['psi_phase'][wing_name], dtype=float)

    # Span stations including the exact root so both fans meet at a center tip.
    eta = np.linspace(0.0, 1.0, n_eta + 1)

    style = resolve_style(theme=theme)
    apply_matplotlib_style(style)

    if stroke == 'both':
        _plot_combined(
            output_path, phi, phi_dot, psi, phase, psi_amp, psi_phase,
            phi_mean, lb0, eta, style, has_twist, twist_root, twist_ref_eta,
        )
    else:
        _plot_single(
            output_path, phi, phi_dot, psi, phase, psi_amp, psi_phase,
            stroke, lb0, eta, style, has_twist, twist_root, twist_ref_eta,
        )


def _plot_combined(
    output_path, phi, phi_dot, psi, phase, psi_amp, psi_phase,
    phi_mean, lb0, eta, style, has_twist, twist_root, twist_ref_eta,
):
    """Split-fan plot: downstroke right, upstroke left, no frame or axes."""
    # --- select half-strokes ---
    idx_down = select_half(phi, phi_dot, 'right')
    idx_up   = select_half(phi, phi_dot, 'left')
    if len(idx_down) == 0 or len(idx_up) == 0:
        raise ValueError("Both downstroke and upstroke timesteps are required for stroke='both'.")

    phi_d = phi[idx_down]
    phi_u = phi[idx_up]
    pitch_down_raw = _pitch_grid(
        psi[idx_down], phase[idx_down], psi_amp, psi_phase,
        has_twist, twist_root, twist_ref_eta, eta,
    )
    pitch_up_raw = _pitch_grid(
        psi[idx_up], phase[idx_up], psi_amp, psi_phase,
        has_twist, twist_root, twist_ref_eta, eta,
    )
    phi_d, pitch_down = resample_field(phi_d, pitch_down_raw)
    phi_u, pitch_up = resample_field(phi_u, pitch_up_raw)

    # Down fan on the right, up fan mirrored to the left.
    X_down, Y_down = fan_grid_from_phi(phi_d, phi_mean, eta, lb0, side='right')
    X_up,   Y_up   = fan_grid_from_phi(phi_u, phi_mean, eta, lb0, side='left')

    # Shared contour levels across both fans
    all_pitch = np.concatenate([pitch_down.ravel(), pitch_up.ravel()])
    finite = all_pitch[np.isfinite(all_pitch)]
    if finite.size == 0:
        raise ValueError("Computed pitch contains no finite values.")
    step = 5.0
    abs_lim = np.nanpercentile(np.abs(finite), 90)
    hi = step * np.ceil(abs_lim / step)
    lo = -hi
    if not np.isfinite(lo) or not np.isfinite(hi):
        raise ValueError("Failed to compute finite pitch contour levels.")
    if hi <= 0.0:
        hi = step
        lo = -step
    levels = np.arange(lo, hi + 0.5 * step, step)
    label_levels = levels[::2]

    fig, ax = plt.subplots(figsize=figure_size(height_over_width=0.45))
    fig.patch.set_facecolor('white')
    ax.set_facecolor('white')

    lc = style.text_color
    lw_contour = 1.6
    lw_silhouette = 1
    lw_arrow = 1
    ct_down = ax.contour(
        X_down, Y_down, pitch_down,
        levels=levels, colors=lc, linewidths=lw_contour, linestyles='solid',
    )
    ct_up = ax.contour(
        X_up, Y_up, pitch_up,
        levels=levels, colors=lc, linewidths=lw_contour, linestyles='solid',
    )
    ax.clabel(
        ct_down, levels=label_levels, fmt='%.0f', fontsize=9, inline=True,
        rightside_up=True, use_clabeltext=True,
    )
    ax.clabel(
        ct_up, levels=label_levels, fmt='%.0f', fontsize=9, inline=True,
        rightside_up=True, use_clabeltext=True,
    )

    for X, Y in ((X_down, Y_down), (X_up, Y_up)):
        ax.plot(X[0, :], Y[0, :], color=lc, lw=lw_silhouette)
        ax.plot(X[-1, :], Y[-1, :], color=lc, lw=lw_silhouette)
        ax.plot(X[:, -1], Y[:, -1], color=lc, lw=lw_silhouette)

    ax.set_aspect('equal')
    ax.set_axis_off()

    x_max = np.nanmax(np.abs(np.concatenate([X_down.ravel(), X_up.ravel()])))
    y_max = np.nanmax(np.abs(np.concatenate([Y_down.ravel(), Y_up.ravel()])))
    x_pad = 0.35 * x_max
    y_pad = 0.15 * y_max
    ax.set_xlim(-x_max - x_pad, x_max + x_pad)
    ax.set_ylim(-y_max - y_pad, y_max + y_pad)

    # Draw concentric direction arrows with radius set slightly outside the fan.
    r_wing = float(lb0 * eta[-1])
    r_arrow = 1.06 * r_wing
    theta_span = np.degrees(np.arcsin(np.clip(0.78 * y_max / max(r_arrow, 1e-12), 0.0, 1.0)))
    theta_span *= 0.5

    def _draw_circular_arrow(theta_start_deg, theta_end_deg):
        theta = np.deg2rad(np.linspace(theta_start_deg, theta_end_deg, 120))
        x = r_arrow * np.cos(theta)
        y = r_arrow * np.sin(theta)
        body_end = -5
        ax.plot(x[:body_end], y[:body_end], color=lc, lw=lw_arrow, solid_capstyle='butt')
        head = FancyArrowPatch(
            (x[body_end], y[body_end]), (x[-1], y[-1]),
            arrowstyle='Simple,head_length=0.45,head_width=0.225,tail_width=0.05',
            mutation_scale=14,
            fc=lc,
            ec=lc,
            lw=0.0,
        )
        ax.add_patch(head)

    # Up arrow (left): bottom -> top.
    _draw_circular_arrow(180.0 + theta_span, 180.0 - theta_span)
    # Down arrow (right): top -> bottom.
    _draw_circular_arrow(theta_span, -theta_span)

    ax.text(-1.1 * r_wing, 0.02 * y_max, 'Up', color=lc, ha='right', va='center')
    ax.text(1.1 * r_wing, 0.02 * y_max, 'Down', color=lc, ha='left', va='center')

    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(str(output_path), dpi=300, bbox_inches='tight')
    plt.close(fig)


def _plot_single(
    output_path, phi, phi_dot, psi, phase, psi_amp, psi_phase,
    stroke, lb0, eta, style, has_twist, twist_root, twist_ref_eta,
):
    """Single half-stroke fan plot (downstroke or upstroke)."""
    mask = (phi_dot < 0) if stroke == 'downstroke' else (phi_dot >= 0)
    if not np.any(mask):
        raise ValueError(
            f"No timesteps found for stroke='{stroke}'. "
            "Ensure the simulation has oscillatory phi kinematics."
        )
    idx = np.where(mask)[0]
    phi_sel = phi[idx]
    sort_order = np.argsort(phi_sel)
    idx = idx[sort_order]
    phi_sel = phi_sel[sort_order]

    pitch_raw = _pitch_grid(
        psi[idx], phase[idx], psi_amp, psi_phase,
        has_twist, twist_root, twist_ref_eta, eta,
    )
    phi_sel, pitch = resample_field(phi_sel, pitch_raw)
    side = 'right' if stroke == 'downstroke' else 'left'
    X, Y = fan_grid_from_phi(phi_sel, np.mean(phi_sel), eta, lb0, side=side)

    finite = pitch[np.isfinite(pitch)]
    if finite.size == 0:
        raise ValueError("Computed pitch contains no finite values.")
    step = 5.0
    abs_lim = np.nanpercentile(np.abs(finite), 90)
    hi = step * np.ceil(abs_lim / step)
    lo = -hi
    if hi <= 0.0:
        hi = step
        lo = -step
    levels = np.arange(lo, hi + 0.5 * step, step)
    label_levels = levels[::2]

    fig, ax = plt.subplots(figsize=figure_size(height_over_width=0.9))
    fig.patch.set_facecolor('white')
    ax.set_facecolor('white')

    lc = style.text_color
    lw_contour = 1.6
    lw_silhouette = 1.2
    ct = ax.contour(X, Y, pitch, levels=levels, colors=lc, linewidths=lw_contour, linestyles='solid')
    ax.clabel(
        ct, levels=label_levels, fmt='%.0f', fontsize=9, inline=True,
        rightside_up=True, use_clabeltext=True,
    )

    ax.plot(X[:, -1], Y[:, -1], color=lc, linewidth=lw_silhouette)
    ax.plot(X[0, :], Y[0, :], color=lc, linewidth=lw_silhouette)
    ax.plot(X[-1, :], Y[-1, :], color=lc, linewidth=lw_silhouette)
    ax.set_aspect('equal')
    ax.set_axis_off()

    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(str(output_path), dpi=300, bbox_inches='tight')
    plt.close(fig)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument('input',  help='Input HDF5 file (one wingbeat)')
    parser.add_argument('output', help='Output image path (e.g. pitch_stroke.png)')
    parser.add_argument('--wing', required=True,
                        help='Wing name (e.g. fore_right, hind_right)')
    parser.add_argument('--stroke', choices=['both', 'downstroke', 'upstroke'], default='both',
                        help='Which stroke(s) to plot (default: both)')
    parser.add_argument('--n-eta', type=int, default=50,
                        help='Number of span stations for pitch grid (default: 50)')
    parser.add_argument('--theme', choices=['light', 'dark'], default='light')

    args = parser.parse_args()
    print(f"Reading {args.input}...")
    plot_stroke_pitch(
        Path(args.output), args.input, args.wing,
        stroke=args.stroke, n_eta=args.n_eta, theme=args.theme,
    )
    print(f"Saved: {args.output}")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

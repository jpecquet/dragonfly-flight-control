#!/usr/bin/env python3
"""
Plot angle-of-attack contours in the stroke plane over one half-wingbeat.

View is normal to the stroke plane. The swept area of the wing is drawn as a
fan-shaped region and filled with AoA contours, showing how angle of attack
varies along the span and over the stroke. Expects a simulation file exactly
one wingbeat long.

Default mode ('both') shows upstroke and downstroke side by side as a split
fan: the downstroke occupies the right half (phi >= phi_mean) and the upstroke
occupies the left half (phi <= phi_mean), meeting at the mid-stroke position.

Usage:
    python -m post.plot_stroke_aoa <input.h5> <output.png>
        --wing forewing_right [--stroke both|downstroke|upstroke]
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
from post.style import apply_matplotlib_style, figure_size, resolve_style


# ---------------------------------------------------------------------------
# Fourier series helpers (match evaluateHarmonicValue / evaluateHarmonicRate
# in include/kinematics.hpp)
# ---------------------------------------------------------------------------

def _eval_series(cos_c, sin_c, phase):
    """Evaluate sum_k [ c_k*cos(k*phase) + s_k*sin(k*phase) ] over a phase array."""
    val = np.zeros_like(phase)
    for k, (c, s) in enumerate(zip(cos_c, sin_c), 1):
        arg = k * phase
        val += c * np.cos(arg) + s * np.sin(arg)
    return val


def _eval_series_dot(cos_c, sin_c, basis_omega, phase):
    """Time derivative of _eval_series (via chain rule with d_phase/dt = basis_omega)."""
    val = np.zeros_like(phase)
    for k, (c, s) in enumerate(zip(cos_c, sin_c), 1):
        arg = k * phase
        val += k * basis_omega * (-c * np.sin(arg) + s * np.cos(arg))
    return val


# ---------------------------------------------------------------------------
# HDF5 reading
# ---------------------------------------------------------------------------

def _read_angle_params(filename, wing_names_param):
    """Read per-wing phi and gamma Fourier parameters from the HDF5 file."""
    angles = {}
    with h5py.File(filename, 'r') as f:
        for angle in ('phi', 'gamma'):
            mean_arr = f[f'/parameters/wings/{angle}_mean'][:]
            cos_arr  = f[f'/parameters/wings/{angle}_cos'][:]
            sin_arr  = f[f'/parameters/wings/{angle}_sin'][:]
            angles[f'{angle}_mean'] = dict(zip(wing_names_param, mean_arr))
            angles[f'{angle}_cos']  = {
                name: np.asarray(cos_arr[i], dtype=float)
                for i, name in enumerate(wing_names_param)
            }
            angles[f'{angle}_sin']  = {
                name: np.asarray(sin_arr[i], dtype=float)
                for i, name in enumerate(wing_names_param)
            }
    return angles


# ---------------------------------------------------------------------------
# Kinematic angle computation
# ---------------------------------------------------------------------------

def _compute_angles(time, wing_name, params, angle_params):
    """
    Compute phi, phi_dot, gam_raw, gam_raw_dot arrays for a wing over time.

    'gam_raw' is the raw Fourier series value. The simulation uses
    gam = pi - gam_raw and gam_dot = -gam_raw_dot (see wing.cpp).
    """
    omega     = float(params['wing_omega'][wing_name])
    period    = float(params.get('wing_harmonic_period_wingbeats', {}).get(wing_name, 1.0))
    phase_off = float(params.get('wing_phase_offset', {}).get(wing_name, 0.0))
    basis_omega = omega / period
    phase = basis_omega * time + phase_off

    phi = float(angle_params['phi_mean'][wing_name]) + _eval_series(
        angle_params['phi_cos'][wing_name], angle_params['phi_sin'][wing_name], phase)
    phi_dot = _eval_series_dot(
        angle_params['phi_cos'][wing_name], angle_params['phi_sin'][wing_name], basis_omega, phase)

    gam_raw = float(angle_params['gamma_mean'][wing_name]) + _eval_series(
        angle_params['gamma_cos'][wing_name], angle_params['gamma_sin'][wing_name], phase)
    gam_raw_dot = _eval_series_dot(
        angle_params['gamma_cos'][wing_name], angle_params['gamma_sin'][wing_name], basis_omega, phase)

    return phi, phi_dot, gam_raw, gam_raw_dot


# ---------------------------------------------------------------------------
# AoA computation
# ---------------------------------------------------------------------------

def _aoa_grid(e_r, e_s, e_c, states, phi_dot, gam_raw_dot, lb0, is_left, eta):
    """
    Compute angle of attack (degrees) at each (time step, span station) point.

    Reproduces the velocity and AoA computation from wing.cpp / blade_element.cpp:
      uw = ub + v_phi + v_gam
      u  = uw projected onto the plane normal to the span (e_r)
      alpha = atan2(u x e_c . e_r,  u . e_c)

    Parameters
    ----------
    e_r, e_s, e_c  : (N, 3) wing orientation vectors from HDF5
    states         : (N, 6) simulation state [pos, vel]
    phi_dot        : (N,) stroke angular velocity
    gam_raw_dot    : (N,) raw gamma rate; gam_dot = -gam_raw_dot in simulation
    lb0            : wing span (m)
    is_left        : True for left wing
    eta            : (M,) normalized span stations in (0, 1]

    Returns
    -------
    aoa : (N, M) angle of attack in degrees
    """
    N = len(phi_dot)
    M = len(eta)
    aoa = np.zeros((N, M))

    ey = np.array([0.0, 1.0, 0.0])
    gam_sign = -1.0 if is_left else 1.0

    for i in range(N):
        er = e_r[i]
        ec = e_c[i]
        es = e_s[i]
        ub = states[i, 3:6]

        # wing.cpp: gam_dot = -gam_raw_dot; omega_gam = gam_sign * gam_dot * ey
        omega_gam = gam_sign * (-gam_raw_dot[i]) * ey

        for j, eta_j in enumerate(eta):
            r     = eta_j * lb0
            v_phi = r * phi_dot[i] * es
            v_gam = np.cross(omega_gam, r * er)
            uw    = ub + v_phi + v_gam

            # Project velocity onto the plane perpendicular to span
            u = uw - np.dot(uw, er) * er

            U_sq = np.dot(u, u)
            if U_sq < 1e-20:
                continue
            U_inv    = 1.0 / np.sqrt(U_sq)
            c_alpha  = np.dot(u, ec) * U_inv
            s_alpha  = np.dot(np.cross(u, ec), er) * U_inv
            aoa[i, j] = np.degrees(np.arctan2(s_alpha, c_alpha))

    return aoa


def _fan_grid_from_phi(phi_sel, phi_mean, eta, lb0, side):
    """
    Build fan coordinates from stroke angle for a side-oriented split fan.

    side='right' maps contours to the right fan (downstroke).
    side='left'  mirrors contours to the left fan (upstroke).
    """
    theta = np.asarray(phi_sel, dtype=float) - float(phi_mean)
    r = eta * lb0
    x = np.cos(theta)
    y = np.sin(theta)
    if side == 'left':
        x = -x
    X = np.outer(x, r)
    Y = np.outer(y, r)
    return X, Y


def _smooth_along_axis(arr, window, axis):
    """Edge-preserving moving average along one axis."""
    if window <= 1:
        return arr
    pad = window // 2
    pads = [(0, 0)] * arr.ndim
    pads[axis] = (pad, pad)
    padded = np.pad(arr, pads, mode='edge')
    kernel = np.ones(window, dtype=float) / float(window)
    return np.apply_along_axis(lambda x: np.convolve(x, kernel, mode='valid'), axis, padded)


def _resample_aoa(phi_sel, aoa, n_phi=72, smooth_phi=13, smooth_eta=9):
    """
    Interpolate AoA onto a uniform stroke-angle grid and smooth for contours.
    """
    phi_sel = np.asarray(phi_sel, dtype=float)
    aoa = np.asarray(aoa, dtype=float)

    order = np.argsort(phi_sel)
    phi_sorted = phi_sel[order]
    aoa_sorted = aoa[order]

    phi_unique, unique_idx = np.unique(phi_sorted, return_index=True)
    aoa_unique = aoa_sorted[unique_idx]
    if len(phi_unique) < 2:
        return phi_unique, aoa_unique

    n_phi = max(16, int(n_phi))
    phi_target = np.linspace(phi_unique[0], phi_unique[-1], n_phi)
    aoa_target = np.empty((n_phi, aoa_unique.shape[1]), dtype=float)

    for j in range(aoa_unique.shape[1]):
        col = aoa_unique[:, j]
        valid = np.isfinite(col)
        if np.count_nonzero(valid) >= 2:
            aoa_target[:, j] = np.interp(phi_target, phi_unique[valid], col[valid])
        elif np.count_nonzero(valid) == 1:
            aoa_target[:, j] = col[valid][0]
        else:
            aoa_target[:, j] = 0.0

    aoa_target = _smooth_along_axis(aoa_target, window=smooth_phi, axis=0)
    aoa_target = _smooth_along_axis(aoa_target, window=smooth_eta, axis=1)
    return phi_target, aoa_target


# ---------------------------------------------------------------------------
# Main plotting function
# ---------------------------------------------------------------------------

def plot_stroke_aoa(
    output_path: Path,
    filename: str,
    wing_name: str,
    stroke: str = 'both',
    n_eta: int = 50,
    theme: str | None = None,
) -> None:
    """
    Create and save the stroke-plane AoA contour plot.

    stroke='both'      : split fan — downstroke right, upstroke left (default)
    stroke='downstroke': single fan showing only the downstroke
    stroke='upstroke'  : single fan showing only the upstroke
    """
    params, time, states, wings = read_simulation(filename)

    with h5py.File(filename, 'r') as f:
        wing_names_param = decode_string_array(f['/parameters/wings/names'][:])

    if wing_name not in wings:
        raise ValueError(f"Wing '{wing_name}' not found. Available: {sorted(wings.keys())}")

    angle_params = _read_angle_params(filename, wing_names_param)
    lb0     = float(params['wing_lb0'][wing_name])
    is_left = '_left' in wing_name

    phi, phi_dot, _, gam_raw_dot = _compute_angles(time, wing_name, params, angle_params)
    phi_mean = float(angle_params['phi_mean'][wing_name])

    # Span stations including the exact root so both fans meet at a center tip.
    eta = np.linspace(0.0, 1.0, n_eta + 1)

    style = resolve_style(theme=theme)
    apply_matplotlib_style(style)

    if stroke == 'both':
        _plot_combined(
            output_path, wing_name, wings, states,
            phi, phi_dot, gam_raw_dot, phi_mean,
            lb0, is_left, eta, style,
        )
    else:
        _plot_single(
            output_path, wing_name, wings, states,
            phi, phi_dot, gam_raw_dot, stroke,
            lb0, is_left, eta, style,
        )


def _select_half(phi, phi_dot, side):
    """
    Select and sort timestep indices for one half of the stroke.

    side='right': downstroke (phi_dot < 0)
    side='left' : upstroke  (phi_dot >= 0)
    """
    if side == 'right':
        mask = (phi_dot < 0)
    else:
        mask = (phi_dot >= 0)
    idx = np.where(mask)[0]
    idx = idx[np.argsort(phi[idx])]
    return idx


def _plot_combined(
    output_path, wing_name, wings, states,
    phi, phi_dot, gam_raw_dot, phi_mean,
    lb0, is_left, eta, style,
):
    """Split-fan plot: downstroke right, upstroke left, no frame or axes."""
    # --- select half-strokes ---
    idx_down = _select_half(phi, phi_dot, 'right')
    idx_up   = _select_half(phi, phi_dot, 'left')
    if len(idx_down) == 0 or len(idx_up) == 0:
        raise ValueError("Both downstroke and upstroke timesteps are required for stroke='both'.")

    def _extract(idx):
        return (
            wings[wing_name]['e_r'][idx],
            wings[wing_name]['e_s'][idx],
            wings[wing_name]['e_c'][idx],
            states[idx],
            phi_dot[idx],
            gam_raw_dot[idx],
            phi[idx],
        )

    er_d, es_d, ec_d, st_d, pd_d, gd_d, phi_d = _extract(idx_down)
    er_u, es_u, ec_u, st_u, pd_u, gd_u, phi_u = _extract(idx_up)

    aoa_down_raw = _aoa_grid(er_d, es_d, ec_d, st_d, pd_d, gd_d, lb0, is_left, eta)
    aoa_up_raw   = _aoa_grid(er_u, es_u, ec_u, st_u, pd_u, gd_u, lb0, is_left, eta)
    phi_d, aoa_down = _resample_aoa(phi_d, aoa_down_raw)
    phi_u, aoa_up = _resample_aoa(phi_u, aoa_up_raw)

    # Down fan on the right, up fan mirrored to the left.
    X_down, Y_down = _fan_grid_from_phi(phi_d, phi_mean, eta, lb0, side='right')
    X_up,   Y_up   = _fan_grid_from_phi(phi_u, phi_mean, eta, lb0, side='left')

    # Shared contour levels across both fans
    all_aoa = np.concatenate([aoa_down.ravel(), aoa_up.ravel()])
    finite = all_aoa[np.isfinite(all_aoa)]
    if finite.size == 0:
        raise ValueError("Computed AoA contains no finite values.")
    step = 5.0
    abs_lim = np.nanpercentile(np.abs(finite), 90)
    hi = step * np.ceil(abs_lim / step)
    lo = -hi
    if not np.isfinite(lo) or not np.isfinite(hi):
        raise ValueError("Failed to compute finite AoA contour levels.")
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
        X_down, Y_down, aoa_down,
        levels=levels, colors=lc, linewidths=lw_contour, linestyles='solid',
    )
    ct_up = ax.contour(
        X_up, Y_up, aoa_up,
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
    output_path, wing_name, wings, states,
    phi, phi_dot, gam_raw_dot, stroke,
    lb0, is_left, eta, style,
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

    e_r_s    = wings[wing_name]['e_r'][idx]
    e_s_s    = wings[wing_name]['e_s'][idx]
    e_c_s    = wings[wing_name]['e_c'][idx]
    states_s = states[idx]
    pd_s     = phi_dot[idx]
    gd_s     = gam_raw_dot[idx]

    aoa_raw = _aoa_grid(e_r_s, e_s_s, e_c_s, states_s, pd_s, gd_s, lb0, is_left, eta)
    phi_sel, aoa = _resample_aoa(phi_sel, aoa_raw)
    side = 'right' if stroke == 'downstroke' else 'left'
    X, Y = _fan_grid_from_phi(phi_sel, np.mean(phi_sel), eta, lb0, side=side)

    finite = aoa[np.isfinite(aoa)]
    if finite.size == 0:
        raise ValueError("Computed AoA contains no finite values.")
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
    ct = ax.contour(X, Y, aoa, levels=levels, colors=lc, linewidths=lw_contour, linestyles='solid')
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
    parser.add_argument('output', help='Output image path (e.g. aoa_stroke.png)')
    parser.add_argument('--wing', required=True,
                        help='Wing name (e.g. forewing_right, hindwing_left)')
    parser.add_argument('--stroke', choices=['both', 'downstroke', 'upstroke'], default='both',
                        help='Which stroke(s) to plot (default: both)')
    parser.add_argument('--n-eta', type=int, default=50,
                        help='Number of span stations for AoA grid (default: 50)')
    parser.add_argument('--theme', choices=['light', 'dark'], default='light')

    args = parser.parse_args()
    print(f"Reading {args.input}...")
    plot_stroke_aoa(
        Path(args.output), args.input, args.wing,
        stroke=args.stroke, n_eta=args.n_eta, theme=args.theme,
    )
    print(f"Saved: {args.output}")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

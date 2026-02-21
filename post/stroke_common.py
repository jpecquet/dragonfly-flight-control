"""
Shared helpers for stroke-plane contour plots.
"""

from __future__ import annotations

import numpy as np


def fan_grid_from_phi(phi_sel, phi_mean, eta, lb0, side):
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


def smooth_along_axis(arr, window, axis):
    """Edge-preserving moving average along one axis."""
    if window <= 1:
        return arr
    pad = window // 2
    pads = [(0, 0)] * arr.ndim
    pads[axis] = (pad, pad)
    padded = np.pad(arr, pads, mode='edge')
    kernel = np.ones(window, dtype=float) / float(window)
    return np.apply_along_axis(lambda x: np.convolve(x, kernel, mode='valid'), axis, padded)


def resample_field(phi_sel, field, n_phi=72, smooth_phi=13, smooth_eta=9):
    """Interpolate a contour field onto a uniform stroke-angle grid and smooth it."""
    phi_sel = np.asarray(phi_sel, dtype=float)
    field = np.asarray(field, dtype=float)

    order = np.argsort(phi_sel)
    phi_sorted = phi_sel[order]
    field_sorted = field[order]

    phi_unique, unique_idx = np.unique(phi_sorted, return_index=True)
    field_unique = field_sorted[unique_idx]
    if len(phi_unique) < 2:
        return phi_unique, field_unique

    n_phi = max(16, int(n_phi))
    phi_target = np.linspace(phi_unique[0], phi_unique[-1], n_phi)
    field_target = np.empty((n_phi, field_unique.shape[1]), dtype=float)

    for j in range(field_unique.shape[1]):
        col = field_unique[:, j]
        valid = np.isfinite(col)
        if np.count_nonzero(valid) >= 2:
            field_target[:, j] = np.interp(phi_target, phi_unique[valid], col[valid])
        elif np.count_nonzero(valid) == 1:
            field_target[:, j] = col[valid][0]
        else:
            field_target[:, j] = 0.0

    field_target = smooth_along_axis(field_target, window=smooth_phi, axis=0)
    field_target = smooth_along_axis(field_target, window=smooth_eta, axis=1)
    return phi_target, field_target


def select_half(phi, phi_dot, side):
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

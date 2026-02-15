"""Shared data and utilities for Wang 2007 kinematics analysis."""

import os
import numpy as np

# Geometry
gamma_fore = np.radians(53)     # simulator input stroke plane angles (from posterior)
gamma_hind = np.radians(44)

# Nondimensional time
omega = 2 * np.pi               # one cycle per nondimensional time unit

# --- Experimental data ---
data_dir = os.path.dirname(os.path.abspath(__file__))
data = np.genfromtxt(os.path.join(data_dir, "exp_data.csv"), delimiter=",", skip_header=2)

cols = {
    "s_fore":     (0, 1),   "s_hind":     (2, 3),
    "beta_fore":  (4, 5),   "beta_hind":  (6, 7),
    "alpha_fore": (8, 9),   "alpha_hind": (10, 11),
    "d_fore":     (12, 13), "d_hind":     (14, 15),
}


def sorted_xy(name):
    """Load a named variable from the CSV, sorted by time."""
    cx, cy = cols[name]
    x, y = data[:, cx], data[:, cy]
    mask = ~(np.isnan(x) | np.isnan(y))
    x, y = x[mask], y[mask]
    order = np.argsort(x)
    return x[order], y[order]


def fourier_smooth(t_raw, y_raw, n_harmonics=7, n_points=2000):
    """Smooth scattered data by keeping only the first n wingbeat harmonics.

    Interpolates onto a uniform grid, applies FFT truncation, returns smooth curve.
    The wingbeat period is 1 in nondimensional time.
    """
    t = np.linspace(t_raw[0], t_raw[-1], n_points)
    y = np.interp(t, t_raw, y_raw)
    n_wb = t[-1] - t[0]                    # number of wingbeats in data
    n_components = int(round(n_harmonics * n_wb))
    Y = np.fft.rfft(y)
    Y[n_components + 1:] = 0
    y_smooth = np.fft.irfft(Y, len(t))
    return t, y_smooth

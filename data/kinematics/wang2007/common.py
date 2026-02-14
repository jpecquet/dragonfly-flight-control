"""Shared data, constants, and utilities for Wang 2007 kinematics analysis."""

import os
import numpy as np
import matplotlib.pyplot as plt

plt.rcParams.update({
    "font.family": "serif",
    "font.serif": ["STIXGeneral"],
    "mathtext.fontset": "stix",
    "font.size": 12,
})

# Geometry
span = 40.0                     # mm (wing span)
r = (2 / 3) * span              # mm (reference span station)
chord = 10.0                    # mm
gamma_fore = np.radians(53)     # stroke plane angles (from posterior)
gamma_hind = np.radians(44)

# Nondimensional time
omega = 2 * np.pi               # one cycle per nondimensional time unit
delta = np.radians(22)          # hindwing phase lead

# Physical parameters
rho = 1.225                     # kg/m³
c_phys = chord * 1e-3           # m
L_phys = span * 1e-3            # m
f_phys = 33.4                   # Hz
omega_phys = 2 * np.pi * f_phys # rad/s
Cl0 = 1.2
Cd0 = 0.4
m_body = 300e-6                 # kg
mg = m_body * 9.81              # N

# Simulation convention angles
gam_fore = np.pi - gamma_fore
gam_hind = np.pi - gamma_hind

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


def break_discontinuities(x, y, threshold=50):
    """Insert NaN where consecutive y-values jump by more than threshold."""
    x_arr = np.asarray(x)
    y_arr = np.asarray(y)
    if x_arr.size == 0 or y_arr.size == 0:
        return x_arr.copy(), y_arr.copy()

    jump_idx = np.where(np.abs(np.diff(y_arr)) > threshold)[0] + 1
    x_out = np.insert(x_arr, jump_idx, np.nan)
    y_out = np.insert(y_arr, jump_idx, np.nan)
    return x_out, y_out


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


def fit_cosine(t, y):
    """Fit y = C + A*cos(omega*t + phi) via linear least squares.

    Returns (A, phi, C).
    """
    M = np.column_stack([
        np.ones_like(t),
        np.cos(omega * t),
        np.sin(omega * t),
    ])
    a0, a1, a2 = np.linalg.lstsq(M, y, rcond=None)[0]
    A = np.sqrt(a1**2 + a2**2)
    phi = np.arctan2(-a2, a1)
    return A, phi, a0


# --- Sinusoidal fit (forewing reference, hindwing leads by delta) ---
t_sf, sf = sorted_xy("s_fore")
t_bf, bf = sorted_xy("beta_fore")

phi_exp = np.arccos(np.clip(sf / r, -1, 1))
A_phi, psi_phi, C_phi = fit_cosine(t_sf, phi_exp)
A_beta, psi_beta, C_beta = fit_cosine(t_bf, bf)


def model_kinematics(t, phase=0):
    """Evaluate sinusoidal kinematic model.

    Returns (phi_wang, s, beta_deg).  phase=0 for forewing, delta for hindwing.
    """
    phi_wang = C_phi + A_phi * np.cos(omega * t + psi_phi + phase)
    s = r * np.cos(phi_wang)
    beta = C_beta + A_beta * np.cos(omega * t + psi_beta + phase)
    return phi_wang, s, beta


def compute_Fz_pair(phi_sim, phi_dot_phys, psi_sim, gam):
    """Compute Fz for a left+right wing pair using blade element theory.

    phi_sim:       simulation flapping angle (rad)
    phi_dot_phys:  d(phi_sim)/dt in physical units (rad/s)
    psi_sim:       simulation pitch angle = pi/2 - beta_rad
    gam:           simulation stroke plane angle = pi - gamma_wang
    """
    Cd = Cd0 + 2 * np.cos(psi_sim)**2
    Cl = Cl0 * np.sin(2 * psi_sim)
    factor = -phi_dot_phys * np.abs(phi_dot_phys)
    Fz_one = 0.5 * rho * c_phys * (L_phys**3 / 3) * factor * (
        Cd * np.sin(gam) * np.cos(phi_sim) + Cl * np.cos(gam))
    return 2 * Fz_one  # left + right

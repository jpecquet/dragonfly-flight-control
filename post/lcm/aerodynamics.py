"""Aerodynamic coefficient models for the LCM."""

import numpy as np


def cl_sinusoidal(alpha, cl0=1.2):
    """Wang 2004 sinusoidal lift coefficient: Cl = Cl0 * sin(2*alpha)."""
    return cl0 * np.sin(2.0 * alpha)


def cd_sinusoidal(alpha, cd_min=0.4, cd_max=2.4):
    """Wang 2004 sinusoidal drag coefficient: Cd = Cd_min + (Cd_max - Cd_min)*sin^2(alpha)."""
    return cd_min + (cd_max - cd_min) * np.sin(alpha) ** 2

def cl_azuma1988(alpha):
    """Azuma 1988 clamped-linear lift coefficient.

    Cl = clamp(slope * (alpha - neutral), Cl_min, Cl_max)
    with slope = 0.045/deg, neutral = -6.5 deg, Cl_min = -1.2, Cl_max = 1.2.
    Plateaus engage at alpha = -33 deg (Cl = -1.2) and alpha = +20 deg (Cl = +1.2).
    """
    slope = 0.045 * (180.0 / np.pi)  # per radian
    neutral = -6.5 * (np.pi / 180.0)  # radians
    return np.clip(slope * (np.asarray(alpha, dtype=float) - neutral), -1.2, 1.2)


def cd_azuma1988(alpha):
    """Azuma 1988 drag coefficient: 3-harmonic pi-periodic Fourier series.

    Cd(alpha) = a0 + a1*cos(2a) + b1*sin(2a) + a2*cos(4a) + b2*sin(4a) + a3*cos(6a) + b3*sin(6a)

    Coefficients from constrained least-squares fit to Azuma (1988) Fig. 7 data,
    with Cd(+/-90 deg) = 2.0 and min Cd = 0.07 (at alpha ~= -12 deg).
    """
    a = np.asarray(alpha, dtype=float)
    return (
         1.0950622737
        - 1.0121136563 * np.cos(2.0 * a)
        + 0.1376875279 * np.sin(2.0 * a)
        - 0.0266446397 * np.cos(4.0 * a)
        + 0.0846997560 * np.sin(4.0 * a)
        + 0.0805312903 * np.cos(6.0 * a)
        - 0.0120505921 * np.sin(6.0 * a)
    )


def cl_azuma1985(alpha):
    """Piecewise-linear Cl approximation from Azuma 1985 Fig. 8.

    Breakpoints chosen so that Cl is continuous:
      alpha <= -50: linear ramp from large negative alpha to -1.1
      -50 to -20:  constant -1.1
      -20 to  25:  linear from -1.1 to 1.8
       25 to  50:  constant 1.8
      alpha >= 50:  linear ramp down from 1.8
    """
    scalar = np.ndim(alpha) == 0
    a = np.rad2deg(np.atleast_1d(np.asarray(alpha, dtype=float)))
    # Middle linear region: connects (-20, -1.1) to (25, 1.8)
    slope_mid = 2.9 / 45.0  # (1.8 - (-1.1)) / (25 - (-20))
    cl = -1.1 + slope_mid * (a + 20.0)
    # Flat plateaus
    cl[a <= -20] = -1.1
    cl[(a >= 25) & (a <= 50)] = 1.8
    # Outer ramps (continuous at boundaries)
    mask_lo = a < -50
    cl[mask_lo] = -0.03 * a[mask_lo] - 2.6
    mask_hi = a > 50
    cl[mask_hi] = -0.045 * a[mask_hi] + 4.05
    return float(cl[0]) if scalar else cl


def cd_azuma1985(alpha):
    """Piecewise-linear Cd approximation from Azuma 1985 Fig. 8.

    Continuous everywhere:
      alpha < -25: linear ramp (= 0.2 at alpha=-25)
      -25 to  10:  constant 0.2
      alpha >  10: linear ramp (= 0.2 at alpha=10)
    """
    scalar = np.ndim(alpha) == 0
    a = np.rad2deg(np.atleast_1d(np.asarray(alpha, dtype=float)))
    cd = np.full_like(a, 0.2)
    mask_lo = a < -25
    cd[mask_lo] = -0.04 * a[mask_lo] - 0.8
    mask_hi = a > 10
    cd[mask_hi] = 0.04 * a[mask_hi] - 0.2
    return float(cd[0]) if scalar else cd

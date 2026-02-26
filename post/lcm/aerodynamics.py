"""Aerodynamic coefficient models for the LCM."""

import numpy as np


def cl_sinusoidal(alpha, cl0=1.2):
    """Wang 2004 sinusoidal lift coefficient: Cl = Cl0 * sin(2*alpha)."""
    return cl0 * np.sin(2.0 * alpha)


def cd_sinusoidal(alpha, cd_min=0.4, cd_max=2.4):
    """Wang 2004 sinusoidal drag coefficient: Cd = Cd_min + (Cd_max - Cd_min)*sin^2(alpha)."""
    return cd_min + (cd_max - cd_min) * np.sin(alpha) ** 2

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

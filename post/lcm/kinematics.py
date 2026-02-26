"""Fourier-series wing kinematics for the LCM.

Evaluates flapping angle phi(t), pitch angle psi(t), and their derivatives
from harmonic coefficients in the project convention:
    angle(t) = mean + sum_k amp_k * cos(k * omega * t + phase_k)
"""

import numpy as np


def fourier_series(t, omega, mean_deg, harmonics):
    """Evaluate a Fourier cosine series.

    Parameters
    ----------
    t : array_like
        Time array (seconds).
    omega : float
        Angular frequency (rad/s) = 2*pi*f.
    mean_deg : float
        Mean value in degrees.
    harmonics : list of (amplitude_deg, phase_deg)
        Harmonic coefficients [(amp1, phase1), (amp2, phase2), ...].

    Returns
    -------
    value : ndarray
        Angle in radians.
    rate : ndarray
        Angular rate in rad/s.
    """
    t = np.asarray(t, dtype=float)
    value = np.full_like(t, np.radians(mean_deg))
    rate = np.zeros_like(t)
    for k, (amp_deg, phase_deg) in enumerate(harmonics, start=1):
        amp = np.radians(amp_deg)
        phase = np.radians(phase_deg)
        arg = k * omega * t + phase
        value += amp * np.cos(arg)
        rate += -k * omega * amp * np.sin(arg)
    return value, rate

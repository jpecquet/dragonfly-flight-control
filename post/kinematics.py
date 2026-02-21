"""
Harmonic series helpers for kinematic angle evaluation.

Mirrors evaluateHarmonicValue / evaluateHarmonicRate from include/kinematics.hpp.
"""

from __future__ import annotations

import h5py
import numpy as np


def eval_series(amp_c, phase_c, phase):
    """Evaluate sum_k [ A_k*cos(k*phase + B_k) ] over a phase array."""
    val = np.zeros_like(phase)
    for k, (amp, phase_off) in enumerate(zip(amp_c, phase_c), 1):
        val += amp * np.cos((k * phase) + phase_off)
    return val


def eval_series_dot(amp_c, phase_c, basis_omega, phase):
    """Time derivative of eval_series (via chain rule with d_phase/dt = basis_omega)."""
    val = np.zeros_like(phase)
    for k, (amp, phase_off) in enumerate(zip(amp_c, phase_c), 1):
        val += -k * basis_omega * amp * np.sin((k * phase) + phase_off)
    return val


def read_angle_params(filename, wing_names_param, angles):
    """Read per-wing harmonic parameters from the HDF5 file.

    Parameters
    ----------
    filename : path to HDF5 file
    wing_names_param : list of wing name strings
    angles : tuple of angle names to read, e.g. ``('phi', 'psi')``
    """
    result = {}
    with h5py.File(filename, 'r') as f:
        for angle in angles:
            mean_arr = f[f'/parameters/wings/{angle}_mean'][:]
            amp_arr = f[f'/parameters/wings/{angle}_amp'][:]
            phase_arr = f[f'/parameters/wings/{angle}_phase'][:]
            result[f'{angle}_mean'] = dict(zip(wing_names_param, mean_arr))
            result[f'{angle}_amp'] = {
                name: np.asarray(amp_arr[i], dtype=float)
                for i, name in enumerate(wing_names_param)
            }
            result[f'{angle}_phase'] = {
                name: np.asarray(phase_arr[i], dtype=float)
                for i, name in enumerate(wing_names_param)
            }
    return result

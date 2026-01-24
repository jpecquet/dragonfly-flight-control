"""
HDF5 I/O functions for postprocessing.
"""

import h5py
import numpy as np


def read_simulation(filename):
    """
    Read simulation output from HDF5 file.

    Returns:
        params: dict of simulation parameters
        time: 1D array of time values
        states: list of state arrays [x, y, z, ux, uy, uz]
        wings: list of wing vector dicts (one per timestep), keyed by wing name
    """
    with h5py.File(filename, "r") as f:
        # Read parameters
        params = {key: f["/parameters"][key][()] for key in f["/parameters"].keys()}

        # Read time and state
        time = f["/time"][:]
        state_array = f["/state"][:]

        # Convert state array to list of arrays (matching Python code interface)
        states = [state_array[i, :] for i in range(state_array.shape[0])]

        # Read wing data (variable number of wings with user-defined names)
        wing_names = [k for k in f["/wings"].keys() if k != "num_wings"]
        vec_names = ["e_s", "e_r", "e_c", "lift", "drag"]

        # Pre-read all wing data
        wing_arrays = {}
        for wname in wing_names:
            wing_arrays[wname] = {}
            for vname in vec_names:
                wing_arrays[wname][vname] = f[f"/wings/{wname}/{vname}"][:]

        # Convert to list of dicts (one per timestep)
        n_steps = len(time)
        wings = []
        for i in range(n_steps):
            step_wings = {}
            for wname in wing_names:
                step_wings[wname] = {
                    vname: wing_arrays[wname][vname][i, :]
                    for vname in vec_names
                }
                # Add placeholder for 'u' vector (used in visualization but optional)
                step_wings[wname]['u'] = np.zeros(3)
            wings.append(step_wings)

    return params, time, states, wings


def read_wing_rotation(filename):
    """
    Read wing rotation test data from HDF5 file.

    Returns:
        dict with keys: frames_per_phase, total_frames, phase_boundaries,
                       gam_range, phi_range, psi_range, is_left,
                       gam, phi, psi, e_s, e_r, e_c
    """
    with h5py.File(filename, 'r') as f:
        data = {
            'frames_per_phase': f['/parameters/frames_per_phase'][()],
            'total_frames': f['/parameters/total_frames'][()],
            'phase_boundaries': f['/parameters/phase_boundaries'][:],
            'gam_range': (f['/parameters/gam_start'][()], f['/parameters/gam_end'][()]),
            'phi_range': (f['/parameters/phi_start'][()], f['/parameters/phi_end'][()]),
            'psi_range': (f['/parameters/psi_start'][()], f['/parameters/psi_end'][()]),
            'is_left': f['/parameters/is_left'][()] == 1,
            'gam': f['/angles/gam'][:],
            'phi': f['/angles/phi'][:],
            'psi': f['/angles/psi'][:],
            'e_s': f['/wing/e_s'][:],
            'e_r': f['/wing/e_r'][:],
            'e_c': f['/wing/e_c'][:],
        }
    return data


def read_landscape(filename):
    """
    Read optimizer landscape data from HDF5 file.

    Returns:
        dict with keys: ux, param_names, param1_values, objective,
                       and optionally param2_values (for 2D landscapes)
    """
    with h5py.File(filename, 'r') as f:
        data = {
            'ux': f['ux'][()],
            'param_names': [s.decode() if isinstance(s, bytes) else s
                          for s in f['param_names'][:]],
            'param1_values': f['param1_values'][:],
            'objective': f['objective'][:],
        }
        if 'param2_values' in f:
            data['param2_values'] = f['param2_values'][:]
    return data

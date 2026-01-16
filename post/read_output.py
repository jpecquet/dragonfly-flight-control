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

"""
HDF5 I/O functions for postprocessing.
"""

import subprocess
import tempfile
from pathlib import Path

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
        # Read parameters (skip subgroups like 'wings')
        params = {}
        for key in f["/parameters"].keys():
            item = f["/parameters"][key]
            if isinstance(item, h5py.Dataset):
                params[key] = item[()]

        # Extract per-wing parameters for visualization
        wing_names_raw = f["/parameters/wings/names"][:]
        wing_names_param = [n.decode() if isinstance(n, bytes) else n for n in wing_names_raw]
        wing_lb0 = f["/parameters/wings/lb0"][:]
        params['wing_lb0'] = dict(zip(wing_names_param, wing_lb0))

        # Read time and state
        time = f["/time"][:]
        states = f["/state"][:]

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


def read_tracking(filename):
    """
    Read trajectory tracking simulation output from HDF5 file.

    Returns:
        params: dict of simulation parameters
        time: 1D array of time values
        states: list of state arrays [x, y, z, ux, uy, uz]
        wings: list of wing vector dicts (one per timestep)
        controller: dict with target_position, position_error, gamma_mean, psi_mean, phi_amp
                   (or None if not a tracking simulation)
    """
    params, time, states, wings = read_simulation(filename)

    controller = None
    with h5py.File(filename, "r") as f:
        if "/controller" in f and f["/controller/active"][()] == 1:
            controller = {
                'target_position': f["/controller/target_position"][:],
                'position_error': f["/controller/position_error"][:],
                'gamma_mean': f["/controller/gamma_mean"][:],
                'psi_mean': f["/controller/psi_mean"][:],
                'phi_amp': f["/controller/phi_amp"][:],
            }

    return params, time, states, wings, controller


def read_wing_rotation(filename):
    """
    Read wing rotation test data from HDF5 file.

    Returns:
        dict with keys: frames_per_phase, total_frames, phase_boundaries,
                       gam_range, phi_range, psi_range, is_left, sequence,
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
        # Read sequence (with fallback for older files)
        if '/parameters/sequence' in f:
            seq_raw = f['/parameters/sequence'][:]
            data['sequence'] = [s.decode() if isinstance(s, bytes) else s for s in seq_raw]
        else:
            data['sequence'] = ['gam', 'phi', 'psi']  # default for backward compat
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


def read_terminal_velocity(filename):
    """
    Read terminal velocity simulation data from HDF5 file.

    Returns:
        dict with keys: time, x, z, ux, uz, psi, psi_deg, mu0, lb0, Cd0, Cl0,
                       dt, t_max, speed_analytical
    """
    with h5py.File(filename, 'r') as f:
        data = {
            'time': f['/time'][:],
            'z': f['/z'][:],
            'uz': f['/uz'][:],
            'psi': f['/parameters/psi'][()],
            'psi_deg': f['/parameters/psi_deg'][()],
            'mu0': f['/parameters/mu0'][()],
            'lb0': f['/parameters/lb0'][()],
            'Cd0': f['/parameters/Cd0'][()],
            'Cl0': f['/parameters/Cl0'][()],
            'dt': f['/parameters/dt'][()],
            't_max': f['/parameters/t_max'][()],
        }
        # New field (with backward compatibility)
        if '/parameters/speed_analytical' in f:
            data['speed_analytical'] = f['/parameters/speed_analytical'][()]
        elif '/parameters/uz_analytical' in f:
            data['speed_analytical'] = np.abs(f['/parameters/uz_analytical'][()])
        else:
            data['speed_analytical'] = 0.0
        if '/x' in f:
            data['x'] = f['/x'][:]
        else:
            data['x'] = np.zeros_like(data['z'])
        if '/ux' in f:
            data['ux'] = f['/ux'][:]
        else:
            data['ux'] = np.zeros_like(data['uz'])
        # Lift/drag vectors (optional, for animation)
        if '/lift_x' in f:
            data['lift_x'] = f['/lift_x'][:]
            data['lift_z'] = f['/lift_z'][:]
            data['drag_x'] = f['/drag_x'][:]
            data['drag_z'] = f['/drag_z'][:]
    return data


def run_termvel_simulation(psi_deg, dt=0.01, tmax=50.0):
    """
    Run terminal velocity simulation and return data.

    Args:
        psi_deg: Wing pitch angle in degrees
        dt: Time step
        tmax: Maximum simulation time

    Returns:
        dict: Simulation data (see read_terminal_velocity)
    """
    # Find the dragonfly binary
    script_dir = Path(__file__).parent.parent
    binary = script_dir / "build" / "bin" / "dragonfly"
    if not binary.exists():
        raise FileNotFoundError(f"Dragonfly binary not found at {binary}")

    # Run simulation to temp file
    with tempfile.NamedTemporaryFile(suffix=".h5", delete=False) as f:
        output_path = f.name

    cmd = [
        str(binary), "termvel",
        "--psi", str(psi_deg),
        "--dt", str(dt),
        "--tmax", str(tmax),
        "-o", output_path
    ]
    subprocess.run(cmd, check=True, capture_output=True)

    # Load and return data
    data = read_terminal_velocity(output_path)
    Path(output_path).unlink()  # Clean up temp file
    return data

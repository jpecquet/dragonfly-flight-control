"""
HDF5 I/O functions for postprocessing.
"""

import subprocess
import tempfile
from pathlib import Path

import h5py
import numpy as np


def decode_string_array(arr):
    """Decode an array of bytes/strings from HDF5 to a list of Python strings."""
    return [s.decode() if isinstance(s, bytes) else s for s in arr]


def read_simulation(filename):
    """
    Read simulation output from HDF5 file.

    Returns:
        params: dict of simulation parameters
        time: 1D array of time values
        states: ndarray of state vectors [x, y, z, ux, uy, uz] shape (N, 6)
        wings: dict-of-arrays keyed by wing name, e.g.
               wings[wing_name]['e_r'] is an (N, 3) array
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
        wing_names_param = decode_string_array(wing_names_raw)
        wing_lb0 = f["/parameters/wings/lb0"][:]
        params["wing_lb0"] = dict(zip(wing_names_param, wing_lb0))
        if "/parameters/wings/n_blade_elements" in f:
            wing_n_blades = f["/parameters/wings/n_blade_elements"][:]
            params["wing_n_blade_elements"] = dict(zip(wing_names_param, wing_n_blades))
        if "/parameters/wings/has_psi_twist_h1" in f:
            has_twist = f["/parameters/wings/has_psi_twist_h1"][:]
            params["wing_has_psi_twist_h1"] = dict(zip(wing_names_param, has_twist))
        if "/parameters/wings/psi_twist_h1_root" in f:
            twist_root = f["/parameters/wings/psi_twist_h1_root"][:]
            params["wing_psi_twist_h1_root"] = dict(zip(wing_names_param, twist_root))
        if "/parameters/wings/psi_twist_ref_eta" in f:
            twist_ref_eta = f["/parameters/wings/psi_twist_ref_eta"][:]
            params["wing_psi_twist_ref_eta"] = dict(zip(wing_names_param, twist_ref_eta))
        if "/parameters/wings/phase_offset" in f:
            wing_phase = f["/parameters/wings/phase_offset"][:]
            params["wing_phase_offset"] = dict(zip(wing_names_param, wing_phase))
        if "/parameters/wings/harmonic_period_wingbeats" in f:
            wing_period = f["/parameters/wings/harmonic_period_wingbeats"][:]
            params["wing_harmonic_period_wingbeats"] = dict(zip(wing_names_param, wing_period))
        if "/parameters/wings/omega" in f:
            wing_omega = f["/parameters/wings/omega"][:]
            params["wing_omega"] = dict(zip(wing_names_param, wing_omega))
        if "/parameters/wings/psi_amp" in f:
            psi_amp = f["/parameters/wings/psi_amp"][:]
            params["wing_psi_amp"] = {
                name: np.asarray(psi_amp[i], dtype=float)
                for i, name in enumerate(wing_names_param)
            }
        if "/parameters/wings/psi_phase" in f:
            psi_phase = f["/parameters/wings/psi_phase"][:]
            params["wing_psi_phase"] = {
                name: np.asarray(psi_phase[i], dtype=float)
                for i, name in enumerate(wing_names_param)
            }

        # Read time and state
        time = f["/time"][:]
        states = f["/state"][:]

        # Read wing data (variable number of wings with user-defined names)
        wing_names = sorted(k for k in f["/wings"].keys() if k != "num_wings")
        vec_names = ["e_s", "e_r", "e_c", "lift", "drag"]

        n_steps = len(time)
        wings = {}
        for wname in wing_names:
            wings[wname] = {}
            for vname in vec_names:
                wings[wname][vname] = f[f"/wings/{wname}/{vname}"][:]
            wings[wname]["u"] = np.zeros((n_steps, 3))

    return params, time, states, wings


def read_tracking(filename):
    """
    Read trajectory tracking simulation output from HDF5 file.

    Returns:
        params: dict of simulation parameters
        time: 1D array of time values
        states: ndarray of state vectors (N, 6)
        wings: dict-of-arrays keyed by wing name
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
            'sequence': decode_string_array(f['/parameters/sequence'][:]),
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
            'param_names': decode_string_array(f['param_names'][:]),
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
            'x': f['/x'][:],
            'z': f['/z'][:],
            'ux': f['/ux'][:],
            'uz': f['/uz'][:],
            'psi': f['/parameters/psi'][()],
            'psi_deg': f['/parameters/psi_deg'][()],
            'mu0': f['/parameters/mu0'][()],
            'lb0': f['/parameters/lb0'][()],
            'Cd0': f['/parameters/Cd0'][()],
            'Cl0': f['/parameters/Cl0'][()],
            'dt': f['/parameters/dt'][()],
            't_max': f['/parameters/t_max'][()],
            'speed_analytical': f['/parameters/speed_analytical'][()],
        }
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
        output_path = Path(f.name)

    cmd = [
        str(binary), "termvel",
        "--psi", str(psi_deg),
        "--dt", str(dt),
        "--tmax", str(tmax),
        "-o", str(output_path)
    ]
    try:
        subprocess.run(cmd, check=True, capture_output=True)
        return read_terminal_velocity(output_path)
    finally:
        if output_path.exists():
            output_path.unlink()

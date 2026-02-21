"""Shared data and utilities for Wang 2007 kinematics analysis."""

import json
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
case_path = REPO_ROOT / "data" / "case_studies" / "wang2007" / "case.json"
case_data = json.loads(case_path.read_text(encoding="utf-8"))
sim_defaults = case_data["simulation_defaults"]
stroke_plane_angles = sim_defaults["stroke_plane_angles_deg"]

# Geometry
gamma_fore = np.radians(float(stroke_plane_angles["fore"]))  # simulator stroke plane angle
gamma_hind = np.radians(float(stroke_plane_angles["hind"]))

# Nondimensional time
omega = 2 * np.pi               # one cycle per nondimensional time unit

# --- Experimental data ---
timeseries = case_data["kinematics"]["timeseries"]
csv_path = (case_path.parent / timeseries["csv_path"]).resolve()
data = np.genfromtxt(str(csv_path), delimiter=",", skip_header=int(timeseries.get("skip_header", 0)))

cols = {
    name: (int(mapping["time_col"]), int(mapping["value_col"]))
    for name, mapping in timeseries["columns"].items()
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

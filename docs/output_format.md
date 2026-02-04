# Simulation Output Format Specification

## Overview

The dragonfly flight simulation outputs data in HDF5 format. This document specifies the file structure for interoperability between the C++ simulation (`src/`) and Python postprocessing (`post/`).

## File Structure

```
simulation_output.h5
├── /parameters              (group)
│   ├── omega                (scalar) - Wing beat frequency
│   ├── gamma_mean           (scalar) - Mean stroke plane angle
│   ├── gamma_amp            (scalar) - Stroke plane oscillation amplitude
│   ├── gamma_phase          (scalar) - Stroke plane phase offset
│   ├── phi_amp              (scalar) - Stroke amplitude
│   ├── psi_mean             (scalar) - Mean pitch angle
│   ├── psi_amp              (scalar) - Pitch oscillation amplitude
│   ├── psi_phase            (scalar) - Pitch phase offset
│   └── /wings               (group)
│       ├── count            (scalar) - Number of wings
│       ├── names            (array)  - Wing names (e.g., "fore", "hind")
│       ├── sides            (array)  - Wing sides (0=left, 1=right)
│       ├── mu0              (array)  - Mass parameters
│       ├── lb0              (array)  - Span lengths
│       ├── Cd0              (array)  - Drag coefficients
│       ├── Cl0              (array)  - Lift coefficients
│       └── phase_offset     (array)  - Phase offsets (radians)
│
├── /time                    (dataset) [N] - Time values
│
├── /state                   (dataset) [N x 6] - State vectors
│   └── columns: [x, y, z, ux, uy, uz]
│
└── /wings                   (group)
    ├── num_wings            (scalar) - Number of wings
    └── /{name}_{side}       (group)  - Per-wing data (variable number)
        ├── e_s              (dataset) [N x 3] - Stroke direction unit vector
        ├── e_r              (dataset) [N x 3] - Radial direction unit vector
        ├── e_c              (dataset) [N x 3] - Chord direction unit vector
        ├── lift             (dataset) [N x 3] - Lift force vector
        └── drag             (dataset) [N x 3] - Drag force vector
```

## Wing Names

Wing group names are constructed from the config-defined name combined with the side (`_left` or `_right`). Examples:

**4-wing dragonfly:**
- `fore_left`, `fore_right`, `hind_left`, `hind_right`

**6-wing configuration:**
- `fore_left`, `fore_right`, `mid_left`, `mid_right`, `hind_left`, `hind_right`

## Data Types

| Field | HDF5 Type | Description |
|-------|-----------|-------------|
| Kinematic params | `H5T_NATIVE_DOUBLE` | Scalar 64-bit floats |
| names | Variable-length string | Wing name strings |
| Wing config arrays | `H5T_NATIVE_DOUBLE` | 1D arrays, length = num_wings |
| count, num_wings | `H5T_NATIVE_INT` | Number of wings |
| sides | `H5T_NATIVE_INT` | 0=left, 1=right |
| time | `H5T_NATIVE_DOUBLE` | 1D array of size N |
| state | `H5T_NATIVE_DOUBLE` | 2D array of shape [N, 6] |
| Wing vectors | `H5T_NATIVE_DOUBLE` | 2D arrays of shape [N, 3] |

## Coordinate System

Right-handed coordinate system:
- **X**: Forward (body heading direction)
- **Y**: Left (lateral)
- **Z**: Up (vertical)

## Units

All quantities are **nondimensional**:
- Time, distances, and velocities are nondimensionalized
- Angles are in radians

## Usage

### Python (reading)
```python
# Using the postprocessing library
from post.io import read_simulation

params, time, states, wings = read_simulation("output.h5")

# Or directly with h5py
import h5py

with h5py.File("output.h5", "r") as f:
    # Kinematic parameters
    omega = f["/parameters/omega"][()]
    gamma_mean = f["/parameters/gamma_mean"][()]

    # Wing configurations
    wing_count = f["/parameters/wings/count"][()]
    phase_offsets = f["/parameters/wings/phase_offset"][:]

    # Time series data
    time = f["/time"][:]
    state = f["/state"][:]

    # Wing data (variable number of wings)
    wing_names = [k for k in f["/wings"].keys() if k != "num_wings"]
    for name in wing_names:
        e_s = f[f"/wings/{name}/e_s"][:]
        lift = f[f"/wings/{name}/lift"][:]
```

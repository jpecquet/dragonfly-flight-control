# Simulation Output Format Specification

## Overview

The dragonfly flight simulation outputs data in HDF5 format. This document specifies the file structure for interoperability between the C++ simulation (`src/`) and Python postprocessing (`post/`).

## File Structure

```
simulation_output.h5
├── /parameters              (group)
│   ├── omega                (scalar) - Wing beat frequency
│   ├── n_harmonics          (scalar int) - Number of harmonics used
│   ├── gamma_mean           (scalar) - Mean stroke plane angle
│   ├── phi_mean             (scalar) - Mean stroke angle
│   ├── psi_mean             (scalar) - Mean pitch angle
│   ├── gamma_amp            (scalar) - Stroke plane oscillation amplitude
│   ├── gamma_phase          (scalar) - Stroke plane phase offset
│   ├── phi_amp              (scalar) - Stroke amplitude
│   ├── psi_amp              (scalar) - Pitch oscillation amplitude
│   ├── psi_phase            (scalar) - Pitch phase offset
│   ├── gamma_cos            (array)  - gamma cosine coefficients [N]
│   ├── gamma_sin            (array)  - gamma sine coefficients [N]
│   ├── phi_cos              (array)  - phi cosine coefficients [N]
│   ├── phi_sin              (array)  - phi sine coefficients [N]
│   ├── psi_cos              (array)  - psi cosine coefficients [N]
│   ├── psi_sin              (array)  - psi sine coefficients [N]
│   └── /wings               (group)
│       ├── count            (scalar) - Number of wings
│       ├── names            (array)  - Wing names with side (e.g., "fore_left", "hind_right")
│       ├── sides            (array)  - Wing sides (0=left, 1=right)
│       ├── mu0              (array)  - Mass parameters
│       ├── lb0              (array)  - Span lengths
│       ├── Cd0              (array)  - Drag coefficients
│       ├── Cl0              (array)  - Lift coefficients
│       ├── phase_offset     (array)  - Phase offsets (radians)
│       ├── has_custom_motion(array int) - 1 if wing had explicit motion overrides
│       ├── omega            (array)  - Per-wing wingbeat frequencies
│       ├── gamma_mean       (array)  - Per-wing gamma means
│       ├── phi_mean         (array)  - Per-wing phi means
│       ├── psi_mean         (array)  - Per-wing psi means
│       ├── gamma_cos        (matrix) [num_wings x N] - Per-wing gamma cosine coeffs
│       ├── gamma_sin        (matrix) [num_wings x N] - Per-wing gamma sine coeffs
│       ├── phi_cos          (matrix) [num_wings x N] - Per-wing phi cosine coeffs
│       ├── phi_sin          (matrix) [num_wings x N] - Per-wing phi sine coeffs
│       ├── psi_cos          (matrix) [num_wings x N] - Per-wing psi cosine coeffs
│       └── psi_sin          (matrix) [num_wings x N] - Per-wing psi sine coeffs
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

# Optional (tracking runs only)
simulation_output.h5
└── /controller              (group)
    ├── active               (scalar int) - Always 1 when present
    ├── target_position      (dataset) [N x 3] - Desired trajectory position
    ├── position_error       (dataset) [N x 3] - Target minus actual position
    ├── gamma_mean           (dataset) [N] - Controlled stroke-plane angle history
    ├── psi_mean             (dataset) [N] - Controlled mean pitch history
    └── phi_amp              (dataset) [N] - Controlled stroke amplitude history
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
| omega, means, amp/phase aliases | `H5T_NATIVE_DOUBLE` | Scalar 64-bit floats |
| n_harmonics | `H5T_NATIVE_INT` | Number of harmonics in coefficient arrays |
| *_cos, *_sin | `H5T_NATIVE_DOUBLE` | 1D arrays of length N |
| names | Variable-length string | Wing name strings |
| Wing config arrays | `H5T_NATIVE_DOUBLE` | 1D arrays, length = num_wings |
| wing has_custom_motion | `H5T_NATIVE_INT` | 1D array, length = num_wings |
| wing harmonic matrices | `H5T_NATIVE_DOUBLE` | 2D arrays [num_wings, N] |
| count, num_wings | `H5T_NATIVE_INT` | Number of wings |
| sides | `H5T_NATIVE_INT` | 0=left, 1=right |
| time | `H5T_NATIVE_DOUBLE` | 1D array of size N |
| state | `H5T_NATIVE_DOUBLE` | 2D array of shape [N, 6] |
| Wing vectors | `H5T_NATIVE_DOUBLE` | 2D arrays of shape [N, 3] |
| controller/active | `H5T_NATIVE_INT` | Present only for tracking output |
| controller/target_position | `H5T_NATIVE_DOUBLE` | 2D array [N, 3], tracking only |
| controller/position_error | `H5T_NATIVE_DOUBLE` | 2D array [N, 3], tracking only |
| controller/gamma_mean | `H5T_NATIVE_DOUBLE` | 1D array [N], tracking only |
| controller/psi_mean | `H5T_NATIVE_DOUBLE` | 1D array [N], tracking only |
| controller/phi_amp | `H5T_NATIVE_DOUBLE` | 1D array [N], tracking only |

## Coordinate System

Right-handed coordinate system:
- **X**: Forward (body heading direction)
- **Y**: Left (lateral)
- **Z**: Up (vertical)

## Units

All quantities are **nondimensional**:
- Time, distances, and velocities are nondimensionalized
- Angles are in radians

`/parameters/*` stores the global/default kinematic inputs. When per-wing motion overrides are used in config, resolved per-wing values are stored in `/parameters/wings/*`.

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

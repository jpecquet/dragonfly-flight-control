# Simulation Output Format Specification

## Overview

The dragonfly flight simulation outputs data in HDF5 format. This document specifies the file structure for interoperability between the C++ simulation (`src/`) and Python postprocessing (`post/`).

## File Structure

```
simulation_output.h5
├── /parameters          (group)
│   ├── lb0_f            (scalar) - Forewing length
│   ├── lb0_h            (scalar) - Hindwing length
│   ├── mu0_f            (scalar) - Forewing mass parameter
│   ├── mu0_h            (scalar) - Hindwing mass parameter
│   ├── Cd0              (scalar) - Base drag coefficient
│   ├── Cl0              (scalar) - Base lift coefficient
│   ├── omg0             (scalar) - Wing beat frequency
│   ├── gam0             (scalar) - Stroke plane angle
│   ├── phi0             (scalar) - Stroke amplitude
│   ├── psim             (scalar) - Mean pitch angle
│   ├── dpsi             (scalar) - Pitch oscillation amplitude
│   ├── sig0             (scalar) - Fore/hindwing phase offset
│   └── dlt0             (scalar) - Pitch phase offset
│
├── /time                (dataset) [N] - Time values
│
├── /state               (dataset) [N x 6] - State vectors
│   └── columns: [x, y, z, ux, uy, uz]
│
└── /wings               (group)
    ├── /fl              (group) - Forewing left
    │   ├── e_s          (dataset) [N x 3] - Stroke direction unit vector
    │   ├── e_r          (dataset) [N x 3] - Radial direction unit vector
    │   ├── e_c          (dataset) [N x 3] - Chord direction unit vector
    │   ├── lift         (dataset) [N x 3] - Lift force vector
    │   └── drag         (dataset) [N x 3] - Drag force vector
    ├── /fr              (group) - Forewing right
    │   └── ... (same structure)
    ├── /hl              (group) - Hindwing left
    │   └── ... (same structure)
    └── /hr              (group) - Hindwing right
        └── ... (same structure)
```

## Data Types

| Field | HDF5 Type | Description |
|-------|-----------|-------------|
| Parameters | `H5T_NATIVE_DOUBLE` | All scalar parameters are 64-bit floats |
| time | `H5T_NATIVE_DOUBLE` | 1D array of size N |
| state | `H5T_NATIVE_DOUBLE` | 2D array of shape [N, 6] |
| Wing vectors | `H5T_NATIVE_DOUBLE` | 2D arrays of shape [N, 3] |

## Coordinate System

- **X**: Forward (body heading direction)
- **Y**: Right (lateral)
- **Z**: Up (vertical)

## Units

All quantities are **nondimensional**:
- Time, distances, and velocities are nondimensionalized
- Angles are in radians

## Usage

### C++ (writing)
```cpp
#include <HighFive/HighFive.hpp>

HighFive::File file("output.h5", HighFive::File::Overwrite);
auto params = file.createGroup("/parameters");
params.createDataSet("lb0_f", lb0_f);
// ...
file.createDataSet("/time", time_vec);
file.createDataSet("/state", state_matrix);
```

### Python (reading)
```python
import h5py

with h5py.File("output.h5", "r") as f:
    params = {k: f["/parameters"][k][()] for k in f["/parameters"].keys()}
    time = f["/time"][:]
    state = f["/state"][:]
    wings = {
        name: {vec: f[f"/wings/{name}/{vec}"][:] for vec in ["e_s", "e_r", "e_c", "lift", "drag"]}
        for name in ["fl", "fr", "hl", "hr"]
    }
```

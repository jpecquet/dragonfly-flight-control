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
    ├── num_wings        (scalar) - Number of wings
    ├── /fore_left       (group) - Forewing left
    │   ├── e_s          (dataset) [N x 3] - Stroke direction unit vector
    │   ├── e_r          (dataset) [N x 3] - Radial direction unit vector
    │   ├── e_c          (dataset) [N x 3] - Chord direction unit vector
    │   ├── lift         (dataset) [N x 3] - Lift force vector
    │   └── drag         (dataset) [N x 3] - Drag force vector
    ├── /fore_right      (group) - Forewing right
    │   └── ... (same structure)
    ├── /hind_left       (group) - Hindwing left
    │   └── ... (same structure)
    └── /hind_right      (group) - Hindwing right
        └── ... (same structure)
```

## Wing Names

Wing names are constructed from the user-defined identifier passed to the Wing constructor in `main.cpp` combined with the wing's sidedness (`_left` or `_right`). For a standard 4-wing dragonfly:
- `fore_left`: Forewing left
- `fore_right`: Forewing right
- `hind_left`: Hindwing left
- `hind_right`: Hindwing right

## Data Types

| Field | HDF5 Type | Description |
|-------|-----------|-------------|
| Parameters | `H5T_NATIVE_DOUBLE` | All scalar parameters are 64-bit floats |
| num_wings | `H5T_NATIVE_INT` | Number of wings |
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

// Wings (variable number)
file.createGroup("/wings");
H5Easy::dump(file, "/wings/num_wings", num_wings);
for (int i = 0; i < num_wings; ++i) {
    std::string group = "/wings/" + std::to_string(i);
    file.createGroup(group);
    file.createDataSet(group + "/e_s", e_s_data);
    // ...
}
```

### Python (reading)
```python
import h5py

with h5py.File("output.h5", "r") as f:
    params = {k: f["/parameters"][k][()] for k in f["/parameters"].keys()}
    time = f["/time"][:]
    state = f["/state"][:]

    # Wing names are the subgroup names under /wings (excluding num_wings)
    wing_names = [k for k in f["/wings"].keys() if k != "num_wings"]
    wings = {
        name: {vec: f[f"/wings/{name}/{vec}"][:] for vec in ["e_s", "e_r", "e_c", "lift", "drag"]}
        for name in wing_names
    }
```

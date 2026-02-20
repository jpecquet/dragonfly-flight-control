# Modeling Overview

This project simulates dragonfly flight with a reduced-order model where each wing contributes aerodynamic force and the body state integrates under those forces plus gravity.

## State and dynamics

The simulator uses translational state:

- position: $\mathbf{x} = [x, y, z]^T$
- velocity: $\mathbf{u} = [u_x, u_y, u_z]^T$

The equations of motion are assembled in `src/eom.cpp`:

$$
\dot{\mathbf{x}} = \mathbf{u}
$$

$$
\dot{\mathbf{u}} = \sum_{i=1}^{N_{\text{wings}}} \mathbf{f}_i - \hat{\mathbf{z}}
$$

where gravity is nondimensionalized to magnitude 1 in $+z$-up coordinates.

## Wing force pipeline

At each timestep, per wing:

1. Evaluate wing angles from harmonic kinematics (`include/kinematics.hpp`)
2. Build wing orientation vectors (`include/rotation.hpp`)
3. Compute wing-point velocity at one or more span stations (`src/wing.cpp`)
4. Compute lift/drag via blade-element model and sum over stations (`src/blade_element.cpp`, `src/wing.cpp`)

## Integration

State integration is explicit Euler or RK4 (`src/integrator.cpp`).

## Configuration path

Simulation inputs are parsed from `.cfg` files (`src/config.cpp`, `src/sim_setup.cpp`) and mapped into per-wing motion/geometry (`include/wing.hpp`).

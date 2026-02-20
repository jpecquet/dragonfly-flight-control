# Wing Kinematics

Wing angles are represented as harmonic series (`include/kinematics.hpp`).

## Harmonic form

For angle series with mean $m$, harmonic amplitudes $A_k$, and per-harmonic phase offsets $B_k$:

$$
\theta(t) = m + \sum_{k=1}^{N} A_k\cos\left(k\,\phi(t) + B_k\right)
$$

with phase argument:

$$
\phi(t) = \omega t + \phi_0
$$

where $\phi_0$ is per-wing phase offset.

## Time derivative

The implementation computes:

$$
\dot{\theta}(t) = \sum_{k=1}^{N} -A_k\,k\omega\,\sin\left(k\,\phi(t) + B_k\right)
$$

## Config mapping

Global keys in `.cfg`:

- `n_harmonics`
- `gamma_mean`, `gamma_amp`, `gamma_phase`
- `phi_mean`, `phi_amp`, `phi_phase`
- `psi_mean`, `psi_amp`, `psi_phase`

Per-wing `[[wing]]` blocks can override these values (`src/sim_setup.cpp`).

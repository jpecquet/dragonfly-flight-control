# Wing Kinematics

Wing angles are represented as harmonic series (`include/kinematics.hpp`).

## Harmonic form

For angle series with mean $m$, cosine coefficients $a_k$, and sine coefficients $b_k$:

$$
\theta(t) = m + \sum_{k=1}^{N}\left[a_k\cos\left(k\,\phi(t)\right) + b_k\sin\left(k\,\phi(t)\right)\right]
$$

with phase argument:

$$
\phi(t) = \omega t + \phi_0
$$

where $\phi_0$ is per-wing phase offset.

## Time derivative

The implementation computes:

$$
\dot{\theta}(t) = \sum_{k=1}^{N} k\omega\left[-a_k\sin\left(k\,\phi(t)\right) + b_k\cos\left(k\,\phi(t)\right)\right]
$$

## Config mapping

Global keys in `.cfg`:

- `n_harmonics`
- `gamma_mean`, `gamma_cos`, `gamma_sin`
- `phi_mean`, `phi_cos`, `phi_sin`
- `psi_mean`, `psi_cos`, `psi_sin`

Per-wing `[[wing]]` blocks can override these values (`src/sim_setup.cpp`).

Legacy first-harmonic parameters (`*_amp`, `*_phase`) are still supported and converted internally to first-harmonic coefficients.

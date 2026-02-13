# Nondimensionalization

This project uses nondimensional variables so the governing equations and parameters are scale-consistent across scenarios.

## Reference scales

Let:

- $L$: body length scale
- $g$: gravitational acceleration
- $\tau = \sqrt{L/g}$: time scale

Then:

- nondimensional time: $t^* = t/\tau$
- nondimensional angular frequency: $\omega^* = \omega\,\tau$

For physical wingbeat frequency $f$ (Hz):

$$
\omega^* = 2\pi f\sqrt{L/g}
$$

This is exactly how the Wang 2007 pipeline computes `omega` in `scripts/wang2007_pipeline.py`.

## Wing geometry/aero parameters

The pipeline defines:

$$
\lambda = L_{\text{wing}}/L
$$

$$
\mu = \rho_{\text{air}}\,S_{\text{wing}}\,L_{\text{wing}}/m
$$

These map to config fields:

- `lb0 = \lambda`
- `mu0 = \mu`

with force prefactor per wing:

$$
\frac{1}{2}\frac{\mu}{\lambda}
$$

matching `force_coefficient = 0.5 * mu0 / lb0` in `src/wing.cpp`.

## Gravity normalization

In nondimensional form, gravity becomes unity in magnitude, and the EOM subtracts `1.0` from vertical acceleration (`src/eom.cpp`).

## Practical implications

- Changing physical scales in the Wang pipeline (`--body-length-mm`, `--wing-length-mm`, etc.) changes nondimensional `omega`, `mu0`, and `lb0`.
- Config files consumed by the simulator should be interpreted as nondimensional values unless explicitly documented otherwise.

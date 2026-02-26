# Blade Elements

## Overview

Aerodynamic forces are computed with a blade-element model. Each wing is discretized into a set of spanwise blade elements, and the aerodynamic force is evaluated at one representative point per element. The total wing force is the sum of the blade-element contributions.

The model stores both:

- per-wing aggregate quantities (orientation vectors, total lift, total drag, reference AoA), and
- per-blade quantities (orientation vectors, lift, drag, AoA, and span station)

which can be used to visualize spanwise force and angle-of-attack distributions.

## Velocity And Angle Of Attack

At a span station $\eta \in [0,1]$ with radius vector $\mathbf{r} = \eta R \mathbf{e}_r$, the local wing velocity in the inertial frame is modeled as

$$
\mathbf{u}_w = \mathbf{u}_b + \mathbf{v}_{\phi} + \mathbf{v}_{\gamma}
$$

where $\mathbf{u}_b$ is the body velocity, $\mathbf{v}_{\phi}$ is the flapping contribution, and $\mathbf{v}_{\gamma}$ is the stroke-plane rotation contribution.

The velocity used for aerodynamic loading is the component normal to the spanwise direction:

$$
\mathbf{u} = \mathbf{u}_w - (\mathbf{u}_w \cdot \mathbf{e}_r)\mathbf{e}_r
$$

The simulator angle of attack is then defined by the signed angle from the chord direction $\mathbf{e}_c$ to the projected velocity $\mathbf{u}$ about the local span axis $\mathbf{e}_r$:

$$
\alpha = \operatorname{atan2}\left((\mathbf{u} \times \mathbf{e}_c)\cdot\mathbf{e}_r,\ \mathbf{u}\cdot\mathbf{e}_c\right)
$$

This is the internal simulator sign convention. Because the sign is defined about the local span axis, left and right wings can have opposite signs for otherwise mirrored motions.

## Aerodynamic Coefficients

For each blade element, drag and lift coefficients are evaluated from the configured coefficient model using the local angle of attack $\alpha$.

Supported forms include:

- sinusoidal drag with neutral angle shift
- sinusoidal lift with neutral angle shift
- linear (clamped) lift with neutral angle shift
- piecewise-linear presets (e.g. Azuma 1985)

The blade-element force vectors are then accumulated into total wing lift and drag.

## Spanwise Discretization

Each wing is divided into `n_blade_elements` bins along the normalized span coordinate $\eta$.

The representative span station of each blade element is not the simple bin midpoint. Instead, the code uses an ellipse-chord-weighted centroid for each bin, consistent with the spanwise weighting used in force integration.

Let the normalized chord weighting be

$$
w(\eta) \propto \sqrt{1 - (2\eta - 1)^2}, \quad \eta \in [0,1]
$$

For a blade-element bin $\eta \in [a,b]$, the representative station is the chord centroid

$$
\eta_c = \frac{\int_a^b \eta\, w(\eta)\, d\eta}{\int_a^b w(\eta)\, d\eta}
$$

The implementation evaluates these integrals analytically (not numerically) using the change of variables $x = 2\eta - 1$, which reduces them to semicircle integrals over $x \in [-1,1]$.

The same bin integral is also used to form the blade-element area weights for force accumulation, so the quadrature locations and weights are consistent.

Special case:

- If `n_blade_elements = 1`, the representative station is fixed at $\eta = 2/3$.

This means per-blade outputs should be interpreted as values at quadrature points, not arbitrary sample points.

## Twist And Per-Blade Orientation

When pitch twist is enabled, the pitching angle $\psi$ varies with span station. As a result, the per-blade chord direction $\mathbf{e}_c$ differs across blade elements at the same timestep.

To support visualization and diagnostics, the simulation output stores per-blade orientation vectors (`e_s`, `e_r`, `e_c`) in addition to per-blade forces and AoA.

## HDF5 Output (Per Wing)

The simulation output file stores aggregate wing quantities under:

- `/wings/<wing>/e_s`, `/e_r`, `/e_c` (shape `(N, 3)`)
- `/wings/<wing>/lift`, `/drag` (shape `(N, 3)`)
- `/wings/<wing>/alpha` (shape `(N,)`, radians)

`/wings/<wing>/alpha` is a reference AoA value evaluated at $\eta = 2/3$.

Per-blade quantities are stored under:

- `/wings/<wing>/blade/eta` (shape `(B,)`)
- `/wings/<wing>/blade/alpha` (shape `(N, B)`, radians)
- `/wings/<wing>/blade/e_s`, `/e_r`, `/e_c`, `/lift`, `/drag`

For compatibility with the current HDF5 writer implementation, the per-blade vector datasets are stored as flattened matrices of shape `(N, 3B)` and reshaped in Python postprocessing to `(N, B, 3)`.

## References

```{bibliography}
:filter: docname in docnames
```

# Aerodynamic and Force Model

## Wing-point velocity decomposition

For each wing, velocity at evaluation point ($2/3$ span) is composed in `src/wing.cpp` as:

$$
\mathbf{u}_w = \mathbf{u}_b + \mathbf{v}_{\phi} + \mathbf{v}_{\gamma}
$$

where:

- $\mathbf{u}_b$: body velocity
- $\mathbf{v}_{\phi}$: flapping contribution from $\dot{\phi}$
- $\mathbf{v}_{\gamma}$: stroke-plane-rotation contribution from $\dot{\gamma}$ via cross product

The component along spanwise direction $\mathbf{e}_r$ is removed before aerodynamic force evaluation:

$$
\mathbf{u} = \mathbf{u}_w - (\mathbf{u}_w\cdot\mathbf{e}_r)\mathbf{e}_r
$$

## Angle of attack, lift, and drag

In `src/blade_element.cpp`:

- $c_\alpha = \cos\alpha = \hat{\mathbf{u}}\cdot\mathbf{e}_c$
- $s_\alpha = \sin\alpha = (\hat{\mathbf{u}}\times\mathbf{e}_c)\cdot\mathbf{e}_r$

Coefficient model:

$$
C_d = (C_{d0}+1) - \cos(2\alpha)
$$

$$
C_l = C_{l0}\sin(2\alpha)
$$

Force vectors:

$$
\mathbf{F}_d = \left(\frac{1}{2}\frac{\mu_0}{\lambda_0}\right) C_d \|\mathbf{u}\|^2\,\mathbf{e}_d
$$

$$
\mathbf{F}_l = \left(\frac{1}{2}\frac{\mu_0}{\lambda_0}\right) C_l \|\mathbf{u}\|^2\,\mathbf{e}_l
$$

Total per-wing force is $\mathbf{F}_l + \mathbf{F}_d$.

## Net acceleration

The simulator sums all wing forces and applies gravity in `src/eom.cpp`.


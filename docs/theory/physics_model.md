# Aerodynamic and Force Model

## Wing-point velocity decomposition

For each wing, velocity at one or more spanwise evaluation points is composed in `src/wing.cpp` as:

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

## Spanwise blade-element option

Each `[[wing]]` block can optionally specify:

- `n_blade_elements = N` (integer, `N >= 1`)
- `psi_twist_h1_root_deg = value` (optional root coefficient for first pitch harmonic)
- `psi_twist_ref_eta = value` (optional reference span station, default convention uses `0.75`)

There is also a global fallback key:

- `n_blade_elements = N`

If unspecified, the default is `N=1` (legacy single-point behavior at $2/3$ span).
For `N > 1`, the model splits the wing into equal span bins and area-weights each
bin using a composite full-ellipse chord distribution:

$$
c(\eta) \propto \sqrt{1 - (2\eta - 1)^2},\quad \eta \in [0,1]
$$

with the pitching axis at quarter-chord, i.e. leading/trailing half-chords scale as
$\frac{1}{4}c(\eta)$ and $\frac{3}{4}c(\eta)$.

When pitch twist is enabled, the first pitch-harmonic coefficient magnitude is scaled
linearly along span:

$$
C_1(\eta)=\left(C_1(\eta_\mathrm{ref})-C_1(0)\right)\frac{\eta}{\eta_\mathrm{ref}}+C_1(0)
$$

while preserving the harmonic phase (equivalently scaling both first-harmonic cosine
and sine coefficients by $C_1(\eta)/C_1(\eta_\mathrm{ref})$).

## Net acceleration

The simulator sums all wing forces and applies gravity in `src/eom.cpp`.

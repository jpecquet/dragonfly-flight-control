# Reachability Analysis

## Overview

We investigate the region of $(\tilde{u}_x, \tilde{u}_z)$ velocity space that the dragonfly can sustain as a steady-state equilibrium by adjusting its wing control parameters. For each point on a regular grid in velocity space, we run a multi-start optimizer to find control parameters that bring the net body force to zero. A grid point is considered reachable if the optimizer achieves a residual below a specified tolerance.

The velocity is nondimensionalized by $\sqrt{gL}$: $\tilde{u}_x$ is the horizontal and $\tilde{u}_z$ is the vertical body velocity. The wing control parameters are:

- $\gamma_0$: stroke plane angle
- $\phi_0$: mean flapping angle
- $\phi_1$: flapping amplitude
- $\psi_0$: mean pitch angle
- $\psi_1$: pitching amplitude
- $\delta_\psi$: pitch-flap phase offset

The flapping phase difference between the hindwing and the forewing does not affect the results here, since the optimizer computes the mean force over one wingbeat assuming constant body velocity. We find that fixing $\phi_0 = \psi_0 = 0$ does not significantly restrict the reachable set, leaving four free parameters: $\gamma_0$, $\phi_1$, $\psi_1$, and $\delta_\psi$.

## Effect of morphological parameters

### Effect of aerodynamic loading

The aerodynamic loading parameter $\mu_0 = \rho S R / m$ is the ratio of the characteristic aerodynamic mass $\rho S R$ to the total dragonfly mass, where $\rho$ is air density, $S$ is the single-wing planform area, and $R$ is the wing span. Fig. 1 shows the reachable set for $\mu_0 = 0.005$, $0.02$, and $0.08$.

```{raw} html
<div style="margin-bottom:1.5rem;">
  <img
    class="case-study-image"
    src="../_static/media/reachable/reachable_mu0.dark.png"
    alt="Reachable set: effect of wing mass ratio"
    data-light-src="../_static/media/reachable/reachable_mu0.light.png"
    data-dark-src="../_static/media/reachable/reachable_mu0.dark.png"
  />
  <div style="font-size:0.85em; line-height:1.2; margin-top:0.3rem; text-align:center;">Fig. 1. Effect of aerodynamic loading parameter $\mu_0$ on the reachable set (4-parameter control).</div>
</div>
```

### Effect of wingbeat frequency

Fig. 2 shows the reachable set for $\omega_0 = 6\pi$, $8\pi$, and $10\pi$.

```{raw} html
<div style="margin-bottom:1.5rem;">
  <img
    class="case-study-image"
    src="../_static/media/reachable/reachable_omega.dark.png"
    alt="Reachable set: effect of wingbeat frequency"
    data-light-src="../_static/media/reachable/reachable_omega.light.png"
    data-dark-src="../_static/media/reachable/reachable_omega.dark.png"
  />
  <div style="font-size:0.85em; line-height:1.2; margin-top:0.3rem; text-align:center;">Fig. 2. Effect of wingbeat frequency $\omega_0$ on the reachable set (4-parameter control).</div>
</div>
```

### Effect of wing span ratio

Fig. 2 shows the reachable set for $\lambda_0 = 0.5$, $0.75$, and $1.0$.

```{raw} html
<div style="margin-bottom:1.5rem;">
  <img
    class="case-study-image"
    src="../_static/media/reachable/reachable_lb0.dark.png"
    alt="Reachable set: effect of wing span ratio"
    data-light-src="../_static/media/reachable/reachable_lb0.light.png"
    data-dark-src="../_static/media/reachable/reachable_lb0.dark.png"
  />
  <div style="font-size:0.85em; line-height:1.2; margin-top:0.3rem; text-align:center;">Fig. 3. Effect of wing span ratio $\lambda_0$ on the reachable set (4-parameter control).</div>
</div>
```

## Effect of wing parameters

### Effect of pitch-flap phase offset

Fixing $\delta_\psi$ to a specific value shifts the reachable set in velocity space. With $\delta_\psi = -\pi/2$ the set shifts toward positive $\tilde{u}_x$ (forward flight); with $\delta_\psi = +\pi/2$ it shifts toward negative $\tilde{u}_x$ (backward flight).

```{raw} html
<div style="margin-bottom:1.5rem;">
  <img
    class="case-study-image"
    src="../_static/media/reachable/reachable_phase.dark.png"
    alt="Reachable set: effect of pitch-flap phase offset"
    data-light-src="../_static/media/reachable/reachable_phase.light.png"
    data-dark-src="../_static/media/reachable/reachable_phase.dark.png"
  />
  <div style="font-size:0.85em; line-height:1.2; margin-top:0.3rem; text-align:center;">Fig. 4. Effect of pitch-flap phase offset on the reachable set.</div>
</div>
```

### Effect of stroke plane angle

Restricting the stroke plane angle $\gamma_0$ to $[0, \pi/2]$ (backward tilt) or $[\pi/2, \pi]$ (forward tilt) restricts the reachable set to two quadrants (about $\tilde{u}_z = \tilde{u}_x$ for backward tilt, and about $\tilde{u}_z = -\tilde{u}_x$ for foward tilt).

```{raw} html
<div style="margin-bottom:1.5rem;">
  <img
    class="case-study-image"
    src="../_static/media/reachable/reachable_gamma.dark.png"
    alt="Reachable set: effect of stroke plane angle range"
    data-light-src="../_static/media/reachable/reachable_gamma.light.png"
    data-dark-src="../_static/media/reachable/reachable_gamma.dark.png"
  />
  <div style="font-size:0.85em; line-height:1.2; margin-top:0.3rem; text-align:center;">Fig. 5. Effect of stroke plane angle range on the reachable set.</div>
</div>
```

### Effect of pitching amplitude

Restricting the pitching amplitude $\psi_1$ to below or above $\pi/4$ shows how much pitch authority the dragonfly needs to access different regions of velocity space. Small pitching amplitudes ($\psi_1 < \pi/4$) produce a reduced reachable set, while larger amplitudes ($\psi_1 > \pi/4$) recover most of the full set.

```{raw} html
<div style="margin-bottom:1.5rem;">
  <img
    class="case-study-image"
    src="../_static/media/reachable/reachable_psi1.dark.png"
    alt="Reachable set: effect of pitching amplitude"
    data-light-src="../_static/media/reachable/reachable_psi1.light.png"
    data-dark-src="../_static/media/reachable/reachable_psi1.dark.png"
  />
  <div style="font-size:0.85em; line-height:1.2; margin-top:0.3rem; text-align:center;">Fig. 6. Effect of pitching amplitude $\psi_1$ on the reachable set (split at $\psi_1 = \pi/4$).</div>
</div>
```

### Effect of flapping amplitude

Restricting the flapping amplitude $\phi_1$ to below or above $\pi/4$ shows the role of stroke amplitude in covering velocity space. Small amplitudes ($\phi_1 < \pi/4$) yield a much reduced reachable set, while larger amplitudes ($\phi_1 > \pi/4$) recover the full set.

```{raw} html
<div style="margin-bottom:1.5rem;">
  <img
    class="case-study-image"
    src="../_static/media/reachable/reachable_phi1.dark.png"
    alt="Reachable set: effect of flapping amplitude"
    data-light-src="../_static/media/reachable/reachable_phi1.light.png"
    data-dark-src="../_static/media/reachable/reachable_phi1.dark.png"
  />
  <div style="font-size:0.85em; line-height:1.2; margin-top:0.3rem; text-align:center;">Fig. 7. Effect of flapping amplitude $\phi_1$ on the reachable set (split at $\phi_1 = \pi/4$).</div>
</div>
```

In Fig. 8, the pitching amplitude is now capped at $\psi_1 \leq \pi/16$.

```{raw} html
<div style="margin-bottom:1.5rem;">
  <img
    class="case-study-image"
    src="../_static/media/reachable/reachable_phi1_psi1pi16.dark.png"
    alt="Reachable set: effect of flapping amplitude with psi1 capped at pi/16"
    data-light-src="../_static/media/reachable/reachable_phi1_psi1pi16.light.png"
    data-dark-src="../_static/media/reachable/reachable_phi1_psi1pi16.dark.png"
  />
  <div style="font-size:0.85em; line-height:1.2; margin-top:0.3rem; text-align:center;">Fig. 8. Effect of flapping amplitude $\phi_1$ with $\psi_1 \leq \pi/16$ (split at $\phi_1 = \pi/4$).</div>
</div>
```


## Reproduction Commands

```bash
# Regenerate docs media for this case
python -m scripts.docs_media_runner cases/reachable/post.yaml
```

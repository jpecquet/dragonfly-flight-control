# Pursuit Problem

## Motivation

We develop a control scheme so that our model dragonfly can intercept a moving target. This is a dual-mode control scheme with a hover phase, and an interception phase. The dragonfly starts at rest at the origin in a hovering equilibrium and must detect, pursue, and come to within a short distance of a moving target. This combines two control problems: maintaining a stable hover while scanning for targets, and switching to an aggressive pursuit mode upon detection.

## Setup

### Pursuit Controller

The pursuit controller follows the same philosophy as the model developped for the pursuit of prey by tiger beetles in {cite}`haselsteiner2014` and {cite}`noest2017`. Upon detection of the target, the controller switches from the hover wing kinematics to kinematics that allow high-speed flight, with the only variable parameter being $\gamma$. The fixed parameters are a high flapping amplitude $\phi_1 = 35°$, $\psi_0 = 0°$ for a symmetric wing stroke, a low pitch amplitude $\psi_1 = 20°$, and $\delta_0 = 90^\circ$.

$\gamma_0$ is modulated via proportional control on the signed angle $\alpha$ between the averaged velocity $\bar{\mathbf{v}}$ and the line-of-sight to the target $\mathbf{r} = \mathbf{x}_\text{target} - \mathbf{x}$. It is bound by a minimum of $0°$ and a maximum of $90°$:

$$\gamma_0 = \text{clamp}\!\left(\gamma_\text{hover} + K_p \cdot \text{sign}(\alpha) \cdot |\alpha|,\ \gamma_\text{min},\ \gamma_\text{max}\right)$$

Reducing $\gamma_0$ below the hover baseline tilts the stroke plane back toward horizontal, increasing the vertical force component and steering the dragonfly upward. Increasing $\gamma_0$ toward $90°$ tilts the stroke plane more forward, producing a larger horizontal force component for forward or downward flight.

### Target Detection and Interception

Target detection uses an angular field-of-view model: the target must lie within a $60°$ half-cone centered on the forward ($+x$) body axis. Interception is declared when the distance drops below $0.1\,\tilde{L}$ (one tenth of a body length). After interception, the controller returns to hover mode and the simulation runs for 30 additional wingbeats before ending.

## Results

### Run 1: Horizontal Target

The target begins at $(0, 0, 5)$ (five body lengths above the origin) and moves horizontally at unit velocity along $x$.

```{raw} html
<div style="margin-bottom:1.5rem;">
  <video
    class="case-study-video"
    loop
    autoplay
    muted
    playsinline
    preload="metadata"
    data-light-src="../_static/media/pursuit/pursuit_animation.light.mp4"
    data-dark-src="../_static/media/pursuit/pursuit_animation.dark.mp4"
  >
    <source src="../_static/media/pursuit/pursuit_animation.dark.mp4" type="video/mp4">
    Your browser does not support the video tag.
  </video>
  <div style="font-size:0.85em; line-height:1.2; margin-top:0.3rem; text-align:center;">Fig. 1. Dual-mode pursuit. The dragonfly starts at rest (hover mode), detects the target entering its field of view, switches to pursuit mode, and intercepts.</div>
</div>
```

```{image} ../_static/media/pursuit/pursuit_control.light.png
:align: center
:class: only-light
:width: 80%
```
```{image} ../_static/media/pursuit/pursuit_control.dark.png
:align: center
:class: only-dark
:width: 80%
```
<div style="font-size:0.85em; line-height:1.2; margin-top:0.3rem; text-align:center; margin-bottom:1.5rem;">Fig. 2. Controller state for Run 1. Top: stroke plane angle $\gamma_0$. Middle: signed angle error $\alpha$ between velocity and line-of-sight. Bottom: distance to target. Dashed lines mark detection and interception.</div>

### Run 2: Descending Target

The target begins at $(0, 0, 5)$ and moves forward at unit velocity while descending at $-1$ m/s. The controller must first steer the dragonfly upward (reducing $\gamma_0$) and then forward (increasing $\gamma_0$) as the target descends through the dragonfly's altitude.

```{raw} html
<div style="margin-bottom:1.5rem;">
  <video
    class="case-study-video"
    loop
    autoplay
    muted
    playsinline
    preload="metadata"
    data-light-src="../_static/media/pursuit/pursuit_descending.light.mp4"
    data-dark-src="../_static/media/pursuit/pursuit_descending.dark.mp4"
  >
    <source src="../_static/media/pursuit/pursuit_descending.dark.mp4" type="video/mp4">
    Your browser does not support the video tag.
  </video>
  <div style="font-size:0.85em; line-height:1.2; margin-top:0.3rem; text-align:center;">Fig. 3. Pursuit of a descending target. The dragonfly climbs to intercept.</div>
</div>
```

```{image} ../_static/media/pursuit/pursuit_descending_control.light.png
:align: center
:class: only-light
:width: 80%
```
```{image} ../_static/media/pursuit/pursuit_descending_control.dark.png
:align: center
:class: only-dark
:width: 80%
```
<div style="font-size:0.85em; line-height:1.2; margin-top:0.3rem; text-align:center; margin-bottom:1.5rem;">Fig. 4. Controller state for Run 2. The stroke plane angle actively modulates to steer the dragonfly onto the target.</div>

## References

```{bibliography}
:filter: docname in docnames
```

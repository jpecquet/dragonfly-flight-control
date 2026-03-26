# Pursuit Problem

## Motivation

We develop a control scheme so that our model dragonfly can intercept a moving target. This is a dual-mode control scheme with a hover phase, and an interception phase. The dragonfly starts at rest at the origin in a hovering equilibrium and must detect, pursue, and come to within a short distance of a moving target. This combines two control problems: maintaining a stable hover while scanning for targets, and switching to an aggressive pursuit mode upon detection.

## Setup

### Pursuit Controller

The controller follows the same philosophy as the model developped for the pursuit of prey by tiger betles in {cite}`haselsteiner2014` and {cite}`noest2017`.

The controller uses the same three-layer physiological feedback model as the {doc}`hover stabilization study <hover>` — rolling velocity average, neural sensing delay, and neuromuscular lag — which naturally smooths the transition between modes.

The controller operates in two modes with four runtime-controllable wing kinematic parameters: stroke plane angle $\gamma_0$, flapping amplitude $\phi_1$, mean pitch angle $\psi_0$, and pitch amplitude $\psi_1$.

**Hover mode.** At startup, the optimizer finds the minimum-power hover equilibrium at $\gamma_0 = 45°$, yielding equilibrium values for $\phi_1$ and $\psi_0$. Proportional feedback holds the dragonfly at rest:

$$\phi_1^\text{target} = \phi_1^\text{eq} - K_{p,z} \, \bar{u}_z, \qquad
\psi_0^\text{target} = \psi_0^\text{eq} + K_{p,x} \, \bar{u}_x$$

where $\bar{u}_x$ and $\bar{u}_z$ are the delayed wingbeat-averaged velocities.

**Pursuit mode.** Upon detection, the controller switches to fixed $\phi_1 = 35°$, $\psi_0 = 0°$, $\psi_1 = 20°$, while modulating $\gamma_0$ via proportional control on the signed angle $\alpha$ between the delayed velocity and the target direction:

$$\gamma_0 = \text{clip}\!\left(\gamma_0^\text{hover} + K_{p,\gamma} \, \alpha,\; \gamma_0^\text{hover},\; 90°\right)$$

The reduced pitch amplitude ($20°$ vs. $57°$ in hover) and moderate flapping amplitude ($35°$) produce efficient forward thrust when the stroke plane is tilted, while the inclined stroke plane provides the vertical force bias needed to maintain altitude during pursuit.

### Target Detection and Interception

Target detection uses an angular field-of-view model: the target must lie within a $60°$ half-cone centered on the forward ($+x$) body axis. Interception is declared when the distance drops below $0.1\,\tilde{L}$ (one tenth of a body length). After interception, the controller returns to hover mode and the simulation runs for 30 additional wingbeats before ending.

## Results

### Linear Target Trajectory

The dragonfly successfully intercepts the target at wingbeat 36.6. The full sequence is: hover at equilibrium, detect the target entering the field of view, pursue with increasing stroke plane tilt, intercept, then return to hover for 30 wingbeats.

The neuromuscular lag (time constant $0.5$ wingbeats) smooths all parameter transitions — when the mode switches, $\gamma_0$, $\phi_1$, $\psi_0$, and $\psi_1$ transition continuously rather than jumping instantaneously, producing realistic flight behavior.

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
  <div style="font-size:0.85em; line-height:1.2; margin-top:0.3rem; text-align:center;">Fig. 1. Dual-mode pursuit. The dragonfly starts at rest (hover mode), detects the target entering its field of view, switches to pursuit mode with tilted stroke plane, and intercepts at wingbeat 36.6. The green marker shows the target moving at unit velocity along $x$ at altitude $z=5$.</div>
</div>
```

## References

```{bibliography}
:filter: docname in docnames
```

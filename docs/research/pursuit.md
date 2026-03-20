# Pursuit Problem

## Overview

We study whether a dragonfly can intercept a moving target using PID feedback control over its wing kinematic parameters. The dragonfly starts at rest at the origin and must pursue a target that begins at $\tilde{z} = 5$ (five body lengths above) and moves horizontally at nondimensional velocity $\tilde{u}_x = 1$ along the $x$-axis.

This is a classical pursuit problem: the dragonfly must simultaneously climb to the target's initial altitude, accelerate forward to match its speed, and reduce the position error to zero — all through aerodynamic control alone.

## Setup

### Initial conditions

| Quantity | Value |
|----------|-------|
| Dragonfly position | $(0, 0, 0)$ |
| Dragonfly velocity | $(0, 0, 0)$ |
| Target position at $\tilde{t}=0$ | $(\tilde{x}, \tilde{y}, \tilde{z}) = (0, 0, 5)$ |
| Target velocity | $(\tilde{u}_x, \tilde{u}_y, \tilde{u}_z) = (1, 0, 0)$ |

The dragonfly specimen uses the same morphological parameters as the reachability study (body length 40 mm, body mass 0.325 g, wingbeat frequency 62.6 Hz).

### Wing kinematics

The pitch angle $\psi$ and flapping amplitude $\phi_1$ are held constant throughout the maneuver; only the stroke plane angle $\gamma_0$ is varied. We found empirically (by sweeping the parameter space) that a baseline stroke plane tilt of $\gamma_0^\text{base} = 65°$ produces sufficient combined upward and forward force for the dragonfly to reach the target altitude while accelerating in $x$. This is a forward-flight configuration: at $\gamma_0 = 65°$ the net aerodynamic force has both upward and forward components, generating positive vertical acceleration (the dragonfly climbs) and positive horizontal acceleration simultaneously. The horizontal equilibrium — where vertical force exactly equals gravity — lies near $\gamma_0 \approx 74°$; below this angle the dragonfly ascends, above it descends.

### Target trajectory

The target follows a linear trajectory:

$$\tilde{\mathbf{r}}_\text{target}(\tilde{t}) = (\tilde{t},\; 0,\; 5)$$

This is specified in the case config as `trajectory: "linear 0.0 0.0 5.0 1.0 0.0 0.0"`.

### Controller

The mean pitch angle $\psi_0$ and flapping amplitude $\phi_1$ are held fixed. Only the stroke plane angle $\gamma_0$ is modulated using proportional control on the signed angle $\alpha$ between the wingbeat-averaged body velocity $\bar{\mathbf{v}}$ and the target direction $\mathbf{r} = \mathbf{r}_\text{target} - \mathbf{r}_\text{body}$:

$$\gamma_0 = \text{clip}\!\left(\gamma_0^\text{base} + K_p \, \alpha,\; \gamma_\text{min},\; \gamma_\text{max}\right)$$

where $\alpha$ is signed via the $y$-component of $\hat{\bar{\mathbf{v}}} \times \hat{\mathbf{r}}$ (positive when the target lies above the current velocity direction in the $xz$-plane), $K_p = 0.5$, and $[\gamma_\text{min}, \gamma_\text{max}] = [0°, 114.6°]$. Velocity is smoothed with an exponential moving average whose time constant equals one wingbeat period, suppressing within-wingbeat oscillation that would otherwise cause chattering.

### Success criterion

Interception is declared when the closest approach distance drops below $0.1 \tilde{L}$ (one tenth of a body length, approximately 4 mm).

## Results

The dragonfly successfully intercepts the target at wingbeat 68.5, achieving a closest approach of $0.033\,\tilde{L}$ (1.3 mm), well below the 0.1 body-length tolerance. At the moment of closest approach, the dragonfly is at $(\tilde{x}, \tilde{z}) = (17.11, 5.03)$ and the target is at $(17.13, 5.00)$ — less than $0.1\,\tilde{L}$ separation in both $x$ and $z$ simultaneously.

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
  <div style="font-size:0.85em; line-height:1.2; margin-top:0.3rem; text-align:center;">Fig. 1. Pursuit trajectory. The dragonfly (blue trail) starts at rest at the origin and intercepts the target (green marker) which begins at $z=5$ and moves along $x$ at unit velocity. The dashed green line shows the full target trajectory. Closest approach (0.033 body lengths) occurs at wingbeat 68.</div>
</div>
```

## Reproduction Commands

```bash
# Build
mkdir -p build && cd build && cmake .. && make

# Regenerate docs media (runs simulation + renders animation)
python -m scripts.docs_media_runner cases/pursuit/post.yaml
```

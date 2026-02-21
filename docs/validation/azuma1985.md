# Azuma and Azuma 1985

Flight simulation from experimental wing kinematics from {cite}`azuma1985`.

## Description

```{include} generated/azuma1985_description.md
```

## Pre-processing

### Kinematics Data

The flapping angle $\psi$ and pitch angle $\theta$ below are the Fourier cosine
series from the paper, evaluated over one wingbeat.

```{raw} html
<img
  class="case-study-image"
  src="../_static/media/azuma1985/kinematics_inputs.dark.png"
  alt="Azuma 1985 paper-convention kinematics for psi and theta"
  data-light-src="../_static/media/azuma1985/kinematics_inputs.light.png"
  data-dark-src="../_static/media/azuma1985/kinematics_inputs.dark.png"
/>
```

### Mapped Simulator Angles (`phi`, `psi`)

The simulator motion inputs are constructed from the paper angles using:

- `phi_sim = -psi_paper` (sign flip)
- `psi_sim = theta_paper - 90 deg`

```{raw} html
<img
  class="case-study-image"
  src="../_static/media/azuma1985/motion_mapping.dark.png"
  alt="Azuma 1985 mapped simulator angles phi and psi"
  data-light-src="../_static/media/azuma1985/motion_mapping.light.png"
  data-dark-src="../_static/media/azuma1985/motion_mapping.dark.png"
/>
```

## Results

### Wing Motion 3D Visualization

```{raw} html
<video
  class="case-study-video"
  controls
  loop
  autoplay
  muted
  preload="metadata"
  data-light-src="../_static/media/azuma1985/simulation.light.mp4"
  data-dark-src="../_static/media/azuma1985/simulation.dark.mp4"
>
  <source src="../_static/media/azuma1985/simulation.dark.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>
```

### Wing Motion Stick Plot

```{raw} html
<video
  class="case-study-video"
  controls
  loop
  autoplay
  muted
  preload="metadata"
  data-light-src="../_static/media/azuma1985/stick.light.mp4"
  data-dark-src="../_static/media/azuma1985/stick.dark.mp4"
>
  <source src="../_static/media/azuma1985/stick.dark.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>
```

### Body Speed and Direction vs Experiment

The left panel shows dimensional body speed magnitude from simulation compared to
the experimental reference. The right panel shows the center-of-mass direction
angle in the `XZ` plane, `atan2(z, x)` in degrees, compared to experiment.

```{raw} html
<img
  class="case-study-image"
  src="../_static/media/azuma1985/flight_metrics.dark.png"
  alt="Azuma 1985 body speed and direction comparison against experimental references"
  data-light-src="../_static/media/azuma1985/flight_metrics.light.png"
  data-dark-src="../_static/media/azuma1985/flight_metrics.dark.png"
/>
```

## Reproduction Commands

```bash
# Regenerate and sync all media
python scripts/update_docs_media.py --only azuma1985
```

## References

```{bibliography}
:filter: docname in docnames
```

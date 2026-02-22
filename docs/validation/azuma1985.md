# Azuma and Azuma 1985

Flight simulation from experimental wing kinematics from {cite}`azuma1985`.

## Description

### Dragonfly Specimen

| Parameter | Value |
|---|---|
| Body length $L$ | `0.04 m` |
| Body mass $m$ | `2.600e-04 kg` |
| Forewing span $R_{fw}$ | `0.0335 m` |
| Forewing area $S_{fw}$ | `2.210e-04 m^2` |
| Hindwing span $R_{hw}$ | `0.0325 m` |
| Hindwing area $S_{hw}$ | `2.720e-04 m^2` |

### Wing Kinematics

Flapping frequency: `41.5 Hz`

Coning angles:
- Fore: `8 deg`
- Hind: `-2 deg`

Paper-convention Fourier data (degrees):

| Wing | Series | mean (deg) | Fourier terms |
|---|---|---:|---|
| fore | stroke plane angle | 37 | `-` |
| fore | flapping angle theta (paper figure typo labels this phi) | -3 | k=1: A=-43, phase=0 deg |
| fore | pitch angle | 98 | k=1: A=-77, phase=-49 deg<br>k=2: A=-3, phase=67 deg<br>k=3: A=-8, phase=29 deg |
| hind | stroke plane angle | 40 | `-` |
| hind | flapping angle theta (paper figure typo labels this phi) | 2 | k=1: A=-47, phase=77 deg |
| hind | pitch angle | 93 | k=1: A=-65, phase=18 deg<br>k=2: A=8, phase=74 deg<br>k=3: A=8, phase=28 deg |

### Experimental Output References

- `body_speed_and_direction`: speed `0.54 m/s`, direction `60 deg`
  - Definition: `atan2(z, x) in XZ plane`

## Pre-processing

### Kinematics

```{raw} html
<img
  class="case-study-image"
  src="../_static/media/azuma1985/kinematics.dark.png"
  alt="Azuma 1985 fore/hind kinematics (phi, psi)"
  data-light-src="../_static/media/azuma1985/kinematics.light.png"
  data-dark-src="../_static/media/azuma1985/kinematics.dark.png"
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
# Regenerate docs media for this case
python -m scripts.docs_media_runner --run-all --only azuma1985
```

## References

```{bibliography}
:filter: docname in docnames
```

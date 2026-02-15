# (Azuma, 1985)

Flight simulation from experimental wing kinematics from {cite}`azuma1985`.

## Description

### Dragonfly Specimen

| Parameter | Dimensions |
|---|---|
| $L$ | $4\times10^{-2}$ m |
| $m$ | $2.6\times10^{-4}$ kg |
| $R_{fw}$ | $3.35\times10^{-2}$ m |
| $S_{fw}$ | $2.21\times10^{-4}$ m$^2$ |
| $AR_{fw}$ | $10.2$ |
| $R_{hw}$ | $3.25\times10^{-2}$ m |
| $S_{hw}$ | $2.72\times10^{-4}$ m$^2$ |
| $AR_{hw}$ | $7.8$ |

### Wing Kinematics

Flapping frequency: $f = 41.5$ Hz

Stroke planes

$$\gamma_{fw} = 37 \text{deg}$$
$$\gamma_{hw} = 40 \text{deg}$$

Wing flapping angle Fourier series (in degrees)

$$\psi_{fw} = -3 - 43 \cos{(\omega t)}$$
$$\psi_{hw} = 2 - 47 \cos{(\omega t + 77)}$$

Wing pitch angle Fourier series (in degrees)

$$\theta_{fw} = 98 - 77 \cos{(\omega t - 49)} - 3 \cos{(2\omega t + 67)} - 8\cos{(3\omega t + 29)}$$
$$\theta_{hw} = 93 - 65 \cos{(\omega t + 18)} + 8 \cos{(2\omega t + 74)} + 8\cos{(3\omega t + 28)}$$

## Artifacts

- {download}`manifest.json <azuma1985/artifacts/manifest.json>`
- {download}`sim_azuma1985.cfg <azuma1985/artifacts/sim/sim_azuma1985.cfg>`
- {download}`translate_summary.json <azuma1985/artifacts/sim/translate_summary.json>`
- {download}`output.h5 <azuma1985/artifacts/sim/output.h5>`
- {download}`simulation.mp4 <azuma1985/artifacts/post/simulation.mp4>`
- {download}`stick_two_wing.mp4 <azuma1985/artifacts/post/stick_two_wing.mp4>`

## 3D Animation

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

## Two-Wing Stick Animation

```{raw} html
<video
  class="case-study-video"
  controls
  loop
  autoplay
  muted
  preload="metadata"
  data-light-src="../_static/media/azuma1985/stick_two_wing.light.mp4"
  data-dark-src="../_static/media/azuma1985/stick_two_wing.dark.mp4"
>
  <source src="../_static/media/azuma1985/stick_two_wing.dark.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>
```

## Config Extract

```{literalinclude} azuma1985/artifacts/sim/sim_azuma1985.cfg
:language: ini
```

## Translation Summary Extract

```{literalinclude} azuma1985/artifacts/sim/translate_summary.json
:language: json
```

## Reproduction Commands

```bash
# Regenerate and sync all Azuma docs media/artifacts
python scripts/update_docs_media.py --only azuma1985_translate_sim azuma1985_animation_light azuma1985_animation_dark azuma1985_stick_two_wing_light azuma1985_stick_two_wing_dark

# Or only one themed animation entry
python scripts/update_docs_media.py --only azuma1985_animation_light
python scripts/update_docs_media.py --only azuma1985_animation_dark
python scripts/update_docs_media.py --only azuma1985_stick_two_wing_light
python scripts/update_docs_media.py --only azuma1985_stick_two_wing_dark
```

## References

```{bibliography}
:filter: docname in docnames
```

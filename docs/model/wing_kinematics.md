# Wing Kinematics

## Model of wing motion

Dragonflies are equipped with four wings arranged in pairs and attached to the thorax. When wing motion is observed from the side, the wingtips appear to be mostly constrained to a straight line (Fig. 1A). Generalizing this to three dimensions, the wingtips appear to be bound to a plane called the stroke plane (Fig. 1B). The orientation of the stroke plane is defined by the angle $\gamma$ which can vary during flight. Higher values of $\gamma$ (closer to the vertical) are generally associated with higher flight velocities, since the net aerodynamic force vector is roughly normal to the stroke plane. For this reason it is convenient to define $\gamma$ relative to the horizontal axis, not the body axis.

<!--
Fig. 1A and 1B side to side
-->

```{raw} html
<video
  class="case-study-video"
  controls
  loop
  autoplay
  muted
  preload="metadata"
  data-light-src="../_static/media/modeling/wing_kinematics/wing_kinematics_section1_anim1_side_y.light.mp4"
  data-dark-src="../_static/media/modeling/wing_kinematics/wing_kinematics_section1_anim1_side_y.dark.mp4"
>
  <source src="../_static/media/modeling/wing_kinematics/wing_kinematics_section1_anim1_side_y.dark.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>
```

If the wingtip is bound to a plane, it follows from simple geometry that the line extending from the wing root joint to the wingtip must be bound to a cone whose top vertex coincides with the root joint, and whose axis is normal to the stroke plane. A *coning angle* $\beta$ {cite}`azuma1988` can be defined from the stroke plane to wing cone. $\beta$ generally has a small positive value for the forewings, and a small negative value for the hindwings (Fig. 2A). The flapping of the wings within the wing cone is described by the flapping angle $\phi$ (Fig. 2B). For $\gamma = 0$, $\phi > 0$ means the wings are raised above the horizontal plane.

<!--
Fig. 2A and 2B side to side
-->

In addition to flapping, the wing motion has a pitching component. Here we take the approach of {cite}`azuma1988` and assume that the root-to-tip line introduced to describe flapping is also the pitching axis. Following {cite}`azuma1988`, we further assume that it corresponds to the quarter-chord line (the quarter-chord point at each spanwise location being the chordwise aerodynamic center). Pitching about this spanwise axis is described by the pitching angle $\psi$. $\psi = 0$ means the wing chord axis is normal to the wing cone surface. $\psi > 0$ is pitched up and $\psi < 0$ is pitched down. A classical way to visualize wing pitch, and wing motion in general, is the stick plot (Fig. 3), showing the wing crossection is a line segment with a small circle at the leading edge. The stroke planes and wing root joints are also shown in the below plot.

<!--
Fig. 3
-->

## References

```{bibliography}
:filter: docname in docnames
```

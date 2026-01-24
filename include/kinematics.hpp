#pragma once

#include "eom.hpp"

#include <cmath>

// Factory function for wing angle kinematics
// Creates AngleFunc lambda with captured kinematic parameters
//
// Parameters:
//   gam0        - Stroke plane angle
//   phi0        - Flapping amplitude
//   psim        - Mean pitch angle
//   dpsi        - Pitch oscillation amplitude
//   dlt0        - Pitch phase offset
//   phaseOffset - Per-wing phase offset (e.g., 0 for forewing, pi for hindwing)
//   omg0        - Wing beat frequency
//
inline AngleFunc makeAngleFunc(double gam0, double phi0, double psim,
                               double dpsi, double dlt0, double phaseOffset, double omg0) {
    return [=](double t) -> WingAngles {
        return {
            gam0,
            phi0 * std::cos(omg0 * t + phaseOffset),
            -phi0 * omg0 * std::sin(omg0 * t + phaseOffset),
            psim + dpsi * std::cos(omg0 * t + dlt0 + phaseOffset)
        };
    };
}

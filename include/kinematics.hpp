#pragma once

#include "eom.hpp"

#include <cmath>

// Factory function for wing angle kinematics
// Creates AngleFunc lambda with captured kinematic parameters
//
// Parameters:
//   gam0        - Mean stroke plane angle
//   dgam        - Stroke plane oscillation amplitude
//   dlt_gam     - Stroke plane phase offset
//   phi0        - Flapping amplitude
//   psim        - Mean pitch angle
//   dpsi        - Pitch oscillation amplitude
//   dlt0        - Pitch phase offset
//   phaseOffset - Per-wing phase offset (e.g., 0 for forewing, pi for hindwing)
//   omg0        - Wing beat frequency
//
inline AngleFunc makeAngleFunc(double gam0, double dgam, double dlt_gam,
                               double phi0, double psim,
                               double dpsi, double dlt0, double phaseOffset, double omg0) {
    return [=](double t) -> WingAngles {
        double phase_gam = omg0 * t + dlt_gam + phaseOffset;
        double phase_phi = omg0 * t + phaseOffset;
        double phase_psi = omg0 * t + dlt0 + phaseOffset;
        return {
            gam0 + dgam * std::cos(phase_gam),
            -dgam * omg0 * std::sin(phase_gam),
            phi0 * std::cos(phase_phi),
            -phi0 * omg0 * std::sin(phase_phi),
            psim + dpsi * std::cos(phase_psi)
        };
    };
}

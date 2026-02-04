#pragma once

#include "eom.hpp"

#include <cmath>

// Factory function for wing angle kinematics
// Creates AngleFunc lambda with captured kinematic parameters
//
// Parameters:
//   gamma_mean  - Mean stroke plane angle
//   gamma_amp   - Stroke plane oscillation amplitude
//   gamma_phase - Stroke plane phase offset
//   phi_amp     - Flapping amplitude
//   psi_mean    - Mean pitch angle
//   psi_amp     - Pitch oscillation amplitude
//   psi_phase   - Pitch phase offset
//   phaseOffset - Per-wing phase offset (e.g., 0 for forewing, pi for hindwing)
//   omega       - Wing beat frequency
//
inline AngleFunc makeAngleFunc(double gamma_mean, double gamma_amp, double gamma_phase,
                               double phi_amp, double psi_mean,
                               double psi_amp, double psi_phase, double phaseOffset, double omega) {
    return [=](double t) -> WingAngles {
        double phase_gam = omega * t + gamma_phase + phaseOffset;
        double phase_phi = omega * t + phaseOffset;
        double phase_psi = omega * t + psi_phase + phaseOffset;
        return {
            gamma_mean + gamma_amp * std::cos(phase_gam),
            -gamma_amp * omega * std::sin(phase_gam),
            phi_amp * std::cos(phase_phi),
            -phi_amp * omega * std::sin(phase_phi),
            psi_mean + psi_amp * std::cos(phase_psi)
        };
    };
}

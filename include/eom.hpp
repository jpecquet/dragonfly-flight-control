#pragma once

#include "rotation.hpp"
#include "wingforce.hpp"

#include <array>

struct Parameters {
    double lb0_f;   // Forewing length
    double lb0_h;   // Hindwing length
    double mu0_f;   // Forewing mass parameter
    double mu0_h;   // Hindwing mass parameter
    double Cd0;     // Base drag coefficient
    double Cl0;     // Base lift coefficient
    double omg0;    // Wing beat frequency
    double gam0;    // Stroke plane angle
    double phi0;    // Stroke amplitude
    double psim;    // Mean pitch angle
    double dpsi;    // Pitch oscillation amplitude
    double sig0;    // Fore/hindwing phase offset
    double dlt0;    // Pitch phase offset
};

// State vector: [x, y, z, ux, uy, uz]
using State = std::array<double, 6>;

// State derivative: [ux, uy, uz, ax, ay, az]
using StateDerivative = std::array<double, 6>;

struct WingVectors {
    SingleWingVectors fl;  // Forewing left
    SingleWingVectors fr;  // Forewing right
    SingleWingVectors hl;  // Hindwing left
    SingleWingVectors hr;  // Hindwing right
};

// Compute state derivatives (equations of motion)
StateDerivative equationOfMotion(
    double t,
    const State& state,
    const Parameters& params,
    WingVectors& wings
);

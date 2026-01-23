#pragma once

#include <cmath>

// Fixed kinematic and geometric parameters (not optimized)
struct FixedParams {
    double phi0;    // Stroke amplitude
    double sig0;    // Fore/hindwing phase offset
    double dlt0;    // Pitch phase offset
    double omg0;    // Wing beat frequency

    double lb0_f;   // Forewing length
    double lb0_h;   // Hindwing length
    double mu0_f;   // Forewing mass parameter
    double mu0_h;   // Hindwing mass parameter

    double Cd0;     // Drag coefficient at zero angle of attack
    double Cl0;     // Lift coefficient slope

    // Default values matching Python optimize.py
    static FixedParams defaults() {
        return {
            M_PI / 8.0,     // phi0
            M_PI,           // sig0
            M_PI / 2.0,     // dlt0
            8.0 * M_PI,     // omg0
            0.75,           // lb0_f
            0.75,           // lb0_h
            0.075,          // mu0_f
            0.075,          // mu0_h
            0.4,            // Cd0
            1.2             // Cl0
        };
    }
};

// Parameters being optimized
struct OptimParams {
    double gam0;    // Stroke plane angle
    double psim;    // Mean pitch angle
    double dpsi;    // Pitch amplitude
};

// Flight condition (body velocity)
struct FlightCondition {
    double ux;      // Forward velocity
    double uz;      // Vertical velocity
};

// Bounds for optimization
struct OptimBounds {
    double gam0_min, gam0_max;
    double psim_min, psim_max;
    double dpsi_min, dpsi_max;

    // Default bounds matching Python optimize.py
    static OptimBounds defaults() {
        return {
            M_PI / 2.0, M_PI / 2.0,    // gam0 fixed at 90 degrees
            0.0, M_PI / 2.0,            // psim: 0 to 90 degrees
            -M_PI / 2.0, M_PI / 2.0     // dpsi: -90 to 90 degrees
        };
    }
};

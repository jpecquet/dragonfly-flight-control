#pragma once

#include "rotation.hpp"

struct SingleWingVectors {
    Vec3 e_s;    // Stroke direction unit vector
    Vec3 e_r;    // Radial direction unit vector
    Vec3 e_c;    // Chord direction unit vector
    Vec3 lift;   // Lift force vector
    Vec3 drag;   // Drag force vector
};

// Calculate aerodynamic force on a single wing
Vec3 wingForce(
    const Vec3& ub,
    double lb0,
    double mu0,
    double Cd0,
    double Cl0,
    double phi_dot,
    const Vec3& e_s,
    const Vec3& e_r,
    const Vec3& e_c,
    SingleWingVectors& vecs
);

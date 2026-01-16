#pragma once

#include "linalg.hpp"

#include <array>
#include <functional>
#include <vector>

// Forward declaration
class Wing;

// Wing angles at a given time
struct WingAngles {
    double gam;      // Stroke plane angle
    double phi;      // Stroke angle
    double phi_dot;  // Stroke angular velocity
    double psi;      // Pitch angle
};

// Function type for computing wing angles from time
using AngleFunc = std::function<WingAngles(double t)>;

// State vector: [x, y, z, ux, uy, uz]
using State = std::array<double, 6>;

// State derivative: [ux, uy, uz, ax, ay, az]
using StateDerivative = std::array<double, 6>;

struct SingleWingVectors {
    Vec3 e_s;    // Stroke direction unit vector
    Vec3 e_r;    // Radial direction unit vector
    Vec3 e_c;    // Chord direction unit vector
    Vec3 lift;   // Lift force vector
    Vec3 drag;   // Drag force vector
};

// Compute state derivatives (equations of motion)
StateDerivative equationOfMotion(
    double t,
    const State& state,
    const std::vector<Wing>& wings,
    std::vector<SingleWingVectors>& wing_outputs
);

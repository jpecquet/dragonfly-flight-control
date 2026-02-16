#pragma once

#include "linalg.hpp"

#include <vector>

// Forward declaration
class Wing;

// State vector with position and velocity
struct State {
    Vec3 pos;  // Position [x, y, z]
    Vec3 vel;  // Velocity [ux, uy, uz]

    // Default constructor
    State() : pos(Vec3::Zero()), vel(Vec3::Zero()) {}

    // Construct from components
    State(const Vec3& p, const Vec3& v) : pos(p), vel(v) {}
};

// State derivative: [ux, uy, uz, ax, ay, az]
struct StateDerivative {
    Vec3 vel;   // Velocity (derivative of position)
    Vec3 accel; // Acceleration (derivative of velocity)

    StateDerivative() : vel(Vec3::Zero()), accel(Vec3::Zero()) {}
    StateDerivative(const Vec3& v, const Vec3& a) : vel(v), accel(a) {}
};

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

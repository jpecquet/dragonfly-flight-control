#pragma once

#include <Eigen/Dense>
#include <array>

using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;

// Rotation matrices about principal axes
Mat3 rotX(double angle);
Mat3 rotY(double angle);
Mat3 rotZ(double angle);

// Composite Euler rotations (intrinsic, matching scipy convention)
Mat3 eulerYX(double y_angle, double x_angle);
Mat3 eulerYXY(double y1_angle, double x_angle, double y2_angle);

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

struct SingleWingVectors {
    Vec3 e_s;    // Stroke direction unit vector
    Vec3 e_r;    // Radial direction unit vector
    Vec3 e_c;    // Chord direction unit vector
    Vec3 lift;   // Lift force vector
    Vec3 drag;   // Drag force vector
};

struct WingVectors {
    SingleWingVectors fl;  // Forewing left
    SingleWingVectors fr;  // Forewing right
    SingleWingVectors hl;  // Hindwing left
    SingleWingVectors hr;  // Hindwing right
};

// Calculate aerodynamic force on a single wing
Vec3 wingForce(
    const Vec3& ub,
    double lb0,
    double mu0,
    double Cd0,
    double Cl0,
    double psi_dot,
    const Vec3& e_s,
    const Vec3& e_r,
    const Vec3& e_c,
    SingleWingVectors& vecs
);

// Compute state derivatives (equations of motion)
StateDerivative equationOfMotion(
    double t,
    const State& state,
    const Parameters& params,
    WingVectors& wings
);

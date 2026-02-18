#pragma once

#include "linalg.hpp"

// Rotation matrices about principal axes
Mat3 rotX(double angle);
Mat3 rotY(double angle);
Mat3 rotZ(double angle);

// Wing orientation basis vectors
struct WingOrientation {
    Vec3 e_s;  // Stroke direction
    Vec3 e_r;  // Radial (span) direction
    Vec3 e_c;  // Chord direction
};

// Compute wing orientation from Euler angles (radians)
// gam: stroke plane angle, phi: stroke angle, psi: pitch angle, cone: coning angle
WingOrientation computeWingOrientation(double gam, double phi, double psi, double cone, bool is_left);

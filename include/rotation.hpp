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
// gam: stroke plane angle
// phi: flapping azimuth around cone axis
// psi: pitch angle
// cone: beta angle from stroke plane, giving cone half-angle (pi/2 - cone)
WingOrientation computeWingOrientation(double gam, double phi, double psi, double cone, bool is_left);

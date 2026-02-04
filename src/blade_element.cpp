#include "blade_element.hpp"

#include <cmath>

namespace {
constexpr double VELOCITY_THRESHOLD_SQ = 1e-20;  // Squared threshold for zero velocity
}

BladeElement::BladeElement(double Cd0, double Cl0)
    : Cd0_(Cd0), Cl0_(Cl0) {}

Vec3 BladeElement::computeForce(const Vec3& u, const Vec3& e_r, const Vec3& e_c,
                                 double force_coefficient, Vec3& lift, Vec3& drag) const {
    // Use squared norm to avoid sqrt, then compute inverse once
    double U_sq = u.squaredNorm();

    // Handle zero velocity case
    if (U_sq < VELOCITY_THRESHOLD_SQ) {
        lift = Vec3::Zero();
        drag = Vec3::Zero();
        return Vec3::Zero();
    }

    double U_inv = 1.0 / std::sqrt(U_sq);

    // Angle of attack: alpha is angle between velocity u and chord e_c
    // c_alpha = cos(alpha), s_alpha = sin(alpha)
    double c_alpha = u.dot(e_c) * U_inv;
    double s_alpha = u.cross(e_c).dot(e_r) * U_inv;

    // Drag and lift coefficients (Cd0 + 2*sin^2(alpha), Cl0*sin(2*alpha))
    double Cd = (Cd0_ + 1.0) - (2.0 * c_alpha * c_alpha - 1.0);
    double Cl = 2.0 * Cl0_ * s_alpha * c_alpha;

    // Drag direction (opposite to velocity, reuse U_inv)
    Vec3 e_d = -u * U_inv;

    // Lift direction (perpendicular to drag and spanwise direction)
    Vec3 e_l = e_d.cross(e_r);

    // Force vectors (U_sq already computed)
    drag = force_coefficient * Cd * U_sq * e_d;
    lift = force_coefficient * Cl * U_sq * e_l;

    return lift + drag;
}

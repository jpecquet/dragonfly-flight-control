#include "blade_element.hpp"

BladeElement::BladeElement(double Cd0, double Cl0)
    : Cd0_(Cd0), Cl0_(Cl0) {}

Vec3 BladeElement::computeForce(const Vec3& u, const Vec3& e_r, const Vec3& e_c,
                                 double force_coefficient, Vec3& lift, Vec3& drag) const {
    double U = u.norm();
    
    // Handle zero velocity case
    if (U < 1e-10) {
        lift = Vec3::Zero();
        drag = Vec3::Zero();
        return Vec3::Zero();
    }

    // Angle of attack components
    double c_alpha = u.dot(e_c) / U;
    double s_alpha = u.cross(e_c).dot(e_r);

    // Drag and lift coefficients
    double Cd = (Cd0_ + 1.0) - (2.0 * c_alpha * c_alpha - 1.0);
    double Cl = 2.0 * Cl0_ * s_alpha * c_alpha;

    // Drag direction (opposite to velocity)
    Vec3 e_d = -u / U;

    // Lift direction (perpendicular to drag and spanwise direction)
    Vec3 e_l = e_d.cross(e_r);

    // Force vectors
    drag = force_coefficient * Cd * U * U * e_d;
    lift = force_coefficient * Cl * U * U * e_l;

    return lift + drag;
}

#include "wingforce.hpp"

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
) {
    // Store orientation vectors
    vecs.e_s = e_s;
    vecs.e_r = e_r;
    vecs.e_c = e_c;

    // Wing velocity at 2/3 span
    Vec3 uw = ub + (2.0 / 3.0) * lb0 * phi_dot * e_s;

    // Velocity component perpendicular to radial direction
    Vec3 u = uw - uw.dot(e_r) * e_r;
    double U = u.norm();

    if (U < 1e-10) {
        vecs.lift = Vec3::Zero();
        vecs.drag = Vec3::Zero();
        return Vec3::Zero();
    }

    // Angle of attack components
    double c_alpha = u.dot(e_c) / U;
    double s_alpha = u.cross(e_c).dot(e_r);

    // Drag and lift coefficients (angle-dependent)
    double Cd = (Cd0 + 1.0) - (2.0 * c_alpha * c_alpha - 1.0);
    double Cl = 2.0 * Cl0 * s_alpha * c_alpha;

    // Drag direction (opposite to velocity)
    Vec3 e_d = -u / U;

    // Lift direction (perpendicular to velocity and radial)
    Vec3 e_l = e_d.cross(e_r);

    // Force magnitude coefficient
    double coeff = 0.5 * mu0 / lb0 * U * U;

    // Force vectors
    vecs.drag = coeff * Cd * e_d;
    vecs.lift = coeff * Cl * e_l;

    return vecs.lift + vecs.drag;
}

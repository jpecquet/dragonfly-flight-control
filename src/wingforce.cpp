#include "wingforce.hpp"
#include "blade_element.hpp"

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

    // Wing velocity projected on plane normal to spanwise direction
    Vec3 u = uw - uw.dot(e_r) * e_r;

    double force_coefficient = 0.5 * mu0 / lb0;

    // Compute aerodynamic force using blade element
    BladeElement blade(Cd0, Cl0);
    return blade.computeForce(u, e_r, e_c, force_coefficient, vecs.lift, vecs.drag);
}

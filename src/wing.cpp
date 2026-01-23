#include "wing.hpp"

Wing::Wing(const std::string& name, double mu0, double lb0, WingSide side,
           double Cd0, double Cl0, AngleFunc angleFunc)
    : name_(name), mu0_(mu0), lb0_(lb0), side_(side), blade_(Cd0, Cl0), angleFunc_(std::move(angleFunc)) {}

Vec3 Wing::computeForce(
    double t,
    const Vec3& ub,
    SingleWingVectors& vecs
) const {
    // Get angles from the angle function
    WingAngles angles = angleFunc_(t);

    // Compute wing orientation using shared rotation logic
    WingOrientation orient = computeWingOrientation(
        angles.gam, angles.phi, angles.psi, side_ == WingSide::Left);

    // Store orientation vectors
    vecs.e_s = orient.e_s;
    vecs.e_r = orient.e_r;
    vecs.e_c = orient.e_c;

    // Wing velocity at 2/3 span
    Vec3 uw = ub + (2.0 / 3.0) * lb0_ * angles.phi_dot * orient.e_s;

    // Wing velocity projected on plane normal to spanwise direction
    Vec3 u = uw - uw.dot(orient.e_r) * orient.e_r;

    double force_coefficient = 0.5 * mu0_ / lb0_;

    // Compute aerodynamic force using blade element
    return blade_.computeForce(u, orient.e_r, orient.e_c, force_coefficient, vecs.lift, vecs.drag);
}

#include "wing.hpp"

namespace {
constexpr double SPAN_EVAL_POINT = 2.0 / 3.0;
}  // namespace

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
    // Contribution from flapping (phi_dot): velocity in stroke direction
    Vec3 v_phi = SPAN_EVAL_POINT * lb0_ * angles.phi_dot * orient.e_s;

    // Contribution from stroke plane rotation (gam_dot): omega_gam x r
    // omega_gam = sign * gam_dot * ey (sign depends on wing side due to rotation convention)
    // r = (2/3) * lb0 * e_r
    // Note: v_gam is proportional to sin(phi) since e_r depends on phi
    double gam_sign = (side_ == WingSide::Left) ? -1.0 : 1.0;
    Vec3 ey(0, 1, 0);
    Vec3 omega_gam = gam_sign * angles.gam_dot * ey;
    Vec3 r = SPAN_EVAL_POINT * lb0_ * orient.e_r;
    Vec3 v_gam = omega_gam.cross(r);

    Vec3 uw = ub + v_phi + v_gam;

    // Wing velocity projected on plane normal to spanwise direction
    Vec3 u = uw - uw.dot(orient.e_r) * orient.e_r;

    double force_coefficient = 0.5 * mu0_ / lb0_;

    // Compute aerodynamic force using blade element
    return blade_.computeForce(u, orient.e_r, orient.e_c, force_coefficient, vecs.lift, vecs.drag);
}

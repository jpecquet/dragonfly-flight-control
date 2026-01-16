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

    // Base vectors
    Vec3 ex(1, 0, 0);
    Vec3 ey(0, 1, 0);
    Vec3 ez(0, 0, 1);

    Mat3 Rs, Rp;
    Vec3 e_c;

    if (side_ == WingSide::Left) {
        // Left wing: standard orientation
        Rs = rotY(-angles.gam) * rotZ(-angles.phi);
        Rp = Rs * rotY(-angles.psi);
        e_c = Rp * ez * -1.0;  // Chord direction negated for left
    } else {
        // Right wing: flipped about X axis
        Rs = rotX(-M_PI) * rotY(angles.gam) * rotZ(-angles.phi);
        Rp = Rs * rotY(angles.psi);
        e_c = Rp * ez;
    }

    Vec3 e_s = Rs * ex;  // Stroke direction
    Vec3 e_r = Rs * ey;  // Radial (span) direction

    // Store orientation vectors
    vecs.e_s = e_s;
    vecs.e_r = e_r;
    vecs.e_c = e_c;

    // Wing velocity at 2/3 span
    Vec3 uw = ub + (2.0 / 3.0) * lb0_ * angles.phi_dot * e_s;

    // Wing velocity projected on plane normal to spanwise direction
    Vec3 u = uw - uw.dot(e_r) * e_r;

    double force_coefficient = 0.5 * mu0_ / lb0_;

    // Compute aerodynamic force using blade element
    return blade_.computeForce(u, e_r, e_c, force_coefficient, vecs.lift, vecs.drag);
}

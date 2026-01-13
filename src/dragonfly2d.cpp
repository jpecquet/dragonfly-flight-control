#include "dragonfly2d.hpp"

#include <cmath>

// Rotation matrix about X axis
Mat3 rotX(double angle) {
    double c = std::cos(angle);
    double s = std::sin(angle);
    Mat3 R;
    R << 1,  0,  0,
         0,  c, -s,
         0,  s,  c;
    return R;
}

// Rotation matrix about Y axis
Mat3 rotY(double angle) {
    double c = std::cos(angle);
    double s = std::sin(angle);
    Mat3 R;
    R <<  c, 0, s,
          0, 1, 0,
         -s, 0, c;
    return R;
}

// Rotation matrix about Z axis
Mat3 rotZ(double angle) {
    double c = std::cos(angle);
    double s = std::sin(angle);
    Mat3 R;
    R << c, -s, 0,
         s,  c, 0,
         0,  0, 1;
    return R;
}

// Intrinsic Euler rotation: first Y, then X (in rotated frame)
// R = Rx(x_angle) * Ry(y_angle)
Mat3 eulerYX(double y_angle, double x_angle) {
    return rotX(x_angle) * rotY(y_angle);
}

// Intrinsic Euler rotation: Y, then X, then Y (in successively rotated frames)
// R = Ry(y2_angle) * Rx(x_angle) * Ry(y1_angle)
Mat3 eulerYXY(double y1_angle, double x_angle, double y2_angle) {
    return rotY(y2_angle) * rotX(x_angle) * rotY(y1_angle);
}

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

StateDerivative equationOfMotion(
    double t,
    const State& state,
    const Parameters& params,
    WingVectors& wings
) {
    // Extract velocity from state
    Vec3 ub(state[3], state[4], state[5]);

    const auto& p = params;

    // Stroke plane angles (same for fore and hind)
    double gam_f = p.gam0;
    double gam_h = p.gam0;

    // Stroke angles
    double phi_f = p.phi0 * std::cos(p.omg0 * t);
    double phi_h = p.phi0 * std::cos(p.omg0 * t + p.sig0);

    // Pitch angles
    double psi_f = p.psim + p.dpsi * std::cos(p.omg0 * t + p.dlt0);
    double psi_h = p.psim + p.dpsi * std::cos(p.omg0 * t + p.dlt0 + p.sig0);

    // Stroke angular velocities
    double phi_dot_f = -p.phi0 * p.omg0 * std::sin(p.omg0 * t);
    double phi_dot_h = -p.phi0 * p.omg0 * std::sin(p.omg0 * t + p.sig0);

    // Base vectors
    Vec3 ex(1, 0, 0);
    Vec3 ey(0, 1, 0);
    Vec3 ez(0, 0, 1);

    // Right forewing
    Mat3 Rr = rotZ(M_PI);
    Mat3 Rs_fr = Rr * eulerYX(-(M_PI / 2.0 - gam_f), phi_f);
    Mat3 Rp_fr = Rr * eulerYXY(-(M_PI / 2.0 - gam_f), phi_f, psi_f);
    Vec3 e_s_fr = Rs_fr * ez;
    Vec3 e_r_fr = Rs_fr * ey;
    Vec3 e_c_fr = Rp_fr * ex * -1.0;
    Vec3 a_fr = wingForce(ub, p.lb0_f, p.mu0_f, p.Cd0, p.Cl0, phi_dot_f,
                          e_s_fr, e_r_fr, e_c_fr, wings.fr);

    // Left forewing
    Mat3 Rs_fl = eulerYX(M_PI / 2.0 - gam_f, phi_f);
    Mat3 Rp_fl = eulerYXY(M_PI / 2.0 - gam_f, phi_f, -psi_f);
    Vec3 e_s_fl = Rs_fl * ez;
    Vec3 e_r_fl = Rs_fl * ey;
    Vec3 e_c_fl = Rp_fl * ex;
    Vec3 a_fl = wingForce(ub, p.lb0_f, p.mu0_f, p.Cd0, p.Cl0, phi_dot_f,
                          e_s_fl, e_r_fl, e_c_fl, wings.fl);

    // Right hindwing
    Mat3 Rs_hr = Rr * eulerYX(-(M_PI / 2.0 - gam_h), phi_h);
    Mat3 Rp_hr = Rr * eulerYXY(-(M_PI / 2.0 - gam_h), phi_h, psi_h);
    Vec3 e_s_hr = Rs_hr * ez;
    Vec3 e_r_hr = Rs_hr * ey;
    Vec3 e_c_hr = Rp_hr * ex * -1.0;
    Vec3 a_hr = wingForce(ub, p.lb0_h, p.mu0_h, p.Cd0, p.Cl0, phi_dot_h,
                          e_s_hr, e_r_hr, e_c_hr, wings.hr);

    // Left hindwing
    Mat3 Rs_hl = eulerYX(M_PI / 2.0 - gam_h, phi_h);
    Mat3 Rp_hl = eulerYXY(M_PI / 2.0 - gam_h, phi_h, -psi_h);
    Vec3 e_s_hl = Rs_hl * ez;
    Vec3 e_r_hl = Rs_hl * ey;
    Vec3 e_c_hl = Rp_hl * ex;
    Vec3 a_hl = wingForce(ub, p.lb0_h, p.mu0_h, p.Cd0, p.Cl0, phi_dot_h,
                          e_s_hl, e_r_hl, e_c_hl, wings.hl);

    // Total acceleration (aerodynamic forces + gravity)
    Vec3 a = a_fl + a_fr + a_hl + a_hr - Vec3(0.0, 0.0, 1.0);

    return {state[3], state[4], state[5], a.x(), a.y(), a.z()};
}

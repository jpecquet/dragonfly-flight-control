#include "rotation.hpp"

#include <cmath>

Mat3 rotX(double angle) {
    double c = std::cos(angle);
    double s = std::sin(angle);
    Mat3 R;
    R << 1,  0,  0,
         0,  c, -s,
         0,  s,  c;
    return R;
}

Mat3 rotY(double angle) {
    double c = std::cos(angle);
    double s = std::sin(angle);
    Mat3 R;
    R <<  c, 0, s,
          0, 1, 0,
         -s, 0, c;
    return R;
}

Mat3 rotZ(double angle) {
    double c = std::cos(angle);
    double s = std::sin(angle);
    Mat3 R;
    R << c, -s, 0,
         s,  c, 0,
         0,  0, 1;
    return R;
}

WingOrientation computeWingOrientation(double gam, double phi, double psi, double cone, bool is_left) {
    Vec3 ex(1, 0, 0);
    Vec3 ey(0, 1, 0);
    Vec3 ez(0, 0, 1);

    Mat3 Rs, Rp;
    Vec3 e_c;

    if (is_left) {
        Rs = rotY(-gam) * rotX(-cone) * rotZ(-phi);
        Rp = Rs * rotY(-psi);
        e_c = Rp * ez * -1.0;
    } else {
        Rs = rotX(-M_PI) * rotY(gam) * rotX(cone) * rotZ(-phi);
        Rp = Rs * rotY(psi);
        e_c = Rp * ez;
    }

    return {Rs * ex, Rs * ey, e_c};
}

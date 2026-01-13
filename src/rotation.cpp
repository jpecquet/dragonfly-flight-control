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

Mat3 eulerYX(double y_angle, double x_angle) {
    return rotX(x_angle) * rotY(y_angle);
}

Mat3 eulerYXY(double y1_angle, double x_angle, double y2_angle) {
    return rotY(y2_angle) * rotX(x_angle) * rotY(y1_angle);
}

#pragma once

#include <Eigen/Dense>

using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;

// Rotation matrices about principal axes
Mat3 rotX(double angle);
Mat3 rotY(double angle);
Mat3 rotZ(double angle);

// Composite Euler rotations (intrinsic, matching scipy convention)
Mat3 eulerYX(double y_angle, double x_angle);
Mat3 eulerYXY(double y1_angle, double x_angle, double y2_angle);

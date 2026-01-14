#pragma once

#include "linalg.hpp"

class BladeElement {
public:
    BladeElement(double Cd0, double Cl0);

    // Compute aerodynamic force from velocity perpendicular to radial direction
    // Returns total force (lift + drag), also outputs individual components
    Vec3 computeForce(const Vec3& u, const Vec3& e_r, const Vec3& e_c,
                      double force_coefficient, Vec3& lift, Vec3& drag) const;

private:
    double Cd0_;
    double Cl0_;
};

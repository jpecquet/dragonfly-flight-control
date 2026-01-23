#pragma once

#include "eom.hpp"

#include <cmath>

// Factory functions for wing angle kinematics
// These create AngleFunc lambdas with captured kinematic parameters

inline AngleFunc makeForewingAngleFunc(double gam0, double phi0, double psim,
                                        double dpsi, double dlt0, double omg0) {
    return [=](double t) -> WingAngles {
        return {
            gam0,
            phi0 * std::cos(omg0 * t),
            -phi0 * omg0 * std::sin(omg0 * t),
            psim + dpsi * std::cos(omg0 * t + dlt0)
        };
    };
}

inline AngleFunc makeHindwingAngleFunc(double gam0, double phi0, double psim,
                                        double dpsi, double dlt0, double sig0, double omg0) {
    return [=](double t) -> WingAngles {
        return {
            gam0,
            phi0 * std::cos(omg0 * t + sig0),
            -phi0 * omg0 * std::sin(omg0 * t + sig0),
            psim + dpsi * std::cos(omg0 * t + dlt0 + sig0)
        };
    };
}

#include "blade_element.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace {
constexpr double VELOCITY_THRESHOLD_SQ = 1e-20;  // Squared threshold for zero velocity

double dragCoefficient(const BladeElementAeroParams& params, double alpha) {
    switch (params.drag_model) {
        case DragCoefficientModel::Sinusoidal: {
            const double s = std::sin(alpha - params.Cd_alpha_neutral);
            return params.Cd_min + (params.Cd_max - params.Cd_min) * s * s;
        }
    }
    throw std::runtime_error("Unsupported drag coefficient model");
}

double liftCoefficient(const BladeElementAeroParams& params, double alpha) {
    switch (params.lift_model) {
        case LiftCoefficientModel::Sinusoidal:
            return params.Cl0 * std::sin(2.0 * (alpha - params.Cl_alpha_neutral));
        case LiftCoefficientModel::Linear: {
            const double unclamped = params.Cl_alpha_slope * (alpha - params.Cl_alpha_neutral);
            return std::max(params.Cl_min, std::min(params.Cl_max, unclamped));
        }
    }
    throw std::runtime_error("Unsupported lift coefficient model");
}
}

BladeElement::BladeElement(const BladeElementAeroParams& params)
    : params_(params) {
    if (!std::isfinite(params_.Cd_min)) {
        throw std::runtime_error("Cd_min must be finite");
    }
    if (!std::isfinite(params_.Cd_max)) {
        throw std::runtime_error("Cd_max must be finite");
    }
    if (params_.Cd_max < params_.Cd_min) {
        throw std::runtime_error("Cd_max must be >= Cd_min");
    }
    if (!std::isfinite(params_.Cd_alpha_neutral)) {
        throw std::runtime_error("Cd_alpha_neutral must be finite");
    }
    if (!std::isfinite(params_.Cl0)) {
        throw std::runtime_error("Cl0 must be finite");
    }
    if (params_.lift_model == LiftCoefficientModel::Linear) {
        if (!std::isfinite(params_.Cl_alpha_slope) || !std::isfinite(params_.Cl_alpha_neutral)) {
            throw std::runtime_error("Linear lift model requires finite Cl_alpha_slope and Cl_alpha_neutral");
        }
        if (!std::isfinite(params_.Cl_min) || !std::isfinite(params_.Cl_max)) {
            throw std::runtime_error("Linear lift model requires finite Cl_min and Cl_max");
        }
        if (params_.Cl_min > params_.Cl_max) {
            throw std::runtime_error("Linear lift model requires Cl_min <= Cl_max");
        }
    }
}

BladeElement::BladeElement(double Cd_min, double Cl0)
    : BladeElement(BladeElementAeroParams::sinusoidal(Cd_min, Cl0)) {}

Vec3 BladeElement::computeForce(const Vec3& u, const Vec3& e_r, const Vec3& e_c,
                                 double force_coefficient, Vec3& lift, Vec3& drag) const {
    // Use squared norm to avoid sqrt, then compute inverse once
    double U_sq = u.squaredNorm();

    // Handle zero velocity case
    if (U_sq < VELOCITY_THRESHOLD_SQ) {
        lift = Vec3::Zero();
        drag = Vec3::Zero();
        return Vec3::Zero();
    }

    double U_inv = 1.0 / std::sqrt(U_sq);

    // Angle of attack: alpha is angle between velocity u and chord e_c
    // c_alpha = cos(alpha), s_alpha = sin(alpha)
    double c_alpha = u.dot(e_c) * U_inv;
    double s_alpha = u.cross(e_c).dot(e_r) * U_inv;
    double alpha = std::atan2(s_alpha, c_alpha);

    // Drag and lift coefficients from configured models.
    double Cd = dragCoefficient(params_, alpha);
    double Cl = liftCoefficient(params_, alpha);

    // Drag direction (opposite to velocity, reuse U_inv)
    Vec3 e_d = -u * U_inv;

    // Lift direction (perpendicular to drag and spanwise direction)
    Vec3 e_l = e_d.cross(e_r);

    // Force vectors (U_sq already computed)
    drag = force_coefficient * Cd * U_sq * e_d;
    lift = force_coefficient * Cl * U_sq * e_l;

    return lift + drag;
}

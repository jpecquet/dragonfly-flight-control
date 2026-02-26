#pragma once

#include "linalg.hpp"

enum class DragCoefficientModel {
    Sinusoidal,
    PiecewiseLinear
};

enum class LiftCoefficientModel {
    Sinusoidal,
    Linear,
    PiecewiseLinear
};

inline const char* toString(DragCoefficientModel model) {
    switch (model) {
        case DragCoefficientModel::Sinusoidal: return "sinusoidal";
        case DragCoefficientModel::PiecewiseLinear: return "piecewise_linear";
    }
    return "unknown";
}

inline const char* toString(LiftCoefficientModel model) {
    switch (model) {
        case LiftCoefficientModel::Sinusoidal: return "sinusoidal";
        case LiftCoefficientModel::Linear: return "linear";
        case LiftCoefficientModel::PiecewiseLinear: return "piecewise_linear";
    }
    return "unknown";
}

namespace aero_defaults {
constexpr double kWang2004CdMin = 0.4;
constexpr double kWang2004Cl0 = 1.2;
}

struct BladeElementAeroParams {
    DragCoefficientModel drag_model = DragCoefficientModel::Sinusoidal;
    LiftCoefficientModel lift_model = LiftCoefficientModel::Sinusoidal;

    // Sinusoidal coefficients:
    //   Cd(alpha) = Cd_min + (Cd_max - Cd_min)*sin^2(alpha - Cd_alpha_neutral)
    //   Cl(alpha) = Cl0*sin(2*(alpha - Cl_alpha_neutral))
    double Cd_min = aero_defaults::kWang2004CdMin;
    double Cd_max = aero_defaults::kWang2004CdMin + 2.0;
    double Cd_alpha_neutral = 0.0;
    double Cl0 = aero_defaults::kWang2004Cl0;

    // Linear lift model:
    //   Cl(alpha) = Cl_alpha_slope * (alpha - Cl_alpha_neutral), clamped to [Cl_min, Cl_max]
    double Cl_alpha_slope = 0.0;
    double Cl_alpha_neutral = 0.0;
    double Cl_min = -1.0e12;
    double Cl_max = 1.0e12;

    static BladeElementAeroParams sinusoidal(double Cd_min, double Cl0) {
        BladeElementAeroParams params;
        params.drag_model = DragCoefficientModel::Sinusoidal;
        params.lift_model = LiftCoefficientModel::Sinusoidal;
        params.Cd_min = Cd_min;
        params.Cd_max = Cd_min + 2.0;
        params.Cl0 = Cl0;
        return params;
    }
};

class BladeElement {
public:
    explicit BladeElement(const BladeElementAeroParams& params);
    BladeElement(double Cd_min, double Cl0);

    // Compute aerodynamic force from velocity perpendicular to radial direction
    // Returns total force (lift + drag), also outputs individual components
    Vec3 computeForce(const Vec3& u, const Vec3& e_r, const Vec3& e_c,
                      double force_coefficient, Vec3& lift, Vec3& drag) const;

private:
    BladeElementAeroParams params_;
};

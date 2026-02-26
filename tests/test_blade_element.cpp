#include "blade_element.hpp"

#include <cmath>
#include <iostream>
#include <vector>

// Analytical formulas for lift and drag coefficients
// Cd(α) = Cd_min + (Cd_max - Cd_min)*sin²(α)
// Cl(α) = Cl0*sin(2α)

double analyticalCd(double Cd_min, double Cd_max, double alpha) {
    double s = std::sin(alpha);
    return Cd_min + (Cd_max - Cd_min) * s * s;
}

double analyticalCdShifted(double Cd_min, double Cd_max, double alpha, double alpha_neutral) {
    const double a = alpha - alpha_neutral;
    const double s = std::sin(a);
    return Cd_min + (Cd_max - Cd_min) * s * s;
}

double analyticalCl(double Cl0, double alpha) {
    return Cl0 * std::sin(2.0 * alpha);
}

double analyticalClShifted(double Cl0, double alpha, double alpha_neutral) {
    return Cl0 * std::sin(2.0 * (alpha - alpha_neutral));
}

double analyticalClLinear(double slope, double alpha_neutral, double alpha, double cl_min, double cl_max) {
    double cl = slope * (alpha - alpha_neutral);
    if (cl < cl_min) cl = cl_min;
    if (cl > cl_max) cl = cl_max;
    return cl;
}

int main() {
    std::cout << "Blade Element Test: Verifying Cd and Cl against analytical formulas\n";
    std::cout << "===================================================================\n\n";

    // Aerodynamic coefficients
    double Cd_min = 0.4;
    double Cl0 = 1.2;

    std::cout << "Parameters: Cd_min = " << Cd_min << ", Cl0 = " << Cl0 << "\n\n";

    BladeElement blade(Cd_min, Cl0);

    // Set up coordinate frame:
    // e_r = radial direction (spanwise), pointing along Y
    // e_c = chord direction, pointing along X
    // Velocity will be in the X-Z plane (perpendicular to span)
    Vec3 e_r(0.0, 1.0, 0.0);
    Vec3 e_c(1.0, 0.0, 0.0);

    // Use unit force coefficient and unit velocity magnitude for easy extraction
    double force_coefficient = 1.0;
    double U = 1.0;

    // Sweep angle of attack from 0 to 90 degrees
    int n_angles = 19;  // 0, 5, 10, ..., 90 degrees
    double max_Cd_error = 0.0;
    double max_Cl_error = 0.0;

    std::cout << "  Alpha(deg)     Cd_sim    Cd_exact      Cl_sim    Cl_exact\n";
    std::cout << "  ---------------------------------------------------------\n";

    for (int i = 0; i <= n_angles; ++i) {
        double alpha_deg = i * 5.0;
        double alpha = alpha_deg * M_PI / 180.0;

        // Velocity at angle alpha to chord direction
        // When alpha = 0: u aligned with e_c (flow along chord)
        // When alpha = 90: u perpendicular to e_c (flow normal to chord)
        Vec3 u = U * (std::cos(alpha) * e_c + std::sin(alpha) * Vec3(0.0, 0.0, 1.0));

        Vec3 lift, drag;
        blade.computeForce(u, e_r, e_c, force_coefficient, lift, drag);

        // Extract coefficients from force magnitudes
        // |drag| = force_coefficient * Cd * U^2, with force_coefficient = U = 1
        double Cd_sim = drag.norm();
        double Cl_sim = lift.norm();

        // Analytical values (magnitudes)
        double Cd_exact = analyticalCd(Cd_min, Cd_min + 2.0, alpha);
        double Cl_exact = std::abs(analyticalCl(Cl0, alpha));

        // Track errors
        double Cd_error = std::abs(Cd_sim - Cd_exact);
        double Cl_error = std::abs(Cl_sim - Cl_exact);
        max_Cd_error = std::max(max_Cd_error, Cd_error);
        max_Cl_error = std::max(max_Cl_error, Cl_error);

        printf("  %6.1f      %8.5f  %8.5f    %8.5f  %8.5f\n",
               alpha_deg, Cd_sim, Cd_exact, Cl_sim, Cl_exact);
    }

    std::cout << "\nMaximum errors:\n";
    std::cout << "  Cd error: " << max_Cd_error << "\n";
    std::cout << "  Cl error: " << max_Cl_error << "\n\n";

    // Pass/fail criteria
    double tolerance = 1e-10;
    bool passed = (max_Cd_error < tolerance) && (max_Cl_error < tolerance);

    // Shifted sinusoidal test for drag/lift neutral angles
    BladeElementAeroParams shifted_params;
    shifted_params.drag_model = DragCoefficientModel::Sinusoidal;
    shifted_params.lift_model = LiftCoefficientModel::Sinusoidal;
    shifted_params.Cd_min = Cd_min;
    shifted_params.Cd_max = 2.05;
    shifted_params.Cd_alpha_neutral = 0.2;
    shifted_params.Cl0 = Cl0;
    shifted_params.Cl_alpha_neutral = -0.15;
    BladeElement blade_shifted(shifted_params);

    std::cout << "\nShifted sinusoidal model spot checks:\n";
    for (double alpha_deg : std::vector<double>{-20.0, 10.0, 35.0, 70.0}) {
        const double alpha = alpha_deg * M_PI / 180.0;
        Vec3 u = U * (std::cos(alpha) * e_c + std::sin(alpha) * Vec3(0.0, 0.0, 1.0));
        Vec3 lift, drag;
        blade_shifted.computeForce(u, e_r, e_c, force_coefficient, lift, drag);

        const double Cd_sim = drag.norm();
        const double Cl_sim = lift.norm();
        const double Cd_exact = analyticalCdShifted(
            shifted_params.Cd_min, shifted_params.Cd_max, alpha, shifted_params.Cd_alpha_neutral
        );
        const double Cl_exact = std::abs(analyticalClShifted(Cl0, alpha, shifted_params.Cl_alpha_neutral));

        const double cd_err = std::abs(Cd_sim - Cd_exact);
        const double cl_err = std::abs(Cl_sim - Cl_exact);
        max_Cd_error = std::max(max_Cd_error, cd_err);
        max_Cl_error = std::max(max_Cl_error, cl_err);

        printf("  alpha=%6.1f deg: Cd_sim=%8.5f Cd_exact=%8.5f Cl_sim=%8.5f |Cl_exact|=%8.5f\n",
               alpha_deg, Cd_sim, Cd_exact, Cl_sim, Cl_exact);
    }

    // Linear lift test with sinusoidal drag
    BladeElementAeroParams linear_params;
    linear_params.drag_model = DragCoefficientModel::Sinusoidal;
    linear_params.lift_model = LiftCoefficientModel::Linear;
    linear_params.Cd_min = Cd_min;
    linear_params.Cl_alpha_slope = 2.0;
    linear_params.Cl_alpha_neutral = 0.1;
    linear_params.Cl_min = -0.4;
    linear_params.Cl_max = 0.8;
    BladeElement blade_linear(linear_params);

    std::cout << "\nLinear lift model spot checks:\n";
    const std::vector<double> test_alpha_deg = {-30.0, 0.0, 15.0, 30.0, 60.0};
    for (double alpha_deg : test_alpha_deg) {
        const double alpha = alpha_deg * M_PI / 180.0;
        Vec3 u = U * (std::cos(alpha) * e_c + std::sin(alpha) * Vec3(0.0, 0.0, 1.0));
        Vec3 lift, drag;
        blade_linear.computeForce(u, e_r, e_c, force_coefficient, lift, drag);

        const double Cd_sim = drag.norm();
        const double Cl_sim = lift.norm();
        const double Cd_exact = analyticalCd(Cd_min, Cd_min + 2.0, alpha);
        const double Cl_exact_signed = analyticalClLinear(
            linear_params.Cl_alpha_slope,
            linear_params.Cl_alpha_neutral,
            alpha,
            linear_params.Cl_min,
            linear_params.Cl_max
        );
        const double Cl_exact = std::abs(Cl_exact_signed);

        const double cd_err = std::abs(Cd_sim - Cd_exact);
        const double cl_err = std::abs(Cl_sim - Cl_exact);
        max_Cd_error = std::max(max_Cd_error, cd_err);
        max_Cl_error = std::max(max_Cl_error, cl_err);

        printf("  alpha=%6.1f deg: Cd_sim=%8.5f Cd_exact=%8.5f Cl_sim=%8.5f |Cl_exact|=%8.5f\n",
               alpha_deg, Cd_sim, Cd_exact, Cl_sim, Cl_exact);
    }

    passed = passed && (max_Cd_error < tolerance) && (max_Cl_error < tolerance);

    // Piecewise-linear (Azuma 1985) spot checks
    // Verify Cl and Cd at specific alpha values against known formulas
    BladeElementAeroParams pw_params;
    pw_params.drag_model = DragCoefficientModel::PiecewiseLinear;
    pw_params.lift_model = LiftCoefficientModel::PiecewiseLinear;
    BladeElement blade_pw(pw_params);

    std::cout << "\nPiecewise-linear (Azuma 1985) spot checks:\n";

    struct PwCheck {
        double alpha_deg;
        double expected_Cl;
        double expected_Cd;
    };
    std::vector<PwCheck> pw_checks = {
        { -60.0, -0.03*(-60.0) - 2.6, -0.04*(-60.0) - 0.8 },  // outer ramps
        { -50.0, -1.1,                 -0.04*(-50.0) - 0.8 },   // Cl plateau start, Cd ramp
        { -35.0, -1.1,                 -0.04*(-35.0) - 0.8 },   // Cl plateau, Cd ramp
        { -25.0, -1.1,                 0.2 },                    // Cl plateau, Cd plateau start
        {   0.0, -1.1 + (2.9/45.0)*20.0, 0.2 },                 // Cl linear ramp, Cd plateau
        {  10.0, -1.1 + (2.9/45.0)*30.0, 0.2 },                 // Cl linear ramp, Cd plateau end
        {  25.0,  1.8,                 0.04*25.0 - 0.2 },        // Cl plateau start, Cd ramp
        {  50.0,  1.8,                 0.04*50.0 - 0.2 },        // Cl plateau end, Cd ramp
        {  60.0, -0.045*60.0 + 4.05,  0.04*60.0 - 0.2 },        // outer ramps
    };

    double max_pw_error = 0.0;
    for (const auto& check : pw_checks) {
        const double alpha = check.alpha_deg * M_PI / 180.0;
        Vec3 u = U * (std::cos(alpha) * e_c + std::sin(alpha) * Vec3(0.0, 0.0, 1.0));
        Vec3 lift, drag;
        blade_pw.computeForce(u, e_r, e_c, force_coefficient, lift, drag);

        // Extract signed coefficients using direction
        // Drag magnitude is always positive; lift sign comes from direction relative to e_l
        const double Cd_sim = drag.norm();
        const double Cl_sim = lift.norm();
        const double Cd_exact = std::abs(check.expected_Cd);
        const double Cl_exact = std::abs(check.expected_Cl);

        const double cd_err = std::abs(Cd_sim - Cd_exact);
        const double cl_err = std::abs(Cl_sim - Cl_exact);
        max_pw_error = std::max(max_pw_error, std::max(cd_err, cl_err));

        printf("  alpha=%6.1f deg: Cd_sim=%8.5f Cd_exact=%8.5f Cl_sim=%8.5f Cl_exact=%8.5f\n",
               check.alpha_deg, Cd_sim, Cd_exact, Cl_sim, Cl_exact);
    }

    std::cout << "  Max piecewise-linear error: " << max_pw_error << "\n";
    passed = passed && (max_pw_error < tolerance);

    if (passed) {
        std::cout << "PASSED: Errors within tolerance (" << tolerance << ")\n";
        return 0;
    } else {
        std::cout << "FAILED: Errors exceed tolerance\n";
        return 1;
    }
}

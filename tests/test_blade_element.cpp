#include "blade_element.hpp"

#include <cmath>
#include <iostream>

// Analytical formulas for lift and drag coefficients
// Cd(α) = Cd0 + 2*sin²(α)
// Cl(α) = Cl0*sin(2α)

double analyticalCd(double Cd0, double alpha) {
    double s = std::sin(alpha);
    return Cd0 + 2.0 * s * s;
}

double analyticalCl(double Cl0, double alpha) {
    return Cl0 * std::sin(2.0 * alpha);
}

int main() {
    std::cout << "Blade Element Test: Verifying Cd and Cl against analytical formulas\n";
    std::cout << "===================================================================\n\n";

    // Aerodynamic coefficients
    double Cd0 = 0.4;
    double Cl0 = 1.2;

    std::cout << "Parameters: Cd0 = " << Cd0 << ", Cl0 = " << Cl0 << "\n\n";

    BladeElement blade(Cd0, Cl0);

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
        double Cd_exact = analyticalCd(Cd0, alpha);
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

    if (passed) {
        std::cout << "PASSED: Errors within tolerance (" << tolerance << ")\n";
        return 0;
    } else {
        std::cout << "FAILED: Errors exceed tolerance\n";
        return 1;
    }
}

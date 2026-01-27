#include "eom.hpp"
#include "integrator.hpp"
#include "wing.hpp"

#include <cmath>
#include <iostream>
#include <vector>

// Analytical terminal velocity for vertical fall with fixed wing orientation
// For psi = pi/2 (horizontal chord): |u_z| = sqrt(2*lb0 / (mu0 * (Cd0 + 2)))
// For psi = 0 (vertical chord):      |u_z| = sqrt(2*lb0 / (mu0 * Cd0))
double analyticalTerminalVelocity(double mu0, double lb0, double Cd0, double psi) {
    double Cd;
    if (std::abs(psi - M_PI / 2.0) < 1e-10) {
        // Horizontal chord: alpha = pi/2, Cd = Cd0 + 2
        Cd = Cd0 + 2.0;
    } else if (std::abs(psi) < 1e-10 || std::abs(psi - M_PI) < 1e-10) {
        // Vertical chord: alpha = 0 or pi, Cd = Cd0
        Cd = Cd0;
    } else {
        // General case not supported (would have horizontal drift)
        return -1.0;
    }
    return std::sqrt(2.0 * lb0 / (mu0 * Cd));
}

// Test terminal velocity for a given wing pitch angle
bool testTerminalVelocity(double psi, const std::string& case_name) {
    std::cout << "Testing " << case_name << " (psi = " << psi * 180.0 / M_PI << " deg)...\n";

    // Wing parameters
    double mu0 = 0.1;
    double lb0 = 1.0;
    double Cd0 = 0.4;
    double Cl0 = 1.2;

    // Create wing with fixed orientation (no flapping)
    // gam = 0, gam_dot = 0, phi = 0, phi_dot = 0, psi = pitch
    auto fixedAngles = [psi](double t) -> WingAngles {
        (void)t;
        return {0.0, 0.0, 0.0, 0.0, psi};
    };

    Wing wing("test", mu0, lb0, WingSide::Left, Cd0, Cl0, fixedAngles);
    std::vector<Wing> wings = {wing};

    // Integration parameters
    double dt = 0.001;
    double t_max = 100.0;  // Long enough to reach terminal velocity
    double t = 0.0;

    // Start from rest at origin
    State state(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    // Scratch buffer for integration
    std::vector<SingleWingVectors> scratch(1);

    // Track velocity to detect steady state
    double prev_uz = 0.0;
    double steady_state_tol = 1e-8;
    int steady_count = 0;
    int required_steady_steps = 100;

    // Integrate until steady state
    while (t < t_max) {
        state = stepRK4(t, dt, state, wings, scratch);
        t += dt;

        // Check for steady state (velocity not changing)
        double uz_change = std::abs(state.vel.z() - prev_uz);
        if (uz_change < steady_state_tol * dt) {
            steady_count++;
            if (steady_count >= required_steady_steps) {
                break;
            }
        } else {
            steady_count = 0;
        }
        prev_uz = state.vel.z();
    }

    // Get simulated terminal velocity (magnitude of z-velocity)
    double uz_sim = std::abs(state.vel.z());
    double ux_sim = std::abs(state.vel.x());

    // Analytical terminal velocity
    double uz_analytical = analyticalTerminalVelocity(mu0, lb0, Cd0, psi);

    std::cout << "  Simulation time: " << t << " s\n";
    std::cout << "  Simulated u_z:   " << uz_sim << "\n";
    std::cout << "  Simulated u_x:   " << ux_sim << " (should be ~0)\n";
    std::cout << "  Analytical u_z:  " << uz_analytical << "\n";

    // Check results
    double rel_error = std::abs(uz_sim - uz_analytical) / uz_analytical;
    std::cout << "  Relative error:  " << rel_error * 100.0 << "%\n";

    bool passed = true;

    // Check vertical velocity matches analytical
    double tolerance = 1e-3;  // 0.1% tolerance
    if (rel_error > tolerance) {
        std::cout << "  FAILED: vertical velocity error exceeds " << tolerance * 100 << "%\n";
        passed = false;
    }

    // Check horizontal velocity is negligible
    if (ux_sim > 1e-6) {
        std::cout << "  FAILED: unexpected horizontal drift (u_x = " << ux_sim << ")\n";
        passed = false;
    }

    // Check we actually reached steady state
    if (t >= t_max - dt) {
        std::cout << "  WARNING: may not have reached steady state (t_max reached)\n";
    }

    if (passed) {
        std::cout << "  PASSED\n";
    }

    return passed;
}

int main() {
    std::cout << "Terminal Velocity Test: Falling wing reaches analytical steady state\n";
    std::cout << "====================================================================\n\n";

    bool all_passed = true;

    // Test case 1: Horizontal chord (psi = pi/2)
    // Cd = Cd0 + 2 = 2.4, maximum drag configuration
    all_passed &= testTerminalVelocity(M_PI / 2.0, "horizontal chord");
    std::cout << "\n";

    // Test case 2: Vertical chord (psi = 0)
    // Cd = Cd0 = 0.4, minimum drag configuration
    all_passed &= testTerminalVelocity(0.0, "vertical chord");
    std::cout << "\n";

    // Test case 3: Vertical chord pointing up (psi = pi)
    // Should give same result as psi = 0
    all_passed &= testTerminalVelocity(M_PI, "vertical chord (inverted)");
    std::cout << "\n";

    std::cout << "====================================================================\n";
    if (all_passed) {
        std::cout << "PASSED: All terminal velocity tests passed\n";
        return 0;
    } else {
        std::cout << "FAILED: Some tests failed\n";
        return 1;
    }
}

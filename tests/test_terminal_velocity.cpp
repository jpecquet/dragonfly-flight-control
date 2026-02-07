#include "eom.hpp"
#include "integrator.hpp"
#include "terminal_velocity.hpp"
#include "wing.hpp"

#include <cmath>
#include <iostream>
#include <vector>

// Test terminal velocity for a given wing pitch angle
bool testTerminalVelocity(double psi, const std::string& case_name) {
    std::cout << "Testing " << case_name << " (psi = " << psi * 180.0 / M_PI << " deg)...\n";

    // Wing parameters
    double mu0 = 0.1;
    double lb0 = 1.0;
    double Cd0 = 0.4;
    double Cl0 = 1.2;

    // Compute analytical solution
    TerminalVelocitySolution analytical = solveTerminalVelocity(psi, mu0, lb0, Cd0, Cl0);

    if (!analytical.converged) {
        std::cout << "  WARNING: Analytical solver did not converge\n";
    }

    // Create wing with fixed orientation (gam = 90° so psi = 0 means horizontal chord)
    auto fixedAngles = [psi](double t) -> WingAngles {
        (void)t;
        return {M_PI / 2.0, 0.0, 0.0, 0.0, psi};
    };

    Wing wing("test", mu0, lb0, WingSide::Left, Cd0, Cl0, fixedAngles);
    std::vector<Wing> wings = {wing};

    // Integration parameters
    double dt = 0.001;
    double t_max = 100.0;
    double t = 0.0;

    // Start from rest at origin
    State state;

    // Scratch buffer for integration
    std::vector<SingleWingVectors> scratch(1);

    // Track speed magnitude for steady state detection
    double prev_speed = 0.0;
    double steady_state_tol = 1e-10;
    int steady_count = 0;
    int required_steady_steps = 100;

    // Integrate until steady state
    while (t < t_max) {
        state = stepRK4(t, dt, state, wings, scratch);
        t += dt;

        // Check for steady state (speed not changing)
        double speed = std::sqrt(state.vel.x() * state.vel.x() + state.vel.z() * state.vel.z());
        double speed_change = std::abs(speed - prev_speed);
        if (speed_change < steady_state_tol * dt) {
            steady_count++;
            if (steady_count >= required_steady_steps) {
                break;
            }
        } else {
            steady_count = 0;
        }
        prev_speed = speed;
    }

    // Get simulated terminal velocity components
    double ux_sim = state.vel.x();
    double uz_sim = state.vel.z();
    double speed_sim = std::sqrt(ux_sim * ux_sim + uz_sim * uz_sim);
    double theta_sim = std::atan2(ux_sim, -uz_sim);

    std::cout << "  Simulation time:      " << t << " s\n";
    std::cout << "  Analytical solution:\n";
    std::cout << "    Glide angle theta:  " << analytical.theta * 180.0 / M_PI << " deg\n";
    std::cout << "    Angle of attack:    " << analytical.alpha * 180.0 / M_PI << " deg\n";
    std::cout << "    Speed U:            " << analytical.speed << "\n";
    std::cout << "    u_x:                " << analytical.ux << "\n";
    std::cout << "    u_z:                " << analytical.uz << "\n";
    std::cout << "  Simulated:\n";
    std::cout << "    Glide angle theta:  " << theta_sim * 180.0 / M_PI << " deg\n";
    std::cout << "    Speed U:            " << speed_sim << "\n";
    std::cout << "    u_x:                " << ux_sim << "\n";
    std::cout << "    u_z:                " << uz_sim << "\n";

    // Compute errors
    double speed_error = std::abs(speed_sim - analytical.speed) / analytical.speed;
    double ux_error = (std::abs(analytical.ux) > 1e-6) ?
        std::abs(ux_sim - analytical.ux) / std::abs(analytical.ux) :
        std::abs(ux_sim - analytical.ux);
    double uz_error = std::abs(uz_sim - analytical.uz) / std::abs(analytical.uz);
    double theta_error = std::abs(theta_sim - analytical.theta);

    std::cout << "  Errors:\n";
    std::cout << "    Speed:              " << speed_error * 100.0 << "%\n";
    std::cout << "    u_z:                " << uz_error * 100.0 << "%\n";
    if (std::abs(analytical.ux) > 1e-6) {
        std::cout << "    u_x:                " << ux_error * 100.0 << "%\n";
        std::cout << "    Glide angle:        " << theta_error * 180.0 / M_PI << " deg\n";
    }

    bool passed = true;
    double tolerance = 1e-3;  // 0.1% tolerance

    // Check speed
    if (speed_error > tolerance) {
        std::cout << "  FAILED: speed error exceeds " << tolerance * 100 << "%\n";
        passed = false;
    }

    // Check vertical velocity
    if (uz_error > tolerance) {
        std::cout << "  FAILED: u_z error exceeds " << tolerance * 100 << "%\n";
        passed = false;
    }

    // Check horizontal velocity (only if significant)
    if (std::abs(analytical.ux) > 1e-6 && ux_error > tolerance) {
        std::cout << "  FAILED: u_x error exceeds " << tolerance * 100 << "%\n";
        passed = false;
    }

    // Check we reached steady state
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

    // Test case 1: Horizontal chord (psi = 0)
    // Cl = 0, so no drift
    all_passed &= testTerminalVelocity(0.0, "horizontal chord");
    std::cout << "\n";

    // Test case 2: Vertical chord (psi = pi/2)
    // Cl = 0, so no drift
    all_passed &= testTerminalVelocity(M_PI / 2.0, "vertical chord");
    std::cout << "\n";

    // Test case 3: Vertical chord pointing down (psi = -pi/2)
    // Same as psi = pi/2
    all_passed &= testTerminalVelocity(-M_PI / 2.0, "vertical chord (inverted)");
    std::cout << "\n";

    // Test case 4: 45 degree pitch (psi = pi/4)
    // Drifting case
    all_passed &= testTerminalVelocity(M_PI / 4.0, "45 deg pitch");
    std::cout << "\n";

    // Test case 5: 30 degree pitch (psi = pi/6)
    // Drifting case
    all_passed &= testTerminalVelocity(M_PI / 6.0, "30 deg pitch");
    std::cout << "\n";

    // Test case 6: 60 degree pitch (psi = pi/3)
    // Drifting case
    all_passed &= testTerminalVelocity(M_PI / 3.0, "60 deg pitch");
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

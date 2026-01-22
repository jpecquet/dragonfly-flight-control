#include "eom.hpp"
#include "integrator.hpp"
#include "wing.hpp"

#include <cmath>
#include <iostream>
#include <vector>

// Analytical solution for free fall under constant gravity
// x(t) = x0 + ux0*t
// y(t) = y0 + uy0*t
// z(t) = z0 + uz0*t - 0.5*g*t^2  (g = 1 in nondimensional units)
// ux(t) = ux0
// uy(t) = uy0
// uz(t) = uz0 - g*t

State analyticalFreeFall(const State& initial, double t) {
    double g = 1.0;  // Nondimensional gravity
    return {
        initial[0] + initial[3] * t,           // x
        initial[1] + initial[4] * t,           // y
        initial[2] + initial[5] * t - 0.5 * g * t * t,  // z
        initial[3],                            // ux (constant)
        initial[4],                            // uy (constant)
        initial[5] - g * t                     // uz
    };
}

int main() {
    std::cout << "Free Fall Test: Verifying physics against analytical solution\n";
    std::cout << "============================================================\n\n";

    // Empty wings vector - no aerodynamic forces
    std::vector<Wing> wings;

    // Initial state with non-zero velocity to make test more interesting
    // [x, y, z, ux, uy, uz]
    State initial = {0.0, 0.0, 10.0, 1.0, 0.5, 2.0};

    std::cout << "Initial state:\n";
    std::cout << "  Position: (" << initial[0] << ", " << initial[1] << ", " << initial[2] << ")\n";
    std::cout << "  Velocity: (" << initial[3] << ", " << initial[4] << ", " << initial[5] << ")\n\n";

    // Integration parameters
    double dt = 0.001;   // Small timestep for accuracy
    double T = 2.0;      // Total simulation time
    int nsteps = static_cast<int>(T / dt);

    State state = initial;
    double t = 0.0;

    // Track maximum errors
    double max_pos_error = 0.0;
    double max_vel_error = 0.0;

    // Integrate and compare at each step
    for (int i = 0; i < nsteps; ++i) {
        state = stepRK4(t, dt, state, wings);
        t += dt;

        // Analytical solution at current time
        State exact = analyticalFreeFall(initial, t);

        // Position error (Euclidean distance)
        double pos_error = std::sqrt(
            std::pow(state[0] - exact[0], 2) +
            std::pow(state[1] - exact[1], 2) +
            std::pow(state[2] - exact[2], 2)
        );

        // Velocity error (Euclidean distance)
        double vel_error = std::sqrt(
            std::pow(state[3] - exact[3], 2) +
            std::pow(state[4] - exact[4], 2) +
            std::pow(state[5] - exact[5], 2)
        );

        max_pos_error = std::max(max_pos_error, pos_error);
        max_vel_error = std::max(max_vel_error, vel_error);
    }

    // Final comparison
    State exact_final = analyticalFreeFall(initial, T);

    std::cout << "After t = " << T << " seconds:\n\n";

    std::cout << "Simulated state:\n";
    std::cout << "  Position: (" << state[0] << ", " << state[1] << ", " << state[2] << ")\n";
    std::cout << "  Velocity: (" << state[3] << ", " << state[4] << ", " << state[5] << ")\n\n";

    std::cout << "Analytical solution:\n";
    std::cout << "  Position: (" << exact_final[0] << ", " << exact_final[1] << ", " << exact_final[2] << ")\n";
    std::cout << "  Velocity: (" << exact_final[3] << ", " << exact_final[4] << ", " << exact_final[5] << ")\n\n";

    std::cout << "Maximum errors over simulation:\n";
    std::cout << "  Position error: " << max_pos_error << "\n";
    std::cout << "  Velocity error: " << max_vel_error << "\n\n";

    // Pass/fail criteria
    // RK4 with dt=0.001 should achieve very high accuracy for this simple problem
    double pos_tolerance = 1e-10;
    double vel_tolerance = 1e-10;

    bool passed = (max_pos_error < pos_tolerance) && (max_vel_error < vel_tolerance);

    if (passed) {
        std::cout << "PASSED: Errors within tolerance (pos < " << pos_tolerance
                  << ", vel < " << vel_tolerance << ")\n";
        return 0;
    } else {
        std::cout << "FAILED: Errors exceed tolerance\n";
        return 1;
    }
}

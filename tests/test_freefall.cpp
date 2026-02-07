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
    return State(
        initial.pos + initial.vel * t - Vec3(0, 0, 0.5 * g * t * t),
        initial.vel - Vec3(0, 0, g * t)
    );
}

int main() {
    std::cout << "Free Fall Test: Verifying physics against analytical solution\n";
    std::cout << "============================================================\n\n";

    // Empty wings vector - no aerodynamic forces
    std::vector<Wing> wings;

    // Initial state with non-zero velocity to make test more interesting
    State initial(0.0, 0.0, 10.0, 1.0, 0.5, 2.0);

    std::cout << "Initial state:\n";
    std::cout << "  Position: (" << initial.pos.x() << ", " << initial.pos.y() << ", " << initial.pos.z() << ")\n";
    std::cout << "  Velocity: (" << initial.vel.x() << ", " << initial.vel.y() << ", " << initial.vel.z() << ")\n\n";

    // Integration parameters
    double dt = 0.001;   // Small timestep for accuracy
    double T = 2.0;      // Total simulation time
    int nsteps = static_cast<int>(T / dt);

    State state = initial;
    double t = 0.0;

    // Pre-allocate scratch buffer
    std::vector<SingleWingVectors> scratch(wings.size());

    // Track maximum errors
    double max_pos_error = 0.0;
    double max_vel_error = 0.0;

    // Integrate and compare at each step
    for (int i = 0; i < nsteps; ++i) {
        state = stepRK4(t, dt, state, wings, scratch);
        t += dt;

        // Analytical solution at current time
        State exact = analyticalFreeFall(initial, t);

        // Position error (Euclidean distance)
        double pos_error = (state.pos - exact.pos).norm();

        // Velocity error (Euclidean distance)
        double vel_error = (state.vel - exact.vel).norm();

        max_pos_error = std::max(max_pos_error, pos_error);
        max_vel_error = std::max(max_vel_error, vel_error);
    }

    // Final comparison
    State exact_final = analyticalFreeFall(initial, T);

    std::cout << "After t = " << T << " seconds:\n\n";

    std::cout << "Simulated state:\n";
    std::cout << "  Position: (" << state.pos.x() << ", " << state.pos.y() << ", " << state.pos.z() << ")\n";
    std::cout << "  Velocity: (" << state.vel.x() << ", " << state.vel.y() << ", " << state.vel.z() << ")\n\n";

    std::cout << "Analytical solution:\n";
    std::cout << "  Position: (" << exact_final.pos.x() << ", " << exact_final.pos.y() << ", " << exact_final.pos.z() << ")\n";
    std::cout << "  Velocity: (" << exact_final.vel.x() << ", " << exact_final.vel.y() << ", " << exact_final.vel.z() << ")\n\n";

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

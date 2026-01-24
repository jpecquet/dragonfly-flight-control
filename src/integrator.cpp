#include "integrator.hpp"
#include "eom.hpp"
#include "wing.hpp"

// Euler with scratch buffer (core implementation)
State stepEuler(double t, double h, const State& y,
                const std::vector<Wing>& wings,
                std::vector<SingleWingVectors>& scratch) {
    StateDerivative k1 = equationOfMotion(t, y, wings, scratch);

    return State(
        y.pos + h * k1.vel,
        y.vel + h * k1.accel
    );
}

// Euler without scratch buffer (convenience wrapper)
State stepEuler(double t, double h, const State& y,
                const std::vector<Wing>& wings) {
    std::vector<SingleWingVectors> scratch;
    return stepEuler(t, h, y, wings, scratch);
}

// RK4 with scratch buffer (core implementation - reuses allocation across 4 EOM calls)
State stepRK4(double t, double h, const State& y,
              const std::vector<Wing>& wings,
              std::vector<SingleWingVectors>& scratch) {
    // k1
    StateDerivative k1 = equationOfMotion(t, y, wings, scratch);

    // k2
    State y2(y.pos + 0.5 * h * k1.vel, y.vel + 0.5 * h * k1.accel);
    StateDerivative k2 = equationOfMotion(t + 0.5 * h, y2, wings, scratch);

    // k3
    State y3(y.pos + 0.5 * h * k2.vel, y.vel + 0.5 * h * k2.accel);
    StateDerivative k3 = equationOfMotion(t + 0.5 * h, y3, wings, scratch);

    // k4
    State y4(y.pos + h * k3.vel, y.vel + h * k3.accel);
    StateDerivative k4 = equationOfMotion(t + h, y4, wings, scratch);

    // Combine: y_new = y + (h/6) * (k1 + 2*k2 + 2*k3 + k4)
    return State(
        y.pos + (h / 6.0) * (k1.vel + 2.0 * k2.vel + 2.0 * k3.vel + k4.vel),
        y.vel + (h / 6.0) * (k1.accel + 2.0 * k2.accel + 2.0 * k3.accel + k4.accel)
    );
}

// RK4 without scratch buffer (convenience wrapper)
State stepRK4(double t, double h, const State& y,
              const std::vector<Wing>& wings) {
    std::vector<SingleWingVectors> scratch;
    return stepRK4(t, h, y, wings, scratch);
}

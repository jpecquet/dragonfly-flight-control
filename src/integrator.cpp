#include "integrator.hpp"
#include "eom.hpp"
#include "wing.hpp"

State stepEuler(double t, double h, const State& y,
                const std::vector<Wing>& wings) {
    std::vector<SingleWingVectors> wing_outputs;
    StateDerivative k1 = equationOfMotion(t, y, wings, wing_outputs);

    return State(
        y.pos + h * k1.vel,
        y.vel + h * k1.accel
    );
}

State stepRK4(double t, double h, const State& y,
              const std::vector<Wing>& wings) {
    std::vector<SingleWingVectors> wing_outputs;

    // k1
    StateDerivative k1 = equationOfMotion(t, y, wings, wing_outputs);

    // k2
    State y2(y.pos + 0.5 * h * k1.vel, y.vel + 0.5 * h * k1.accel);
    StateDerivative k2 = equationOfMotion(t + 0.5 * h, y2, wings, wing_outputs);

    // k3
    State y3(y.pos + 0.5 * h * k2.vel, y.vel + 0.5 * h * k2.accel);
    StateDerivative k3 = equationOfMotion(t + 0.5 * h, y3, wings, wing_outputs);

    // k4
    State y4(y.pos + h * k3.vel, y.vel + h * k3.accel);
    StateDerivative k4 = equationOfMotion(t + h, y4, wings, wing_outputs);

    // Combine: y_new = y + (h/6) * (k1 + 2*k2 + 2*k3 + k4)
    return State(
        y.pos + (h / 6.0) * (k1.vel + 2.0 * k2.vel + 2.0 * k3.vel + k4.vel),
        y.vel + (h / 6.0) * (k1.accel + 2.0 * k2.accel + 2.0 * k3.accel + k4.accel)
    );
}

#include "integrator.hpp"
#include "eom.hpp"

State stepEuler(double t, double h, const State& y, const Parameters& params) {
    WingVectors wings;
    StateDerivative k1 = equationOfMotion(t, y, params, wings);

    State y_new;
    for (int i = 0; i < 6; ++i) {
        y_new[i] = y[i] + h * k1[i];
    }
    return y_new;
}

State stepRK4(double t, double h, const State& y, const Parameters& params) {
    WingVectors wings;

    // k1
    StateDerivative k1 = equationOfMotion(t, y, params, wings);

    // k2
    State y2;
    for (int i = 0; i < 6; ++i) {
        y2[i] = y[i] + 0.5 * h * k1[i];
    }
    StateDerivative k2 = equationOfMotion(t + 0.5 * h, y2, params, wings);

    // k3
    State y3;
    for (int i = 0; i < 6; ++i) {
        y3[i] = y[i] + 0.5 * h * k2[i];
    }
    StateDerivative k3 = equationOfMotion(t + 0.5 * h, y3, params, wings);

    // k4
    State y4;
    for (int i = 0; i < 6; ++i) {
        y4[i] = y[i] + h * k3[i];
    }
    StateDerivative k4 = equationOfMotion(t + h, y4, params, wings);

    // Combine
    State y_new;
    for (int i = 0; i < 6; ++i) {
        y_new[i] = y[i] + (h / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
    }
    return y_new;
}

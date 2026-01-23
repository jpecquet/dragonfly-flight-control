#include "optimizer/objective.hpp"
#include "eom.hpp"
#include "wing.hpp"

#include <cmath>
#include <vector>

Vec3 instantAccel(double t, const OptimParams& optim, const FlightCondition& flight,
                  const FixedParams& fixed) {
    // Angle function for forewing (phase offset = 0)
    auto angleFunc_f = [&](double time) -> WingAngles {
        return {
            optim.gam0,
            fixed.phi0 * std::cos(fixed.omg0 * time),
            -fixed.phi0 * fixed.omg0 * std::sin(fixed.omg0 * time),
            optim.psim + optim.dpsi * std::cos(fixed.omg0 * time + fixed.dlt0)
        };
    };

    // Angle function for hindwing (phase offset = sig0)
    auto angleFunc_h = [&](double time) -> WingAngles {
        return {
            optim.gam0,
            fixed.phi0 * std::cos(fixed.omg0 * time + fixed.sig0),
            -fixed.phi0 * fixed.omg0 * std::sin(fixed.omg0 * time + fixed.sig0),
            optim.psim + optim.dpsi * std::cos(fixed.omg0 * time + fixed.dlt0 + fixed.sig0)
        };
    };

    // Create wings vector
    std::vector<Wing> wings;
    wings.emplace_back("fore", fixed.mu0_f, fixed.lb0_f, WingSide::Left, fixed.Cd0, fixed.Cl0, angleFunc_f);
    wings.emplace_back("fore", fixed.mu0_f, fixed.lb0_f, WingSide::Right, fixed.Cd0, fixed.Cl0, angleFunc_f);
    wings.emplace_back("hind", fixed.mu0_h, fixed.lb0_h, WingSide::Left, fixed.Cd0, fixed.Cl0, angleFunc_h);
    wings.emplace_back("hind", fixed.mu0_h, fixed.lb0_h, WingSide::Right, fixed.Cd0, fixed.Cl0, angleFunc_h);

    // Create state with specified velocity
    State state(0.0, 0.0, 0.0, flight.ux, 0.0, flight.uz);

    // Compute equations of motion
    std::vector<SingleWingVectors> wing_outputs;
    StateDerivative deriv = equationOfMotion(t, state, wings, wing_outputs);

    // Return acceleration
    return deriv.accel;
}

double wingBeatAccel(const OptimParams& optim, const FlightCondition& flight,
                     const FixedParams& fixed, int N) {
    // Wing beat period
    double T = 2.0 * M_PI / fixed.omg0;
    double dt = T / N;

    // Trapezoidal integration of acceleration over one period
    Vec3 a_mean(0.0, 0.0, 0.0);
    double t = 0.0;

    while (t < T) {
        Vec3 a1 = instantAccel(t, optim, flight, fixed);
        Vec3 a2 = instantAccel(t + dt, optim, flight, fixed);
        a_mean += (dt / T) * (a1 + a2) / 2.0;
        t += dt;
    }

    // Return squared magnitude
    return a_mean.squaredNorm();
}

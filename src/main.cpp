#include "eom.hpp"
#include "integrator.hpp"
#include "output.hpp"
#include "wing.hpp"

#include <cmath>
#include <iostream>
#include <vector>

int main() {
    // Wing geometry parameters
    double mu0_f = 0.075;  // Forewing mass parameter
    double lb0_f = 0.75;   // Forewing length
    double mu0_h = 0.075;  // Hindwing mass parameter
    double lb0_h = 0.75;   // Hindwing length

    // Aerodynamic coefficients
    double Cd0 = 0.4;
    double Cl0 = 1.2;

    // Kinematic parameters
    double omg0 = 8.0 * M_PI;          // Wing beat frequency
    double gam0 = M_PI / 3.0;          // Stroke plane angle
    double phi0 = M_PI / 8.0;          // Stroke amplitude
    double psim = 10.0 * M_PI / 180.0; // Mean pitch angle
    double dpsi = 30.0 * M_PI / 180.0; // Pitch oscillation amplitude
    double dlt0 = M_PI / 3.0;          // Pitch phase offset
    double sig0 = M_PI / 3.0;          // Fore/hindwing phase offset

    // Angle function for forewing (phase offset = 0)
    auto angleFunc_f = [=](double t) -> WingAngles {
        return {
            gam0,
            phi0 * std::cos(omg0 * t),
            -phi0 * omg0 * std::sin(omg0 * t),
            psim + dpsi * std::cos(omg0 * t + dlt0)
        };
    };

    // Angle function for hindwing (phase offset = sig0)
    auto angleFunc_h = [=](double t) -> WingAngles {
        return {
            gam0,
            phi0 * std::cos(omg0 * t + sig0),
            -phi0 * omg0 * std::sin(omg0 * t + sig0),
            psim + dpsi * std::cos(omg0 * t + dlt0 + sig0)
        };
    };

    // Create wings vector
    std::vector<Wing> wings;
    wings.emplace_back("fore", mu0_f, lb0_f, WingSide::Left, Cd0, Cl0, angleFunc_f);
    wings.emplace_back("fore", mu0_f, lb0_f, WingSide::Right, Cd0, Cl0, angleFunc_f);
    wings.emplace_back("hind", mu0_h, lb0_h, WingSide::Left, Cd0, Cl0, angleFunc_h);
    wings.emplace_back("hind", mu0_h, lb0_h, WingSide::Right, Cd0, Cl0, angleFunc_h);

    // Initial state: [x, y, z, ux, uy, uz]
    State state = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // Time integration setup
    double Twb = 2.0 * M_PI / omg0;  // Wing beat period
    double dt = Twb / 50.0;          // 50 steps per wing beat
    double T = 5.0 * Twb;            // 5 wing beat cycles
    int nsteps = static_cast<int>(T / dt);

    // Output storage
    SimulationOutput output;
    output.lb0_f = lb0_f;
    output.lb0_h = lb0_h;
    output.mu0_f = mu0_f;
    output.mu0_h = mu0_h;
    output.Cd0 = Cd0;
    output.Cl0 = Cl0;
    output.omg0 = omg0;
    output.gam0 = gam0;
    output.phi0 = phi0;
    output.psim = psim;
    output.dpsi = dpsi;
    output.sig0 = sig0;
    output.dlt0 = dlt0;
    output.time.reserve(nsteps + 1);
    output.states.reserve(nsteps + 1);
    output.wing_data.reserve(nsteps + 1);
    
    // Store initial state
    std::vector<SingleWingVectors> wing_data;
    equationOfMotion(t, state, wings, wing_data);

    output.time.push_back(0.0);
    output.states.push_back(state);
    output.wing_data.push_back(wing_data);

    // Time integration loop
    double t = 0.0;
    for (int i = 0; i < nsteps; ++i) {

        // Integrate
        state = stepRK4(t, dt, state, wings);
        t += dt;

        // Store new state
        std::vector<SingleWingVectors> wing_data;
        equationOfMotion(t, state, wings, wing_data);

        output.time.push_back(t);
        output.states.push_back(state);
        output.wing_data.push_back(wing_data);
    }

    // Write output
    writeHDF5("output.h5", output, wings);

    std::cout << "Simulation complete. Output written to output.h5\n";
    return 0;
}

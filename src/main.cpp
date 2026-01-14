#include "eom.hpp"
#include "integrator.hpp"
#include "output.hpp"

#include <cmath>
#include <iostream>
#include <vector>

int main() {
    // Simulation parameters
    Parameters params;
    params.lb0_f = 0.75;
    params.lb0_h = 0.75;
    params.mu0_f = 0.075;
    params.mu0_h = 0.075;
    params.Cd0 = 0.4;
    params.Cl0 = 1.2;
    params.omg0 = 8.0 * M_PI;
    params.gam0 = M_PI / 3.0;
    params.phi0 = M_PI / 8.0;
    params.psim = 10.0 * M_PI / 180.0;
    params.dpsi = 30.0 * M_PI / 180.0;
    params.sig0 = M_PI / 3.0;
    params.dlt0 = M_PI / 3.0;

    // Initial state: [x, y, z, ux, uy, uz]
    State state = {0.0, 0.0, 0.0, 0.0, 0.0, 2.0};

    // Time integration setup
    double Twb = 2.0 * M_PI / params.omg0;  // Wing beat period
    double dt = Twb / 50.0;                  // 50 steps per wing beat
    double T = 5.0 * Twb;                    // 5 wing beat cycles
    int nsteps = static_cast<int>(T / dt);

    // Output storage
    SimulationOutput output;
    output.params = params;
    output.time.reserve(nsteps);
    output.states.reserve(nsteps);
    output.wing_data.reserve(nsteps);

    // Time integration loop
    double t = 0.0;
    for (int i = 0; i < nsteps; ++i) {
        // Store current state
        WingVectors wings;
        StateDerivative deriv = equationOfMotion(t, state, params, wings);

        output.time.push_back(t);
        output.states.push_back(state);
        output.wing_data.push_back(wings);

        // Integrate
        if (i == 0) {
            state = stepEuler(t, dt, state, params);
        } else {
            state = stepRK4(t, dt, state, params);
        }
        t += dt;
    }

    // Write output
    writeHDF5("output.h5", output);

    std::cout << "Simulation complete. Output written to output.h5\n";
    return 0;
}

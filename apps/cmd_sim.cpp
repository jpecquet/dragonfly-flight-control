#include "cmd_sim.hpp"
#include "eom.hpp"
#include "integrator.hpp"
#include "output.hpp"
#include "wing.hpp"

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

int runSim(const Config& cfg) {
    // Wing geometry
    double mu0_f = cfg.getDouble("mu0_f");
    double lb0_f = cfg.getDouble("lb0_f");
    double mu0_h = cfg.getDouble("mu0_h");
    double lb0_h = cfg.getDouble("lb0_h");

    // Aerodynamic coefficients
    double Cd0 = cfg.getDouble("Cd0");
    double Cl0 = cfg.getDouble("Cl0");

    // Kinematic parameters
    double omg0 = cfg.getDouble("omg0");
    double gam0 = cfg.getDouble("gam0");
    double phi0 = cfg.getDouble("phi0");
    double psim = cfg.getDouble("psim");
    double dpsi = cfg.getDouble("dpsi");
    double dlt0 = cfg.getDouble("dlt0");
    double sig0 = cfg.getDouble("sig0");

    // Initial conditions
    double x0 = cfg.getDouble("x0", 0.0);
    double y0 = cfg.getDouble("y0", 0.0);
    double z0 = cfg.getDouble("z0", 0.0);
    double ux0 = cfg.getDouble("ux0", 0.0);
    double uy0 = cfg.getDouble("uy0", 0.0);
    double uz0 = cfg.getDouble("uz0", 0.0);

    // Time integration
    int n_wingbeats = cfg.getInt("n_wingbeats", 5);
    int steps_per_wingbeat = cfg.getInt("steps_per_wingbeat", 50);

    // Output
    std::string output_file = cfg.getString("output");

    // Angle function for forewing
    auto angleFunc_f = [=](double t) -> WingAngles {
        return {
            gam0,
            phi0 * std::cos(omg0 * t),
            -phi0 * omg0 * std::sin(omg0 * t),
            psim + dpsi * std::cos(omg0 * t + dlt0)
        };
    };

    // Angle function for hindwing
    auto angleFunc_h = [=](double t) -> WingAngles {
        return {
            gam0,
            phi0 * std::cos(omg0 * t + sig0),
            -phi0 * omg0 * std::sin(omg0 * t + sig0),
            psim + dpsi * std::cos(omg0 * t + dlt0 + sig0)
        };
    };

    // Create wings
    std::vector<Wing> wings;
    wings.emplace_back("fore", mu0_f, lb0_f, WingSide::Left, Cd0, Cl0, angleFunc_f);
    wings.emplace_back("fore", mu0_f, lb0_f, WingSide::Right, Cd0, Cl0, angleFunc_f);
    wings.emplace_back("hind", mu0_h, lb0_h, WingSide::Left, Cd0, Cl0, angleFunc_h);
    wings.emplace_back("hind", mu0_h, lb0_h, WingSide::Right, Cd0, Cl0, angleFunc_h);

    // Initial state
    State state(x0, y0, z0, ux0, uy0, uz0);

    // Time setup
    double Twb = 2.0 * M_PI / omg0;
    double dt = Twb / steps_per_wingbeat;
    double T = n_wingbeats * Twb;
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
    double t = 0.0;
    std::vector<SingleWingVectors> wing_data;
    equationOfMotion(t, state, wings, wing_data);
    output.time.push_back(t);
    output.states.push_back(state);
    output.wing_data.push_back(wing_data);

    // Time integration
    std::cout << "Running simulation for " << n_wingbeats << " wingbeats..." << std::endl;
    for (int i = 0; i < nsteps; ++i) {
        state = stepRK4(t, dt, state, wings);
        t += dt;

        std::vector<SingleWingVectors> wd;
        equationOfMotion(t, state, wings, wd);
        output.time.push_back(t);
        output.states.push_back(state);
        output.wing_data.push_back(wd);
    }

    // Write output
    writeHDF5(output_file, output, wings);
    std::cout << "Output written to " << output_file << std::endl;

    return 0;
}

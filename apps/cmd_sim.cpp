#include "cmd_sim.hpp"
#include "eom.hpp"
#include "integrator.hpp"
#include "kinematics.hpp"
#include "output.hpp"
#include "wing.hpp"

#include <iostream>
#include <string>
#include <vector>

int runSim(const Config& cfg) {
    // Kinematic parameters
    double omega = cfg.getDouble("omega");
    double gamma_mean = cfg.getDouble("gamma_mean");
    double gamma_amp = cfg.getDouble("gamma_amp", 0.0);
    double gamma_phase = cfg.getDouble("gamma_phase", 0.0);
    double phi_amp = cfg.getDouble("phi_amp");
    double psi_mean = cfg.getDouble("psi_mean");
    double psi_amp = cfg.getDouble("psi_amp");
    double psi_phase = cfg.getDouble("psi_phase");

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

    // Tether mode: fix position/velocity (zero acceleration)
    bool tether = cfg.getBool("tether", false);

    // Output
    std::string output_file = cfg.getString("output");

    // Build wing configurations from [[wing]] sections
    if (!cfg.hasWings()) {
        throw std::runtime_error("Config must define wings using [[wing]] sections");
    }

    std::vector<WingConfig> wingConfigs;
    for (const auto& entry : cfg.getWingEntries()) {
        WingSide side = (entry.side == "left") ? WingSide::Left : WingSide::Right;
        wingConfigs.emplace_back(entry.name, side, entry.mu0, entry.lb0,
                                 entry.Cd0, entry.Cl0, entry.phase);
    }
    std::cout << "Loaded " << wingConfigs.size() << " wings from config" << std::endl;

    // Create wings from configurations
    std::vector<Wing> wings;
    wings.reserve(wingConfigs.size());
    for (const auto& wc : wingConfigs) {
        auto angleFunc = makeAngleFunc(gamma_mean, gamma_amp, gamma_phase, phi_amp, psi_mean, psi_amp, psi_phase, wc.phaseOffset, omega);
        wings.emplace_back(wc.name, wc.mu0, wc.lb0, wc.side, wc.Cd0, wc.Cl0, angleFunc);
    }

    // Initial state
    State state(x0, y0, z0, ux0, uy0, uz0);

    // Time setup
    double Twb = 2.0 * M_PI / omega;
    double dt = Twb / steps_per_wingbeat;
    double T = n_wingbeats * Twb;
    int nsteps = static_cast<int>(T / dt);

    // Output storage
    SimulationOutput output;
    output.wingConfigs = wingConfigs;
    output.omega = omega;
    output.gamma_mean = gamma_mean;
    output.gamma_amp = gamma_amp;
    output.gamma_phase = gamma_phase;
    output.phi_amp = phi_amp;
    output.psi_mean = psi_mean;
    output.psi_amp = psi_amp;
    output.psi_phase = psi_phase;
    output.time.reserve(nsteps + 1);
    output.states.reserve(nsteps + 1);
    output.wing_data.reserve(nsteps + 1);

    // Pre-allocate scratch buffer for integrator (avoids repeated allocation)
    std::vector<SingleWingVectors> scratch(wings.size());

    // Store initial state
    double t = 0.0;
    std::vector<SingleWingVectors> wing_data(wings.size());
    equationOfMotion(t, state, wings, wing_data);
    output.time.push_back(t);
    output.states.push_back(state);
    output.wing_data.push_back(wing_data);

    // Time integration
    std::cout << "Running simulation for " << n_wingbeats << " wingbeats";
    if (tether) {
        std::cout << " (tethered)";
    }
    std::cout << "..." << std::endl;

    for (int i = 0; i < nsteps; ++i) {
        if (!tether) {
            state = stepRK4(t, dt, state, wings, scratch);
        }
        t += dt;

        equationOfMotion(t, state, wings, wing_data);
        output.time.push_back(t);
        output.states.push_back(state);
        output.wing_data.push_back(wing_data);
    }

    // Write output
    writeHDF5(output_file, output, wings);
    std::cout << "Output written to " << output_file << std::endl;

    return 0;
}

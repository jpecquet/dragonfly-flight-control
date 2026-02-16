#include "cmd_sim.hpp"
#include "integrator.hpp"
#include "sim_setup.hpp"

#include <iostream>
#include <string>

int runSim(const Config& cfg) {
    auto kin = readKinematicParams(cfg);
    auto state = readInitialState(cfg);
    auto tp = readTimeParams(cfg, kin.omega);
    bool tether = cfg.getBool("tether", false);
    std::string output_file = cfg.getString("output");

    auto wingConfigs = buildWingConfigs(cfg, kin);
    auto wings = createWings(wingConfigs, kin);
    auto output = initOutput(wingConfigs, kin, tp.nsteps);

    // Pre-allocate scratch buffers
    std::vector<SingleWingVectors> scratch(wings.size());
    std::vector<SingleWingVectors> wing_data(wings.size());

    // Store initial state
    double t = 0.0;
    storeTimestep(output, t, state, wings, wing_data);

    // Time integration
    std::cout << "Running simulation for " << cfg.getInt("n_wingbeats", 5) << " wingbeats";
    if (tether) {
        std::cout << " (tethered)";
    }
    std::cout << "..." << std::endl;

    for (int i = 0; i < tp.nsteps; ++i) {
        if (!tether) {
            state = stepRK4(t, tp.dt, state, wings, scratch);
        }
        t += tp.dt;
        storeTimestep(output, t, state, wings, wing_data);
    }

    // Write output
    writeHDF5(output_file, output, wings);
    std::cout << "Output written to " << output_file << std::endl;

    return 0;
}

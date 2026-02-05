#include "cmd_track.hpp"
#include "eom.hpp"
#include "integrator.hpp"
#include "kinematics.hpp"
#include "output.hpp"
#include "trajectory_controller.hpp"
#include "wing.hpp"

#include <iostream>
#include <string>
#include <vector>

int runTrack(const Config& cfg) {
    // Kinematic parameters (baseline values)
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

    // Output
    std::string output_file = cfg.getString("output");

    // PID gains
    PIDGains x_gains(
        cfg.getDouble("pid_x_kp", 2.0),
        cfg.getDouble("pid_x_ki", 0.5),
        cfg.getDouble("pid_x_kd", 0.8),
        cfg.getDouble("pid_x_imax", 1.0)
    );
    PIDGains y_gains(
        cfg.getDouble("pid_y_kp", 2.0),
        cfg.getDouble("pid_y_ki", 0.5),
        cfg.getDouble("pid_y_kd", 0.8),
        cfg.getDouble("pid_y_imax", 1.0)
    );
    PIDGains z_gains(
        cfg.getDouble("pid_z_kp", 4.0),
        cfg.getDouble("pid_z_ki", 1.0),
        cfg.getDouble("pid_z_kd", 1.2),
        cfg.getDouble("pid_z_imax", 2.0)
    );

    // Trajectory
    std::string traj_spec = cfg.getString("trajectory", "hover 0.0 0.0 0.0");
    TrajectoryFunc trajectory = trajectories::parse(traj_spec);

    // Parameter bounds
    ParameterBounds bounds;
    bounds.gamma_mean_min = cfg.getDouble("gamma_mean_min", 0.5);
    bounds.gamma_mean_max = cfg.getDouble("gamma_mean_max", 2.5);
    bounds.psi_mean_min = cfg.getDouble("psi_mean_min", 0.0);
    bounds.psi_mean_max = cfg.getDouble("psi_mean_max", 1.5708);
    bounds.phi_amp_min = cfg.getDouble("phi_amp_min", 0.1);
    bounds.phi_amp_max = cfg.getDouble("phi_amp_max", 0.8);

    // Mixing matrix (optional override)
    MixingMatrix mixing;
    if (cfg.has("mix_gamma_x")) {
        mixing.gamma_mean_mix = Vec3(
            cfg.getDouble("mix_gamma_x"),
            cfg.getDouble("mix_gamma_y", 0.0),
            cfg.getDouble("mix_gamma_z")
        );
    }
    if (cfg.has("mix_psi_x")) {
        mixing.psi_mean_mix = Vec3(
            cfg.getDouble("mix_psi_x"),
            cfg.getDouble("mix_psi_y", 0.0),
            cfg.getDouble("mix_psi_z")
        );
    }
    if (cfg.has("mix_phi_x")) {
        mixing.phi_amp_mix = Vec3(
            cfg.getDouble("mix_phi_x"),
            cfg.getDouble("mix_phi_y", 0.0),
            cfg.getDouble("mix_phi_z")
        );
    }

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

    // Create wings from configurations (initial angle functions)
    std::vector<Wing> wings;
    wings.reserve(wingConfigs.size());
    for (const auto& wc : wingConfigs) {
        auto angleFunc = makeAngleFunc(gamma_mean, gamma_amp, gamma_phase, phi_amp,
                                       psi_mean, psi_amp, psi_phase, wc.phaseOffset, omega);
        wings.emplace_back(wc.name, wc.mu0, wc.lb0, wc.side, wc.Cd0, wc.Cl0, angleFunc);
    }

    // Setup controller
    TrajectoryController controller;
    controller.setGains(x_gains, y_gains, z_gains);
    controller.setTrajectory(std::move(trajectory));
    controller.setBaseline(gamma_mean, psi_mean, phi_amp);
    controller.setMixing(mixing);
    controller.setBounds(bounds);

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
    output.gamma_mean = gamma_mean;  // Baseline (will vary during tracking)
    output.gamma_amp = gamma_amp;
    output.gamma_phase = gamma_phase;
    output.phi_amp = phi_amp;        // Baseline (will vary during tracking)
    output.psi_mean = psi_mean;      // Baseline (will vary during tracking)
    output.psi_amp = psi_amp;
    output.psi_phase = psi_phase;
    output.time.reserve(nsteps + 1);
    output.states.reserve(nsteps + 1);
    output.wing_data.reserve(nsteps + 1);

    // Controller data storage
    output.controller_active = true;
    output.target_positions.reserve(nsteps + 1);
    output.position_errors.reserve(nsteps + 1);
    output.param_gamma_mean.reserve(nsteps + 1);
    output.param_psi_mean.reserve(nsteps + 1);
    output.param_phi_amp.reserve(nsteps + 1);

    // Pre-allocate scratch buffer
    std::vector<SingleWingVectors> scratch(wings.size());

    // Store initial state
    double t = 0.0;
    std::vector<SingleWingVectors> wing_data(wings.size());
    equationOfMotion(t, state, wings, wing_data);
    output.time.push_back(t);
    output.states.push_back(state);
    output.wing_data.push_back(wing_data);

    // Initial controller state
    auto ctrl = controller.compute(t, dt, state);
    const auto& cs = controller.lastState();
    output.target_positions.push_back(cs.target_pos);
    output.position_errors.push_back(cs.pos_error);
    output.param_gamma_mean.push_back(ctrl.gamma_mean);
    output.param_psi_mean.push_back(ctrl.psi_mean);
    output.param_phi_amp.push_back(ctrl.phi_amp);

    // Time integration with controller
    std::cout << "Running trajectory tracking for " << n_wingbeats << " wingbeats..." << std::endl;

    for (int i = 0; i < nsteps; ++i) {
        // 1. Controller update
        ctrl = controller.compute(t, dt, state);

        // 2. Update wing angle functions with new parameters
        for (size_t w = 0; w < wings.size(); ++w) {
            auto angleFunc = makeAngleFunc(ctrl.gamma_mean, gamma_amp, gamma_phase,
                                           ctrl.phi_amp, ctrl.psi_mean, psi_amp,
                                           psi_phase, wingConfigs[w].phaseOffset, omega);
            wings[w].setAngleFunc(std::move(angleFunc));
        }

        // 3. Physics step
        state = stepRK4(t, dt, state, wings, scratch);
        t += dt;

        // Store outputs
        equationOfMotion(t, state, wings, wing_data);
        output.time.push_back(t);
        output.states.push_back(state);
        output.wing_data.push_back(wing_data);

        // Store controller state
        const auto& cs2 = controller.lastState();
        output.target_positions.push_back(cs2.target_pos);
        output.position_errors.push_back(cs2.pos_error);
        output.param_gamma_mean.push_back(ctrl.gamma_mean);
        output.param_psi_mean.push_back(ctrl.psi_mean);
        output.param_phi_amp.push_back(ctrl.phi_amp);
    }

    // Report final tracking error
    const auto& final_cs = controller.lastState();
    double final_error = final_cs.pos_error.norm();
    std::cout << "Final position error: " << final_error << std::endl;

    // Write output
    writeHDF5(output_file, output, wings);
    std::cout << "Output written to " << output_file << std::endl;

    return 0;
}

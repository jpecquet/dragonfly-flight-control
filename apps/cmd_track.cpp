#include "cmd_track.hpp"
#include "integrator.hpp"
#include "kinematics.hpp"
#include "output.hpp"
#include "sim_setup.hpp"
#include "trajectory_controller.hpp"

#include <iostream>
#include <string>

int runTrack(const Config& cfg) {
    auto kin = readKinematicParams(cfg);
    auto state = readInitialState(cfg);
    auto tp = readTimeParams(cfg, kin.omega);
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

    auto wingConfigs = buildWingConfigs(cfg);
    auto wings = createWings(wingConfigs, kin);
    auto output = initOutput(wingConfigs, kin, tp.nsteps);

    // Setup controller
    TrajectoryController controller;
    controller.setGains(x_gains, y_gains, z_gains);
    controller.setTrajectory(std::move(trajectory));
    controller.setBaseline(kin.gamma_mean, kin.psi_mean, kin.phi_amp);
    controller.setMixing(mixing);
    controller.setBounds(bounds);

    // Controller data storage
    output.controller_active = true;
    output.target_positions.reserve(tp.nsteps + 1);
    output.position_errors.reserve(tp.nsteps + 1);
    output.param_gamma_mean.reserve(tp.nsteps + 1);
    output.param_psi_mean.reserve(tp.nsteps + 1);
    output.param_phi_amp.reserve(tp.nsteps + 1);

    // Pre-allocate scratch buffers
    std::vector<SingleWingVectors> scratch(wings.size());
    std::vector<SingleWingVectors> wing_data(wings.size());

    // Store initial state
    double t = 0.0;
    storeTimestep(output, t, state, wings, wing_data);

    // Initial controller state
    auto ctrl = controller.compute(t, tp.dt, state);
    const auto& cs = controller.lastState();
    output.target_positions.push_back(cs.target_pos);
    output.position_errors.push_back(cs.pos_error);
    output.param_gamma_mean.push_back(ctrl.gamma_mean);
    output.param_psi_mean.push_back(ctrl.psi_mean);
    output.param_phi_amp.push_back(ctrl.phi_amp);

    // Time integration with controller
    std::cout << "Running trajectory tracking for " << cfg.getInt("n_wingbeats", 5)
              << " wingbeats..." << std::endl;

    for (int i = 0; i < tp.nsteps; ++i) {
        // 1. Controller update
        ctrl = controller.compute(t, tp.dt, state);

        // 2. Update wing angle functions with new parameters
        for (size_t w = 0; w < wings.size(); ++w) {
            auto angleFunc = makeAngleFunc(ctrl.gamma_mean, kin.gamma_amp, kin.gamma_phase,
                                           ctrl.phi_amp, ctrl.psi_mean, kin.psi_amp,
                                           kin.psi_phase, wingConfigs[w].phaseOffset, kin.omega);
            wings[w].setAngleFunc(std::move(angleFunc));
        }

        // 3. Physics step
        state = stepRK4(t, tp.dt, state, wings, scratch);
        t += tp.dt;

        // Store outputs
        storeTimestep(output, t, state, wings, wing_data);

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

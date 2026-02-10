#include "cmd_track.hpp"
#include "integrator.hpp"
#include "kinematics.hpp"
#include "output.hpp"
#include "sim_setup.hpp"
#include "trajectory_controller.hpp"

#include <cmath>
#include <iostream>
#include <string>

int runTrack(const Config& cfg) {
    auto kin = readKinematicParams(cfg);
    auto state = readInitialState(cfg);
    auto tp = readTimeParams(cfg, kin.omega);
    std::string output_file = cfg.getString("output");

    // PID gains
    PIDGains x_gains = readPIDGains(cfg, "pid_x_", {2.0, 0.5, 0.8, 1.0});
    PIDGains y_gains = readPIDGains(cfg, "pid_y_", {2.0, 0.5, 0.8, 1.0});
    PIDGains z_gains = readPIDGains(cfg, "pid_z_", {4.0, 1.0, 1.2, 2.0});

    // Trajectory
    std::string traj_spec = cfg.getString("trajectory", "hover 0.0 0.0 0.0");
    TrajectoryFunc trajectory = trajectories::parse(traj_spec);

    // Parameter bounds and mixing matrix
    ParameterBounds bounds = readParameterBounds(cfg);
    MixingMatrix mixing = readMixingMatrix(cfg);

    auto wingConfigs = buildWingConfigs(cfg);
    auto wings = createWings(wingConfigs, kin);
    auto output = initOutput(wingConfigs, kin, tp.nsteps);

    // Controller-updated parameters shared by all pre-bound wing angle lambdas.
    double gamma_mean_cmd = kin.gamma_mean;
    double psi_mean_cmd = kin.psi_mean;
    double phi_amp_cmd = kin.phi_amp;
    for (size_t w = 0; w < wings.size(); ++w) {
        const double phase_offset = wingConfigs[w].phaseOffset;
        auto angleFunc = [&gamma_mean_cmd, &psi_mean_cmd, &phi_amp_cmd,
                          omega = kin.omega,
                          gamma_amp = kin.gamma_amp,
                          gamma_phase = kin.gamma_phase,
                          psi_amp = kin.psi_amp,
                          psi_phase = kin.psi_phase,
                          phase_offset](double t) -> WingAngles {
            double phase_gam = omega * t + gamma_phase + phase_offset;
            double phase_phi = omega * t + phase_offset;
            double phase_psi = omega * t + psi_phase + phase_offset;
            return {
                gamma_mean_cmd + gamma_amp * std::cos(phase_gam),
                -gamma_amp * omega * std::sin(phase_gam),
                phi_amp_cmd * std::cos(phase_phi),
                -phi_amp_cmd * omega * std::sin(phase_phi),
                psi_mean_cmd + psi_amp * std::cos(phase_psi)
            };
        };
        wings[w].setAngleFunc(std::move(angleFunc));
    }

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
    gamma_mean_cmd = ctrl.gamma_mean;
    psi_mean_cmd = ctrl.psi_mean;
    phi_amp_cmd = ctrl.phi_amp;
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
        gamma_mean_cmd = ctrl.gamma_mean;
        psi_mean_cmd = ctrl.psi_mean;
        phi_amp_cmd = ctrl.phi_amp;

        // 2. Physics step
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

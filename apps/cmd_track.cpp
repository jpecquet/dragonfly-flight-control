#include "cmd_track.hpp"
#include "integrator.hpp"
#include "kinematics.hpp"
#include "output.hpp"
#include "sim_setup.hpp"
#include "trajectory_controller.hpp"

#include <cmath>
#include <iostream>
#include <string>

namespace {

void applyControlOutput(const ControlOutput& ctrl,
                        double& gamma_mean_cmd,
                        double& psi_mean_cmd,
                        double& phi_amp_cmd) {
    gamma_mean_cmd = ctrl.gamma_mean;
    psi_mean_cmd = ctrl.psi_mean;
    phi_amp_cmd = ctrl.phi_amp;
}

void appendControllerState(SimulationOutput& output,
                           const TrajectoryController& controller,
                           const ControlOutput& ctrl) {
    const auto& cs = controller.lastState();
    output.target_positions.push_back(cs.target_pos);
    output.position_errors.push_back(cs.pos_error);
    output.param_gamma_mean.push_back(ctrl.gamma_mean);
    output.param_psi_mean.push_back(ctrl.psi_mean);
    output.param_phi_amp.push_back(ctrl.phi_amp);
}

}  // namespace

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

    auto wingConfigs = buildWingConfigs(cfg, kin);
    auto wings = createWings(wingConfigs, kin);
    auto output = initOutput(wingConfigs, kin, tp.nsteps);

    constexpr double EPS = 1e-12;
    const double kin_phi_c1 = kin.phi_cos.empty() ? 0.0 : kin.phi_cos[0];
    const double kin_phi_s1 = kin.phi_sin.empty() ? 0.0 : kin.phi_sin[0];
    const double kin_phi_amp = std::hypot(kin_phi_c1, kin_phi_s1);

    // Controller baseline is the average effective wing motion baseline.
    double gamma_mean_base = 0.0;
    double psi_mean_base = 0.0;
    double phi_amp_base = 0.0;
    for (const auto& wcfg : wingConfigs) {
        if (hasWingMotionSeries(wcfg)) {
            gamma_mean_base += wcfg.gamma_mean;
            psi_mean_base += wcfg.psi_mean;
            double c1 = wcfg.phi_cos.empty() ? 0.0 : wcfg.phi_cos[0];
            double s1 = wcfg.phi_sin.empty() ? 0.0 : wcfg.phi_sin[0];
            phi_amp_base += std::hypot(c1, s1);
        } else {
            gamma_mean_base += kin.gamma_mean;
            psi_mean_base += kin.psi_mean;
            phi_amp_base += kin_phi_amp;
        }
    }
    if (!wingConfigs.empty()) {
        const double inv_count = 1.0 / static_cast<double>(wingConfigs.size());
        gamma_mean_base *= inv_count;
        psi_mean_base *= inv_count;
        phi_amp_base *= inv_count;
    } else {
        gamma_mean_base = kin.gamma_mean;
        psi_mean_base = kin.psi_mean;
        phi_amp_base = kin_phi_amp;
    }

    // Controller-updated parameters shared by all pre-bound wing angle lambdas.
    double gamma_mean_cmd = gamma_mean_base;
    double psi_mean_cmd = psi_mean_base;
    double phi_amp_cmd = phi_amp_base;
    for (size_t w = 0; w < wings.size(); ++w) {
        const auto& wcfg = wingConfigs[w];
        const bool use_local = hasWingMotionSeries(wcfg);
        const MotionParams* base_motion = use_local
            ? static_cast<const MotionParams*>(&wcfg)
            : static_cast<const MotionParams*>(&kin);
        const auto base_series = base_motion->toHarmonicSeries();
        const double omega_w = base_motion->omega;
        const double harmonic_period_wingbeats_w = base_motion->harmonic_period_wingbeats;
        const HarmonicSeries gamma_base = base_series.gamma;
        const HarmonicSeries phi_base = base_series.phi;
        const HarmonicSeries psi_base = base_series.psi;
        const double phi_c1 = phi_base.cos_coeff.empty() ? 0.0 : phi_base.cos_coeff[0];
        const double phi_s1 = phi_base.sin_coeff.empty() ? 0.0 : phi_base.sin_coeff[0];
        const double wing_phi_amp_base = std::hypot(phi_c1, phi_s1);
        const double phase_offset = wingConfigs[w].phase_offset;
        auto angleFunc = makeControlledAngleFunc(
            gamma_base,
            phi_base,
            psi_base,
            phase_offset,
            omega_w,
            harmonic_period_wingbeats_w,
            gamma_mean_base,
            psi_mean_base,
            phi_amp_base,
            wing_phi_amp_base,
            gamma_mean_cmd,
            psi_mean_cmd,
            phi_amp_cmd,
            EPS
        );
        wings[w].setAngleFunc(std::move(angleFunc));
    }

    // Setup controller
    TrajectoryController controller;
    controller.setGains(x_gains, y_gains, z_gains);
    controller.setTrajectory(std::move(trajectory));
    controller.setBaseline(gamma_mean_base, psi_mean_base, phi_amp_base);
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
    applyControlOutput(ctrl, gamma_mean_cmd, psi_mean_cmd, phi_amp_cmd);
    appendControllerState(output, controller, ctrl);

    // Time integration with controller
    std::cout << "Running trajectory tracking for " << cfg.getInt("n_wingbeats", 5)
              << " wingbeats..." << std::endl;

    for (int i = 0; i < tp.nsteps; ++i) {
        // Controller is already computed for i=0 at t=0.
        if (i > 0) {
            ctrl = controller.compute(t, tp.dt, state);
            applyControlOutput(ctrl, gamma_mean_cmd, psi_mean_cmd, phi_amp_cmd);
        }

        // 2. Physics step
        state = stepRK4(t, tp.dt, state, wings, scratch);
        t += tp.dt;

        // Store outputs
        storeTimestep(output, t, state, wings, wing_data);

        // Store controller state
        appendControllerState(output, controller, ctrl);
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

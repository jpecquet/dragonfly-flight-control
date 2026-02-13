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

    auto wingConfigs = buildWingConfigs(cfg, kin);
    auto wings = createWings(wingConfigs, kin);
    auto output = initOutput(wingConfigs, kin, tp.nsteps);

    constexpr double EPS = 1e-12;
    auto hasWingMotionSeries = [](const WingConfig& w) {
        return !w.gamma_cos.empty() && !w.gamma_sin.empty() &&
               !w.phi_cos.empty() && !w.phi_sin.empty() &&
               !w.psi_cos.empty() && !w.psi_sin.empty();
    };

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
            phi_amp_base += kin.phi_amp;
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
        phi_amp_base = kin.phi_amp;
    }

    // Controller-updated parameters shared by all pre-bound wing angle lambdas.
    double gamma_mean_cmd = gamma_mean_base;
    double psi_mean_cmd = psi_mean_base;
    double phi_amp_cmd = phi_amp_base;
    for (size_t w = 0; w < wings.size(); ++w) {
        const auto& wcfg = wingConfigs[w];
        const bool use_local = hasWingMotionSeries(wcfg);
        const double omega_w = use_local ? wcfg.omega : kin.omega;
        const HarmonicSeries gamma_base{
            use_local ? wcfg.gamma_mean : kin.gamma_mean,
            use_local ? wcfg.gamma_cos : kin.gamma_cos,
            use_local ? wcfg.gamma_sin : kin.gamma_sin
        };
        const HarmonicSeries phi_base{
            use_local ? wcfg.phi_mean : kin.phi_mean,
            use_local ? wcfg.phi_cos : kin.phi_cos,
            use_local ? wcfg.phi_sin : kin.phi_sin
        };
        const HarmonicSeries psi_base{
            use_local ? wcfg.psi_mean : kin.psi_mean,
            use_local ? wcfg.psi_cos : kin.psi_cos,
            use_local ? wcfg.psi_sin : kin.psi_sin
        };
        const double phi_c1 = phi_base.cos_coeff.empty() ? 0.0 : phi_base.cos_coeff[0];
        const double phi_s1 = phi_base.sin_coeff.empty() ? 0.0 : phi_base.sin_coeff[0];
        const double wing_phi_amp_base = std::hypot(phi_c1, phi_s1);
        const double phase_offset = wingConfigs[w].phaseOffset;
        auto angleFunc = [&gamma_mean_cmd, &psi_mean_cmd, &phi_amp_cmd,
                          omega_w,
                          gamma_mean_base,
                          psi_mean_base,
                          phi_amp_base,
                          wing_phi_amp_base,
                          gamma_base,
                          phi_base,
                          psi_base,
                          eps = EPS,
                          phase_offset](double t) -> WingAngles {
            const double phase = omega_w * t + phase_offset;
            const double gamma_shift = gamma_mean_cmd - gamma_mean_base;
            const double psi_shift = psi_mean_cmd - psi_mean_base;

            const double phi_ref_amp = (std::abs(phi_amp_base) > eps) ? phi_amp_base : wing_phi_amp_base;
            double phi_scale = 1.0;
            if (std::abs(phi_ref_amp) > eps) {
                phi_scale = phi_amp_cmd / phi_ref_amp;
            }

            double gam = evaluateHarmonicValue(gamma_base, phase) + gamma_shift;
            double gam_dot = evaluateHarmonicRate(gamma_base, phase, omega_w);
            double phi = evaluateHarmonicValue(phi_base, phase, phi_scale);
            double phi_dot = evaluateHarmonicRate(phi_base, phase, omega_w, phi_scale);

            // If both baseline references are zero, allow controller to inject a first harmonic.
            if (std::abs(phi_ref_amp) <= eps && std::abs(phi_amp_cmd) > eps) {
                phi += phi_amp_cmd * std::cos(phase);
                phi_dot += -phi_amp_cmd * omega_w * std::sin(phase);
            }

            double psi = evaluateHarmonicValue(psi_base, phase) + psi_shift;
            return {
                gam,
                gam_dot,
                phi,
                phi_dot,
                psi
            };
        };
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

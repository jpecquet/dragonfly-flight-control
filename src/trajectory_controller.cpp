#include "trajectory_controller.hpp"

#include <algorithm>

TrajectoryController::TrajectoryController()
    : trajectory_(trajectories::hover(Vec3::Zero())) {}

void TrajectoryController::setGains(const PIDGains& x_gains, const PIDGains& y_gains, const PIDGains& z_gains) {
    pid_x_.setGains(x_gains);
    pid_y_.setGains(y_gains);
    pid_z_.setGains(z_gains);
}

void TrajectoryController::setTrajectory(TrajectoryFunc traj) {
    trajectory_ = std::move(traj);
}

void TrajectoryController::setBaseline(double gamma_mean, double psi_mean, double phi_amp) {
    gamma_mean_base_ = gamma_mean;
    psi_mean_base_ = psi_mean;
    phi_amp_base_ = phi_amp;
}

void TrajectoryController::setMixing(const MixingMatrix& mix) {
    mixing_ = mix;
}

void TrajectoryController::setBounds(const ParameterBounds& bounds) {
    bounds_ = bounds;
}

ControlOutput TrajectoryController::compute(double t, double dt, const State& state) {
    // Get desired trajectory point
    TrajectoryPoint target = trajectory_(t);

    // Compute position and velocity errors
    Vec3 pos_error = target.position - state.pos;
    Vec3 vel_error = target.velocity - state.vel;

    // Compute PID outputs for each axis
    double cmd_x = pid_x_.compute(dt, pos_error.x(), vel_error.x());
    double cmd_y = pid_y_.compute(dt, pos_error.y(), vel_error.y());
    double cmd_z = pid_z_.compute(dt, pos_error.z(), vel_error.z());

    Vec3 pid_output(cmd_x, cmd_y, cmd_z);

    // Apply mixing matrix to get parameter deltas
    double delta_gamma_mean = mixing_.gamma_mean_mix.dot(pid_output);
    double delta_psi_mean = mixing_.psi_mean_mix.dot(pid_output);
    double delta_phi_amp = mixing_.phi_amp_mix.dot(pid_output);

    // Compute final parameters
    double gamma_mean = gamma_mean_base_ + delta_gamma_mean;
    double psi_mean = psi_mean_base_ + delta_psi_mean;
    double phi_amp = phi_amp_base_ + delta_phi_amp;

    // Clamp to bounds
    gamma_mean = std::clamp(gamma_mean, bounds_.gamma_mean_min, bounds_.gamma_mean_max);
    psi_mean = std::clamp(psi_mean, bounds_.psi_mean_min, bounds_.psi_mean_max);
    phi_amp = std::clamp(phi_amp, bounds_.phi_amp_min, bounds_.phi_amp_max);

    // Store state for logging
    last_state_ = {
        target.position,
        target.velocity,
        pos_error,
        vel_error,
        pid_output,
        {gamma_mean, psi_mean, phi_amp, gamma_mean_base_, psi_mean_base_, phi_amp_base_}
    };

    return {gamma_mean, psi_mean, phi_amp, gamma_mean_base_, psi_mean_base_, phi_amp_base_};
}

void TrajectoryController::reset() {
    pid_x_.reset();
    pid_y_.reset();
    pid_z_.reset();
}

// Config loading helpers

PIDGains readPIDGains(const Config& cfg, const std::string& prefix, const PIDGains& defaults) {
    return PIDGains(
        cfg.getDouble(prefix + "kp", defaults.Kp),
        cfg.getDouble(prefix + "ki", defaults.Ki),
        cfg.getDouble(prefix + "kd", defaults.Kd),
        cfg.getDouble(prefix + "imax", defaults.i_max)
    );
}

ParameterBounds readParameterBounds(const Config& cfg) {
    ParameterBounds bounds;
    bounds.gamma_mean_min = cfg.getDouble("gamma_mean_min", bounds.gamma_mean_min);
    bounds.gamma_mean_max = cfg.getDouble("gamma_mean_max", bounds.gamma_mean_max);
    bounds.psi_mean_min = cfg.getDouble("psi_mean_min", bounds.psi_mean_min);
    bounds.psi_mean_max = cfg.getDouble("psi_mean_max", bounds.psi_mean_max);
    bounds.phi_amp_min = cfg.getDouble("phi_amp_min", bounds.phi_amp_min);
    bounds.phi_amp_max = cfg.getDouble("phi_amp_max", bounds.phi_amp_max);
    return bounds;
}

MixingMatrix readMixingMatrix(const Config& cfg) {
    MixingMatrix mixing;

    auto readVec3 = [&](const std::string& prefix, Vec3& target) {
        if (cfg.has(prefix + "_x")) {
            target = Vec3(
                cfg.getDouble(prefix + "_x"),
                cfg.getDouble(prefix + "_y", 0.0),
                cfg.getDouble(prefix + "_z")
            );
        }
    };

    readVec3("mix_gamma", mixing.gamma_mean_mix);
    readVec3("mix_psi", mixing.psi_mean_mix);
    readVec3("mix_phi", mixing.phi_amp_mix);
    return mixing;
}

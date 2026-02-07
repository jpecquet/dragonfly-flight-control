#pragma once

#include "config.hpp"
#include "controller.hpp"
#include "eom.hpp"
#include "trajectory.hpp"

#include <cmath>
#include <string>

// Control output: kinematic parameters to apply
struct ControlOutput {
    double gamma_mean;  // Stroke plane angle
    double psi_mean;    // Mean pitch angle
    double phi_amp;     // Stroke amplitude

    // Baseline values (for output/logging)
    double gamma_mean_base;
    double psi_mean_base;
    double phi_amp_base;
};

// Mixing matrix: maps axis commands to parameter deltas
// Each row is [x_coeff, y_coeff, z_coeff]
struct MixingMatrix {
    Vec3 gamma_mean_mix;  // How x,y,z commands affect gamma_mean
    Vec3 psi_mean_mix;    // How x,y,z commands affect psi_mean
    Vec3 phi_amp_mix;     // How x,y,z commands affect phi_amp

    MixingMatrix()
        : gamma_mean_mix(0.2, 0.0, 0.3),   // x affects forward tilt, z affects lift
          psi_mean_mix(-0.5, 0.0, 0.2),    // x affects pitch for thrust direction
          phi_amp_mix(0.0, 0.0, 0.8) {}    // z affects stroke amplitude for lift
};

// Parameter bounds for clamping
struct ParameterBounds {
    double gamma_mean_min = 0.5;
    double gamma_mean_max = 2.5;
    double psi_mean_min = 0.0;
    double psi_mean_max = M_PI / 2.0;
    double phi_amp_min = 0.1;
    double phi_amp_max = 0.8;
};

// Controller state for logging
struct ControllerState {
    Vec3 target_pos;
    Vec3 target_vel;
    Vec3 pos_error;
    Vec3 vel_error;
    Vec3 pid_output;  // Raw PID outputs before mixing
    ControlOutput params;
};

// Trajectory tracking controller
// Coordinates 3 PID controllers (one per axis) and maps outputs to kinematic parameters
class TrajectoryController {
public:
    TrajectoryController();

    // Configure PID gains for each axis
    void setGains(const PIDGains& x_gains, const PIDGains& y_gains, const PIDGains& z_gains);

    // Set trajectory to track
    void setTrajectory(TrajectoryFunc traj);

    // Set baseline kinematic parameters (from config)
    void setBaseline(double gamma_mean, double psi_mean, double phi_amp);

    // Set mixing matrix
    void setMixing(const MixingMatrix& mix);

    // Set parameter bounds
    void setBounds(const ParameterBounds& bounds);

    // Compute control output
    // t: current time
    // dt: timestep
    // state: current state (position and velocity)
    // Returns: kinematic parameters to use
    ControlOutput compute(double t, double dt, const State& state);

    // Get last controller state (for logging)
    const ControllerState& lastState() const { return last_state_; }

    // Reset all integral states
    void reset();

private:
    PIDController pid_x_;
    PIDController pid_y_;
    PIDController pid_z_;

    TrajectoryFunc trajectory_;
    MixingMatrix mixing_;
    ParameterBounds bounds_;

    // Baseline parameters
    double gamma_mean_base_ = M_PI / 2.0;
    double psi_mean_base_ = M_PI / 4.0;
    double phi_amp_base_ = M_PI / 8.0;

    ControllerState last_state_;
};

// Config loading helpers
PIDGains readPIDGains(const Config& cfg, const std::string& prefix, const PIDGains& defaults);
ParameterBounds readParameterBounds(const Config& cfg);
MixingMatrix readMixingMatrix(const Config& cfg);

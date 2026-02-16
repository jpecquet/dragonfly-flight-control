#pragma once

#include "config.hpp"
#include "eom.hpp"
#include "kinematics.hpp"
#include "wing.hpp"

#include <string>
#include <vector>

// Kinematic values read from config (plain doubles, distinct from optimizer's KinematicParam)
struct SimKinematicParams : MotionParams {
    int n_harmonics = 1;
};

struct SimulationOutput {
    // Wing configurations (supports variable number of wings)
    std::vector<WingConfig> wingConfigs;

    // Kinematic parameters (shared across all wings)
    SimKinematicParams kin;

    std::vector<double> time;
    std::vector<State> states;
    std::vector<std::vector<SingleWingVectors>> wing_data;

    // Controller data (only populated when tracking)
    bool controller_active = false;
    std::vector<Vec3> target_positions;   // Trajectory targets
    std::vector<Vec3> position_errors;    // Tracking errors
    std::vector<double> param_gamma_mean; // Time history of gamma_mean
    std::vector<double> param_psi_mean;   // Time history of psi_mean
    std::vector<double> param_phi_amp;    // Time history of phi_amp
};

// Write simulation output to HDF5 file
void writeHDF5(const std::string& filename, const SimulationOutput& output,
               const std::vector<Wing>& wings);

struct TimeParams {
    double Twb = 0.0, dt = 0.0, T = 0.0;
    int nsteps = 0;
};

SimKinematicParams readKinematicParams(const Config& cfg);
State readInitialState(const Config& cfg);
TimeParams readTimeParams(const Config& cfg, double omega);
std::vector<WingConfig> buildWingConfigs(const Config& cfg, const SimKinematicParams& default_kin);
// True when a wing declared explicit per-wing motion overrides in config.
bool hasWingMotionSeries(const WingConfig& w);
std::vector<Wing> createWings(const std::vector<WingConfig>& wc, const SimKinematicParams& kin);
SimulationOutput initOutput(const std::vector<WingConfig>& wc, const SimKinematicParams& kin, int nsteps);
void storeTimestep(SimulationOutput& out, double t, const State& state,
                   const std::vector<Wing>& wings, std::vector<SingleWingVectors>& wing_data);

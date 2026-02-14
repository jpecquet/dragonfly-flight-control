#pragma once

#include "eom.hpp"
#include "wing.hpp"

#include <string>
#include <vector>

// Kinematic values read from config (plain doubles, distinct from optimizer's KinematicParam)
struct SimKinematicParams {
    double omega = 0.0;
    int n_harmonics = 1;
    double gamma_mean = 0.0;
    double phi_mean = 0.0;
    double psi_mean = 0.0;
    std::vector<double> gamma_cos;
    std::vector<double> gamma_sin;
    std::vector<double> phi_cos;
    std::vector<double> phi_sin;
    std::vector<double> psi_cos;
    std::vector<double> psi_sin;
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

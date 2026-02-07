#pragma once

#include "eom.hpp"
#include "wing.hpp"

#include <string>
#include <vector>

struct SimulationOutput {
    // Wing configurations (supports variable number of wings)
    std::vector<WingConfig> wingConfigs;

    // Kinematic parameters (shared across all wings)
    double omega = 0.0;       // Wing beat frequency
    double gamma_mean = 0.0;  // Mean stroke plane angle (baseline for tracking)
    double gamma_amp = 0.0;   // Stroke plane oscillation amplitude
    double gamma_phase = 0.0; // Stroke plane phase offset
    double phi_amp = 0.0;     // Stroke amplitude (baseline for tracking)
    double psi_mean = 0.0;    // Mean pitch angle (baseline for tracking)
    double psi_amp = 0.0;     // Pitch oscillation amplitude
    double psi_phase = 0.0;   // Pitch phase offset

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

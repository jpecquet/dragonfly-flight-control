#pragma once

#include "eom.hpp"
#include "wing.hpp"

#include <string>
#include <vector>

struct SimulationOutput {
    // Wing configurations (supports variable number of wings)
    std::vector<WingConfig> wingConfigs;

    // Kinematic parameters (shared across all wings)
    double omega;       // Wing beat frequency
    double gamma_mean;  // Mean stroke plane angle (baseline for tracking)
    double gamma_amp;   // Stroke plane oscillation amplitude
    double gamma_phase; // Stroke plane phase offset
    double phi_amp;     // Stroke amplitude (baseline for tracking)
    double psi_mean;    // Mean pitch angle (baseline for tracking)
    double psi_amp;     // Pitch oscillation amplitude
    double psi_phase;   // Pitch phase offset

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

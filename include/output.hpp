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
    double gamma_mean;  // Mean stroke plane angle
    double gamma_amp;   // Stroke plane oscillation amplitude
    double gamma_phase; // Stroke plane phase offset
    double phi_amp;     // Stroke amplitude
    double psi_mean;    // Mean pitch angle
    double psi_amp;     // Pitch oscillation amplitude
    double psi_phase;   // Pitch phase offset

    std::vector<double> time;
    std::vector<State> states;
    std::vector<std::vector<SingleWingVectors>> wing_data;
};

// Write simulation output to HDF5 file
void writeHDF5(const std::string& filename, const SimulationOutput& output,
               const std::vector<Wing>& wings);

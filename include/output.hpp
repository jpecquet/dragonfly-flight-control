#pragma once

#include "eom.hpp"
#include "wing.hpp"

#include <string>
#include <vector>

struct SimulationOutput {
    // Wing configurations (supports variable number of wings)
    std::vector<WingConfig> wingConfigs;

    // Kinematic parameters (shared across all wings)
    double omg0;    // Wing beat frequency
    double gam0;    // Mean stroke plane angle
    double dgam;    // Stroke plane oscillation amplitude
    double dlt_gam; // Stroke plane phase offset
    double phi0;    // Stroke amplitude
    double psim;    // Mean pitch angle
    double dpsi;    // Pitch oscillation amplitude
    double dlt0;    // Pitch phase offset

    std::vector<double> time;
    std::vector<State> states;
    std::vector<std::vector<SingleWingVectors>> wing_data;
};

// Write simulation output to HDF5 file
void writeHDF5(const std::string& filename, const SimulationOutput& output,
               const std::vector<Wing>& wings);

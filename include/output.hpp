#pragma once

#include "eom.hpp"

#include <string>
#include <vector>

struct SimulationOutput {
    Parameters params;
    std::vector<double> time;
    std::vector<State> states;
    std::vector<WingVectors> wing_data;
};

// Write simulation output to HDF5 file
void writeHDF5(const std::string& filename, const SimulationOutput& output);

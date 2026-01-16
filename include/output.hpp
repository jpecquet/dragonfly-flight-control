#pragma once

#include "eom.hpp"
#include "wing.hpp"

#include <string>
#include <vector>

struct SimulationOutput {
    // Wing geometry parameters
    double lb0_f;  // Forewing length
    double lb0_h;  // Hindwing length
    double mu0_f;  // Forewing mass parameter
    double mu0_h;  // Hindwing mass parameter

    // Aerodynamic coefficients
    double Cd0;    // Base drag coefficient
    double Cl0;    // Base lift coefficient

    // Kinematic parameters
    double omg0;   // Wing beat frequency
    double gam0;   // Stroke plane angle
    double phi0;   // Stroke amplitude
    double psim;   // Mean pitch angle
    double dpsi;   // Pitch oscillation amplitude
    double sig0;   // Fore/hindwing phase offset
    double dlt0;   // Pitch phase offset

    std::vector<double> time;
    std::vector<State> states;
    std::vector<std::vector<SingleWingVectors>> wing_data;
};

// Write simulation output to HDF5 file
void writeHDF5(const std::string& filename, const SimulationOutput& output,
               const std::vector<Wing>& wings);

#pragma once

#include "config.hpp"
#include "eom.hpp"
#include "kinematics.hpp"
#include "output.hpp"
#include "wing.hpp"

#include <vector>

// Kinematic values read from config (plain doubles, distinct from optimizer's KinematicParam)
struct SimKinematicParams {
    double omega, gamma_mean, gamma_amp, gamma_phase;
    double phi_amp, psi_mean, psi_amp, psi_phase;
};

struct TimeParams {
    double Twb, dt, T;
    int nsteps;
};

SimKinematicParams readKinematicParams(const Config& cfg);
State readInitialState(const Config& cfg);
TimeParams readTimeParams(const Config& cfg, double omega);
std::vector<WingConfig> buildWingConfigs(const Config& cfg);
std::vector<Wing> createWings(const std::vector<WingConfig>& wc, const SimKinematicParams& kin);
SimulationOutput initOutput(const std::vector<WingConfig>& wc, const SimKinematicParams& kin, int nsteps);
void storeTimestep(SimulationOutput& out, double t, const State& state,
                   const std::vector<Wing>& wings, std::vector<SingleWingVectors>& wing_data);

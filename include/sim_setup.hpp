#pragma once

#include "config.hpp"
#include "eom.hpp"
#include "kinematics.hpp"
#include "output.hpp"
#include "wing.hpp"

#include <vector>

// Kinematic values read from config (plain doubles, distinct from optimizer's KinematicParam)
struct SimKinematicParams {
    double omega = 0.0, gamma_mean = 0.0, gamma_amp = 0.0, gamma_phase = 0.0;
    double phi_amp = 0.0, psi_mean = 0.0, psi_amp = 0.0, psi_phase = 0.0;
};

struct TimeParams {
    double Twb = 0.0, dt = 0.0, T = 0.0;
    int nsteps = 0;
};

SimKinematicParams readKinematicParams(const Config& cfg);
State readInitialState(const Config& cfg);
TimeParams readTimeParams(const Config& cfg, double omega);
std::vector<WingConfig> buildWingConfigs(const Config& cfg);
std::vector<Wing> createWings(const std::vector<WingConfig>& wc,
                              double omega, double gamma_mean, double gamma_amp,
                              double gamma_phase, double phi_amp, double psi_mean,
                              double psi_amp, double psi_phase);
std::vector<Wing> createWings(const std::vector<WingConfig>& wc, const SimKinematicParams& kin);
SimulationOutput initOutput(const std::vector<WingConfig>& wc, const SimKinematicParams& kin, int nsteps);
void storeTimestep(SimulationOutput& out, double t, const State& state,
                   const std::vector<Wing>& wings, std::vector<SingleWingVectors>& wing_data);

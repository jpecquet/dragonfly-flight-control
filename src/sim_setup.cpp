#include "sim_setup.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

SimKinematicParams readKinematicParams(const Config& cfg) {
    return {
        cfg.getDouble("omega"),
        cfg.getDouble("gamma_mean"),
        cfg.getDouble("gamma_amp", 0.0),
        cfg.getDouble("gamma_phase", 0.0),
        cfg.getDouble("phi_amp"),
        cfg.getDouble("psi_mean"),
        cfg.getDouble("psi_amp"),
        cfg.getDouble("psi_phase")
    };
}

State readInitialState(const Config& cfg) {
    return State(
        Vec3(cfg.getDouble("x0", 0.0), cfg.getDouble("y0", 0.0), cfg.getDouble("z0", 0.0)),
        Vec3(cfg.getDouble("ux0", 0.0), cfg.getDouble("uy0", 0.0), cfg.getDouble("uz0", 0.0))
    );
}

TimeParams readTimeParams(const Config& cfg, double omega) {
    int n_wingbeats = cfg.getInt("n_wingbeats", 5);
    int steps_per_wingbeat = cfg.getInt("steps_per_wingbeat", 50);

    if (!std::isfinite(omega) || omega <= 0.0) {
        throw std::runtime_error("omega must be finite and > 0");
    }
    if (n_wingbeats < 0) {
        throw std::runtime_error("n_wingbeats must be >= 0");
    }
    if (steps_per_wingbeat <= 0) {
        throw std::runtime_error("steps_per_wingbeat must be > 0");
    }

    double Twb = 2.0 * M_PI / omega;
    double dt = Twb / steps_per_wingbeat;
    long long nsteps_ll = static_cast<long long>(n_wingbeats) * steps_per_wingbeat;
    if (nsteps_ll > std::numeric_limits<int>::max()) {
        throw std::runtime_error("Too many integration steps; reduce n_wingbeats or steps_per_wingbeat");
    }
    int nsteps = static_cast<int>(nsteps_ll);
    double T = nsteps * dt;

    return {Twb, dt, T, nsteps};
}

std::vector<WingConfig> buildWingConfigs(const Config& cfg) {
    if (!cfg.hasWings()) {
        throw std::runtime_error("Config must define wings using [[wing]] sections");
    }

    std::vector<WingConfig> wingConfigs;
    for (const auto& entry : cfg.getWingEntries()) {
        WingSide side = parseSide(entry.side);
        wingConfigs.emplace_back(entry.name, side, entry.mu0, entry.lb0,
                                 entry.Cd0, entry.Cl0, entry.phase);
    }
    std::cout << "Loaded " << wingConfigs.size() << " wings from config" << std::endl;
    return wingConfigs;
}

std::vector<Wing> createWings(const std::vector<WingConfig>& wc, const SimKinematicParams& kin) {
    std::vector<Wing> wings;
    wings.reserve(wc.size());
    for (const auto& w : wc) {
        auto angleFunc = makeAngleFunc(kin.gamma_mean, kin.gamma_amp, kin.gamma_phase,
                                        kin.phi_amp, kin.psi_mean, kin.psi_amp,
                                        kin.psi_phase, w.phaseOffset, kin.omega);
        wings.emplace_back(w.name, w.mu0, w.lb0, w.side, w.Cd0, w.Cl0, angleFunc);
    }
    return wings;
}

SimulationOutput initOutput(const std::vector<WingConfig>& wc, const SimKinematicParams& kin, int nsteps) {
    SimulationOutput output;
    output.wingConfigs = wc;
    output.kin = kin;
    output.time.reserve(nsteps + 1);
    output.states.reserve(nsteps + 1);
    output.wing_data.reserve(nsteps + 1);
    return output;
}

void storeTimestep(SimulationOutput& out, double t, const State& state,
                   const std::vector<Wing>& wings, std::vector<SingleWingVectors>& wing_data) {
    equationOfMotion(t, state, wings, wing_data);
    out.time.push_back(t);
    out.states.push_back(state);
    out.wing_data.push_back(wing_data);
}

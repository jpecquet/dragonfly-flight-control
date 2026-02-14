#include "sim_setup.hpp"
#include "parse_utils.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <stdexcept>

namespace {

std::vector<double> zeroHarmonics(int n) {
    return std::vector<double>(static_cast<size_t>(n), 0.0);
}

void requireHarmonicLength(const std::vector<double>& values, int expected,
                           const std::string& key) {
    if (values.size() != static_cast<size_t>(expected)) {
        throw std::runtime_error(key + " expects " + std::to_string(expected) +
                                 " values, got " + std::to_string(values.size()));
    }
}

const std::string* findOverride(const WingConfigEntry& entry, const std::string& key) {
    auto it = entry.motion_overrides.find(key);
    return (it == entry.motion_overrides.end()) ? nullptr : &it->second;
}

void loadAngleSeries(const Config& cfg, const std::string& prefix,
                     int n_harmonics, double default_mean,
                     double& mean_out, std::vector<double>& cos_out, std::vector<double>& sin_out) {
    const std::string mean_key = prefix + "_mean";
    const std::string cos_key = prefix + "_cos";
    const std::string sin_key = prefix + "_sin";
    mean_out = cfg.getDouble(mean_key, default_mean);
    cos_out = cfg.getDoubleList(cos_key, zeroHarmonics(n_harmonics));
    sin_out = cfg.getDoubleList(sin_key, zeroHarmonics(n_harmonics));
    requireHarmonicLength(cos_out, n_harmonics, cos_key);
    requireHarmonicLength(sin_out, n_harmonics, sin_key);
}

void applyWingAngleOverrides(
    const WingConfigEntry& entry,
    const std::string& wing_label,
    const std::string& prefix,
    int n_harmonics,
    double& mean,
    std::vector<double>& cos_coeff,
    std::vector<double>& sin_coeff
) {
    const std::string mean_key = prefix + "_mean";
    const std::string cos_key = prefix + "_cos";
    const std::string sin_key = prefix + "_sin";

    const bool has_cos = findOverride(entry, cos_key) != nullptr;
    const bool has_sin = findOverride(entry, sin_key) != nullptr;

    if (const std::string* mean_val = findOverride(entry, mean_key)) {
        mean = parseutil::parseDoubleStrict(*mean_val, "wing '" + wing_label + "' key '" + mean_key + "'");
    }

    if (has_cos) {
        cos_coeff = parseutil::parseDoubleListStrict(*findOverride(entry, cos_key),
                                                     "wing '" + wing_label + "' key '" + cos_key + "'");
        requireHarmonicLength(cos_coeff, n_harmonics, "wing '" + wing_label + "' key '" + cos_key + "'");
    }
    if (has_sin) {
        sin_coeff = parseutil::parseDoubleListStrict(*findOverride(entry, sin_key),
                                                     "wing '" + wing_label + "' key '" + sin_key + "'");
        requireHarmonicLength(sin_coeff, n_harmonics, "wing '" + wing_label + "' key '" + sin_key + "'");
    }
}

void populateWingMotion(
    const WingConfigEntry& entry,
    const SimKinematicParams& default_kin,
    WingConfig& out
) {
    const int n_harmonics = default_kin.n_harmonics;

    out.omega = default_kin.omega;
    out.gamma_mean = default_kin.gamma_mean;
    out.phi_mean = default_kin.phi_mean;
    out.psi_mean = default_kin.psi_mean;

    out.gamma_cos = default_kin.gamma_cos;
    out.gamma_sin = default_kin.gamma_sin;
    out.phi_cos = default_kin.phi_cos;
    out.phi_sin = default_kin.phi_sin;
    out.psi_cos = default_kin.psi_cos;
    out.psi_sin = default_kin.psi_sin;

    auto ensure_size = [n_harmonics](std::vector<double>& values) {
        if (values.size() != static_cast<size_t>(n_harmonics)) {
            values.resize(static_cast<size_t>(n_harmonics), 0.0);
        }
    };
    ensure_size(out.gamma_cos);
    ensure_size(out.gamma_sin);
    ensure_size(out.phi_cos);
    ensure_size(out.phi_sin);
    ensure_size(out.psi_cos);
    ensure_size(out.psi_sin);

    if (entry.motion_overrides.empty()) {
        out.has_custom_motion = false;
        return;
    }

    std::string wing_label = entry.name + "_" + entry.side;
    if (const std::string* omega_override = findOverride(entry, "omega")) {
        out.omega = parseutil::parseDoubleStrict(*omega_override, "wing '" + wing_label + "' key 'omega'");
    }

    applyWingAngleOverrides(entry, wing_label, "gamma", n_harmonics,
                            out.gamma_mean, out.gamma_cos, out.gamma_sin);
    applyWingAngleOverrides(entry, wing_label, "phi", n_harmonics,
                            out.phi_mean, out.phi_cos, out.phi_sin);
    applyWingAngleOverrides(entry, wing_label, "psi", n_harmonics,
                            out.psi_mean, out.psi_cos, out.psi_sin);

    out.has_custom_motion = true;
}

}  // namespace

bool hasWingMotionSeries(const WingConfig& w) {
    return !w.gamma_cos.empty() && !w.gamma_sin.empty() &&
           !w.phi_cos.empty() && !w.phi_sin.empty() &&
           !w.psi_cos.empty() && !w.psi_sin.empty();
}

SimKinematicParams readKinematicParams(const Config& cfg) {
    SimKinematicParams kin;
    kin.omega = cfg.getDouble("omega");
    kin.n_harmonics = cfg.getInt("n_harmonics", 1);
    if (kin.n_harmonics <= 0) {
        throw std::runtime_error("n_harmonics must be >= 1");
    }

    loadAngleSeries(
        cfg, "gamma", kin.n_harmonics, cfg.getDouble("gamma_mean", 0.0),
        kin.gamma_mean, kin.gamma_cos, kin.gamma_sin
    );

    loadAngleSeries(
        cfg, "phi", kin.n_harmonics, cfg.getDouble("phi_mean", 0.0),
        kin.phi_mean, kin.phi_cos, kin.phi_sin
    );

    loadAngleSeries(
        cfg, "psi", kin.n_harmonics, cfg.getDouble("psi_mean", 0.0),
        kin.psi_mean, kin.psi_cos, kin.psi_sin
    );

    return kin;
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

std::vector<WingConfig> buildWingConfigs(const Config& cfg, const SimKinematicParams& default_kin) {
    if (!cfg.hasWings()) {
        throw std::runtime_error("Config must define wings using [[wing]] sections");
    }

    std::vector<WingConfig> wingConfigs;
    for (const auto& entry : cfg.getWingEntries()) {
        WingSide side = parseSide(entry.side);
        WingConfig config(entry.name, side, entry.mu0, entry.lb0,
                          entry.Cd0, entry.Cl0, entry.phase);
        populateWingMotion(entry, default_kin, config);
        wingConfigs.push_back(std::move(config));
    }
    std::cout << "Loaded " << wingConfigs.size() << " wings from config" << std::endl;
    return wingConfigs;
}

std::vector<Wing> createWings(const std::vector<WingConfig>& wc, const SimKinematicParams& kin) {
    std::vector<Wing> wings;
    wings.reserve(wc.size());
    const auto kin_series = kin.toHarmonicSeries();
    for (const auto& w : wc) {
        HarmonicSeries gamma = kin_series.gamma;
        HarmonicSeries phi = kin_series.phi;
        HarmonicSeries psi = kin_series.psi;
        double omega = kin.omega;

        if (hasWingMotionSeries(w)) {
            const auto wing_series = w.toHarmonicSeries();
            gamma = wing_series.gamma;
            phi = wing_series.phi;
            psi = wing_series.psi;
            omega = w.omega;
        }

        auto angleFunc = makeAngleFunc(gamma, phi, psi, w.phase_offset, omega);
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

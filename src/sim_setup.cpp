#include "sim_setup.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <sstream>
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

double firstOrZero(const std::vector<double>& v) {
    return v.empty() ? 0.0 : v.front();
}

std::string trimToken(const std::string& s) {
    size_t start = s.find_first_not_of(" \t");
    if (start == std::string::npos) return "";
    size_t end = s.find_last_not_of(" \t");
    return s.substr(start, end - start + 1);
}

double parseDoubleString(const std::string& value, const std::string& context) {
    std::string token = trimToken(value);
    try {
        size_t idx = 0;
        double parsed = std::stod(token, &idx);
        if (idx != token.size()) {
            throw std::runtime_error("trailing characters");
        }
        return parsed;
    } catch (const std::exception&) {
        throw std::runtime_error("Invalid value for " + context + ": expected number, got '" + value + "'");
    }
}

std::vector<double> parseDoubleListString(const std::string& raw_value, const std::string& context) {
    std::string value = trimToken(raw_value);
    if (value.size() >= 2 && value.front() == '[' && value.back() == ']') {
        value = trimToken(value.substr(1, value.size() - 2));
    }
    if (value.empty()) {
        throw std::runtime_error("Invalid value for " + context +
                                 ": expected comma-separated numbers, got empty");
    }

    std::vector<double> values;
    std::stringstream ss(value);
    std::string token;
    while (std::getline(ss, token, ',')) {
        token = trimToken(token);
        if (token.empty()) {
            throw std::runtime_error("Invalid value for " + context + ": empty list item");
        }
        values.push_back(parseDoubleString(token, context));
    }
    return values;
}

const std::string* findOverride(const WingConfigEntry& entry, const std::string& key) {
    auto it = entry.motion_overrides.find(key);
    return (it == entry.motion_overrides.end()) ? nullptr : &it->second;
}

void setLegacyAliases(SimKinematicParams& kin) {
    const double gamma_c1 = firstOrZero(kin.gamma_cos);
    const double gamma_s1 = firstOrZero(kin.gamma_sin);
    kin.gamma_amp = std::hypot(gamma_c1, gamma_s1);
    kin.gamma_phase = std::atan2(-gamma_s1, gamma_c1);

    const double phi_c1 = firstOrZero(kin.phi_cos);
    const double phi_s1 = firstOrZero(kin.phi_sin);
    kin.phi_amp = std::hypot(phi_c1, phi_s1);

    const double psi_c1 = firstOrZero(kin.psi_cos);
    const double psi_s1 = firstOrZero(kin.psi_sin);
    kin.psi_amp = std::hypot(psi_c1, psi_s1);
    kin.psi_phase = std::atan2(-psi_s1, psi_c1);
}

void loadAngleSeries(const Config& cfg, const std::string& prefix,
                     int n_harmonics, double legacy_mean,
                     bool require_mean_when_legacy,
                     const std::function<void(std::vector<double>&, std::vector<double>&)>& legacy_loader,
                     double& mean_out, std::vector<double>& cos_out, std::vector<double>& sin_out) {
    const std::string mean_key = prefix + "_mean";
    const std::string cos_key = prefix + "_cos";
    const std::string sin_key = prefix + "_sin";
    const bool has_harmonics = cfg.has(cos_key) || cfg.has(sin_key);

    if (!has_harmonics && require_mean_when_legacy && !cfg.has(mean_key)) {
        throw std::runtime_error("Missing config key: " + mean_key);
    }

    mean_out = cfg.getDouble(mean_key, legacy_mean);
    if (has_harmonics) {
        cos_out = cfg.getDoubleList(cos_key, zeroHarmonics(n_harmonics));
        sin_out = cfg.getDoubleList(sin_key, zeroHarmonics(n_harmonics));
        requireHarmonicLength(cos_out, n_harmonics, cos_key);
        requireHarmonicLength(sin_out, n_harmonics, sin_key);
        return;
    }

    cos_out = zeroHarmonics(n_harmonics);
    sin_out = zeroHarmonics(n_harmonics);
    legacy_loader(cos_out, sin_out);
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
    const std::string amp_key = prefix + "_amp";
    const std::string phase_key = prefix + "_phase";

    const bool has_cos = findOverride(entry, cos_key) != nullptr;
    const bool has_sin = findOverride(entry, sin_key) != nullptr;
    const bool has_amp = findOverride(entry, amp_key) != nullptr;
    const bool has_phase = findOverride(entry, phase_key) != nullptr;

    if ((has_cos || has_sin) && (has_amp || has_phase)) {
        throw std::runtime_error(
            "Wing '" + wing_label + "' mixes harmonic and legacy overrides for " + prefix +
            " (use either " + cos_key + "/" + sin_key + " or " + amp_key + "/" + phase_key + ")"
        );
    }

    if (const std::string* mean_val = findOverride(entry, mean_key)) {
        mean = parseDoubleString(*mean_val, "wing '" + wing_label + "' key '" + mean_key + "'");
    }

    if (has_cos) {
        cos_coeff = parseDoubleListString(*findOverride(entry, cos_key),
                                          "wing '" + wing_label + "' key '" + cos_key + "'");
        requireHarmonicLength(cos_coeff, n_harmonics, "wing '" + wing_label + "' key '" + cos_key + "'");
    }
    if (has_sin) {
        sin_coeff = parseDoubleListString(*findOverride(entry, sin_key),
                                          "wing '" + wing_label + "' key '" + sin_key + "'");
        requireHarmonicLength(sin_coeff, n_harmonics, "wing '" + wing_label + "' key '" + sin_key + "'");
    }

    if (!has_amp && !has_phase) {
        return;
    }

    if (prefix == "phi" && has_phase) {
        throw std::runtime_error(
            "Wing '" + wing_label + "': phi_phase is not supported; use phi_cos/phi_sin for phased phi"
        );
    }

    const double c1 = firstOrZero(cos_coeff);
    const double s1 = firstOrZero(sin_coeff);
    double amp = std::hypot(c1, s1);
    double phase = std::atan2(-s1, c1);

    if (has_amp) {
        amp = parseDoubleString(*findOverride(entry, amp_key),
                                "wing '" + wing_label + "' key '" + amp_key + "'");
    }
    if (has_phase) {
        phase = parseDoubleString(*findOverride(entry, phase_key),
                                  "wing '" + wing_label + "' key '" + phase_key + "'");
    }

    if (cos_coeff.empty()) cos_coeff = zeroHarmonics(n_harmonics);
    if (sin_coeff.empty()) sin_coeff = zeroHarmonics(n_harmonics);

    if (prefix == "phi") {
        cos_coeff[0] = amp;
        sin_coeff[0] = 0.0;
    } else {
        cos_coeff[0] = amp * std::cos(phase);
        sin_coeff[0] = -amp * std::sin(phase);
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
        out.hasCustomMotion = false;
        return;
    }

    std::string wing_label = entry.name + "_" + entry.side;
    if (const std::string* omega_override = findOverride(entry, "omega")) {
        out.omega = parseDoubleString(*omega_override, "wing '" + wing_label + "' key 'omega'");
    }

    applyWingAngleOverrides(entry, wing_label, "gamma", n_harmonics,
                            out.gamma_mean, out.gamma_cos, out.gamma_sin);
    applyWingAngleOverrides(entry, wing_label, "phi", n_harmonics,
                            out.phi_mean, out.phi_cos, out.phi_sin);
    applyWingAngleOverrides(entry, wing_label, "psi", n_harmonics,
                            out.psi_mean, out.psi_cos, out.psi_sin);

    out.hasCustomMotion = true;
}

bool hasWingMotionSeries(const WingConfig& w) {
    return !w.gamma_cos.empty() && !w.gamma_sin.empty() &&
           !w.phi_cos.empty() && !w.phi_sin.empty() &&
           !w.psi_cos.empty() && !w.psi_sin.empty();
}

}  // namespace

SimKinematicParams readKinematicParams(const Config& cfg) {
    SimKinematicParams kin;
    kin.omega = cfg.getDouble("omega");
    kin.n_harmonics = cfg.getInt("n_harmonics", 1);
    if (kin.n_harmonics <= 0) {
        throw std::runtime_error("n_harmonics must be >= 1");
    }

    loadAngleSeries(
        cfg, "gamma", kin.n_harmonics, cfg.getDouble("gamma_mean", 0.0), false,
        [&](std::vector<double>& cos_coeff, std::vector<double>& sin_coeff) {
            const double amp = cfg.getDouble("gamma_amp", 0.0);
            const double phase = cfg.getDouble("gamma_phase", 0.0);
            cos_coeff[0] = amp * std::cos(phase);
            sin_coeff[0] = -amp * std::sin(phase);
        },
        kin.gamma_mean, kin.gamma_cos, kin.gamma_sin
    );

    loadAngleSeries(
        cfg, "phi", kin.n_harmonics, cfg.getDouble("phi_mean", 0.0), false,
        [&](std::vector<double>& cos_coeff, std::vector<double>& sin_coeff) {
            const double amp = cfg.getDouble("phi_amp", 0.0);
            cos_coeff[0] = amp;
            sin_coeff[0] = 0.0;
        },
        kin.phi_mean, kin.phi_cos, kin.phi_sin
    );

    loadAngleSeries(
        cfg, "psi", kin.n_harmonics, cfg.getDouble("psi_mean", 0.0), false,
        [&](std::vector<double>& cos_coeff, std::vector<double>& sin_coeff) {
            const double amp = cfg.getDouble("psi_amp", 0.0);
            const double phase = cfg.getDouble("psi_phase", 0.0);
            cos_coeff[0] = amp * std::cos(phase);
            sin_coeff[0] = -amp * std::sin(phase);
        },
        kin.psi_mean, kin.psi_cos, kin.psi_sin
    );

    setLegacyAliases(kin);
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
    for (const auto& w : wc) {
        HarmonicSeries gamma{kin.gamma_mean, kin.gamma_cos, kin.gamma_sin};
        HarmonicSeries phi{kin.phi_mean, kin.phi_cos, kin.phi_sin};
        HarmonicSeries psi{kin.psi_mean, kin.psi_cos, kin.psi_sin};
        double omega = kin.omega;

        if (hasWingMotionSeries(w)) {
            gamma = {w.gamma_mean, w.gamma_cos, w.gamma_sin};
            phi = {w.phi_mean, w.phi_cos, w.phi_sin};
            psi = {w.psi_mean, w.psi_cos, w.psi_sin};
            omega = w.omega;
        }

        auto angleFunc = makeAngleFunc(gamma, phi, psi, w.phaseOffset, omega);
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

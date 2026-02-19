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
                     HarmonicSeries& out) {
    const std::string mean_key = prefix + "_mean";
    const std::string cos_key = prefix + "_cos";
    const std::string sin_key = prefix + "_sin";
    out.mean = cfg.getDouble(mean_key, default_mean);
    out.cos_coeff = cfg.getDoubleList(cos_key, zeroHarmonics(n_harmonics));
    out.sin_coeff = cfg.getDoubleList(sin_key, zeroHarmonics(n_harmonics));
    requireHarmonicLength(out.cos_coeff, n_harmonics, cos_key);
    requireHarmonicLength(out.sin_coeff, n_harmonics, sin_key);
}

void applyWingAngleOverrides(
    const WingConfigEntry& entry,
    const std::string& wing_label,
    const std::string& prefix,
    int n_harmonics,
    HarmonicSeries& series
) {
    const std::string mean_key = prefix + "_mean";
    const std::string cos_key = prefix + "_cos";
    const std::string sin_key = prefix + "_sin";

    const bool has_cos = findOverride(entry, cos_key) != nullptr;
    const bool has_sin = findOverride(entry, sin_key) != nullptr;

    if (const std::string* mean_val = findOverride(entry, mean_key)) {
        series.mean = parseutil::parseDoubleStrict(*mean_val, "wing '" + wing_label + "' key '" + mean_key + "'");
    }

    if (has_cos) {
        series.cos_coeff = parseutil::parseDoubleListStrict(*findOverride(entry, cos_key),
                                                     "wing '" + wing_label + "' key '" + cos_key + "'");
        requireHarmonicLength(series.cos_coeff, n_harmonics, "wing '" + wing_label + "' key '" + cos_key + "'");
    }
    if (has_sin) {
        series.sin_coeff = parseutil::parseDoubleListStrict(*findOverride(entry, sin_key),
                                                     "wing '" + wing_label + "' key '" + sin_key + "'");
        requireHarmonicLength(series.sin_coeff, n_harmonics, "wing '" + wing_label + "' key '" + sin_key + "'");
    }
}

void ensureHarmonicSize(HarmonicSeries& series, int n_harmonics) {
    const auto n = static_cast<size_t>(n_harmonics);
    if (series.cos_coeff.size() != n) series.cos_coeff.resize(n, 0.0);
    if (series.sin_coeff.size() != n) series.sin_coeff.resize(n, 0.0);
}

void populateWingMotion(
    const WingConfigEntry& entry,
    const SimKinematicParams& default_kin,
    WingConfig& out
) {
    const int n_harmonics = default_kin.n_harmonics;

    out.omega = default_kin.omega;
    out.harmonic_period_wingbeats = default_kin.harmonic_period_wingbeats;
    out.gamma = default_kin.gamma;
    out.phi = default_kin.phi;
    out.psi = default_kin.psi;

    ensureHarmonicSize(out.gamma, n_harmonics);
    ensureHarmonicSize(out.phi, n_harmonics);
    ensureHarmonicSize(out.psi, n_harmonics);

    if (entry.motion_overrides.empty()) {
        out.has_custom_motion = false;
        return;
    }

    std::string wing_label = entry.name + "_" + entry.side;
    if (const std::string* omega_override = findOverride(entry, "omega")) {
        out.omega = parseutil::parseDoubleStrict(*omega_override, "wing '" + wing_label + "' key 'omega'");
    }
    if (const std::string* period_override = findOverride(entry, "harmonic_period_wingbeats")) {
        out.harmonic_period_wingbeats = parseutil::parseDoubleStrict(
            *period_override,
            "wing '" + wing_label + "' key 'harmonic_period_wingbeats'"
        );
    }
    if (!std::isfinite(out.harmonic_period_wingbeats) || out.harmonic_period_wingbeats <= 0.0) {
        throw std::runtime_error("wing '" + wing_label + "' harmonic_period_wingbeats must be finite and > 0");
    }

    applyWingAngleOverrides(entry, wing_label, "gamma", n_harmonics, out.gamma);
    applyWingAngleOverrides(entry, wing_label, "phi", n_harmonics, out.phi);
    applyWingAngleOverrides(entry, wing_label, "psi", n_harmonics, out.psi);

    out.has_custom_motion = true;
}

}  // namespace

bool hasWingMotionSeries(const WingConfig& w) {
    return w.has_custom_motion;
}

SimKinematicParams readKinematicParams(const Config& cfg) {
    SimKinematicParams kin;
    kin.omega = cfg.getDouble("omega");
    kin.harmonic_period_wingbeats = cfg.getDouble("harmonic_period_wingbeats", 1.0);
    kin.n_harmonics = cfg.getInt("n_harmonics", 1);
    if (kin.n_harmonics <= 0) {
        throw std::runtime_error("n_harmonics must be >= 1");
    }
    if (!std::isfinite(kin.harmonic_period_wingbeats) || kin.harmonic_period_wingbeats <= 0.0) {
        throw std::runtime_error("harmonic_period_wingbeats must be finite and > 0");
    }

    loadAngleSeries(cfg, "gamma", kin.n_harmonics, cfg.getDouble("gamma_mean", 0.0), kin.gamma);
    loadAngleSeries(cfg, "phi", kin.n_harmonics, cfg.getDouble("phi_mean", 0.0), kin.phi);
    loadAngleSeries(cfg, "psi", kin.n_harmonics, cfg.getDouble("psi_mean", 0.0), kin.psi);

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

    const int global_n_blade_elements = cfg.getInt("n_blade_elements", 1);
    if (global_n_blade_elements <= 0) {
        throw std::runtime_error("n_blade_elements must be > 0");
    }

    std::vector<WingConfig> wingConfigs;
    for (const auto& entry : cfg.getWingEntries()) {
        WingSide side = parseSide(entry.side);
        const int n_blade_elements =
            (entry.n_blade_elements > 0) ? entry.n_blade_elements : global_n_blade_elements;
        const bool has_twist = entry.has_psi_twist_h1_root_deg;
        const double twist_root_rad = has_twist ? (entry.psi_twist_h1_root_deg * M_PI / 180.0) : 0.0;
        const double twist_ref_eta = entry.psi_twist_ref_eta;
        if (has_twist && (!std::isfinite(twist_ref_eta) || twist_ref_eta <= 0.0)) {
            throw std::runtime_error(
                "wing '" + entry.name + "_" + entry.side + "' psi_twist_ref_eta must be finite and > 0"
            );
        }
        WingConfig config(entry.name, side, entry.mu0, entry.lb0,
                          entry.Cd0, entry.Cl0, entry.phase, entry.cone, n_blade_elements,
                          has_twist, twist_root_rad, twist_ref_eta);
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
        const MotionParams& motion = hasWingMotionSeries(w)
            ? static_cast<const MotionParams&>(w) : static_cast<const MotionParams&>(kin);

        auto angleFunc = makeAngleFunc(
            motion.gamma, motion.phi, motion.psi,
            w.phase_offset, motion.omega, motion.harmonic_period_wingbeats
        );
        PitchTwistH1Model twist;
        if (w.has_psi_twist_h1) {
            if (motion.psi.cos_coeff.empty() || motion.psi.sin_coeff.empty()) {
                throw std::runtime_error(
                    "wing '" + w.name + "' pitch twist requires at least 1 psi harmonic coefficient"
                );
            }
            twist.enabled = true;
            twist.root_coeff = w.psi_twist_h1_root;
            twist.ref_eta = w.psi_twist_ref_eta;
            twist.c1 = motion.psi.cos_coeff[0];
            twist.s1 = motion.psi.sin_coeff[0];
            twist.basis_omega = motion.omega / motion.harmonic_period_wingbeats;
            twist.phase_offset = w.phase_offset;
        }
        wings.emplace_back(
            w.name, w.mu0, w.lb0, w.side, w.Cd0, w.Cl0, w.cone_angle,
            std::move(angleFunc), w.n_blade_elements, twist
        );
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

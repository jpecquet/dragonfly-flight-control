#include "sim_setup.hpp"
#include "parse_utils.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <stdexcept>

namespace {

enum class CoeffSet {
    Custom,
    Wang2004,
    Azuma1988,
    Azuma1985
};

std::vector<double> zeroHarmonics(int n) {
    return std::vector<double>(static_cast<size_t>(n), 0.0);
}

std::string wingLabel(const WingConfigEntry& entry) {
    return entry.name + "_" + entry.side;
}

CoeffSet parseCoeffSet(const std::string& value, const std::string& key,
                       const std::string& wing_label) {
    if (value == "custom") return CoeffSet::Custom;
    if (value == "wang2004") return CoeffSet::Wang2004;
    if (value == "azuma1988") return CoeffSet::Azuma1988;
    if (value == "azuma1985") return CoeffSet::Azuma1985;
    throw std::runtime_error(
        "wing '" + wing_label + "' key '" + key +
        "' must be 'custom', 'wang2004', 'azuma1988', or 'azuma1985' (got '" + value + "')"
    );
}

DragCoefficientModel parseDragModel(const std::string& value, const std::string& wing_label) {
    if (value == "sinusoidal") return DragCoefficientModel::Sinusoidal;
    if (value == "piecewise_linear") return DragCoefficientModel::PiecewiseLinear;
    throw std::runtime_error(
        "wing '" + wing_label + "' key 'drag_model' must be 'sinusoidal' or 'piecewise_linear' (got '" + value + "')"
    );
}

LiftCoefficientModel parseLiftModel(const std::string& value, const std::string& wing_label) {
    if (value == "sinusoidal") return LiftCoefficientModel::Sinusoidal;
    if (value == "linear") return LiftCoefficientModel::Linear;
    if (value == "piecewise_linear") return LiftCoefficientModel::PiecewiseLinear;
    throw std::runtime_error(
        "wing '" + wing_label + "' key 'lift_model' must be 'sinusoidal', 'linear', or 'piecewise_linear' (got '" + value + "')"
    );
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
    const std::string amp_key = prefix + "_amp";
    const std::string phase_key = prefix + "_phase";

    out.mean = cfg.getDouble(mean_key, default_mean);
    out.amplitude_coeff = cfg.getDoubleList(amp_key, zeroHarmonics(n_harmonics));
    out.phase_coeff = cfg.getDoubleList(phase_key, zeroHarmonics(n_harmonics));
    requireHarmonicLength(out.amplitude_coeff, n_harmonics, amp_key);
    requireHarmonicLength(out.phase_coeff, n_harmonics, phase_key);
}

void applyWingAngleOverrides(
    const WingConfigEntry& entry,
    const std::string& wing_label,
    const std::string& prefix,
    int n_harmonics,
    HarmonicSeries& series
) {
    const std::string mean_key = prefix + "_mean";
    const std::string amp_key = prefix + "_amp";
    const std::string phase_key = prefix + "_phase";

    const std::string* mean_val = findOverride(entry, mean_key);
    const std::string* amp_val = findOverride(entry, amp_key);
    const std::string* phase_val = findOverride(entry, phase_key);

    if (mean_val != nullptr) {
        series.mean = parseutil::parseDoubleStrict(*mean_val, "wing '" + wing_label + "' key '" + mean_key + "'");
    }

    if (amp_val != nullptr) {
        series.amplitude_coeff = parseutil::parseDoubleListStrict(
            *amp_val,
            "wing '" + wing_label + "' key '" + amp_key + "'"
        );
        requireHarmonicLength(series.amplitude_coeff, n_harmonics, "wing '" + wing_label + "' key '" + amp_key + "'");
    }
    if (phase_val != nullptr) {
        series.phase_coeff = parseutil::parseDoubleListStrict(
            *phase_val,
            "wing '" + wing_label + "' key '" + phase_key + "'"
        );
        requireHarmonicLength(series.phase_coeff, n_harmonics, "wing '" + wing_label + "' key '" + phase_key + "'");
    }
}

void ensureHarmonicSize(HarmonicSeries& series, int n_harmonics) {
    const auto n = static_cast<size_t>(n_harmonics);
    if (series.amplitude_coeff.size() != n) series.amplitude_coeff.resize(n, 0.0);
    if (series.phase_coeff.size() != n) series.phase_coeff.resize(n, 0.0);
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

    std::string wing_label = wingLabel(entry);
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

WingConfig buildWingConfigFromEntry(const WingConfigEntry& entry, int global_n_blade_elements) {
    if (global_n_blade_elements <= 0) {
        throw std::runtime_error("n_blade_elements must be > 0");
    }

    const std::string label = wingLabel(entry);
    const WingSide side = parseSide(entry.side);
    const int n_blade_elements =
        (entry.n_blade_elements > 0) ? entry.n_blade_elements : global_n_blade_elements;
    DragCoefficientModel drag_model = parseDragModel(entry.drag_model, label);
    LiftCoefficientModel lift_model = parseLiftModel(entry.lift_model, label);
    const CoeffSet drag_coeff_set = parseCoeffSet(entry.drag_coeff_set, "drag_coeff_set", label);
    const CoeffSet lift_coeff_set = parseCoeffSet(entry.lift_coeff_set, "lift_coeff_set", label);

    // Azuma1985 coeff sets override the model to PiecewiseLinear
    if (drag_coeff_set == CoeffSet::Azuma1985) {
        if (entry.has_Cd_min || entry.has_Cd_max || entry.has_Cd_alpha_neutral) {
            throw std::runtime_error(
                "wing '" + label + "' does not allow Cd parameters when drag_coeff_set=azuma1985"
            );
        }
        drag_model = DragCoefficientModel::PiecewiseLinear;
    }
    if (lift_coeff_set == CoeffSet::Azuma1985) {
        if (entry.has_Cl0 || entry.has_Cl_alpha_slope || entry.has_Cl_alpha_neutral ||
            entry.has_Cl_min || entry.has_Cl_max) {
            throw std::runtime_error(
                "wing '" + label + "' does not allow Cl parameters when lift_coeff_set=azuma1985"
            );
        }
        lift_model = LiftCoefficientModel::PiecewiseLinear;
    }

    double Cd_min = 0.0;
    double Cd_max = 0.0;
    double Cd_alpha_neutral = 0.0;
    if (drag_model == DragCoefficientModel::Sinusoidal) {
        if (drag_coeff_set == CoeffSet::Wang2004) {
            if (entry.has_Cd_min || entry.has_Cd_max) {
                throw std::runtime_error(
                    "wing '" + label + "' does not allow Cd_min/Cd_max when drag_coeff_set=wang2004"
                );
            }
            if (entry.has_Cd_alpha_neutral && std::abs(entry.Cd_alpha_neutral) > 1e-12) {
                throw std::runtime_error(
                    "wing '" + label + "' requires Cd_alpha_neutral=0 when drag_coeff_set=wang2004"
                );
            }
            Cd_min = aero_defaults::kWang2004CdMin;
            Cd_max = aero_defaults::kWang2004CdMin + 2.0;
            Cd_alpha_neutral = 0.0;
        } else if (drag_coeff_set == CoeffSet::Custom) {
            if (!entry.has_Cd_min) {
                throw std::runtime_error(
                    "wing '" + label + "' requires Cd_min (or legacy Cd0) when drag_coeff_set=custom"
                );
            }
            Cd_min = entry.Cd_min;
            Cd_max = entry.has_Cd_max ? entry.Cd_max : (Cd_min + 2.0);
            Cd_alpha_neutral = entry.has_Cd_alpha_neutral ? entry.Cd_alpha_neutral : 0.0;
        } else if (drag_coeff_set == CoeffSet::Azuma1988) {
            if (entry.has_Cd_min || entry.has_Cd_max || entry.has_Cd_alpha_neutral) {
                throw std::runtime_error(
                    "wing '" + label + "' does not allow Cd_min/Cd_max/Cd_alpha_neutral when drag_coeff_set=azuma1988"
                );
            }
            Cd_min = 0.07;
            Cd_max = 2.0;
            Cd_alpha_neutral = 7.0 * (M_PI / 180.0);
        } else {
            throw std::runtime_error("wing '" + label + "' unknown drag coefficient set");
        }
    }
    if (drag_model != DragCoefficientModel::PiecewiseLinear) {
        if (!std::isfinite(Cd_min) || !std::isfinite(Cd_max) || !std::isfinite(Cd_alpha_neutral)) {
            throw std::runtime_error(
                "wing '" + label + "' requires finite Cd_min, Cd_max, and Cd_alpha_neutral"
            );
        }
        if (Cd_max < Cd_min) {
            throw std::runtime_error("wing '" + label + "' requires Cd_max >= Cd_min");
        }
    }

    double Cl0 = 0.0;
    double Cl_alpha_slope = 0.0;
    double Cl_alpha_neutral = 0.0;
    double Cl_min = -1.0e12;
    double Cl_max = 1.0e12;
    if (lift_model == LiftCoefficientModel::PiecewiseLinear) {
        // No parameters needed — coefficients are hardcoded in blade_element.cpp
    } else if (lift_model == LiftCoefficientModel::Sinusoidal) {
        if (entry.has_Cl_alpha_slope || entry.has_Cl_min || entry.has_Cl_max) {
            throw std::runtime_error(
                "wing '" + label + "' Cl_alpha_slope/Cl_min/Cl_max are only valid when lift_model=linear"
            );
        }
        if (lift_coeff_set == CoeffSet::Wang2004) {
            if (entry.has_Cl0) {
                throw std::runtime_error("wing '" + label + "' does not allow Cl0 when lift_coeff_set=wang2004");
            }
            if (entry.has_Cl_alpha_neutral && std::abs(entry.Cl_alpha_neutral) > 1e-12) {
                throw std::runtime_error(
                    "wing '" + label + "' requires Cl_alpha_neutral=0 when lift_coeff_set=wang2004"
                );
            }
            Cl0 = aero_defaults::kWang2004Cl0;
            Cl_alpha_neutral = 0.0;
        } else if (lift_coeff_set == CoeffSet::Custom) {
            if (!entry.has_Cl0) {
                throw std::runtime_error("wing '" + label + "' requires Cl0 when lift_coeff_set=custom");
            }
            Cl0 = entry.Cl0;
            Cl_alpha_neutral = entry.has_Cl_alpha_neutral ? entry.Cl_alpha_neutral : 0.0;
        } else {
            throw std::runtime_error(
                "wing '" + label + "' lift_coeff_set=azuma1988 is only supported when lift_model=linear"
            );
        }
        if (!std::isfinite(Cl0) || !std::isfinite(Cl_alpha_neutral)) {
            throw std::runtime_error("wing '" + label + "' requires finite Cl0 and Cl_alpha_neutral");
        }
    } else {
        if (entry.has_Cl0) {
            throw std::runtime_error("wing '" + label + "' does not allow Cl0 when lift_model=linear");
        }
        if (lift_coeff_set == CoeffSet::Wang2004) {
            throw std::runtime_error(
                "wing '" + label + "' lift_coeff_set=wang2004 is not supported when lift_model=linear"
            );
        }
        if (lift_coeff_set == CoeffSet::Azuma1988) {
            if (entry.has_Cl_alpha_slope || entry.has_Cl_alpha_neutral || entry.has_Cl_min || entry.has_Cl_max) {
                throw std::runtime_error(
                    "wing '" + label + "' does not allow linear lift override parameters when lift_coeff_set=azuma1988"
                );
            }
            // (Azuma, 1988) preset:
            //   Cl_min = -1.2, Cl_max = 1.2, alpha_neutral = -7 deg, slope = 0.052 / deg
            Cl_alpha_slope = 0.052 * (180.0 / M_PI);  // convert from per-degree to per-radian
            Cl_alpha_neutral = -7.0 * (M_PI / 180.0);
            Cl_min = -1.2;
            Cl_max = 1.2;
        } else {
            if (!entry.has_Cl_alpha_slope || !entry.has_Cl_alpha_neutral) {
                throw std::runtime_error(
                    "wing '" + label + "' requires Cl_alpha_slope and Cl_alpha_neutral for lift_model=linear"
                );
            }
            Cl_alpha_slope = entry.Cl_alpha_slope;
            Cl_alpha_neutral = entry.Cl_alpha_neutral;
            Cl_min = entry.has_Cl_min ? entry.Cl_min : -1.0e12;
            Cl_max = entry.has_Cl_max ? entry.Cl_max : 1.0e12;
        }
        if (!std::isfinite(Cl_alpha_slope) || !std::isfinite(Cl_alpha_neutral) ||
            !std::isfinite(Cl_min) || !std::isfinite(Cl_max)) {
            throw std::runtime_error("wing '" + label + "' linear lift parameters must be finite");
        }
        if (Cl_min > Cl_max) {
            throw std::runtime_error("wing '" + label + "' requires Cl_min <= Cl_max");
        }
    }

    const bool has_twist = entry.has_psi_twist_h1_root_deg;
    const double twist_root_rad = has_twist ? (entry.psi_twist_h1_root_deg * M_PI / 180.0) : 0.0;
    const double twist_ref_eta = entry.psi_twist_ref_eta;
    if (has_twist && (!std::isfinite(twist_ref_eta) || twist_ref_eta <= 0.0)) {
        throw std::runtime_error(
            "wing '" + label + "' psi_twist_ref_eta must be finite and > 0"
        );
    }

    WingConfig config(entry.name, side, entry.mu0, entry.lb0,
                      Cd_min, Cl0, entry.phase, entry.cone, n_blade_elements,
                      has_twist, twist_root_rad, twist_ref_eta);
    config.drag_model = drag_model;
    config.lift_model = lift_model;
    config.Cd_max = Cd_max;
    config.Cd_alpha_neutral = Cd_alpha_neutral;
    config.Cl_alpha_slope = Cl_alpha_slope;
    config.Cl_alpha_neutral = Cl_alpha_neutral;
    config.Cl_min = Cl_min;
    config.Cl_max = Cl_max;
    return config;
}

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

    std::vector<WingConfig> wingConfigs;
    wingConfigs.reserve(cfg.getWingEntries().size());
    for (const auto& entry : cfg.getWingEntries()) {
        WingConfig config = buildWingConfigFromEntry(entry, global_n_blade_elements);
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
            if (motion.psi.amplitude_coeff.empty() || motion.psi.phase_coeff.empty()) {
                throw std::runtime_error(
                    "wing '" + w.name + "' pitch twist requires at least 1 psi harmonic coefficient"
                );
            }
            twist.enabled = true;
            twist.root_coeff = w.psi_twist_h1_root;
            twist.ref_eta = w.psi_twist_ref_eta;
            const double amp1 = motion.psi.amplitude_coeff[0];
            const double phase1 = motion.psi.phase_coeff[0];
            twist.c1 = amp1 * std::cos(phase1);
            twist.s1 = -amp1 * std::sin(phase1);
            twist.basis_omega = motion.omega / motion.harmonic_period_wingbeats;
            twist.phase_offset = w.phase_offset;
        }
        BladeElementAeroParams aero;
        aero.drag_model = w.drag_model;
        aero.lift_model = w.lift_model;
        aero.Cd_min = w.Cd_min;
        aero.Cd_max = w.Cd_max;
        aero.Cd_alpha_neutral = w.Cd_alpha_neutral;
        aero.Cl0 = w.Cl0;
        aero.Cl_alpha_slope = w.Cl_alpha_slope;
        aero.Cl_alpha_neutral = w.Cl_alpha_neutral;
        aero.Cl_min = w.Cl_min;
        aero.Cl_max = w.Cl_max;
        wings.emplace_back(
            w.name, w.mu0, w.lb0, w.side, aero, w.cone_angle,
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

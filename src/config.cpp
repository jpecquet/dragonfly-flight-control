#include "config.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <fstream>
#include <set>
#include <sstream>
#include <vector>

namespace {

struct WingRequiredFields {
    bool name = false;
    bool side = false;
    bool mu0 = false;
    bool lb0 = false;
};

std::string stripInlineComment(const std::string& s) {
    size_t hash = s.find('#');
    if (hash == std::string::npos) {
        return s;
    }
    return s.substr(0, hash);
}

void validateWingSection(const WingRequiredFields& fields, int section_start_line) {
    std::vector<std::string> missing;
    if (!fields.name) missing.push_back("name");
    if (!fields.side) missing.push_back("side");
    if (!fields.mu0) missing.push_back("mu0");
    if (!fields.lb0) missing.push_back("lb0");

    if (missing.empty()) {
        return;
    }

    std::string msg = "Missing required wing parameter(s) in [[wing]] section starting at line " +
                      std::to_string(section_start_line) + ": ";
    for (size_t i = 0; i < missing.size(); ++i) {
        msg += missing[i];
        if (i + 1 < missing.size()) {
            msg += ", ";
        }
    }
    throw std::runtime_error(msg);
}

bool isWingMotionKey(const std::string& key) {
    static const std::set<std::string> kMotionKeys = {
        "omega", "harmonic_period_wingbeats",
        "gamma_mean", "gamma_amp", "gamma_phase",
        "phi_mean", "phi_amp", "phi_phase", "phi_waveform", "phi_k",
        "psi_mean", "psi_amp", "psi_phase", "psi_waveform", "psi_k",
        "cone_mean", "cone_amp", "cone_phase"
    };
    return kMotionKeys.find(key) != kMotionKeys.end();
}

double parseDoubleAtLine(const std::string& value, const std::string& key, int line_num) {
    try {
        return parseutil::parseDoubleStrict(value, "'" + key + "'");
    } catch (const std::exception&) {
        throw std::runtime_error("Invalid value for '" + key + "' at line " +
                                 std::to_string(line_num) + ": " + value);
    }
}

int parseIntAtLine(const std::string& value, const std::string& key, int line_num) {
    try {
        return parseutil::parseIntStrict(value, "'" + key + "'");
    } catch (const std::exception&) {
        throw std::runtime_error("Invalid value for '" + key + "' at line " +
                                 std::to_string(line_num) + ": " + value);
    }
}

}  // namespace

namespace {

// Convert a vector of doubles to a comma-separated string
std::string joinDoubles(const std::vector<double>& vals) {
    std::ostringstream oss;
    for (size_t i = 0; i < vals.size(); ++i) {
        if (i > 0) oss << ",";
        oss << vals[i];
    }
    return oss.str();
}

// Parse one angle series (mean + harmonics list) from YAML, angles in degrees → radians
// Returns {n_harmonics, mean_rad, amp_csv_rad, phase_csv_rad}
struct AngleSeries { int n; double mean; std::string amp; std::string phase; };

AngleSeries parseAngleSeries(const YAML::Node& node) {
    double mean_deg = node["mean"] ? node["mean"].as<double>() : 0.0;
    double mean_rad = mean_deg * M_PI / 180.0;

    std::vector<double> amps, phases;
    if (node["harmonics"]) {
        for (const auto& h : node["harmonics"]) {
            amps.push_back(h[0].as<double>() * M_PI / 180.0);
            phases.push_back(h[1].as<double>() * M_PI / 180.0);
        }
    }
    if (amps.empty()) {
        amps.push_back(0.0);
        phases.push_back(0.0);
    }
    return {static_cast<int>(amps.size()), mean_rad, joinDoubles(amps), joinDoubles(phases)};
}

}  // end anonymous namespace for YAML helpers

Config Config::loadYAML(const std::string& filename) {
    YAML::Node yaml = YAML::LoadFile(filename);
    Config config;

    // Specimen
    const auto& spec = yaml["specimen"];
    double body_length = spec["body_length"].as<double>();
    double body_mass = spec["body_mass"].as<double>();
    double frequency = spec["frequency"].as<double>();

    // Environment
    double rho_air = 1.225;
    double gravity = 9.81;
    if (yaml["environment"]) {
        if (yaml["environment"]["rho_air"]) rho_air = yaml["environment"]["rho_air"].as<double>();
        if (yaml["environment"]["gravity"]) gravity = yaml["environment"]["gravity"].as<double>();
    }

    // Nondimensional omega
    double omega = 2.0 * M_PI * frequency * std::sqrt(body_length / gravity);
    config.values_["omega"] = std::to_string(omega);

    // Simulation
    if (yaml["simulation"]) {
        const auto& sim = yaml["simulation"];
        if (sim["n_wingbeats"])
            config.values_["n_wingbeats"] = std::to_string(sim["n_wingbeats"].as<int>());
        if (sim["steps_per_wingbeat"])
            config.values_["steps_per_wingbeat"] = std::to_string(sim["steps_per_wingbeat"].as<int>());
        if (sim["n_blade_elements"])
            config.values_["n_blade_elements"] = std::to_string(sim["n_blade_elements"].as<int>());
    }

    // Tether
    if (yaml["tether"]) {
        config.values_["tether"] = yaml["tether"].as<bool>() ? "true" : "false";
    }

    // Initial state (already nondimensional in case files)
    if (yaml["initial_state"]) {
        const auto& ic = yaml["initial_state"];
        for (const auto& k : {"x", "y", "z"}) {
            config.values_[std::string(k) + "0"] =
                std::to_string(ic[k] ? ic[k].as<double>() : 0.0);
        }
        for (const auto& k : {"ux", "uy", "uz"}) {
            config.values_[std::string(k) + "0"] =
                std::to_string(ic[k] ? ic[k].as<double>() : 0.0);
        }
    }

    // Trajectory (string, passed through as-is)
    if (yaml["trajectory"]) {
        config.values_["trajectory"] = yaml["trajectory"].as<std::string>();
    }

    // Output path
    if (yaml["output"]) {
        config.values_["output"] = yaml["output"].as<std::string>();
    }

    // PID gains
    if (yaml["pid"]) {
        const auto& pid = yaml["pid"];
        for (const auto& axis : {"x", "y", "z"}) {
            if (pid[axis]) {
                const auto& g = pid[axis];
                std::string pfx = std::string("pid_") + axis + "_";
                if (g["kp"]) config.values_[pfx + "kp"] = std::to_string(g["kp"].as<double>());
                if (g["ki"]) config.values_[pfx + "ki"] = std::to_string(g["ki"].as<double>());
                if (g["kd"]) config.values_[pfx + "kd"] = std::to_string(g["kd"].as<double>());
                if (g["imax"]) config.values_[pfx + "imax"] = std::to_string(g["imax"].as<double>());
            }
        }
    }

    // Mixing matrix
    if (yaml["mixing"]) {
        const auto& mix = yaml["mixing"];
        for (const auto& param : {"gamma", "psi", "phi"}) {
            if (mix[param]) {
                const auto& m = mix[param];
                std::string pfx = std::string("mix_") + param;
                if (m["x"]) config.values_[pfx + "_x"] = std::to_string(m["x"].as<double>());
                if (m["y"]) config.values_[pfx + "_y"] = std::to_string(m["y"].as<double>());
                if (m["z"]) config.values_[pfx + "_z"] = std::to_string(m["z"].as<double>());
            }
        }
    }

    // Parameter bounds
    if (yaml["bounds"]) {
        const auto& b = yaml["bounds"];
        const std::vector<std::string> bkeys = {
            "gamma_mean_min", "gamma_mean_max",
            "psi_mean_min", "psi_mean_max",
            "phi_amp_min", "phi_amp_max"
        };
        for (const auto& k : bkeys) {
            if (b[k]) config.values_[k] = std::to_string(b[k].as<double>());
        }
    }

    // Wings — determine n_harmonics from first wing's phi harmonics
    int n_harmonics = 1;
    if (yaml["wings"]) {
        const auto& wings_node = yaml["wings"];
        // Use the first wing to determine n_harmonics
        for (const auto& wname_pair : wings_node) {
            const auto& wkin = wname_pair.second["kinematics"];
            if (wkin && wkin["phi"] && wkin["phi"]["harmonics"]) {
                n_harmonics = static_cast<int>(wkin["phi"]["harmonics"].size());
                break;
            }
        }
        config.values_["n_harmonics"] = std::to_string(n_harmonics);

        // Global kinematics defaults (use first wing's kinematics)
        bool global_set = false;
        for (const auto& wname_pair : wings_node) {
            const auto& wkin = wname_pair.second["kinematics"];
            if (wkin && !global_set) {
                auto gamma = parseAngleSeries(wkin["gamma"] ? wkin["gamma"] : YAML::Node());
                auto phi   = parseAngleSeries(wkin["phi"]   ? wkin["phi"]   : YAML::Node());
                auto psi   = parseAngleSeries(wkin["psi"]   ? wkin["psi"]   : YAML::Node());
                config.values_["gamma_mean"] = std::to_string(gamma.mean);
                config.values_["gamma_amp"]  = gamma.amp;
                config.values_["gamma_phase"]= gamma.phase;
                config.values_["phi_mean"]   = std::to_string(phi.mean);
                config.values_["phi_amp"]    = phi.amp;
                config.values_["phi_phase"]  = phi.phase;
                config.values_["psi_mean"]   = std::to_string(psi.mean);
                config.values_["psi_amp"]    = psi.amp;
                config.values_["psi_phase"]  = psi.phase;
                global_set = true;
            }

            // Create a WingConfigEntry for each side (left + right)
            std::string wname = wname_pair.first.as<std::string>();
            const auto& wnode = wname_pair.second;
            double span = wnode["span"].as<double>();
            double area = wnode["area"].as<double>();
            double mu0 = rho_air * area * span / body_mass;
            double lb0 = span / body_length;

            for (const std::string& side : {"left", "right"}) {
                WingConfigEntry entry;
                entry.name = wname;
                entry.side = side;
                entry.mu0 = mu0;
                entry.lb0 = lb0;
                if (wnode["drag_coeff_set"])
                    entry.drag_coeff_set = wnode["drag_coeff_set"].as<std::string>();
                if (wnode["lift_coeff_set"])
                    entry.lift_coeff_set = wnode["lift_coeff_set"].as<std::string>();
                if (wnode["phase"]) entry.phase = wnode["phase"].as<double>() * M_PI / 180.0;
                if (wnode["cone"])  entry.cone  = wnode["cone"].as<double>() * M_PI / 180.0;

                // Per-wing kinematics as motion overrides
                if (wnode["kinematics"]) {
                    const auto& wkin = wnode["kinematics"];
                    auto gamma = parseAngleSeries(wkin["gamma"] ? wkin["gamma"] : YAML::Node());
                    auto phi   = parseAngleSeries(wkin["phi"]   ? wkin["phi"]   : YAML::Node());
                    auto psi   = parseAngleSeries(wkin["psi"]   ? wkin["psi"]   : YAML::Node());
                    entry.motion_overrides["gamma_mean"]  = std::to_string(gamma.mean);
                    entry.motion_overrides["gamma_amp"]   = gamma.amp;
                    entry.motion_overrides["gamma_phase"] = gamma.phase;
                    entry.motion_overrides["phi_mean"]    = std::to_string(phi.mean);
                    entry.motion_overrides["phi_amp"]     = phi.amp;
                    entry.motion_overrides["phi_phase"]   = phi.phase;
                    entry.motion_overrides["psi_mean"]    = std::to_string(psi.mean);
                    entry.motion_overrides["psi_amp"]     = psi.amp;
                    entry.motion_overrides["psi_phase"]   = psi.phase;
                }
                config.wings_.push_back(entry);
            }
        }
    }

    // Pass-through: any top-level scalar not already handled
    const std::set<std::string> handled = {
        "specimen", "environment", "simulation", "tether",
        "initial_state", "trajectory", "output", "pid", "mixing", "bounds", "wings"
    };
    for (const auto& kv : yaml) {
        std::string key = kv.first.as<std::string>();
        if (handled.count(key) == 0 && kv.second.IsScalar() && !config.has(key)) {
            config.values_[key] = kv.second.as<std::string>();
        }
    }

    return config;
}

namespace {
bool hasYamlExtension(const std::string& filename) {
    if (filename.size() < 5) return false;
    return filename.substr(filename.size() - 5) == ".yaml";
}
}  // namespace

std::vector<double> Config::getDoubleList(const std::string& key) const {
    return parseutil::parseDoubleListStrict(getString(key), "'" + key + "'");
}

Config Config::load(const std::string& filename) {
    if (hasYamlExtension(filename)) {
        return Config::loadYAML(filename);
    }
    Config config;
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open config file: " + filename);
    }

    std::string line;
    int line_num = 0;
    bool in_wing_section = false;
    int wing_section_start_line = -1;
    WingConfigEntry current_wing;
    WingRequiredFields wing_fields;
    auto finalizeWingSection = [&]() {
        validateWingSection(wing_fields, wing_section_start_line);
        config.wings_.push_back(current_wing);
    };

    while (std::getline(file, line)) {
        line_num++;

        // Strip inline comments, then skip empty lines
        std::string uncommented = stripInlineComment(line);
        std::string trimmed = parseutil::trimCopy(uncommented);
        if (trimmed.empty()) {
            continue;
        }

        // Check for [[wing]] section marker
        if (trimmed == "[[wing]]") {
            // Save previous wing if we were in a section
            if (in_wing_section) {
                finalizeWingSection();
            }
            // Start new wing section
            in_wing_section = true;
            wing_section_start_line = line_num;
            current_wing = WingConfigEntry();
            wing_fields = WingRequiredFields();
            continue;
        }

        // Find '='
        size_t eq = uncommented.find('=');
        if (eq == std::string::npos) {
            throw std::runtime_error("Invalid config line " + std::to_string(line_num) + ": " + line);
        }

        // Extract key and value
        std::string key = parseutil::trimCopy(uncommented.substr(0, eq));
        std::string value = parseutil::trimCopy(uncommented.substr(eq + 1));

        if (key.empty()) {
            throw std::runtime_error("Empty key at line " + std::to_string(line_num));
        }

        // If in wing section, parse wing-specific keys
        if (in_wing_section) {
            if (key == "name") {
                current_wing.name = value;
                wing_fields.name = true;
            } else if (key == "side") {
                current_wing.side = value;
                wing_fields.side = true;
            } else if (key == "mu0") {
                current_wing.mu0 = parseDoubleAtLine(value, key, line_num);
                wing_fields.mu0 = true;
            } else if (key == "lb0") {
                current_wing.lb0 = parseDoubleAtLine(value, key, line_num);
                wing_fields.lb0 = true;
            } else if (key == "Cd_min" || key == "Cd0") {
                current_wing.Cd_min = parseDoubleAtLine(value, key, line_num);
                current_wing.has_Cd_min = true;
            } else if (key == "Cd_max") {
                current_wing.Cd_max = parseDoubleAtLine(value, key, line_num);
                current_wing.has_Cd_max = true;
            } else if (key == "Cd_alpha_neutral") {
                current_wing.Cd_alpha_neutral = parseDoubleAtLine(value, key, line_num);
                current_wing.has_Cd_alpha_neutral = true;
            } else if (key == "Cl0") {
                current_wing.Cl0 = parseDoubleAtLine(value, key, line_num);
                current_wing.has_Cl0 = true;
            } else if (key == "drag_model") {
                current_wing.drag_model = value;
            } else if (key == "drag_coeff_set") {
                current_wing.drag_coeff_set = value;
            } else if (key == "lift_model") {
                current_wing.lift_model = value;
            } else if (key == "lift_coeff_set") {
                current_wing.lift_coeff_set = value;
            } else if (key == "Cl_alpha_slope") {
                current_wing.Cl_alpha_slope = parseDoubleAtLine(value, key, line_num);
                current_wing.has_Cl_alpha_slope = true;
            } else if (key == "Cl_alpha_neutral") {
                current_wing.Cl_alpha_neutral = parseDoubleAtLine(value, key, line_num);
                current_wing.has_Cl_alpha_neutral = true;
            } else if (key == "Cl_min") {
                current_wing.Cl_min = parseDoubleAtLine(value, key, line_num);
                current_wing.has_Cl_min = true;
            } else if (key == "Cl_max") {
                current_wing.Cl_max = parseDoubleAtLine(value, key, line_num);
                current_wing.has_Cl_max = true;
            } else if (key == "phase") {
                current_wing.phase = parseDoubleAtLine(value, key, line_num);
            } else if (key == "cone") {
                current_wing.cone = parseDoubleAtLine(value, key, line_num);
            } else if (key == "n_blade_elements") {
                current_wing.n_blade_elements = parseIntAtLine(value, key, line_num);
                if (current_wing.n_blade_elements <= 0) {
                    throw std::runtime_error("Invalid value for 'n_blade_elements' at line " +
                                             std::to_string(line_num) + ": must be > 0");
                }
            } else if (key == "psi_twist_h1_root_deg") {
                current_wing.psi_twist_h1_root_deg = parseDoubleAtLine(value, key, line_num);
                current_wing.has_psi_twist_h1_root_deg = true;
            } else if (key == "psi_twist_ref_eta") {
                current_wing.psi_twist_ref_eta = parseDoubleAtLine(value, key, line_num);
            } else if (isWingMotionKey(key)) {
                current_wing.motion_overrides[key] = value;
            } else {
                throw std::runtime_error("Unknown wing parameter '" + key + "' at line " + std::to_string(line_num));
            }
        } else {
            // Global key=value
            config.values_[key] = value;
        }
    }

    // Save last wing if we were in a section
    if (in_wing_section) {
        finalizeWingSection();
    }

    return config;
}

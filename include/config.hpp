#pragma once

#include "parse_utils.hpp"

#include <map>
#include <stdexcept>
#include <string>
#include <vector>

// Wing configuration from config file
struct WingConfigEntry {
    std::string name;
    std::string side;  // "left" or "right"
    double mu0 = 0.0;
    double lb0 = 0.0;
    std::string drag_model = "sinusoidal";
    std::string drag_coeff_set = "custom";  // "custom", "wang2004", or "azuma1988"
    bool has_Cd_min = false;
    double Cd_min = 0.0;  // Sinusoidal drag model minimum coefficient
    bool has_Cd_max = false;
    double Cd_max = 0.0;  // Sinusoidal drag model max coefficient
    bool has_Cd_alpha_neutral = false;
    double Cd_alpha_neutral = 0.0;  // Neutral AoA (rad) for sinusoidal drag
    std::string lift_model = "sinusoidal";
    std::string lift_coeff_set = "custom";  // "custom", "wang2004", or "azuma1988"
    bool has_Cl0 = false;
    double Cl0 = 0.0;  // Sinusoidal lift model coefficient
    bool has_Cl_alpha_slope = false;
    double Cl_alpha_slope = 0.0;
    bool has_Cl_alpha_neutral = false;
    double Cl_alpha_neutral = 0.0;  // Neutral AoA (rad) for lift models
    bool has_Cl_min = false;
    double Cl_min = 0.0;
    bool has_Cl_max = false;
    double Cl_max = 0.0;
    double phase = 0.0;
    double cone = 0.0;  // Beta angle from stroke plane (radians): sets flap-cone half-angle (pi/2 - beta)
    int n_blade_elements = 0;  // Optional per-wing override. 0 means "use global/default"
    bool has_psi_twist_h1_root_deg = false;
    double psi_twist_h1_root_deg = 0.0;  // Optional first-harmonic pitch coefficient at root (degrees)
    double psi_twist_ref_eta = 0.75;     // Normalized span station where input pitch Fourier coefficient is defined
    std::map<std::string, std::string> motion_overrides;  // Optional per-wing kinematic overrides
};

// Simple key=value config file parser with [[wing]] section support
class Config {
public:
    static Config load(const std::string& filename);

    // Check if wing sections were defined
    bool hasWings() const {
        return !wings_.empty();
    }

    // Get wing configurations
    const std::vector<WingConfigEntry>& getWingEntries() const {
        return wings_;
    }

    bool has(const std::string& key) const {
        return values_.find(key) != values_.end();
    }

    std::string getString(const std::string& key) const {
        auto it = values_.find(key);
        if (it == values_.end()) {
            throw std::runtime_error("Missing config key: " + key);
        }
        return it->second;
    }

    std::string getString(const std::string& key, const std::string& default_val) const {
        auto it = values_.find(key);
        return (it != values_.end()) ? it->second : default_val;
    }

    double getDouble(const std::string& key) const {
        return parseutil::parseDoubleStrict(getString(key), "'" + key + "'");
    }

    double getDouble(const std::string& key, double default_val) const {
        return has(key) ? getDouble(key) : default_val;
    }

    std::vector<double> getDoubleList(const std::string& key) const;

    std::vector<double> getDoubleList(const std::string& key,
                                      const std::vector<double>& default_val) const {
        return has(key) ? getDoubleList(key) : default_val;
    }

    int getInt(const std::string& key) const {
        return parseutil::parseIntStrict(getString(key), "'" + key + "'");
    }

    int getInt(const std::string& key, int default_val) const {
        return has(key) ? getInt(key) : default_val;
    }

    bool getBool(const std::string& key, bool default_val) const {
        if (!has(key)) return default_val;
        std::string val = getString(key);
        if (val == "true" || val == "1" || val == "yes") return true;
        if (val == "false" || val == "0" || val == "no") return false;
        throw std::runtime_error("Invalid value for '" + key + "': expected bool, got '" + val + "'");
    }

private:
    std::map<std::string, std::string> values_;
    std::vector<WingConfigEntry> wings_;
};

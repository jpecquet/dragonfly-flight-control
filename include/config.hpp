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
    double Cd0 = 0.0;
    double Cl0 = 0.0;
    double phase = 0.0;
    double cone = 0.0;  // Coning angle (radians): tilts the wing's flapping plane about the stroke direction
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

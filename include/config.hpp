#pragma once

#include <fstream>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

// Wing configuration from config file
struct WingConfigEntry {
    std::string name;
    std::string side;  // "left" or "right"
    double mu0;
    double lb0;
    double Cd0;
    double Cl0;
    double phase;
};

// Simple key=value config file parser with [[wing]] section support
class Config {
public:
    static Config load(const std::string& filename) {
        Config config;
        std::ifstream file(filename);
        if (!file.is_open()) {
            throw std::runtime_error("Cannot open config file: " + filename);
        }

        std::string line;
        int line_num = 0;
        bool in_wing_section = false;
        WingConfigEntry current_wing;

        while (std::getline(file, line)) {
            line_num++;

            // Skip empty lines and comments
            size_t start = line.find_first_not_of(" \t");
            if (start == std::string::npos || line[start] == '#') {
                continue;
            }

            std::string trimmed = trim(line);

            // Check for [[wing]] section marker
            if (trimmed == "[[wing]]") {
                // Save previous wing if we were in a section
                if (in_wing_section) {
                    config.wings_.push_back(current_wing);
                }
                // Start new wing section
                in_wing_section = true;
                current_wing = WingConfigEntry();
                continue;
            }

            // Find '='
            size_t eq = line.find('=');
            if (eq == std::string::npos) {
                throw std::runtime_error("Invalid config line " + std::to_string(line_num) + ": " + line);
            }

            // Extract key and value
            std::string key = trim(line.substr(0, eq));
            std::string value = trim(line.substr(eq + 1));

            if (key.empty()) {
                throw std::runtime_error("Empty key at line " + std::to_string(line_num));
            }

            // If in wing section, parse wing-specific keys
            if (in_wing_section) {
                if (key == "name") {
                    current_wing.name = value;
                } else if (key == "side") {
                    current_wing.side = value;
                } else if (key == "mu0") {
                    current_wing.mu0 = std::stod(value);
                } else if (key == "lb0") {
                    current_wing.lb0 = std::stod(value);
                } else if (key == "Cd0") {
                    current_wing.Cd0 = std::stod(value);
                } else if (key == "Cl0") {
                    current_wing.Cl0 = std::stod(value);
                } else if (key == "phase") {
                    current_wing.phase = std::stod(value);
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
            config.wings_.push_back(current_wing);
        }

        return config;
    }

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
        std::string val = getString(key);
        try {
            return std::stod(val);
        } catch (const std::exception&) {
            throw std::runtime_error("Invalid value for '" + key + "': expected number, got '" + val + "'");
        }
    }

    double getDouble(const std::string& key, double default_val) const {
        if (!has(key)) return default_val;
        std::string val = getString(key);
        try {
            return std::stod(val);
        } catch (const std::exception&) {
            throw std::runtime_error("Invalid value for '" + key + "': expected number, got '" + val + "'");
        }
    }

    int getInt(const std::string& key) const {
        std::string val = getString(key);
        try {
            return std::stoi(val);
        } catch (const std::exception&) {
            throw std::runtime_error("Invalid value for '" + key + "': expected integer, got '" + val + "'");
        }
    }

    int getInt(const std::string& key, int default_val) const {
        if (!has(key)) return default_val;
        std::string val = getString(key);
        try {
            return std::stoi(val);
        } catch (const std::exception&) {
            throw std::runtime_error("Invalid value for '" + key + "': expected integer, got '" + val + "'");
        }
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

    static std::string trim(const std::string& s) {
        size_t start = s.find_first_not_of(" \t");
        if (start == std::string::npos) return "";
        size_t end = s.find_last_not_of(" \t");
        return s.substr(start, end - start + 1);
    }
};

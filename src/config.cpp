#include "config.hpp"

#include <fstream>

static double parseDouble(const std::string& value, const std::string& key, int line_num) {
    try {
        return std::stod(value);
    } catch (const std::exception&) {
        throw std::runtime_error("Invalid value for '" + key + "' at line " +
                                 std::to_string(line_num) + ": " + value);
    }
}

std::string Config::trim(const std::string& s) {
    size_t start = s.find_first_not_of(" \t");
    if (start == std::string::npos) return "";
    size_t end = s.find_last_not_of(" \t");
    return s.substr(start, end - start + 1);
}

Config Config::load(const std::string& filename) {
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
                current_wing.mu0 = parseDouble(value, key, line_num);
            } else if (key == "lb0") {
                current_wing.lb0 = parseDouble(value, key, line_num);
            } else if (key == "Cd0") {
                current_wing.Cd0 = parseDouble(value, key, line_num);
            } else if (key == "Cl0") {
                current_wing.Cl0 = parseDouble(value, key, line_num);
            } else if (key == "phase") {
                current_wing.phase = parseDouble(value, key, line_num);
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

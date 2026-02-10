#include "config.hpp"

#include <fstream>
#include <vector>

static double parseDouble(const std::string& value, const std::string& key, int line_num) {
    try {
        size_t idx = 0;
        double parsed = std::stod(value, &idx);
        if (idx != value.size()) {
            throw std::runtime_error("trailing characters");
        }
        return parsed;
    } catch (const std::exception&) {
        throw std::runtime_error("Invalid value for '" + key + "' at line " +
                                 std::to_string(line_num) + ": " + value);
    }
}

namespace {

struct WingRequiredFields {
    bool name = false;
    bool side = false;
    bool mu0 = false;
    bool lb0 = false;
    bool Cd0 = false;
    bool Cl0 = false;
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
    if (!fields.Cd0) missing.push_back("Cd0");
    if (!fields.Cl0) missing.push_back("Cl0");

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

}  // namespace

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
    int wing_section_start_line = -1;
    WingConfigEntry current_wing;
    WingRequiredFields wing_fields;

    while (std::getline(file, line)) {
        line_num++;

        // Strip inline comments, then skip empty lines
        std::string uncommented = stripInlineComment(line);
        std::string trimmed = trim(uncommented);
        if (trimmed.empty()) {
            continue;
        }

        // Check for [[wing]] section marker
        if (trimmed == "[[wing]]") {
            // Save previous wing if we were in a section
            if (in_wing_section) {
                validateWingSection(wing_fields, wing_section_start_line);
                config.wings_.push_back(current_wing);
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
        std::string key = trim(uncommented.substr(0, eq));
        std::string value = trim(uncommented.substr(eq + 1));

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
                current_wing.mu0 = parseDouble(value, key, line_num);
                wing_fields.mu0 = true;
            } else if (key == "lb0") {
                current_wing.lb0 = parseDouble(value, key, line_num);
                wing_fields.lb0 = true;
            } else if (key == "Cd0") {
                current_wing.Cd0 = parseDouble(value, key, line_num);
                wing_fields.Cd0 = true;
            } else if (key == "Cl0") {
                current_wing.Cl0 = parseDouble(value, key, line_num);
                wing_fields.Cl0 = true;
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
        validateWingSection(wing_fields, wing_section_start_line);
        config.wings_.push_back(current_wing);
    }

    return config;
}

#include "config.hpp"

#include <fstream>
#include <set>
#include <vector>

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

bool isWingMotionKey(const std::string& key) {
    static const std::set<std::string> kMotionKeys = {
        "omega", "harmonic_period_wingbeats",
        "gamma_mean", "gamma_cos", "gamma_sin",
        "phi_mean", "phi_cos", "phi_sin",
        "psi_mean", "psi_cos", "psi_sin"
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

std::vector<double> Config::getDoubleList(const std::string& key) const {
    return parseutil::parseDoubleListStrict(getString(key), "'" + key + "'");
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
            } else if (key == "Cd0") {
                current_wing.Cd0 = parseDoubleAtLine(value, key, line_num);
                wing_fields.Cd0 = true;
            } else if (key == "Cl0") {
                current_wing.Cl0 = parseDoubleAtLine(value, key, line_num);
                wing_fields.Cl0 = true;
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

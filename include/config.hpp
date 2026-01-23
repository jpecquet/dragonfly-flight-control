#pragma once

#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>

// Simple key=value config file parser
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
        while (std::getline(file, line)) {
            line_num++;

            // Skip empty lines and comments
            size_t start = line.find_first_not_of(" \t");
            if (start == std::string::npos || line[start] == '#') {
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

            config.values_[key] = value;
        }

        return config;
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

private:
    std::map<std::string, std::string> values_;

    static std::string trim(const std::string& s) {
        size_t start = s.find_first_not_of(" \t");
        if (start == std::string::npos) return "";
        size_t end = s.find_last_not_of(" \t");
        return s.substr(start, end - start + 1);
    }
};

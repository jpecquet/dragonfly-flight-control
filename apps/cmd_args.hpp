#pragma once

#include "parse_utils.hpp"

#include <string>

namespace cliarg {

inline bool isHelpFlag(const std::string& arg) {
    return arg == "-h" || arg == "--help";
}

inline std::string requireOptionValue(int argc, char* argv[], int& i, const char* flag) {
    if (i + 1 >= argc) {
        throw std::runtime_error(std::string("Missing value for ") + flag);
    }
    return std::string(argv[++i]);
}

inline double parseDouble(const std::string& raw, const char* flag) {
    return parseutil::parseDoubleStrict(raw, flag);
}

inline int parseInt(const std::string& raw, const char* flag) {
    return parseutil::parseIntStrict(raw, flag);
}

}  // namespace cliarg

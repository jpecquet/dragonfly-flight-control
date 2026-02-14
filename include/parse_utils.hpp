#pragma once

#include <cctype>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace parseutil {

inline std::string trimCopy(std::string_view s) {
    size_t start = 0;
    while (start < s.size() &&
           std::isspace(static_cast<unsigned char>(s[start])) != 0) {
        ++start;
    }
    size_t end = s.size();
    while (end > start &&
           std::isspace(static_cast<unsigned char>(s[end - 1])) != 0) {
        --end;
    }
    return std::string(s.substr(start, end - start));
}

inline double parseDoubleStrict(const std::string& raw_value, const std::string& context) {
    const std::string value = trimCopy(raw_value);
    if (value.empty()) {
        throw std::runtime_error("Invalid value for " + context +
                                 ": expected number, got empty");
    }

    try {
        size_t idx = 0;
        const double parsed = std::stod(value, &idx);
        if (idx != value.size()) {
            throw std::runtime_error("trailing characters");
        }
        return parsed;
    } catch (const std::exception&) {
        throw std::runtime_error("Invalid value for " + context +
                                 ": expected number, got '" + raw_value + "'");
    }
}

inline int parseIntStrict(const std::string& raw_value, const std::string& context) {
    const std::string value = trimCopy(raw_value);
    if (value.empty()) {
        throw std::runtime_error("Invalid value for " + context +
                                 ": expected integer, got empty");
    }

    try {
        size_t idx = 0;
        const int parsed = std::stoi(value, &idx);
        if (idx != value.size()) {
            throw std::runtime_error("trailing characters");
        }
        return parsed;
    } catch (const std::exception&) {
        throw std::runtime_error("Invalid value for " + context +
                                 ": expected integer, got '" + raw_value + "'");
    }
}

inline std::vector<double> parseDoubleListStrict(const std::string& raw_value,
                                                 const std::string& context) {
    std::string value = trimCopy(raw_value);
    if (value.size() >= 2 && value.front() == '[' && value.back() == ']') {
        value = trimCopy(std::string_view(value).substr(1, value.size() - 2));
    }

    if (value.empty()) {
        throw std::runtime_error("Invalid value for " + context +
                                 ": expected comma-separated numbers, got empty");
    }

    std::vector<double> values;
    std::stringstream ss(value);
    std::string token;
    int token_index = 0;
    while (std::getline(ss, token, ',')) {
        token = trimCopy(token);
        if (token.empty()) {
            throw std::runtime_error("Invalid value for " + context +
                                     ": empty token at position " +
                                     std::to_string(token_index));
        }
        try {
            size_t idx = 0;
            const double parsed = std::stod(token, &idx);
            if (idx != token.size()) {
                throw std::runtime_error("trailing characters");
            }
            values.push_back(parsed);
        } catch (const std::exception&) {
            throw std::runtime_error("Invalid value for " + context +
                                     ": expected number in token '" + token + "'");
        }
        ++token_index;
    }

    if (values.empty()) {
        throw std::runtime_error("Invalid value for " + context + ": no values parsed");
    }
    return values;
}

}  // namespace parseutil

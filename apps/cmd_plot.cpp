#include "cmd_plot.hpp"

#include <cstdlib>
#include <iostream>
#include <regex>
#include <string>

static bool isSafePath(const std::string& s) {
    static const std::regex safe_re("^[a-zA-Z0-9._/\\-]+$");
    return std::regex_match(s, safe_re);
}

int runPlot(const Config& cfg) {
    std::string input_file = cfg.getString("input");
    std::string output_file = cfg.getString("output");

    if (!isSafePath(input_file) || !isSafePath(output_file)) {
        std::cerr << "Invalid characters in file path" << std::endl;
        return 1;
    }

    std::string cmd = "python3 -m post.plot_simulation " + input_file + " " + output_file;
    std::cout << "Running: " << cmd << std::endl;

    int ret = std::system(cmd.c_str());
    if (ret != 0) {
        std::cerr << "Plot command failed with exit code " << ret << std::endl;
        return 1;
    }

    std::cout << "Plot written to " << output_file << std::endl;
    return 0;
}

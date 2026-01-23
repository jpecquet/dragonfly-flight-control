#include "cmd_plot.hpp"

#include <cstdlib>
#include <iostream>
#include <string>

int runPlot(const Config& cfg) {
    std::string input_file = cfg.getString("input");
    std::string output_file = cfg.getString("output");
    std::string script_path = cfg.getString("script", "post/main.py");

    std::string cmd = "python3 " + script_path + " " + input_file + " " + output_file;
    std::cout << "Running: " << cmd << std::endl;

    int ret = std::system(cmd.c_str());
    if (ret != 0) {
        std::cerr << "Plot command failed with exit code " << ret << std::endl;
        return 1;
    }

    std::cout << "Plot written to " << output_file << std::endl;
    return 0;
}

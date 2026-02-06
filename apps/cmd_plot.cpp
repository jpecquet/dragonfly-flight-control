#include "cmd_plot.hpp"

#include <iostream>
#include <string>
#include <spawn.h>
#include <sys/wait.h>

extern char** environ;

int runPlot(const Config& cfg) {
    std::string input_file = cfg.getString("input");
    std::string output_file = cfg.getString("output");

    std::cout << "Running: python3 -m post.plot_simulation "
              << input_file << " " << output_file << std::endl;

    const char* argv[] = {
        "python3", "-m", "post.plot_simulation",
        input_file.c_str(), output_file.c_str(), nullptr
    };

    pid_t pid;
    int status = posix_spawn(&pid, "/usr/bin/env", nullptr, nullptr,
                             const_cast<char* const*>(argv), environ);
    if (status != 0) {
        std::cerr << "Failed to spawn python3: " << strerror(status) << std::endl;
        return 1;
    }

    if (waitpid(pid, &status, 0) == -1) {
        std::cerr << "waitpid failed" << std::endl;
        return 1;
    }

    if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
        int code = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
        std::cerr << "Plot command failed with exit code " << code << std::endl;
        return 1;
    }

    std::cout << "Plot written to " << output_file << std::endl;
    return 0;
}

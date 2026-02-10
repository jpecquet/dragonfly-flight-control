#include "cmd_plot.hpp"

#include <iostream>
#include <sstream>
#include <string>
#include <spawn.h>
#include <sys/wait.h>
#include <vector>

extern char** environ;

int runPlot(const Config& cfg) {
    std::string input_file = cfg.getString("input");
    std::string output_file = cfg.getString("output");
    bool no_blender = cfg.getBool("no_blender", false);

    std::vector<std::string> args = {
        "python3", "-m", "post.plot_simulation", input_file, output_file
    };
    if (no_blender) {
        args.push_back("--no-blender");
    }

    std::ostringstream cmd;
    for (size_t i = 0; i < args.size(); ++i) {
        if (i > 0) cmd << " ";
        cmd << args[i];
    }
    std::cout << "Running: " << cmd.str() << std::endl;

    std::vector<char*> argv;
    argv.reserve(args.size() + 1);
    for (auto& arg : args) {
        argv.push_back(const_cast<char*>(arg.c_str()));
    }
    argv.push_back(nullptr);

    pid_t pid;
    int status = posix_spawnp(&pid, "python3", nullptr, nullptr,
                              argv.data(), environ);
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

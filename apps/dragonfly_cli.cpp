#include "cmd_args.hpp"
#include "cmd_optim.hpp"
#include "cmd_plot.hpp"
#include "cmd_sim.hpp"
#include "cmd_termvel.hpp"
#include "cmd_track.hpp"
#include "cmd_wingtest.hpp"
#include "config.hpp"

#include <iostream>
#include <string>

namespace {

enum class ConfigArgStatus { Ok, Help, Error };

ConfigArgStatus parseConfigPath(int argc, char* argv[],
                                std::string& config_path, std::string& error) {
    config_path.clear();
    error.clear();

    for (int i = 2; i < argc; ++i) {
        const std::string arg = argv[i];
        if (cliarg::isHelpFlag(arg)) {
            return ConfigArgStatus::Help;
        }
        if (arg == "-c" || arg == "--config") {
            if (!config_path.empty()) {
                error = "Config specified more than once.";
                return ConfigArgStatus::Error;
            }
            if (i + 1 >= argc) {
                error = "Missing value after " + arg;
                return ConfigArgStatus::Error;
            }
            config_path = argv[++i];
            continue;
        }
        error = "Unknown option: " + arg;
        return ConfigArgStatus::Error;
    }

    if (config_path.empty()) {
        error = "Missing required config path (-c <config> or --config <config>).";
        return ConfigArgStatus::Error;
    }

    return ConfigArgStatus::Ok;
}

}  // namespace

void printUsage(const char* prog) {
    std::cerr << "Usage: " << prog << " <command> [options]\n";
    std::cerr << "\n";
    std::cerr << "Commands:\n";
    std::cerr << "  sim      Run flight simulation (-c/--config <config>)\n";
    std::cerr << "  track    Run trajectory tracking simulation (-c/--config <config>)\n";
    std::cerr << "  optim    Find equilibrium flight conditions (-c/--config <config>)\n";
    std::cerr << "  plot     Generate visualization (-c/--config <config>)\n";
    std::cerr << "  wingtest Generate wing rotation test data (see options below)\n";
    std::cerr << "  termvel  Generate terminal velocity data (see options below)\n";
    std::cerr << "\n";
    std::cerr << "Wingtest options:\n";
    std::cerr << "  --gam START:END    Stroke plane angle range (default: 0:90)\n";
    std::cerr << "  --phi START:END    Flapping angle range (default: 0:25)\n";
    std::cerr << "  --psi START:END    Pitch angle range (default: 0:45)\n";
    std::cerr << "  --seq A,B,C        Sweep order using gam/phi/psi (default: gam,phi,psi)\n";
    std::cerr << "  --frames N         Frames per phase (default: 50)\n";
    std::cerr << "  --right            Use right wing instead of left\n";
    std::cerr << "  -o <file>          Output file (default: wingtest.h5)\n";
    std::cerr << "\n";
    std::cerr << "Termvel options:\n";
    std::cerr << "  --psi DEG          Wing pitch angle in degrees (default: 0)\n";
    std::cerr << "  --dt VALUE         Time step (default: 0.01)\n";
    std::cerr << "  --tmax VALUE       Maximum simulation time (default: 50.0)\n";
    std::cerr << "  -o <file>          Output file (default: termvel.h5)\n";
    std::cerr << "\n";
    std::cerr << "Examples:\n";
    std::cerr << "  " << prog << " sim -c configs/sim_hover.cfg\n";
    std::cerr << "  " << prog << " optim -c configs/optim.cfg\n";
    std::cerr << "  " << prog << " wingtest --gam 0:90 --phi 0:25 -o out.h5\n";
    std::cerr << "  " << prog << " termvel --psi 0 -o termvel.h5\n";
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        printUsage(argv[0]);
        return 1;
    }

    std::string command = argv[1];
    if (cliarg::isHelpFlag(command)) {
        printUsage(argv[0]);
        return 0;
    }

    try {
        // Commands with their own argument parsing
        if (command == "wingtest") {
            return runWingtest(argc, argv);
        }
        if (command == "termvel") {
            return runTermvel(argc, argv);
        }

        // Remaining commands require -c/--config <config>
        std::string config_file;
        std::string config_error;
        ConfigArgStatus config_status = parseConfigPath(argc, argv, config_file, config_error);
        if (config_status == ConfigArgStatus::Help) {
            printUsage(argv[0]);
            return 0;
        }
        if (config_status == ConfigArgStatus::Error) {
            std::cerr << "Error: " << config_error << "\n";
            printUsage(argv[0]);
            return 1;
        }

        Config cfg = Config::load(config_file);

        if (command == "sim") {
            return runSim(cfg);
        } else if (command == "track") {
            return runTrack(cfg);
        } else if (command == "optim") {
            return runOptim(cfg);
        } else if (command == "plot") {
            return runPlot(cfg);
        } else {
            std::cerr << "Unknown command: " << command << std::endl;
            printUsage(argv[0]);
            return 1;
        }
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}

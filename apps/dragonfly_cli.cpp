#include "cmd_optim.hpp"
#include "cmd_plot.hpp"
#include "cmd_sim.hpp"
#include "cmd_termvel.hpp"
#include "cmd_track.hpp"
#include "cmd_wingtest.hpp"
#include "config.hpp"

#include <iostream>
#include <string>

void printUsage(const char* prog) {
    std::cerr << "Usage: " << prog << " <command> [options]\n";
    std::cerr << "\n";
    std::cerr << "Commands:\n";
    std::cerr << "  sim      Run flight simulation (-c <config>)\n";
    std::cerr << "  track    Run trajectory tracking simulation (-c <config>)\n";
    std::cerr << "  optim    Find equilibrium flight conditions (-c <config>)\n";
    std::cerr << "  plot     Generate visualization (-c <config>)\n";
    std::cerr << "  wingtest Generate wing rotation test data (see options below)\n";
    std::cerr << "  termvel  Generate terminal velocity data (see options below)\n";
    std::cerr << "\n";
    std::cerr << "Wingtest options:\n";
    std::cerr << "  --gam START:END    Stroke plane angle range (default: 0:90)\n";
    std::cerr << "  --phi START:END    Flapping angle range (default: 0:25)\n";
    std::cerr << "  --psi START:END    Pitch angle range (default: 0:45)\n";
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

    try {
        // Commands with their own argument parsing
        if (command == "wingtest") {
            return runWingtest(argc, argv);
        }
        if (command == "termvel") {
            return runTermvel(argc, argv);
        }

        // Remaining commands require -c <config>
        if (argc < 4) {
            printUsage(argv[0]);
            return 1;
        }

        std::string config_flag = argv[2];
        std::string config_file = argv[3];

        if (config_flag != "-c") {
            std::cerr << "Error: expected -c <config>\n";
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

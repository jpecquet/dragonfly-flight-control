#include "cmd_termvel.hpp"
#include "eom.hpp"
#include "integrator.hpp"
#include "terminal_velocity.hpp"
#include "wing.hpp"

#include <highfive/H5Easy.hpp>
#include <highfive/H5File.hpp>

#include <cmath>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

namespace {

void printUsage(const char* prog) {
    std::cerr << "Usage: " << prog << " termvel [options]\n";
    std::cerr << "\n";
    std::cerr << "Generate terminal velocity simulation data for visualization.\n";
    std::cerr << "\n";
    std::cerr << "Options:\n";
    std::cerr << "  --psi DEG      Wing pitch angle in degrees (default: 0)\n";
    std::cerr << "  --dt VALUE     Time step (default: 0.01)\n";
    std::cerr << "  --tmax VALUE   Maximum simulation time (default: 50.0)\n";
    std::cerr << "  -o FILE        Output HDF5 file (default: termvel.h5)\n";
    std::cerr << "\n";
    std::cerr << "Examples:\n";
    std::cerr << "  " << prog << " termvel --psi 0 -o horizontal.h5\n";
    std::cerr << "  " << prog << " termvel --psi 90 -o vertical.h5\n";
}

} // namespace

int runTermvel(int argc, char* argv[]) {
    // Default parameters
    double psi_deg = 0.0;
    double dt = 0.01;
    double t_max = 50.0;
    std::string output_file = "termvel.h5";

    // Parse arguments
    for (int i = 2; i < argc; ++i) {
        if (std::strcmp(argv[i], "--psi") == 0 && i + 1 < argc) {
            try {
                psi_deg = std::stod(argv[++i]);
            } catch (const std::exception&) {
                std::cerr << "Invalid value for --psi: " << argv[i] << "\n";
                return 1;
            }
        } else if (std::strcmp(argv[i], "--dt") == 0 && i + 1 < argc) {
            try {
                dt = std::stod(argv[++i]);
            } catch (const std::exception&) {
                std::cerr << "Invalid value for --dt: " << argv[i] << "\n";
                return 1;
            }
        } else if (std::strcmp(argv[i], "--tmax") == 0 && i + 1 < argc) {
            try {
                t_max = std::stod(argv[++i]);
            } catch (const std::exception&) {
                std::cerr << "Invalid value for --tmax: " << argv[i] << "\n";
                return 1;
            }
        } else if (std::strcmp(argv[i], "-o") == 0 && i + 1 < argc) {
            output_file = argv[++i];
        } else if (std::strcmp(argv[i], "-h") == 0 || std::strcmp(argv[i], "--help") == 0) {
            printUsage(argv[0]);
            return 0;
        } else {
            std::cerr << "Unknown option: " << argv[i] << "\n";
            printUsage(argv[0]);
            return 1;
        }
    }

    double psi = psi_deg * M_PI / 180.0;

    // Wing parameters
    double mu0 = 0.1;
    double lb0 = 1.0;
    double Cd0 = 0.4;
    double Cl0 = 1.2;

    std::cout << "Terminal Velocity Simulation\n";
    std::cout << "============================\n";
    std::cout << "  Pitch angle (psi): " << psi_deg << " deg\n";
    std::cout << "  Time step (dt):    " << dt << "\n";
    std::cout << "  Max time (t_max):  " << t_max << "\n";
    std::cout << "  Output file:       " << output_file << "\n";
    std::cout << "\n";

    // Create wing with fixed orientation (gam = 90° so psi = 0 means horizontal chord)
    auto fixedAngles = [psi](double t) -> WingAngles {
        (void)t;
        return {M_PI / 2.0, 0.0, 0.0, 0.0, psi};
    };

    Wing wing("test", mu0, lb0, WingSide::Left, Cd0, Cl0, fixedAngles);
    std::vector<Wing> wings = {wing};

    // Storage for time series
    std::vector<double> time_vec;
    std::vector<double> x_vec;
    std::vector<double> z_vec;
    std::vector<double> ux_vec;
    std::vector<double> uz_vec;
    std::vector<double> lift_x_vec;
    std::vector<double> lift_z_vec;
    std::vector<double> drag_x_vec;
    std::vector<double> drag_z_vec;

    // Estimate capacity
    size_t est_steps = static_cast<size_t>(t_max / dt) + 1;
    time_vec.reserve(est_steps);
    x_vec.reserve(est_steps);
    z_vec.reserve(est_steps);
    ux_vec.reserve(est_steps);
    uz_vec.reserve(est_steps);
    lift_x_vec.reserve(est_steps);
    lift_z_vec.reserve(est_steps);
    drag_x_vec.reserve(est_steps);
    drag_z_vec.reserve(est_steps);

    // Initial state: at rest at origin
    State state(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    double t = 0.0;

    // Scratch buffer for integration
    std::vector<SingleWingVectors> scratch(1);

    // Store initial state (zero forces at rest)
    time_vec.push_back(t);
    x_vec.push_back(state.pos.x());
    z_vec.push_back(state.pos.z());
    ux_vec.push_back(state.vel.x());
    uz_vec.push_back(state.vel.z());
    lift_x_vec.push_back(0.0);
    lift_z_vec.push_back(0.0);
    drag_x_vec.push_back(0.0);
    drag_z_vec.push_back(0.0);

    // Integrate
    std::cout << "Running simulation..." << std::endl;
    while (t < t_max) {
        state = stepRK4(t, dt, state, wings, scratch);
        t += dt;

        // Compute forces at current state
        equationOfMotion(t, state, wings, scratch);

        time_vec.push_back(t);
        x_vec.push_back(state.pos.x());
        z_vec.push_back(state.pos.z());
        ux_vec.push_back(state.vel.x());
        uz_vec.push_back(state.vel.z());
        lift_x_vec.push_back(scratch[0].lift.x());
        lift_z_vec.push_back(scratch[0].lift.z());
        drag_x_vec.push_back(scratch[0].drag.x());
        drag_z_vec.push_back(scratch[0].drag.z());
    }

    // Analytical steady-state speed
    TerminalVelocitySolution analytical = solveTerminalVelocity(psi, mu0, lb0, Cd0, Cl0);
    double speed_analytical = analytical.speed;
    double speed_final = std::sqrt(state.vel.x() * state.vel.x() + state.vel.z() * state.vel.z());
    double glide_angle = std::atan2(state.vel.x(), -state.vel.z()) * 180.0 / M_PI;

    std::cout << "  Final speed:      " << speed_final << "\n";
    std::cout << "  Analytical speed: " << speed_analytical << "\n";
    std::cout << "  Glide angle:      " << glide_angle << " deg from vertical\n";
    std::cout << "\n";

    // Write HDF5
    std::cout << "Writing " << output_file << "..." << std::endl;

    HighFive::File file(output_file, HighFive::File::Overwrite);

    // Parameters
    file.createGroup("/parameters");
    H5Easy::dump(file, "/parameters/psi", psi);
    H5Easy::dump(file, "/parameters/psi_deg", psi_deg);
    H5Easy::dump(file, "/parameters/mu0", mu0);
    H5Easy::dump(file, "/parameters/lb0", lb0);
    H5Easy::dump(file, "/parameters/Cd0", Cd0);
    H5Easy::dump(file, "/parameters/Cl0", Cl0);
    H5Easy::dump(file, "/parameters/dt", dt);
    H5Easy::dump(file, "/parameters/t_max", t_max);
    H5Easy::dump(file, "/parameters/speed_analytical", speed_analytical);

    // Time series data
    file.createDataSet("/time", time_vec);
    file.createDataSet("/x", x_vec);
    file.createDataSet("/z", z_vec);
    file.createDataSet("/ux", ux_vec);
    file.createDataSet("/uz", uz_vec);
    file.createDataSet("/lift_x", lift_x_vec);
    file.createDataSet("/lift_z", lift_z_vec);
    file.createDataSet("/drag_x", drag_x_vec);
    file.createDataSet("/drag_z", drag_z_vec);

    std::cout << "Done. " << time_vec.size() << " timesteps written.\n";

    return 0;
}

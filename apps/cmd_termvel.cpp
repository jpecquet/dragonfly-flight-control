#include "cmd_termvel.hpp"
#include "cmd_args.hpp"
#include "eom.hpp"
#include "integrator.hpp"
#include "terminal_velocity.hpp"
#include "wing.hpp"

#include <highfive/H5Easy.hpp>
#include <highfive/H5File.hpp>

#include <cmath>
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

struct TimeSeries {
    std::vector<double> time, x, z, ux, uz, lift_x, lift_z, drag_x, drag_z;

    void reserve(size_t n) {
        for (auto* v : {&time, &x, &z, &ux, &uz, &lift_x, &lift_z, &drag_x, &drag_z})
            v->reserve(n);
    }

    void store(double t, const State& state, const Vec3& lift, const Vec3& drag) {
        time.push_back(t);
        x.push_back(state.pos.x());   z.push_back(state.pos.z());
        ux.push_back(state.vel.x());   uz.push_back(state.vel.z());
        lift_x.push_back(lift.x());    lift_z.push_back(lift.z());
        drag_x.push_back(drag.x());    drag_z.push_back(drag.z());
    }
};

} // namespace

int runTermvel(int argc, char* argv[]) {
    // Default parameters
    double psi_deg = 0.0;
    double dt = 0.01;
    double t_max = 50.0;
    std::string output_file = "termvel.h5";

    // Parse arguments
    try {
        for (int i = 2; i < argc; ++i) {
            const std::string arg = argv[i];
            if (arg == "--psi") {
                psi_deg = cliarg::parseDouble(
                    cliarg::requireOptionValue(argc, argv, i, "--psi"),
                    "--psi"
                );
            } else if (arg == "--dt") {
                dt = cliarg::parseDouble(
                    cliarg::requireOptionValue(argc, argv, i, "--dt"),
                    "--dt"
                );
            } else if (arg == "--tmax") {
                t_max = cliarg::parseDouble(
                    cliarg::requireOptionValue(argc, argv, i, "--tmax"),
                    "--tmax"
                );
            } else if (arg == "-o") {
                output_file = cliarg::requireOptionValue(argc, argv, i, "-o");
            } else if (cliarg::isHelpFlag(arg)) {
                printUsage(argv[0]);
                return 0;
            } else {
                std::cerr << "Unknown option: " << arg << "\n";
                printUsage(argv[0]);
                return 1;
            }
        }
    } catch (const std::exception& e) {
        std::cerr << e.what() << "\n";
        return 1;
    }

    if (dt <= 0.0) {
        std::cerr << "--dt must be > 0\n";
        return 1;
    }
    if (t_max <= 0.0) {
        std::cerr << "--tmax must be > 0\n";
        return 1;
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
    auto fixedAngles = [psi]([[maybe_unused]] double t) -> WingAngles {
        return {M_PI / 2.0, 0.0, 0.0, 0.0, psi};
    };

    Wing wing("test", mu0, lb0, WingSide::Left, Cd0, Cl0, 0.0, fixedAngles);
    std::vector<Wing> wings = {wing};

    // Storage for time series
    TimeSeries ts;
    ts.reserve(static_cast<size_t>(t_max / dt) + 1);

    // Initial state: at rest at origin
    State state;
    double t = 0.0;

    // Scratch buffer for integration
    std::vector<SingleWingVectors> scratch(1);

    ts.store(t, state, Vec3::Zero(), Vec3::Zero());

    // Integrate
    std::cout << "Running simulation..." << std::endl;
    while (t < t_max) {
        state = stepRK4(t, dt, state, wings, scratch);
        t += dt;

        // Compute forces at current state
        equationOfMotion(t, state, wings, scratch);

        ts.store(t, state, scratch[0].lift, scratch[0].drag);
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
    file.createDataSet("/time", ts.time);
    file.createDataSet("/x", ts.x);
    file.createDataSet("/z", ts.z);
    file.createDataSet("/ux", ts.ux);
    file.createDataSet("/uz", ts.uz);
    file.createDataSet("/lift_x", ts.lift_x);
    file.createDataSet("/lift_z", ts.lift_z);
    file.createDataSet("/drag_x", ts.drag_x);
    file.createDataSet("/drag_z", ts.drag_z);

    std::cout << "Done. " << ts.time.size() << " timesteps written.\n";

    return 0;
}

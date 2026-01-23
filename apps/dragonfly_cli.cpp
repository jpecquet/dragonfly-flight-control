#include "config.hpp"
#include "eom.hpp"
#include "integrator.hpp"
#include "output.hpp"
#include "rotation.hpp"
#include "wing.hpp"
#include "optimizer/flex_optim.hpp"

#include <highfive/H5Easy.hpp>
#include <highfive/H5File.hpp>
#include <nlopt.hpp>

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

void printUsage(const char* prog) {
    std::cerr << "Usage: " << prog << " <command> [options]\n";
    std::cerr << "\n";
    std::cerr << "Commands:\n";
    std::cerr << "  sim                Run flight simulation (-c <config>)\n";
    std::cerr << "  optim              Find equilibrium flight conditions (-c <config>)\n";
    std::cerr << "  plot               Generate visualization (-c <config>)\n";
    std::cerr << "  test_wing_rotation Wing rotation test (see options below)\n";
    std::cerr << "\n";
    std::cerr << "Test options:\n";
    std::cerr << "  --gam START:END    Stroke plane angle range (default: 0:90)\n";
    std::cerr << "  --phi START:END    Flapping angle range (default: 0:25)\n";
    std::cerr << "  --psi START:END    Pitch angle range (default: 0:45)\n";
    std::cerr << "  --frames N         Frames per phase (default: 50)\n";
    std::cerr << "  --right            Use right wing instead of left\n";
    std::cerr << "  -o <file>          Output file (default: wing_rotation.h5)\n";
    std::cerr << "\n";
    std::cerr << "Examples:\n";
    std::cerr << "  " << prog << " sim -c configs/hover.cfg\n";
    std::cerr << "  " << prog << " optim -c configs/optim.cfg\n";
    std::cerr << "  " << prog << " test_wing_rotation --gam 0:90 --phi 0:25 --frames 30 -o out.h5\n";
}

// ============================================================================
// Simulation
// ============================================================================

int runSim(const Config& cfg) {
    // Wing geometry
    double mu0_f = cfg.getDouble("mu0_f");
    double lb0_f = cfg.getDouble("lb0_f");
    double mu0_h = cfg.getDouble("mu0_h");
    double lb0_h = cfg.getDouble("lb0_h");

    // Aerodynamic coefficients
    double Cd0 = cfg.getDouble("Cd0");
    double Cl0 = cfg.getDouble("Cl0");

    // Kinematic parameters
    double omg0 = cfg.getDouble("omg0");
    double gam0 = cfg.getDouble("gam0");
    double phi0 = cfg.getDouble("phi0");
    double psim = cfg.getDouble("psim");
    double dpsi = cfg.getDouble("dpsi");
    double dlt0 = cfg.getDouble("dlt0");
    double sig0 = cfg.getDouble("sig0");

    // Initial conditions
    double x0 = cfg.getDouble("x0", 0.0);
    double y0 = cfg.getDouble("y0", 0.0);
    double z0 = cfg.getDouble("z0", 0.0);
    double ux0 = cfg.getDouble("ux0", 0.0);
    double uy0 = cfg.getDouble("uy0", 0.0);
    double uz0 = cfg.getDouble("uz0", 0.0);

    // Time integration
    int n_wingbeats = cfg.getInt("n_wingbeats", 5);
    int steps_per_wingbeat = cfg.getInt("steps_per_wingbeat", 50);

    // Output
    std::string output_file = cfg.getString("output");

    // Angle function for forewing
    auto angleFunc_f = [=](double t) -> WingAngles {
        return {
            gam0,
            phi0 * std::cos(omg0 * t),
            -phi0 * omg0 * std::sin(omg0 * t),
            psim + dpsi * std::cos(omg0 * t + dlt0)
        };
    };

    // Angle function for hindwing
    auto angleFunc_h = [=](double t) -> WingAngles {
        return {
            gam0,
            phi0 * std::cos(omg0 * t + sig0),
            -phi0 * omg0 * std::sin(omg0 * t + sig0),
            psim + dpsi * std::cos(omg0 * t + dlt0 + sig0)
        };
    };

    // Create wings
    std::vector<Wing> wings;
    wings.emplace_back("fore", mu0_f, lb0_f, WingSide::Left, Cd0, Cl0, angleFunc_f);
    wings.emplace_back("fore", mu0_f, lb0_f, WingSide::Right, Cd0, Cl0, angleFunc_f);
    wings.emplace_back("hind", mu0_h, lb0_h, WingSide::Left, Cd0, Cl0, angleFunc_h);
    wings.emplace_back("hind", mu0_h, lb0_h, WingSide::Right, Cd0, Cl0, angleFunc_h);

    // Initial state
    State state(x0, y0, z0, ux0, uy0, uz0);

    // Time setup
    double Twb = 2.0 * M_PI / omg0;
    double dt = Twb / steps_per_wingbeat;
    double T = n_wingbeats * Twb;
    int nsteps = static_cast<int>(T / dt);

    // Output storage
    SimulationOutput output;
    output.lb0_f = lb0_f;
    output.lb0_h = lb0_h;
    output.mu0_f = mu0_f;
    output.mu0_h = mu0_h;
    output.Cd0 = Cd0;
    output.Cl0 = Cl0;
    output.omg0 = omg0;
    output.gam0 = gam0;
    output.phi0 = phi0;
    output.psim = psim;
    output.dpsi = dpsi;
    output.sig0 = sig0;
    output.dlt0 = dlt0;
    output.time.reserve(nsteps + 1);
    output.states.reserve(nsteps + 1);
    output.wing_data.reserve(nsteps + 1);

    // Store initial state
    double t = 0.0;
    std::vector<SingleWingVectors> wing_data;
    equationOfMotion(t, state, wings, wing_data);
    output.time.push_back(t);
    output.states.push_back(state);
    output.wing_data.push_back(wing_data);

    // Time integration
    std::cout << "Running simulation for " << n_wingbeats << " wingbeats..." << std::endl;
    for (int i = 0; i < nsteps; ++i) {
        state = stepRK4(t, dt, state, wings);
        t += dt;

        std::vector<SingleWingVectors> wd;
        equationOfMotion(t, state, wings, wd);
        output.time.push_back(t);
        output.states.push_back(state);
        output.wing_data.push_back(wd);
    }

    // Write output
    writeHDF5(output_file, output, wings);
    std::cout << "Output written to " << output_file << std::endl;

    return 0;
}

// ============================================================================
// Optimization
// ============================================================================

// Thread-local state for NLopt callback
thread_local KinematicParams* g_kin_ptr = nullptr;
thread_local PhysicalParams g_phys;
thread_local double g_ux = 0.0;
thread_local double g_uz = 0.0;

double flexNloptObjective(const std::vector<double>& x, std::vector<double>& grad, void* data) {
    (void)grad;
    (void)data;
    g_kin_ptr->setVariableValues(x);
    return flexWingBeatAccel(*g_kin_ptr, g_phys, g_ux, g_uz);
}

// Optimize and return the result, modifying kin in place
double runOptimization(KinematicParams& kin, const PhysicalParams& phys,
                       double ux, double uz, int max_eval = 200) {
    g_kin_ptr = &kin;
    g_phys = phys;
    g_ux = ux;
    g_uz = uz;

    size_t n = kin.numVariable();
    if (n == 0) {
        return flexWingBeatAccel(kin, phys, ux, uz);
    }

    nlopt::opt opt(nlopt::LN_COBYLA, n);

    std::vector<double> lb, ub;
    kin.getVariableBounds(lb, ub);
    opt.set_lower_bounds(lb);
    opt.set_upper_bounds(ub);
    opt.set_min_objective(flexNloptObjective, nullptr);
    opt.set_maxeval(max_eval);
    opt.set_xtol_rel(1e-8);
    opt.set_ftol_rel(1e-10);

    std::vector<double> x = kin.variableValues();
    double minf;

    try {
        opt.optimize(x, minf);
        kin.setVariableValues(x);
    } catch (const std::exception& e) {
        std::cerr << "NLopt failed: " << e.what() << std::endl;
    }

    return minf;
}

// Check if solution is new (different from existing branches)
bool isNewBranch(const std::vector<KinematicParams>& branches, const KinematicParams& sol, double tol = 1e-3) {
    for (const auto& prev : branches) {
        std::vector<double> prev_vals = prev.variableValues();
        std::vector<double> sol_vals = sol.variableValues();
        double dist_sq = 0.0;
        for (size_t i = 0; i < prev_vals.size(); ++i) {
            dist_sq += std::pow(prev_vals[i] - sol_vals[i], 2);
        }
        if (std::sqrt(dist_sq) < tol) return false;
    }
    return true;
}

// Write output file for a single branch
void writeBranchOutput(const std::string& filename,
                       const std::vector<std::string>& col_names,
                       const std::vector<std::vector<double>>& data) {
    std::ofstream out(filename);

    // Header
    out << "# ";
    for (size_t i = 0; i < col_names.size(); ++i) {
        out << col_names[i];
        if (i < col_names.size() - 1) out << " ";
    }
    out << "\n";

    // Data
    out << std::fixed << std::setprecision(8);
    for (const auto& row : data) {
        for (size_t i = 0; i < row.size(); ++i) {
            out << row[i];
            if (i < row.size() - 1) out << " ";
        }
        out << "\n";
    }
    out.close();
}

int runOptim(const Config& cfg) {
    // Load parameters from config
    KinematicParams kin_template = KinematicParams::fromConfig(cfg);
    PhysicalParams phys = PhysicalParams::fromConfig(cfg);

    // Velocity range
    double ux_min = cfg.getDouble("ux_min", 0.0);
    double ux_max = cfg.getDouble("ux_max", 10.0);
    int n_velocity = cfg.getInt("n_velocity", 101);

    // Grid search resolution
    int n_grid = cfg.getInt("n_grid", 5);

    // Output base name (will append _branch1.txt, _branch2.txt, etc.)
    std::string output_base = cfg.getString("output");
    // Remove .txt extension if present
    if (output_base.size() > 4 && output_base.substr(output_base.size() - 4) == ".txt") {
        output_base = output_base.substr(0, output_base.size() - 4);
    }

    std::vector<std::string> var_names = kin_template.variableNames();
    std::cout << "Variable parameters: ";
    for (const auto& name : var_names) std::cout << name << " ";
    std::cout << std::endl;

    if (var_names.empty()) {
        std::cerr << "No variable parameters specified!" << std::endl;
        return 1;
    }

    std::cout << "\nSearching for equilibria at ux=" << ux_min << "..." << std::endl;

    // Grid search for initial equilibria
    std::vector<KinematicParams> branches;
    std::vector<double> lb, ub;
    kin_template.getVariableBounds(lb, ub);

    // Generate grid points for each variable parameter
    std::vector<std::vector<double>> grid_values(var_names.size());
    for (size_t i = 0; i < var_names.size(); ++i) {
        for (int j = 0; j < n_grid; ++j) {
            double val = lb[i] + j * (ub[i] - lb[i]) / (n_grid - 1);
            grid_values[i].push_back(val);
        }
    }

    // Recursive grid search (handles arbitrary number of variable parameters)
    std::function<void(size_t, std::vector<double>&)> gridSearch = [&](size_t depth, std::vector<double>& init_vals) {
        if (depth == var_names.size()) {
            KinematicParams kin = kin_template;
            kin.setVariableValues(init_vals);

            runOptimization(kin, phys, ux_min, 0.0);
            double accel = std::sqrt(flexWingBeatAccel(kin, phys, ux_min, 0.0));

            if (accel < 1e-6 && (branches.empty() || isNewBranch(branches, kin))) {
                branches.push_back(kin);
                std::cout << "  Branch " << branches.size() << ": ";
                std::vector<double> vals = kin.variableValues();
                for (size_t i = 0; i < var_names.size(); ++i) {
                    std::cout << var_names[i] << "=" << std::fixed << std::setprecision(2)
                              << (vals[i] * 180.0 / M_PI) << "deg ";
                }
                std::cout << std::endl;
            }
            return;
        }
        for (double val : grid_values[depth]) {
            init_vals.push_back(val);
            gridSearch(depth + 1, init_vals);
            init_vals.pop_back();
        }
    };

    std::vector<double> init_vals;
    gridSearch(0, init_vals);

    if (branches.empty()) {
        std::cerr << "No equilibrium solutions found!" << std::endl;
        return 1;
    }

    std::cout << "\nFound " << branches.size() << " branch(es)" << std::endl;
    std::cout << "Extending from ux=" << ux_min << " to ux=" << ux_max << "...\n" << std::endl;

    // Build column names for output: all kinematic params + ux
    std::vector<std::string> col_names = kin_template.allNames();
    col_names.push_back("ux");

    // Process each branch independently
    double da = M_PI / 16.0;  // Bound tightening for continuation

    for (size_t b = 0; b < branches.size(); ++b) {
        std::cout << "Processing branch " << (b + 1) << "..." << std::endl;

        std::vector<std::vector<double>> branch_data;
        KinematicParams kin = branches[b];

        for (int i = 0; i < n_velocity; ++i) {
            double ux = ux_min + i * (ux_max - ux_min) / (n_velocity - 1);

            if (i > 0) {
                // Tighten bounds around previous solution for continuation
                if (kin.gam0.is_variable) { kin.gam0.min_bound = kin.gam0.value - da; kin.gam0.max_bound = kin.gam0.value + da; }
                if (kin.phi0.is_variable) { kin.phi0.min_bound = kin.phi0.value - da; kin.phi0.max_bound = kin.phi0.value + da; }
                if (kin.psim.is_variable) { kin.psim.min_bound = kin.psim.value - da; kin.psim.max_bound = kin.psim.value + da; }
                if (kin.dpsi.is_variable) { kin.dpsi.min_bound = kin.dpsi.value - da; kin.dpsi.max_bound = kin.dpsi.value + da; }
                if (kin.dlt0.is_variable) { kin.dlt0.min_bound = kin.dlt0.value - da; kin.dlt0.max_bound = kin.dlt0.value + da; }
                if (kin.sig0.is_variable) { kin.sig0.min_bound = kin.sig0.value - da; kin.sig0.max_bound = kin.sig0.value + da; }

                runOptimization(kin, phys, ux, 0.0);
            }

            // Store all values (converted to degrees) plus ux
            std::vector<double> all_vals = kin.allValues();
            std::vector<double> row;
            for (double v : all_vals) {
                row.push_back(v * 180.0 / M_PI);  // Convert to degrees
            }
            row.push_back(ux);
            branch_data.push_back(row);

            if (i > 0 && i % 20 == 0) {
                std::cout << "  ux = " << std::fixed << std::setprecision(2) << ux << std::endl;
            }
        }

        // Write branch output file
        std::string branch_filename = output_base + "_branch" + std::to_string(b + 1) + ".txt";
        writeBranchOutput(branch_filename, col_names, branch_data);
        std::cout << "  Written to " << branch_filename << std::endl;
    }

    std::cout << "\nOptimization complete." << std::endl;
    return 0;
}

// ============================================================================
// Plot
// ============================================================================

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

// ============================================================================
// Wing Rotation Test
// ============================================================================

struct AngleRange {
    double start = 0.0;
    double end = 0.0;
};

struct TestConfig {
    AngleRange gam{0.0, 90.0};
    AngleRange phi{0.0, 25.0};
    AngleRange psi{0.0, 45.0};
    int frames_per_phase = 50;
    bool is_left = true;
    std::string output_file = "wing_rotation.h5";
};

bool parseRange(const char* arg, AngleRange& range) {
    char* end;
    range.start = std::strtod(arg, &end);
    if (*end != ':') return false;
    range.end = std::strtod(end + 1, &end);
    return *end == '\0';
}

TestConfig parseTestArgs(int argc, char* argv[]) {
    TestConfig cfg;

    for (int i = 2; i < argc; ++i) {
        if (std::strcmp(argv[i], "--gam") == 0 && i + 1 < argc) {
            if (!parseRange(argv[++i], cfg.gam)) {
                std::cerr << "Invalid --gam format. Use START:END\n";
                std::exit(1);
            }
        } else if (std::strcmp(argv[i], "--phi") == 0 && i + 1 < argc) {
            if (!parseRange(argv[++i], cfg.phi)) {
                std::cerr << "Invalid --phi format. Use START:END\n";
                std::exit(1);
            }
        } else if (std::strcmp(argv[i], "--psi") == 0 && i + 1 < argc) {
            if (!parseRange(argv[++i], cfg.psi)) {
                std::cerr << "Invalid --psi format. Use START:END\n";
                std::exit(1);
            }
        } else if (std::strcmp(argv[i], "--frames") == 0 && i + 1 < argc) {
            cfg.frames_per_phase = std::atoi(argv[++i]);
            if (cfg.frames_per_phase < 2) {
                std::cerr << "Frames must be >= 2\n";
                std::exit(1);
            }
        } else if (std::strcmp(argv[i], "--right") == 0) {
            cfg.is_left = false;
        } else if (std::strcmp(argv[i], "-o") == 0 && i + 1 < argc) {
            cfg.output_file = argv[++i];
        } else if (argv[i][0] != '-') {
            cfg.output_file = argv[i];
        } else {
            std::cerr << "Unknown option: " << argv[i] << "\n";
            std::exit(1);
        }
    }

    return cfg;
}

constexpr double deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

int runTest(int argc, char* argv[]) {
    TestConfig cfg = parseTestArgs(argc, argv);

    int total_frames = 3 * cfg.frames_per_phase;

    std::cout << "Wing rotation test (" << (cfg.is_left ? "left" : "right") << " wing)\n";
    std::cout << "==================\n";
    std::cout << "Phase 1: Stroke plane (gam) " << cfg.gam.start << " -> " << cfg.gam.end << " deg\n";
    std::cout << "Phase 2: Flapping (phi) " << cfg.phi.start << " -> " << cfg.phi.end << " deg\n";
    std::cout << "Phase 3: Pitch (psi) " << cfg.psi.start << " -> " << cfg.psi.end << " deg\n";
    std::cout << "Frames per phase: " << cfg.frames_per_phase << "\n\n";

    // Pre-allocate storage
    std::vector<int> frame_vec;
    std::vector<double> gam_vec, phi_vec, psi_vec;
    std::vector<std::vector<double>> e_s_vec, e_r_vec, e_c_vec;

    frame_vec.reserve(total_frames);
    gam_vec.reserve(total_frames);
    phi_vec.reserve(total_frames);
    psi_vec.reserve(total_frames);
    e_s_vec.reserve(total_frames);
    e_r_vec.reserve(total_frames);
    e_c_vec.reserve(total_frames);

    // Phase boundaries
    std::vector<int> phase_boundaries = {0, cfg.frames_per_phase, 2 * cfg.frames_per_phase};

    int frame = 0;
    int n = cfg.frames_per_phase;

    // Phase 1: Sweep stroke plane angle
    std::cout << "Phase 1: Stroke plane sweep\n";
    for (int i = 0; i < n; ++i, ++frame) {
        double t = static_cast<double>(i) / (n - 1);
        double gam_deg = cfg.gam.start + t * (cfg.gam.end - cfg.gam.start);
        double phi_deg = cfg.phi.start;
        double psi_deg = cfg.psi.start;

        auto orient = computeWingOrientation(
            deg2rad(gam_deg), deg2rad(phi_deg), deg2rad(psi_deg), cfg.is_left);

        frame_vec.push_back(frame);
        gam_vec.push_back(gam_deg);
        phi_vec.push_back(phi_deg);
        psi_vec.push_back(psi_deg);
        e_s_vec.push_back({orient.e_s.x(), orient.e_s.y(), orient.e_s.z()});
        e_r_vec.push_back({orient.e_r.x(), orient.e_r.y(), orient.e_r.z()});
        e_c_vec.push_back({orient.e_c.x(), orient.e_c.y(), orient.e_c.z()});

        if (i == 0 || i == n - 1) {
            std::cout << "  gam=" << gam_deg << " deg: e_s=[" << orient.e_s.transpose() << "]\n";
        }
    }

    // Phase 2: Sweep flapping angle
    std::cout << "\nPhase 2: Flapping sweep\n";
    double gam_fixed = cfg.gam.end;
    for (int i = 0; i < n; ++i, ++frame) {
        double t = static_cast<double>(i) / (n - 1);
        double gam_deg = gam_fixed;
        double phi_deg = cfg.phi.start + t * (cfg.phi.end - cfg.phi.start);
        double psi_deg = cfg.psi.start;

        auto orient = computeWingOrientation(
            deg2rad(gam_deg), deg2rad(phi_deg), deg2rad(psi_deg), cfg.is_left);

        frame_vec.push_back(frame);
        gam_vec.push_back(gam_deg);
        phi_vec.push_back(phi_deg);
        psi_vec.push_back(psi_deg);
        e_s_vec.push_back({orient.e_s.x(), orient.e_s.y(), orient.e_s.z()});
        e_r_vec.push_back({orient.e_r.x(), orient.e_r.y(), orient.e_r.z()});
        e_c_vec.push_back({orient.e_c.x(), orient.e_c.y(), orient.e_c.z()});

        if (i == 0 || i == n - 1) {
            std::cout << "  phi=" << phi_deg << " deg: e_s=[" << orient.e_s.transpose() << "]\n";
        }
    }

    // Phase 3: Sweep pitch angle
    std::cout << "\nPhase 3: Pitch sweep\n";
    double phi_fixed = cfg.phi.end;
    for (int i = 0; i < n; ++i, ++frame) {
        double t = static_cast<double>(i) / (n - 1);
        double gam_deg = gam_fixed;
        double phi_deg = phi_fixed;
        double psi_deg = cfg.psi.start + t * (cfg.psi.end - cfg.psi.start);

        auto orient = computeWingOrientation(
            deg2rad(gam_deg), deg2rad(phi_deg), deg2rad(psi_deg), cfg.is_left);

        frame_vec.push_back(frame);
        gam_vec.push_back(gam_deg);
        phi_vec.push_back(phi_deg);
        psi_vec.push_back(psi_deg);
        e_s_vec.push_back({orient.e_s.x(), orient.e_s.y(), orient.e_s.z()});
        e_r_vec.push_back({orient.e_r.x(), orient.e_r.y(), orient.e_r.z()});
        e_c_vec.push_back({orient.e_c.x(), orient.e_c.y(), orient.e_c.z()});

        if (i == 0 || i == n - 1) {
            std::cout << "  psi=" << psi_deg << " deg: e_c=[" << orient.e_c.transpose() << "]\n";
        }
    }

    // Write HDF5
    std::cout << "\nWriting to " << cfg.output_file << "...\n";

    HighFive::File file(cfg.output_file, HighFive::File::Overwrite);

    file.createGroup("/parameters");
    H5Easy::dump(file, "/parameters/frames_per_phase", cfg.frames_per_phase);
    H5Easy::dump(file, "/parameters/total_frames", total_frames);
    H5Easy::dump(file, "/parameters/is_left", cfg.is_left ? 1 : 0);
    H5Easy::dump(file, "/parameters/gam_start", cfg.gam.start);
    H5Easy::dump(file, "/parameters/gam_end", cfg.gam.end);
    H5Easy::dump(file, "/parameters/phi_start", cfg.phi.start);
    H5Easy::dump(file, "/parameters/phi_end", cfg.phi.end);
    H5Easy::dump(file, "/parameters/psi_start", cfg.psi.start);
    H5Easy::dump(file, "/parameters/psi_end", cfg.psi.end);
    H5Easy::dump(file, "/parameters/phase_boundaries", phase_boundaries);

    file.createDataSet("/frame", frame_vec);
    file.createGroup("/angles");
    file.createDataSet("/angles/gam", gam_vec);
    file.createDataSet("/angles/phi", phi_vec);
    file.createDataSet("/angles/psi", psi_vec);

    file.createGroup("/wing");
    file.createDataSet("/wing/e_s", e_s_vec);
    file.createDataSet("/wing/e_r", e_r_vec);
    file.createDataSet("/wing/e_c", e_c_vec);

    std::cout << "Done. Total frames: " << total_frames << "\n";

    return 0;
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char* argv[]) {
    if (argc < 2) {
        printUsage(argv[0]);
        return 1;
    }

    std::string command = argv[1];

    // test_wing_rotation command has its own argument parsing
    if (command == "test_wing_rotation") {
        try {
            return runTest(argc, argv);
        } catch (const std::exception& e) {
            std::cerr << "Error: " << e.what() << std::endl;
            return 1;
        }
    }

    // Other commands require -c <config>
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

    Config cfg;
    try {
        cfg = Config::load(config_file);
    } catch (const std::exception& e) {
        std::cerr << "Error loading config: " << e.what() << std::endl;
        return 1;
    }

    try {
        if (command == "sim") {
            return runSim(cfg);
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

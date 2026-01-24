#include "cmd_optim.hpp"
#include "optimize.hpp"
#include "sampling.hpp"

#include <nlopt.hpp>

#include <cmath>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

namespace {

// Context for NLopt objective callback
struct NLoptContext {
    KinematicParams* kin;
    const PhysicalParams* phys;
    OptimBuffers* buffers;  // Pre-allocated buffers to avoid allocation in hot path
    double ux;
    double uz;
};

double nloptObjective(const std::vector<double>& x, std::vector<double>& grad, void* data) {
    (void)grad;
    auto* ctx = static_cast<NLoptContext*>(data);
    ctx->kin->setVariableValues(x);
    return wingBeatAccel(*ctx->kin, *ctx->phys, *ctx->buffers, ctx->ux, ctx->uz);
}

// Optimize and return the result, modifying kin in place
double runOptimization(KinematicParams& kin, const PhysicalParams& phys,
                       OptimBuffers& buffers, double ux, double uz, int max_eval = 200) {
    size_t n = kin.numVariable();
    if (n == 0) {
        return wingBeatAccel(kin, phys, buffers, ux, uz);
    }

    NLoptContext ctx{&kin, &phys, &buffers, ux, uz};

    nlopt::opt opt(nlopt::LN_COBYLA, n);

    std::vector<double> lb, ub;
    kin.getVariableBounds(lb, ub);
    opt.set_lower_bounds(lb);
    opt.set_upper_bounds(ub);
    opt.set_min_objective(nloptObjective, &ctx);
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

// Find branches using Sobol multi-start sampling
std::vector<KinematicParams> findBranchesMultistart(
    const KinematicParams& kin_template,
    const PhysicalParams& phys,
    double ux, const OptimConfig& config) {

    std::vector<KinematicParams> branches;
    std::vector<double> lb, ub;
    kin_template.getVariableBounds(lb, ub);

    if (lb.empty()) {
        return branches;
    }

    auto samples = generateSobolSamples(static_cast<size_t>(config.n_samples), lb, ub);

    OptimBuffers buffers;
    buffers.init(kin_template, phys);

    for (const auto& x0 : samples) {
        KinematicParams kin = kin_template;
        kin.setVariableValues(x0);

        runOptimization(kin, phys, buffers, ux, 0.0, config.max_eval);
        double accel = std::sqrt(wingBeatAccel(kin, phys, buffers, ux, 0.0));

        if (accel < config.equilibrium_tol && isNewBranch(branches, kin)) {
            branches.push_back(kin);
        }
    }

    return branches;
}

// Find branches using NLopt global algorithms
std::vector<KinematicParams> findBranchesGlobal(
    const KinematicParams& kin_template,
    const PhysicalParams& phys,
    double ux, const OptimConfig& config) {

    std::vector<KinematicParams> branches;
    std::vector<double> lb, ub;
    kin_template.getVariableBounds(lb, ub);
    size_t n = lb.size();

    if (n == 0) {
        return branches;
    }

    // Select global algorithm
    nlopt::algorithm alg;
    switch (config.algorithm) {
        case OptimAlgorithm::DIRECT:
            alg = nlopt::GN_DIRECT_L;  // DIRECT with local refinement
            break;
        case OptimAlgorithm::MLSL:
            alg = nlopt::G_MLSL_LDS;   // Multi-level single-linkage with LDS
            break;
        case OptimAlgorithm::CRS2:
            alg = nlopt::GN_CRS2_LM;   // Controlled random search
            break;
        default:
            alg = nlopt::GN_DIRECT_L;
    }

    OptimBuffers buffers;
    buffers.init(kin_template, phys);

    KinematicParams kin = kin_template;
    NLoptContext ctx{&kin, &phys, &buffers, ux, 0.0};

    nlopt::opt opt(alg, n);
    opt.set_lower_bounds(lb);
    opt.set_upper_bounds(ub);
    opt.set_min_objective(nloptObjective, &ctx);
    opt.set_maxeval(config.max_eval * config.n_samples);  // More evals for global
    opt.set_ftol_rel(1e-10);

    // For MLSL, set a local optimizer
    if (config.algorithm == OptimAlgorithm::MLSL) {
        nlopt::opt local_opt(nlopt::LN_COBYLA, n);
        local_opt.set_ftol_rel(1e-10);
        local_opt.set_maxeval(config.max_eval);
        opt.set_local_optimizer(local_opt);
    }

    std::vector<double> x = kin.variableValues();
    double minf;

    try {
        opt.optimize(x, minf);
        kin.setVariableValues(x);

        // Always run local refinement from global optimizer's result
        runOptimization(kin, phys, buffers, ux, 0.0, config.max_eval);
        double accel = std::sqrt(wingBeatAccel(kin, phys, buffers, ux, 0.0));

        if (accel < config.equilibrium_tol) {
            branches.push_back(kin);
        }
    } catch (const std::exception& e) {
        std::cerr << "Global optimization failed: " << e.what() << std::endl;
    }

    // Also sample the full space to find additional branches
    auto samples = generateSobolSamples(static_cast<size_t>(config.n_samples), lb, ub);
    for (const auto& x0 : samples) {
        KinematicParams kin_local = kin_template;
        kin_local.setVariableValues(x0);

        runOptimization(kin_local, phys, buffers, ux, 0.0, config.max_eval);
        double accel = std::sqrt(wingBeatAccel(kin_local, phys, buffers, ux, 0.0));

        if (accel < config.equilibrium_tol && isNewBranch(branches, kin_local)) {
            branches.push_back(kin_local);
        }
    }

    return branches;
}

// Legacy grid search (for COBYLA compatibility)
std::vector<KinematicParams> findBranchesGrid(
    const KinematicParams& kin_template,
    const PhysicalParams& phys,
    double ux, const OptimConfig& config,
    const std::vector<std::string>& var_names) {

    std::vector<KinematicParams> branches;
    std::vector<double> lb, ub;
    kin_template.getVariableBounds(lb, ub);

    // Generate grid points
    std::vector<std::vector<double>> grid_values(var_names.size());
    for (size_t i = 0; i < var_names.size(); ++i) {
        for (int j = 0; j < config.n_grid; ++j) {
            double val = lb[i] + j * (ub[i] - lb[i]) / (config.n_grid - 1);
            grid_values[i].push_back(val);
        }
    }

    OptimBuffers grid_buffers;
    grid_buffers.init(kin_template, phys);

    // Recursive grid search
    std::function<void(size_t, std::vector<double>&)> gridSearch =
        [&](size_t depth, std::vector<double>& init_vals) {
        if (depth == var_names.size()) {
            KinematicParams kin = kin_template;
            kin.setVariableValues(init_vals);

            runOptimization(kin, phys, grid_buffers, ux, 0.0, config.max_eval);
            double accel = std::sqrt(wingBeatAccel(kin, phys, grid_buffers, ux, 0.0));

            if (accel < config.equilibrium_tol && isNewBranch(branches, kin)) {
                branches.push_back(kin);
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

    return branches;
}

}  // namespace

int runOptim(const Config& cfg) {
    // Load parameters from config
    KinematicParams kin_template = KinematicParams::fromConfig(cfg);
    PhysicalParams phys = PhysicalParams::fromConfig(cfg);
    OptimConfig optim_config = OptimConfig::fromConfig(cfg);

    // Velocity range
    double ux_min = cfg.getDouble("ux_min", 0.0);
    double ux_max = cfg.getDouble("ux_max", 10.0);
    int n_velocity = cfg.getInt("n_velocity", 101);

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

    // Generate landscape map if requested
    if (optim_config.map_landscape) {
        std::cout << "\nGenerating objective landscape..." << std::endl;
        LandscapeData landscape = computeLandscape(
            kin_template, phys, ux_min, optim_config.map_resolution);

        std::string landscape_file = output_base + "_landscape.h5";
        landscape.writeHDF5(landscape_file);
        std::cout << "  Written to " << landscape_file << std::endl;
    }

    std::cout << "\nSearching for equilibria at ux=" << ux_min << "..." << std::endl;

    // Algorithm dispatch
    std::string alg_name;
    std::vector<KinematicParams> branches;

    switch (optim_config.algorithm) {
        case OptimAlgorithm::MULTISTART:
            alg_name = "multistart";
            std::cout << "Using multi-start with " << optim_config.n_samples
                      << " Sobol samples" << std::endl;
            branches = findBranchesMultistart(kin_template, phys, ux_min, optim_config);
            break;

        case OptimAlgorithm::DIRECT:
            alg_name = "DIRECT";
            std::cout << "Using DIRECT global optimizer" << std::endl;
            branches = findBranchesGlobal(kin_template, phys, ux_min, optim_config);
            break;

        case OptimAlgorithm::MLSL:
            alg_name = "MLSL";
            std::cout << "Using MLSL global optimizer" << std::endl;
            branches = findBranchesGlobal(kin_template, phys, ux_min, optim_config);
            break;

        case OptimAlgorithm::CRS2:
            alg_name = "CRS2";
            std::cout << "Using CRS2 global optimizer" << std::endl;
            branches = findBranchesGlobal(kin_template, phys, ux_min, optim_config);
            break;

        case OptimAlgorithm::COBYLA:
        default:
            alg_name = "cobyla (grid search)";
            std::cout << "Using grid search with " << optim_config.n_grid
                      << "^" << var_names.size() << " starting points" << std::endl;
            branches = findBranchesGrid(kin_template, phys, ux_min, optim_config, var_names);
            break;
    }

    // Report found branches
    for (size_t b = 0; b < branches.size(); ++b) {
        std::cout << "  Branch " << (b + 1) << ": ";
        std::vector<double> vals = branches[b].variableValues();
        for (size_t i = 0; i < var_names.size(); ++i) {
            std::cout << var_names[i] << "=" << std::fixed << std::setprecision(2)
                      << (vals[i] * 180.0 / M_PI) << "deg ";
        }
        std::cout << std::endl;
    }

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

        // Pre-allocate buffers for this branch's continuation
        OptimBuffers branch_buffers;
        branch_buffers.init(kin, phys);

        for (int i = 0; i < n_velocity; ++i) {
            double ux = ux_min + i * (ux_max - ux_min) / (n_velocity - 1);

            if (i > 0) {
                // Tighten bounds around previous solution for continuation
                if (kin.omg0.is_variable) { kin.omg0.min_bound = kin.omg0.value - da; kin.omg0.max_bound = kin.omg0.value + da; }
                if (kin.gam0.is_variable) { kin.gam0.min_bound = kin.gam0.value - da; kin.gam0.max_bound = kin.gam0.value + da; }
                if (kin.phi0.is_variable) { kin.phi0.min_bound = kin.phi0.value - da; kin.phi0.max_bound = kin.phi0.value + da; }
                if (kin.psim.is_variable) { kin.psim.min_bound = kin.psim.value - da; kin.psim.max_bound = kin.psim.value + da; }
                if (kin.dpsi.is_variable) { kin.dpsi.min_bound = kin.dpsi.value - da; kin.dpsi.max_bound = kin.dpsi.value + da; }
                if (kin.dlt0.is_variable) { kin.dlt0.min_bound = kin.dlt0.value - da; kin.dlt0.max_bound = kin.dlt0.value + da; }

                runOptimization(kin, phys, branch_buffers, ux, 0.0, optim_config.max_eval);
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

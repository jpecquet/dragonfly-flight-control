#include "cmd_optim.hpp"
#include "optimizer/flex_optim.hpp"

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
struct OptimContext {
    KinematicParams* kin;
    const PhysicalParams* phys;
    double ux;
    double uz;
};

double flexNloptObjective(const std::vector<double>& x, std::vector<double>& grad, void* data) {
    (void)grad;
    auto* ctx = static_cast<OptimContext*>(data);
    ctx->kin->setVariableValues(x);
    return flexWingBeatAccel(*ctx->kin, *ctx->phys, ctx->ux, ctx->uz);
}

// Optimize and return the result, modifying kin in place
double runOptimization(KinematicParams& kin, const PhysicalParams& phys,
                       double ux, double uz, int max_eval = 200) {
    size_t n = kin.numVariable();
    if (n == 0) {
        return flexWingBeatAccel(kin, phys, ux, uz);
    }

    OptimContext ctx{&kin, &phys, ux, uz};

    nlopt::opt opt(nlopt::LN_COBYLA, n);

    std::vector<double> lb, ub;
    kin.getVariableBounds(lb, ub);
    opt.set_lower_bounds(lb);
    opt.set_upper_bounds(ub);
    opt.set_min_objective(flexNloptObjective, &ctx);
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

}  // namespace

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
                if (kin.omg0.is_variable) { kin.omg0.min_bound = kin.omg0.value - da; kin.omg0.max_bound = kin.omg0.value + da; }
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

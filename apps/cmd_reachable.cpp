#include "cmd_reachable.hpp"
#include "optimize.hpp"

#include <highfive/H5File.hpp>
#include <yaml-cpp/yaml.h>

#ifdef HAS_OPENMP
#include <omp.h>
#endif

#include <atomic>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace {

struct ControlBounds {
    double min;
    double max;
};

struct ReachableConfig {
    // Morphology
    double omega;
    std::vector<WingConfig> wings;

    // Control bounds (7 params)
    ControlBounds gamma_mean;
    ControlBounds phi_mean;
    ControlBounds phi_amp;
    ControlBounds phi_phase;
    ControlBounds psi_mean;
    ControlBounds psi_amp;
    ControlBounds psi_phase;

    // Grid
    double ux_min, ux_max;
    int n_ux;
    double uz_min, uz_max;
    int n_uz;

    // Algorithm
    int n_samples;
    int max_eval;
    double equilibrium_tol;

    // Output
    std::string output;
};

// Returns {min, max} for variable params, {val, val} for fixed (scalar) params
ControlBounds parseBounds(const YAML::Node& node) {
    if (node.IsSequence()) {
        return {node[0].as<double>(), node[1].as<double>()};
    }
    double val = node.as<double>();
    return {val, val};
}

WingConfig parseWingYAML(const YAML::Node& node) {
    WingConfig wc;
    wc.name = node["name"].as<std::string>();
    wc.side = parseSide(node["side"].as<std::string>());
    wc.mu0 = node["mu0"].as<double>();
    wc.lb0 = node["lb0"].as<double>();
    wc.phase_offset = node["phase"].as<double>(0.0);
    wc.cone_angle = node["cone"].as<double>(0.0);
    wc.n_blade_elements = node["n_blade_elements"].as<int>(1);

    // Aero model: named preset or explicit coefficients
    if (node["aero_model"]) {
        std::string model = node["aero_model"].as<std::string>();
        if (model == "wang2004") {
            wc.drag_model = DragCoefficientModel::Sinusoidal;
            wc.lift_model = LiftCoefficientModel::Sinusoidal;
            wc.Cd_min = 0.4;
            wc.Cd_max = 2.4;
            wc.Cl0 = 1.2;
        } else {
            throw std::runtime_error("Unknown aero_model: " + model);
        }
    } else {
        wc.drag_model = DragCoefficientModel::Sinusoidal;
        wc.lift_model = LiftCoefficientModel::Sinusoidal;
        wc.Cd_min = node["Cd_min"].as<double>();
        wc.Cd_max = node["Cd_max"].as<double>(wc.Cd_min + 2.0);
        wc.Cl0 = node["Cl0"].as<double>();
    }

    return wc;
}

ReachableConfig parseConfig(const std::string& path) {
    YAML::Node yaml = YAML::LoadFile(path);
    ReachableConfig cfg;

    // Morphology
    cfg.omega = yaml["morphology"]["omega"].as<double>();
    for (const auto& w : yaml["morphology"]["wings"]) {
        cfg.wings.push_back(parseWingYAML(w));
    }

    // Control bounds
    auto cb = yaml["control_bounds"];
    cfg.gamma_mean = parseBounds(cb["gamma_mean"]);
    cfg.phi_mean = parseBounds(cb["phi_mean"]);
    cfg.phi_amp = parseBounds(cb["phi_amp"]);
    cfg.phi_phase = parseBounds(cb["phi_phase"]);
    cfg.psi_mean = parseBounds(cb["psi_mean"]);
    cfg.psi_amp = parseBounds(cb["psi_amp"]);
    cfg.psi_phase = parseBounds(cb["psi_phase"]);

    // Grid
    auto grid = yaml["grid"];
    cfg.ux_min = grid["ux_min"].as<double>();
    cfg.ux_max = grid["ux_max"].as<double>();
    cfg.n_ux = grid["n_ux"].as<int>();
    cfg.uz_min = grid["uz_min"].as<double>();
    cfg.uz_max = grid["uz_max"].as<double>();
    cfg.n_uz = grid["n_uz"].as<int>();

    // Algorithm
    auto alg = yaml["algorithm"];
    cfg.n_samples = alg["n_samples"].as<int>(200);
    cfg.max_eval = alg["max_eval"].as<int>(300);
    cfg.equilibrium_tol = alg["equilibrium_tol"].as<double>(1e-6);

    // Output
    cfg.output = yaml["output"].as<std::string>("reachable.h5");

    return cfg;
}

// Build a KinematicParam from control bounds: variable if min != max, fixed otherwise
KinematicParam buildParam(const char* name, const ControlBounds& cb) {
    bool is_var = (cb.min != cb.max);
    double val = is_var ? (cb.min + cb.max) / 2.0 : cb.min;
    return {name, is_var, val, cb.min, cb.max};
}

KinematicParams buildKinTemplate(const ReachableConfig& cfg) {
    KinematicParams kin;

    kin.omega = {"omega", false, cfg.omega, cfg.omega, cfg.omega};

    kin.gamma_mean = buildParam("gamma_mean", cfg.gamma_mean);
    kin.gamma_amp = {"gamma_amp", false, 0.0, 0.0, 0.0};
    kin.gamma_phase = {"gamma_phase", false, 0.0, 0.0, 0.0};

    kin.phi_mean = buildParam("phi_mean", cfg.phi_mean);
    kin.phi_amp = buildParam("phi_amp", cfg.phi_amp);
    kin.phi_phase = buildParam("phi_phase", cfg.phi_phase);

    kin.psi_mean = buildParam("psi_mean", cfg.psi_mean);
    kin.psi_amp = buildParam("psi_amp", cfg.psi_amp);
    kin.psi_phase = buildParam("psi_phase", cfg.psi_phase);

    return kin;
}

PhysicalParams buildPhysParams(const ReachableConfig& cfg) {
    PhysicalParams phys;
    phys.wings = cfg.wings;
    return phys;
}

// Compute 2x7 Jacobian d(ax,az)/d(params) via central finite differences
std::vector<double> computeJacobian(
    const KinematicParams& kin, const PhysicalParams& phys,
    OptimBuffers& buf, double ux, double uz, double h = 1e-4) {

    size_t n_var = kin.numVariable();
    // Layout: [d(ax)/dp0, d(ax)/dp1, ..., d(az)/dp0, d(az)/dp1, ...]
    std::vector<double> jac(2 * n_var, 0.0);

    std::vector<double> x0 = kin.variableValues();

    for (size_t j = 0; j < n_var; ++j) {
        KinematicParams kin_p = kin, kin_m = kin;
        std::vector<double> xp = x0, xm = x0;
        xp[j] += h;
        xm[j] -= h;
        kin_p.setVariableValues(xp);
        kin_m.setVariableValues(xm);

        Vec3 ap = wingBeatAccelVec(kin_p, phys, buf, ux, uz);
        Vec3 am = wingBeatAccelVec(kin_m, phys, buf, ux, uz);

        jac[j] = (ap(0) - am(0)) / (2.0 * h);           // d(ax)/dp_j
        jac[n_var + j] = (ap(2) - am(2)) / (2.0 * h);   // d(az)/dp_j
    }

    return jac;
}

// Compute minimum singular value of a 2xN matrix (stored row-major)
double minSingularValue(const std::vector<double>& jac, size_t n_cols) {
    // For 2xN, use J*J^T (2x2) eigenvalues
    double a11 = 0, a12 = 0, a22 = 0;
    for (size_t j = 0; j < n_cols; ++j) {
        double r0 = jac[j];          // row 0
        double r1 = jac[n_cols + j]; // row 1
        a11 += r0 * r0;
        a12 += r0 * r1;
        a22 += r1 * r1;
    }
    // Eigenvalues of 2x2 symmetric [a11, a12; a12, a22]
    double trace = a11 + a22;
    double det = a11 * a22 - a12 * a12;
    double disc = trace * trace - 4.0 * det;
    if (disc < 0) disc = 0;
    double lambda_min = (trace - std::sqrt(disc)) / 2.0;
    if (lambda_min < 0) lambda_min = 0;
    return std::sqrt(lambda_min);
}

}  // namespace

int runReachable(const std::string& cfg_path) {
    ReachableConfig cfg = parseConfig(cfg_path);

    KinematicParams kin_template = buildKinTemplate(cfg);
    PhysicalParams phys = buildPhysParams(cfg);

    size_t n_var = kin_template.numVariable();
    int n_ux = cfg.n_ux;
    int n_uz = cfg.n_uz;
    int total = n_ux * n_uz;
    constexpr int max_branches = 8;

    std::cout << "Reachable set analysis: " << n_ux << "x" << n_uz << " grid, "
              << cfg.n_samples << " samples/point, " << n_var << " control params\n";

    // Grid coordinates
    std::vector<double> ux_vals(n_ux), uz_vals(n_uz);
    for (int i = 0; i < n_ux; ++i) {
        ux_vals[i] = (n_ux > 1) ? cfg.ux_min + i * (cfg.ux_max - cfg.ux_min) / (n_ux - 1) : cfg.ux_min;
    }
    for (int j = 0; j < n_uz; ++j) {
        uz_vals[j] = (n_uz > 1) ? cfg.uz_min + j * (cfg.uz_max - cfg.uz_min) / (n_uz - 1) : cfg.uz_min;
    }

    // Result arrays (flat, row-major in [i_ux][i_uz])
    constexpr double NaN = std::numeric_limits<double>::quiet_NaN();
    std::vector<double> min_residual(total, NaN);
    std::vector<int> n_branches_arr(total, 0);

    // Per-branch arrays
    struct BranchArrays {
        std::vector<double> params;    // [total * n_var]
        std::vector<double> residual;  // [total]
        std::vector<double> jacobian;  // [total * 2 * n_var]
        std::vector<double> sigma_min; // [total]
    };
    std::vector<BranchArrays> branch_data(max_branches);
    for (auto& ba : branch_data) {
        ba.params.assign(total * n_var, NaN);
        ba.residual.assign(total, NaN);
        ba.jacobian.assign(total * 2 * n_var, NaN);
        ba.sigma_min.assign(total, NaN);
    }

    std::atomic<int> progress(0);

    #pragma omp parallel if(total > 1)
    {
        // Thread-local buffers
        OptimBuffers buf;
        buf.init(kin_template, phys);

        #pragma omp for schedule(dynamic)
        for (int idx = 0; idx < total; ++idx) {
            int i_ux = idx / n_uz;
            int i_uz = idx % n_uz;
            double ux = ux_vals[i_ux];
            double uz = uz_vals[i_uz];

            auto branches = findEquilibria(kin_template, phys, ux, uz,
                                           cfg.n_samples, cfg.max_eval, cfg.equilibrium_tol);

            int nb = std::min(static_cast<int>(branches.size()), max_branches);
            n_branches_arr[idx] = nb;

            double best_res = std::numeric_limits<double>::infinity();
            for (int b = 0; b < nb; ++b) {
                const auto& br = branches[b];
                best_res = std::min(best_res, br.residual);

                // Store params
                std::vector<double> vals = br.kin.variableValues();
                for (size_t p = 0; p < n_var; ++p) {
                    branch_data[b].params[idx * n_var + p] = vals[p];
                }
                branch_data[b].residual[idx] = br.residual;

                // Compute Jacobian
                auto jac = computeJacobian(br.kin, phys, buf, ux, uz);
                for (size_t k = 0; k < 2 * n_var; ++k) {
                    branch_data[b].jacobian[idx * 2 * n_var + k] = jac[k];
                }
                branch_data[b].sigma_min[idx] = minSingularValue(jac, n_var);
            }

            if (nb > 0) {
                min_residual[idx] = best_res;
            }

            int done = ++progress;
            if (done % std::max(1, total / 20) == 0 || done == total) {
                #pragma omp critical
                std::cout << "  " << done << "/" << total << " points done\n";
            }
        }
    }

    // Write HDF5 output
    std::cout << "Writing " << cfg.output << "...\n";
    HighFive::File file(cfg.output, HighFive::File::Overwrite);

    auto grid = file.createGroup("grid");
    grid.createDataSet("ux", ux_vals);
    grid.createDataSet("uz", uz_vals);

    // Reshape min_residual and n_branches to 2D
    std::vector<std::vector<double>> min_res_2d(n_ux, std::vector<double>(n_uz));
    std::vector<std::vector<int>> nb_2d(n_ux, std::vector<int>(n_uz));
    for (int i = 0; i < n_ux; ++i) {
        for (int j = 0; j < n_uz; ++j) {
            min_res_2d[i][j] = min_residual[i * n_uz + j];
            nb_2d[i][j] = n_branches_arr[i * n_uz + j];
        }
    }
    grid.createDataSet("min_residual", min_res_2d);
    grid.createDataSet("n_branches", nb_2d);

    auto branches_grp = file.createGroup("branches");
    for (int b = 0; b < max_branches; ++b) {
        auto bg = branches_grp.createGroup(std::to_string(b));

        // Reshape params to [n_ux, n_uz, n_var]
        std::vector<std::vector<std::vector<double>>> params_3d(
            n_ux, std::vector<std::vector<double>>(n_uz, std::vector<double>(n_var)));
        std::vector<std::vector<double>> res_2d(n_ux, std::vector<double>(n_uz));
        std::vector<std::vector<double>> sig_2d(n_ux, std::vector<double>(n_uz));
        // Jacobian: [n_ux, n_uz, 2, n_var] — store as [n_ux, n_uz, 2*n_var]
        std::vector<std::vector<std::vector<double>>> jac_3d(
            n_ux, std::vector<std::vector<double>>(n_uz, std::vector<double>(2 * n_var)));

        for (int i = 0; i < n_ux; ++i) {
            for (int j = 0; j < n_uz; ++j) {
                int idx = i * n_uz + j;
                for (size_t p = 0; p < n_var; ++p) {
                    params_3d[i][j][p] = branch_data[b].params[idx * n_var + p];
                }
                res_2d[i][j] = branch_data[b].residual[idx];
                sig_2d[i][j] = branch_data[b].sigma_min[idx];
                for (size_t k = 0; k < 2 * n_var; ++k) {
                    jac_3d[i][j][k] = branch_data[b].jacobian[idx * 2 * n_var + k];
                }
            }
        }

        bg.createDataSet("params", params_3d);
        bg.createDataSet("residual", res_2d);
        bg.createDataSet("jacobian", jac_3d);
        bg.createDataSet("sigma_min", sig_2d);
    }

    // Metadata
    auto meta = file.createGroup("metadata");
    meta.createDataSet("param_names", kin_template.variableNames());
    meta.createDataSet("omega", cfg.omega);
    meta.createDataSet("equilibrium_tol", cfg.equilibrium_tol);

    std::cout << "Done.\n";
    return 0;
}

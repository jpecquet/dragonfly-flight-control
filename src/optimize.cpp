#include "optimize.hpp"
#include "eom.hpp"
#include "kinematics.hpp"
#include "wing.hpp"

#include <highfive/H5File.hpp>
#include <stdexcept>

// KinematicParams implementation

std::vector<std::string> KinematicParams::variableNames() const {
    std::vector<std::string> names;
    if (omega.is_variable) names.push_back("omega");
    if (gamma_mean.is_variable) names.push_back("gamma_mean");
    if (gamma_amp.is_variable) names.push_back("gamma_amp");
    if (gamma_phase.is_variable) names.push_back("gamma_phase");
    if (phi_amp.is_variable) names.push_back("phi_amp");
    if (psi_mean.is_variable) names.push_back("psi_mean");
    if (psi_amp.is_variable) names.push_back("psi_amp");
    if (psi_phase.is_variable) names.push_back("psi_phase");
    return names;
}

std::vector<std::string> KinematicParams::allNames() const {
    return {"omega", "gamma_mean", "gamma_amp", "gamma_phase", "phi_amp", "psi_mean", "psi_amp", "psi_phase"};
}

std::vector<double> KinematicParams::allValues() const {
    return {omega.value, gamma_mean.value, gamma_amp.value, gamma_phase.value, phi_amp.value, psi_mean.value, psi_amp.value, psi_phase.value};
}

std::vector<double> KinematicParams::variableValues() const {
    std::vector<double> values;
    if (omega.is_variable) values.push_back(omega.value);
    if (gamma_mean.is_variable) values.push_back(gamma_mean.value);
    if (gamma_amp.is_variable) values.push_back(gamma_amp.value);
    if (gamma_phase.is_variable) values.push_back(gamma_phase.value);
    if (phi_amp.is_variable) values.push_back(phi_amp.value);
    if (psi_mean.is_variable) values.push_back(psi_mean.value);
    if (psi_amp.is_variable) values.push_back(psi_amp.value);
    if (psi_phase.is_variable) values.push_back(psi_phase.value);
    return values;
}

void KinematicParams::setVariableValues(const std::vector<double>& x) {
    size_t i = 0;
    if (omega.is_variable) omega.value = x[i++];
    if (gamma_mean.is_variable) gamma_mean.value = x[i++];
    if (gamma_amp.is_variable) gamma_amp.value = x[i++];
    if (gamma_phase.is_variable) gamma_phase.value = x[i++];
    if (phi_amp.is_variable) phi_amp.value = x[i++];
    if (psi_mean.is_variable) psi_mean.value = x[i++];
    if (psi_amp.is_variable) psi_amp.value = x[i++];
    if (psi_phase.is_variable) psi_phase.value = x[i++];
}

void KinematicParams::getVariableBounds(std::vector<double>& lb, std::vector<double>& ub) const {
    lb.clear();
    ub.clear();
    if (omega.is_variable) { lb.push_back(omega.min_bound); ub.push_back(omega.max_bound); }
    if (gamma_mean.is_variable) { lb.push_back(gamma_mean.min_bound); ub.push_back(gamma_mean.max_bound); }
    if (gamma_amp.is_variable) { lb.push_back(gamma_amp.min_bound); ub.push_back(gamma_amp.max_bound); }
    if (gamma_phase.is_variable) { lb.push_back(gamma_phase.min_bound); ub.push_back(gamma_phase.max_bound); }
    if (phi_amp.is_variable) { lb.push_back(phi_amp.min_bound); ub.push_back(phi_amp.max_bound); }
    if (psi_mean.is_variable) { lb.push_back(psi_mean.min_bound); ub.push_back(psi_mean.max_bound); }
    if (psi_amp.is_variable) { lb.push_back(psi_amp.min_bound); ub.push_back(psi_amp.max_bound); }
    if (psi_phase.is_variable) { lb.push_back(psi_phase.min_bound); ub.push_back(psi_phase.max_bound); }
}

size_t KinematicParams::numVariable() const {
    size_t n = 0;
    if (omega.is_variable) n++;
    if (gamma_mean.is_variable) n++;
    if (gamma_amp.is_variable) n++;
    if (gamma_phase.is_variable) n++;
    if (phi_amp.is_variable) n++;
    if (psi_mean.is_variable) n++;
    if (psi_amp.is_variable) n++;
    if (psi_phase.is_variable) n++;
    return n;
}

static KinematicParam parseParam(const Config& cfg, const std::string& name) {
    KinematicParam param;
    param.name = name;

    std::string val = cfg.getString(name);
    if (val == "variable") {
        param.is_variable = true;
        param.min_bound = cfg.getDouble(name + "_min");
        param.max_bound = cfg.getDouble(name + "_max");
        // Initialize to midpoint
        param.value = (param.min_bound + param.max_bound) / 2.0;
    } else {
        param.is_variable = false;
        param.value = std::stod(val);
        param.min_bound = param.value;
        param.max_bound = param.value;
    }
    return param;
}

static KinematicParam parseParamWithDefault(const Config& cfg, const std::string& name, double default_val) {
    KinematicParam param;
    param.name = name;

    if (!cfg.has(name)) {
        param.is_variable = false;
        param.value = default_val;
        param.min_bound = default_val;
        param.max_bound = default_val;
        return param;
    }

    std::string val = cfg.getString(name);
    if (val == "variable") {
        param.is_variable = true;
        param.min_bound = cfg.getDouble(name + "_min");
        param.max_bound = cfg.getDouble(name + "_max");
        param.value = (param.min_bound + param.max_bound) / 2.0;
    } else {
        param.is_variable = false;
        param.value = std::stod(val);
        param.min_bound = param.value;
        param.max_bound = param.value;
    }
    return param;
}

KinematicParams KinematicParams::fromConfig(const Config& cfg) {
    KinematicParams params;
    params.omega = parseParam(cfg, "omega");
    params.gamma_mean = parseParam(cfg, "gamma_mean");
    params.gamma_amp = parseParamWithDefault(cfg, "gamma_amp", 0.0);
    params.gamma_phase = parseParamWithDefault(cfg, "gamma_phase", 0.0);
    params.phi_amp = parseParam(cfg, "phi_amp");
    params.psi_mean = parseParam(cfg, "psi_mean");
    params.psi_amp = parseParam(cfg, "psi_amp");
    params.psi_phase = parseParam(cfg, "psi_phase");
    return params;
}

// PhysicalParams implementation

PhysicalParams PhysicalParams::fromConfig(const Config& cfg) {
    if (!cfg.hasWings()) {
        throw std::runtime_error("Config must define wings using [[wing]] sections");
    }

    PhysicalParams params;
    for (const auto& entry : cfg.getWingEntries()) {
        WingSide side = (entry.side == "left") ? WingSide::Left : WingSide::Right;
        params.wings.emplace_back(entry.name, side, entry.mu0, entry.lb0,
                                  entry.Cd0, entry.Cl0, entry.phase);
    }
    return params;
}

// Internal helper to create wings

static std::vector<Wing> createWings(const KinematicParams& kin, const PhysicalParams& phys) {
    std::vector<Wing> wings;
    wings.reserve(phys.wings.size());

    double omega = kin.omega.value;
    for (const auto& wc : phys.wings) {
        auto angleFunc = makeAngleFunc(
            kin.gamma_mean.value, kin.gamma_amp.value, kin.gamma_phase.value,
            kin.phi_amp.value, kin.psi_mean.value,
            kin.psi_amp.value, kin.psi_phase.value, wc.phaseOffset, omega);
        wings.emplace_back(wc.name, wc.mu0, wc.lb0, wc.side, wc.Cd0, wc.Cl0, angleFunc);
    }
    return wings;
}

// OptimBuffers implementation

void OptimBuffers::init(const KinematicParams& kin, const PhysicalParams& phys) {
    wings = createWings(kin, phys);
    scratch1.resize(wings.size());
    scratch2.resize(wings.size());
}

// Update angle functions on pre-allocated wings
static void updateWingAngleFuncs(std::vector<Wing>& wings, const KinematicParams& kin,
                                  const PhysicalParams& phys) {
    double omega = kin.omega.value;
    for (size_t i = 0; i < wings.size(); ++i) {
        const auto& wc = phys.wings[i];
        auto angleFunc = makeAngleFunc(
            kin.gamma_mean.value, kin.gamma_amp.value, kin.gamma_phase.value,
            kin.phi_amp.value, kin.psi_mean.value,
            kin.psi_amp.value, kin.psi_phase.value, wc.phaseOffset, omega);
        wings[i].setAngleFunc(std::move(angleFunc));
    }
}

// Objective function (pre-allocated buffers version - use in hot paths)

double wingBeatAccel(const KinematicParams& kin, const PhysicalParams& phys,
                         OptimBuffers& buf, double ux, double uz, int N) {
    updateWingAngleFuncs(buf.wings, kin, phys);

    double omega = kin.omega.value;
    double T = 2.0 * M_PI / omega;
    double dt = T / N;

    Vec3 a_mean = Vec3::Zero();
    double t = 0.0;
    State state(0.0, 0.0, 0.0, ux, 0.0, uz);

    while (t < T) {
        StateDerivative d1 = equationOfMotion(t, state, buf.wings, buf.scratch1);
        StateDerivative d2 = equationOfMotion(t + dt, state, buf.wings, buf.scratch2);

        a_mean += (dt / T) * (d1.accel + d2.accel) / 2.0;
        t += dt;
    }

    return a_mean.squaredNorm();
}

// Convenience overload (allocates internally, for one-off calls)

double wingBeatAccel(const KinematicParams& kin, const PhysicalParams& phys,
                         double ux, double uz, int N) {
    OptimBuffers buf;
    buf.init(kin, phys);
    return wingBeatAccel(kin, phys, buf, ux, uz, N);
}

// Algorithm selection implementation

OptimAlgorithm parseAlgorithm(const std::string& name) {
    if (name == "cobyla") return OptimAlgorithm::COBYLA;
    if (name == "direct") return OptimAlgorithm::DIRECT;
    if (name == "mlsl") return OptimAlgorithm::MLSL;
    if (name == "crs2") return OptimAlgorithm::CRS2;
    if (name == "multistart") return OptimAlgorithm::MULTISTART;
    throw std::runtime_error("Unknown optimization algorithm: " + name);
}

OptimConfig OptimConfig::fromConfig(const Config& cfg) {
    OptimConfig oc;
    oc.algorithm = parseAlgorithm(cfg.getString("algorithm", "cobyla"));
    oc.n_samples = cfg.getInt("n_samples", 100);
    oc.n_grid = cfg.getInt("n_grid", 5);
    oc.max_eval = cfg.getInt("max_eval", 200);
    oc.equilibrium_tol = cfg.getDouble("equilibrium_tol", 1e-6);
    oc.map_landscape = cfg.getString("map_landscape", "false") == "true";
    oc.map_resolution = cfg.getInt("map_resolution", 50);
    return oc;
}

// Landscape computation for visualization

LandscapeData computeLandscape(
    const KinematicParams& kin_template,
    const PhysicalParams& phys,
    double ux, int resolution) {

    LandscapeData data;
    data.ux = ux;
    data.param_names = kin_template.variableNames();

    std::vector<double> lb, ub;
    kin_template.getVariableBounds(lb, ub);

    size_t n_var = lb.size();
    if (n_var == 0) {
        throw std::runtime_error("computeLandscape: no variable parameters");
    }

    OptimBuffers buffers;
    buffers.init(kin_template, phys);

    if (n_var == 1) {
        // 1D landscape
        data.param1_values.resize(resolution);
        data.objective_values.resize(resolution, std::vector<double>(1));

        for (int i = 0; i < resolution; ++i) {
            double t = static_cast<double>(i) / (resolution - 1);
            double val = lb[0] + t * (ub[0] - lb[0]);
            data.param1_values[i] = val;

            KinematicParams kin = kin_template;
            kin.setVariableValues({val});
            double accel = std::sqrt(wingBeatAccel(kin, phys, buffers, ux, 0.0));
            data.objective_values[i][0] = accel;
        }
    } else {
        // 2D landscape (first two variable parameters)
        data.param1_values.resize(resolution);
        data.param2_values.resize(resolution);
        data.objective_values.resize(resolution, std::vector<double>(resolution));

        for (int i = 0; i < resolution; ++i) {
            double t1 = static_cast<double>(i) / (resolution - 1);
            data.param1_values[i] = lb[0] + t1 * (ub[0] - lb[0]);
        }
        for (int j = 0; j < resolution; ++j) {
            double t2 = static_cast<double>(j) / (resolution - 1);
            data.param2_values[j] = lb[1] + t2 * (ub[1] - lb[1]);
        }

        for (int i = 0; i < resolution; ++i) {
            for (int j = 0; j < resolution; ++j) {
                KinematicParams kin = kin_template;

                // Set first two variable parameters; keep others at midpoint
                std::vector<double> x = kin_template.variableValues();
                x[0] = data.param1_values[i];
                x[1] = data.param2_values[j];
                kin.setVariableValues(x);

                double accel = std::sqrt(wingBeatAccel(kin, phys, buffers, ux, 0.0));
                data.objective_values[i][j] = accel;
            }
        }
    }

    return data;
}

void LandscapeData::writeHDF5(const std::string& filename) const {
    HighFive::File file(filename, HighFive::File::Overwrite);

    file.createDataSet("ux", ux);

    // Write parameter names
    file.createDataSet("param_names", param_names);

    // Write parameter values
    file.createDataSet("param1_values", param1_values);
    if (!param2_values.empty()) {
        file.createDataSet("param2_values", param2_values);
    }

    // Write objective values
    file.createDataSet("objective", objective_values);
}

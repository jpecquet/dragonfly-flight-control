#include "optimize.hpp"
#include "eom.hpp"
#include "kinematics.hpp"
#include "wing.hpp"

// KinematicParams implementation

std::vector<std::string> KinematicParams::variableNames() const {
    std::vector<std::string> names;
    if (omg0.is_variable) names.push_back("omg0");
    if (gam0.is_variable) names.push_back("gam0");
    if (phi0.is_variable) names.push_back("phi0");
    if (psim.is_variable) names.push_back("psim");
    if (dpsi.is_variable) names.push_back("dpsi");
    if (dlt0.is_variable) names.push_back("dlt0");
    return names;
}

std::vector<std::string> KinematicParams::allNames() const {
    return {"omg0", "gam0", "phi0", "psim", "dpsi", "dlt0"};
}

std::vector<double> KinematicParams::allValues() const {
    return {omg0.value, gam0.value, phi0.value, psim.value, dpsi.value, dlt0.value};
}

std::vector<double> KinematicParams::variableValues() const {
    std::vector<double> values;
    if (omg0.is_variable) values.push_back(omg0.value);
    if (gam0.is_variable) values.push_back(gam0.value);
    if (phi0.is_variable) values.push_back(phi0.value);
    if (psim.is_variable) values.push_back(psim.value);
    if (dpsi.is_variable) values.push_back(dpsi.value);
    if (dlt0.is_variable) values.push_back(dlt0.value);
    return values;
}

void KinematicParams::setVariableValues(const std::vector<double>& x) {
    size_t i = 0;
    if (omg0.is_variable) omg0.value = x[i++];
    if (gam0.is_variable) gam0.value = x[i++];
    if (phi0.is_variable) phi0.value = x[i++];
    if (psim.is_variable) psim.value = x[i++];
    if (dpsi.is_variable) dpsi.value = x[i++];
    if (dlt0.is_variable) dlt0.value = x[i++];
}

void KinematicParams::getVariableBounds(std::vector<double>& lb, std::vector<double>& ub) const {
    lb.clear();
    ub.clear();
    if (omg0.is_variable) { lb.push_back(omg0.min_bound); ub.push_back(omg0.max_bound); }
    if (gam0.is_variable) { lb.push_back(gam0.min_bound); ub.push_back(gam0.max_bound); }
    if (phi0.is_variable) { lb.push_back(phi0.min_bound); ub.push_back(phi0.max_bound); }
    if (psim.is_variable) { lb.push_back(psim.min_bound); ub.push_back(psim.max_bound); }
    if (dpsi.is_variable) { lb.push_back(dpsi.min_bound); ub.push_back(dpsi.max_bound); }
    if (dlt0.is_variable) { lb.push_back(dlt0.min_bound); ub.push_back(dlt0.max_bound); }
}

size_t KinematicParams::numVariable() const {
    size_t n = 0;
    if (omg0.is_variable) n++;
    if (gam0.is_variable) n++;
    if (phi0.is_variable) n++;
    if (psim.is_variable) n++;
    if (dpsi.is_variable) n++;
    if (dlt0.is_variable) n++;
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

KinematicParams KinematicParams::fromConfig(const Config& cfg) {
    KinematicParams params;
    params.omg0 = parseParam(cfg, "omg0");
    params.gam0 = parseParam(cfg, "gam0");
    params.phi0 = parseParam(cfg, "phi0");
    params.psim = parseParam(cfg, "psim");
    params.dpsi = parseParam(cfg, "dpsi");
    params.dlt0 = parseParam(cfg, "dlt0");
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

    double omg0 = kin.omg0.value;
    for (const auto& wc : phys.wings) {
        auto angleFunc = makeAngleFunc(
            kin.gam0.value, kin.phi0.value, kin.psim.value,
            kin.dpsi.value, kin.dlt0.value, wc.phaseOffset, omg0);
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
    double omg0 = kin.omg0.value;
    for (size_t i = 0; i < wings.size(); ++i) {
        const auto& wc = phys.wings[i];
        auto angleFunc = makeAngleFunc(
            kin.gam0.value, kin.phi0.value, kin.psim.value,
            kin.dpsi.value, kin.dlt0.value, wc.phaseOffset, omg0);
        wings[i].setAngleFunc(std::move(angleFunc));
    }
}

// Objective function (pre-allocated buffers version - use in hot paths)

double wingBeatAccel(const KinematicParams& kin, const PhysicalParams& phys,
                         OptimBuffers& buf, double ux, double uz, int N) {
    updateWingAngleFuncs(buf.wings, kin, phys);

    double omg0 = kin.omg0.value;
    double T = 2.0 * M_PI / omg0;
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

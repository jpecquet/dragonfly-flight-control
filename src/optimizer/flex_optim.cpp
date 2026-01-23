#include "optimizer/flex_optim.hpp"
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
    if (sig0.is_variable) names.push_back("sig0");
    return names;
}

std::vector<std::string> KinematicParams::allNames() const {
    return {"omg0", "gam0", "phi0", "psim", "dpsi", "dlt0", "sig0"};
}

std::vector<double> KinematicParams::allValues() const {
    return {omg0.value, gam0.value, phi0.value, psim.value, dpsi.value, dlt0.value, sig0.value};
}

std::vector<double> KinematicParams::variableValues() const {
    std::vector<double> values;
    if (omg0.is_variable) values.push_back(omg0.value);
    if (gam0.is_variable) values.push_back(gam0.value);
    if (phi0.is_variable) values.push_back(phi0.value);
    if (psim.is_variable) values.push_back(psim.value);
    if (dpsi.is_variable) values.push_back(dpsi.value);
    if (dlt0.is_variable) values.push_back(dlt0.value);
    if (sig0.is_variable) values.push_back(sig0.value);
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
    if (sig0.is_variable) sig0.value = x[i++];
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
    if (sig0.is_variable) { lb.push_back(sig0.min_bound); ub.push_back(sig0.max_bound); }
}

size_t KinematicParams::numVariable() const {
    size_t n = 0;
    if (omg0.is_variable) n++;
    if (gam0.is_variable) n++;
    if (phi0.is_variable) n++;
    if (psim.is_variable) n++;
    if (dpsi.is_variable) n++;
    if (dlt0.is_variable) n++;
    if (sig0.is_variable) n++;
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
    params.sig0 = parseParam(cfg, "sig0");
    return params;
}

// PhysicalParams implementation

PhysicalParams PhysicalParams::fromConfig(const Config& cfg) {
    PhysicalParams params;
    params.lb0_f = cfg.getDouble("lb0_f");
    params.lb0_h = cfg.getDouble("lb0_h");
    params.mu0_f = cfg.getDouble("mu0_f");
    params.mu0_h = cfg.getDouble("mu0_h");
    params.Cd0 = cfg.getDouble("Cd0");
    params.Cl0 = cfg.getDouble("Cl0");
    return params;
}

// Objective function

double flexWingBeatAccel(const KinematicParams& kin, const PhysicalParams& phys,
                         double ux, double uz, int N) {
    // Create angle functions using current kinematic values
    double omg0 = kin.omg0.value;

    auto angleFunc_f = makeForewingAngleFunc(
        kin.gam0.value, kin.phi0.value, kin.psim.value,
        kin.dpsi.value, kin.dlt0.value, omg0);
    auto angleFunc_h = makeHindwingAngleFunc(
        kin.gam0.value, kin.phi0.value, kin.psim.value,
        kin.dpsi.value, kin.dlt0.value, kin.sig0.value, omg0);

    // Create wings
    std::vector<Wing> wings;
    wings.emplace_back("fore", phys.mu0_f, phys.lb0_f, WingSide::Left, phys.Cd0, phys.Cl0, angleFunc_f);
    wings.emplace_back("fore", phys.mu0_f, phys.lb0_f, WingSide::Right, phys.Cd0, phys.Cl0, angleFunc_f);
    wings.emplace_back("hind", phys.mu0_h, phys.lb0_h, WingSide::Left, phys.Cd0, phys.Cl0, angleFunc_h);
    wings.emplace_back("hind", phys.mu0_h, phys.lb0_h, WingSide::Right, phys.Cd0, phys.Cl0, angleFunc_h);

    // Wing beat period
    double T = 2.0 * M_PI / omg0;
    double dt = T / N;

    // Trapezoidal integration
    Vec3 a_mean = Vec3::Zero();
    double t = 0.0;
    State state(0.0, 0.0, 0.0, ux, 0.0, uz);

    while (t < T) {
        std::vector<SingleWingVectors> wo1, wo2;

        StateDerivative d1 = equationOfMotion(t, state, wings, wo1);
        StateDerivative d2 = equationOfMotion(t + dt, state, wings, wo2);

        a_mean += (dt / T) * (d1.accel + d2.accel) / 2.0;
        t += dt;
    }

    return a_mean.squaredNorm();
}

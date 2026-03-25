#include "cmd_hover.hpp"
#include "integrator.hpp"
#include "optimize.hpp"
#include "sim_setup.hpp"

#include <highfive/H5File.hpp>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace {

struct ControlBounds {
    double min;
    double max;
};

struct GammaSweep {
    double min;
    double max;
    int n_points;
};

struct HoverConfig {
    // Morphology
    double omega;
    std::vector<WingConfig> wings;

    // Sweep
    GammaSweep gamma_sweep;

    // Control bounds (variable params only)
    ControlBounds phi_amp;
    ControlBounds psi_mean;
    ControlBounds psi_amp;
    ControlBounds psi_phase;

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
    wc.nu  = node["nu"].as<double>(0.0);
    wc.ar  = node["ar"].as<double>(0.0);
    wc.phase_offset = node["phase"].as<double>(0.0);
    wc.cone_angle = node["cone"].as<double>(0.0);
    wc.n_blade_elements = node["n_blade_elements"].as<int>(1);

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

HoverConfig parseConfig(const std::string& path) {
    YAML::Node yaml = YAML::LoadFile(path);
    HoverConfig cfg;

    cfg.omega = yaml["morphology"]["omega"].as<double>();
    for (const auto& w : yaml["morphology"]["wings"]) {
        cfg.wings.push_back(parseWingYAML(w));
    }

    auto gs = yaml["gamma_sweep"];
    cfg.gamma_sweep = {gs[0].as<double>(), gs[1].as<double>(), gs[2].as<int>()};

    auto cb = yaml["control_bounds"];
    cfg.phi_amp    = parseBounds(cb["phi_amp"]);
    cfg.psi_mean   = parseBounds(cb["psi_mean"]);
    cfg.psi_amp    = parseBounds(cb["psi_amp"]);
    cfg.psi_phase  = parseBounds(cb["psi_phase"]);

    auto alg = yaml["algorithm"];
    cfg.n_samples       = alg["n_samples"].as<int>(200);
    cfg.max_eval        = alg["max_eval"].as<int>(300);
    cfg.equilibrium_tol = alg["equilibrium_tol"].as<double>(1e-6);

    cfg.output = yaml["output"].as<std::string>("hover.h5");
    {
        namespace fs = std::filesystem;
        fs::path out(cfg.output);
        if (out.is_relative())
            cfg.output = (fs::path(path).parent_path() / out).string();
    }

    return cfg;
}

KinematicParam buildParam(const char* name, const ControlBounds& cb) {
    bool is_var = (cb.min != cb.max);
    double val = is_var ? (cb.min + cb.max) / 2.0 : cb.min;
    return {name, is_var, val, cb.min, cb.max};
}

KinematicParams buildKinTemplate(const HoverConfig& cfg, double gamma_value) {
    KinematicParams kin;
    kin.omega       = {"omega",       false, cfg.omega, cfg.omega, cfg.omega};
    kin.gamma_mean  = {"gamma_mean",  false, gamma_value, gamma_value, gamma_value};
    kin.gamma_amp   = {"gamma_amp",   false, 0.0, 0.0, 0.0};
    kin.gamma_phase = {"gamma_phase", false, 0.0, 0.0, 0.0};
    kin.phi_mean    = {"phi_mean",    false, 0.0, 0.0, 0.0};
    kin.phi_amp     = buildParam("phi_amp", cfg.phi_amp);
    kin.phi_phase   = {"phi_phase",   false, 0.0, 0.0, 0.0};
    kin.psi_mean    = buildParam("psi_mean", cfg.psi_mean);
    kin.psi_amp     = buildParam("psi_amp", cfg.psi_amp);
    kin.psi_phase   = buildParam("psi_phase", cfg.psi_phase);
    return kin;
}

PhysicalParams buildPhysParams(const HoverConfig& cfg) {
    PhysicalParams phys;
    phys.wings = cfg.wings;
    return phys;
}

}  // namespace

int runHover(const std::string& cfg_path) {
    HoverConfig cfg = parseConfig(cfg_path);
    PhysicalParams phys = buildPhysParams(cfg);

    const int n_pts = cfg.gamma_sweep.n_points;
    const double g_min = cfg.gamma_sweep.min;
    const double g_max = cfg.gamma_sweep.max;

    // Build one template to report variable param count
    KinematicParams kin0 = buildKinTemplate(cfg, g_min);
    std::cout << "Hover sweep: gamma_mean in [" << g_min << ", " << g_max << "] rad ("
              << n_pts << " points), "
              << kin0.numVariable() << " variable params per point, "
              << cfg.n_samples << " Sobol samples\n\n";

    // Accumulate results
    std::vector<double> gamma_values(n_pts);
    std::vector<double> power_values(n_pts);
    std::vector<double> residual_values(n_pts);
    std::vector<std::vector<double>> all_params(n_pts);

    constexpr double NaN = std::numeric_limits<double>::quiet_NaN();

    // Print header
    std::cout << std::setw(6)  << "i"
              << std::setw(12) << "gamma(deg)"
              << std::setw(14) << "power"
              << std::setw(14) << "residual"
              << "\n";
    std::cout << std::string(46, '-') << "\n";

    for (int i = 0; i < n_pts; ++i) {
        double gamma = (n_pts > 1)
            ? g_min + i * (g_max - g_min) / (n_pts - 1)
            : g_min;

        KinematicParams kin_template = buildKinTemplate(cfg, gamma);

        HoverSolution sol = findMinPowerHover(
            kin_template, phys, cfg.n_samples, cfg.max_eval, cfg.equilibrium_tol);

        bool found = !std::isnan(sol.residual);

        gamma_values[i]    = gamma;
        power_values[i]    = found ? sol.power    : NaN;
        residual_values[i] = found ? sol.residual : NaN;
        all_params[i]      = found ? sol.kin.allValues() : std::vector<double>(10, NaN);

        std::cout << std::setw(6)  << i
                  << std::setw(12) << std::fixed << std::setprecision(1) << gamma * 180.0 / M_PI
                  << std::setw(14) << std::scientific << std::setprecision(4)
                  << power_values[i]
                  << std::setw(14) << residual_values[i]
                  << (found ? "" : "  (no equilibrium)")
                  << "\n";
    }

    // Write HDF5 output
    std::cout << "\nWriting " << cfg.output << "...\n";
    HighFive::File file(cfg.output, HighFive::File::Overwrite);

    file.createDataSet("gamma_values", gamma_values);
    file.createDataSet("power", power_values);
    file.createDataSet("residual", residual_values);
    file.createDataSet("params", all_params);
    file.createDataSet("param_names", kin0.allNames());
    file.createDataSet("equilibrium_tol", cfg.equilibrium_tol);
    file.createDataSet("omega", cfg.omega);

    std::cout << "Done.\n";
    return 0;
}

// ============================================================================
// Hover control simulation
// ============================================================================

namespace {

struct HoverControlConfig {
    // Morphology
    double omega;
    std::vector<WingConfig> wings;

    // Fixed control parameters
    double gamma;       // Stroke plane angle (fixed)
    double psi_amp;     // Pitch amplitude (fixed)
    double psi_phase;   // Pitch phase (fixed)

    // Controller gains
    double Kp_z;        // phi_amp proportional gain on -u_z
    double Kp_x;        // psi_mean proportional gain on -u_x

    // Timing (in wingbeats)
    double avg_window;  // Velocity averaging window (wingbeats)
    double sense_delay; // Sensing-to-actuation delay (wingbeats)
    double muscle_tau;  // Neuromuscular lag time constant (wingbeats)

    // Saturation limits
    double phi_amp_min, phi_amp_max;
    double psi_mean_min, psi_mean_max;

    // Simulation
    int n_wingbeats;
    int steps_per_wingbeat;
    double ux0, uz0;

    // Equilibrium finder
    int n_samples;
    int max_eval;
    double equilibrium_tol;

    // Output
    std::string output;
};

HoverControlConfig parseControlConfig(const std::string& path) {
    YAML::Node yaml = YAML::LoadFile(path);
    HoverControlConfig cfg;

    cfg.omega = yaml["morphology"]["omega"].as<double>();
    for (const auto& w : yaml["morphology"]["wings"]) {
        cfg.wings.push_back(parseWingYAML(w));
    }

    auto ctrl = yaml["control"];
    cfg.gamma     = ctrl["gamma"].as<double>();
    cfg.psi_amp   = ctrl["psi_amp"].as<double>();
    cfg.psi_phase = ctrl["psi_phase"].as<double>();
    cfg.Kp_z      = ctrl["Kp_z"].as<double>(1.0);
    cfg.Kp_x      = ctrl["Kp_x"].as<double>(1.0);
    cfg.avg_window  = ctrl["avg_window"].as<double>(0.5);
    cfg.sense_delay = ctrl["sense_delay"].as<double>(0.25);
    cfg.muscle_tau  = ctrl["muscle_tau"].as<double>(0.5);
    cfg.phi_amp_min  = ctrl["phi_amp_min"].as<double>(0.15);
    cfg.phi_amp_max  = ctrl["phi_amp_max"].as<double>(1.0);
    cfg.psi_mean_min = ctrl["psi_mean_min"].as<double>(-0.2);
    cfg.psi_mean_max = ctrl["psi_mean_max"].as<double>(0.8);

    auto sim = yaml["simulation"];
    cfg.n_wingbeats        = sim["n_wingbeats"].as<int>(30);
    cfg.steps_per_wingbeat = sim["steps_per_wingbeat"].as<int>(200);
    cfg.ux0 = sim["ux0"].as<double>(0.0);
    cfg.uz0 = sim["uz0"].as<double>(0.0);

    auto alg = yaml["algorithm"];
    cfg.n_samples       = alg["n_samples"].as<int>(500);
    cfg.max_eval        = alg["max_eval"].as<int>(500);
    cfg.equilibrium_tol = alg["equilibrium_tol"].as<double>(1e-4);

    cfg.output = yaml["output"].as<std::string>("hover_control.h5");
    {
        namespace fs = std::filesystem;
        fs::path out(cfg.output);
        if (out.is_relative())
            cfg.output = (fs::path(path).parent_path() / out).string();
    }

    return cfg;
}

}  // namespace

int runHoverControl(const std::string& cfg_path) {
    HoverControlConfig cfg = parseControlConfig(cfg_path);

    // ---- Phase 0: Find equilibrium at the target gamma ----
    std::cout << "Phase 0: Finding equilibrium at gamma = "
              << cfg.gamma * 180.0 / M_PI << " deg...\n";

    // Build optimizer template with phi_amp, psi_mean as variable
    KinematicParams kin_template;
    kin_template.omega       = {"omega",       false, cfg.omega, cfg.omega, cfg.omega};
    kin_template.gamma_mean  = {"gamma_mean",  false, cfg.gamma, cfg.gamma, cfg.gamma};
    kin_template.gamma_amp   = {"gamma_amp",   false, 0.0, 0.0, 0.0};
    kin_template.gamma_phase = {"gamma_phase", false, 0.0, 0.0, 0.0};
    kin_template.phi_mean    = {"phi_mean",    false, 0.0, 0.0, 0.0};
    kin_template.phi_amp     = {"phi_amp",     true,
                                (cfg.phi_amp_min + cfg.phi_amp_max) / 2.0,
                                cfg.phi_amp_min, cfg.phi_amp_max};
    kin_template.phi_phase   = {"phi_phase",   false, 0.0, 0.0, 0.0};
    kin_template.psi_mean    = {"psi_mean",    true,
                                (cfg.psi_mean_min + cfg.psi_mean_max) / 2.0,
                                cfg.psi_mean_min, cfg.psi_mean_max};
    kin_template.psi_amp     = {"psi_amp",     false, cfg.psi_amp, cfg.psi_amp, cfg.psi_amp};
    kin_template.psi_phase   = {"psi_phase",   false, cfg.psi_phase, cfg.psi_phase, cfg.psi_phase};

    PhysicalParams phys;
    phys.wings = cfg.wings;

    HoverSolution sol = findMinPowerHover(
        kin_template, phys, cfg.n_samples, cfg.max_eval, cfg.equilibrium_tol);

    if (std::isnan(sol.residual)) {
        std::cerr << "Error: No hover equilibrium found at gamma = "
                  << cfg.gamma * 180.0 / M_PI << " deg\n";
        return 1;
    }

    const double phi_amp_eq  = sol.kin.phi_amp.value;
    const double psi_mean_eq = sol.kin.psi_mean.value;

    std::cout << "  Equilibrium: phi_amp = " << phi_amp_eq * 180.0 / M_PI
              << " deg, psi_mean = " << psi_mean_eq * 180.0 / M_PI
              << " deg, power = " << sol.power
              << ", residual = " << sol.residual << "\n";

    // ---- Phase 1: Build wings with controllable angle functions ----
    // Baseline harmonic series (first harmonic only)
    HarmonicSeries gamma_hs;
    gamma_hs.mean = cfg.gamma;

    HarmonicSeries phi_hs;
    phi_hs.mean = 0.0;
    phi_hs.amplitude_coeff = {phi_amp_eq};
    phi_hs.phase_coeff = {0.0};

    HarmonicSeries psi_hs;
    psi_hs.mean = psi_mean_eq;
    psi_hs.amplitude_coeff = {cfg.psi_amp};
    psi_hs.phase_coeff = {cfg.psi_phase};

    HarmonicSeries cone_hs;  // empty

    // Mutable control variables
    double gamma_mean_cmd = cfg.gamma;  // fixed, never changes
    double psi_mean_cmd   = psi_mean_eq;
    double phi_amp_cmd    = phi_amp_eq;

    // Create wings with per-wing harmonic series populated (needed for HDF5 output)
    SimKinematicParams sim_kin;
    sim_kin.omega = cfg.omega;
    sim_kin.harmonic_period_wingbeats = 1.0;
    sim_kin.n_harmonics = 1;
    sim_kin.gamma = gamma_hs;
    sim_kin.phi = phi_hs;
    sim_kin.psi = psi_hs;

    // Populate per-wing motion for output (WingConfig inherits MotionParams)
    for (auto& wc : cfg.wings) {
        wc.omega = cfg.omega;
        wc.harmonic_period_wingbeats = 1.0;
        wc.gamma = gamma_hs;
        wc.phi = phi_hs;
        wc.psi = psi_hs;
        wc.has_custom_motion = true;
    }

    auto wings = createWings(cfg.wings, sim_kin);

    // Install controlled angle functions on each wing
    for (size_t w = 0; w < wings.size(); ++w) {
        auto angleFunc = makeControlledAngleFunc(
            gamma_hs, phi_hs, psi_hs,
            cfg.wings[w].phase_offset,
            cfg.omega,
            1.0,  // harmonic_period_wingbeats
            cfg.gamma,      // gamma_mean_base
            psi_mean_eq,    // psi_mean_base
            phi_amp_eq,     // phi_amp_base
            phi_amp_eq,     // wing_phi_amp_base
            gamma_mean_cmd, // mutable ref (fixed)
            psi_mean_cmd,   // mutable ref (controlled)
            phi_amp_cmd,    // mutable ref (controlled)
            1e-12,
            cone_hs
        );
        wings[w].setAngleFunc(std::move(angleFunc));
    }

    // ---- Phase 2: Time integration with delayed P control ----
    //
    // Three physiologically motivated layers between sensing and actuation:
    //
    //   1. Rolling average — velocity is averaged over a configurable window
    //      (avg_window wingbeats), modelling optic-flow temporal integration.
    //
    //   2. Sensing delay — the averaged velocity is delayed by sense_delay
    //      wingbeats before reaching the controller, modelling neural latency.
    //
    //   3. Neuromuscular lag — the commanded values track their targets via
    //      a first-order lag with time constant muscle_tau wingbeats,
    //      modelling muscle activation and thorax compliance.

    const double Twb = 2.0 * M_PI / cfg.omega;
    const double dt  = Twb / cfg.steps_per_wingbeat;
    const int nsteps = cfg.n_wingbeats * cfg.steps_per_wingbeat;
    const double lag_alpha = 1.0 - std::exp(-dt / (cfg.muscle_tau * Twb));

    // Rolling average window size (in timesteps)
    const int avg_steps = std::max(1, static_cast<int>(std::round(
        cfg.avg_window * cfg.steps_per_wingbeat)));

    // Delay buffer size (in averaging windows)
    // The delay is measured from the *end* of an averaging window to when
    // the target is applied.  We discretize into averaging-window ticks.
    const int delay_ticks = std::max(0, static_cast<int>(std::round(
        cfg.sense_delay / cfg.avg_window)));

    std::cout << "Phase 1: Running hover control simulation for " << cfg.n_wingbeats
              << " wingbeats (Kp_z=" << cfg.Kp_z << ", Kp_x=" << cfg.Kp_x
              << ", avg=" << cfg.avg_window << "wb, delay=" << cfg.sense_delay
              << "wb, tau=" << cfg.muscle_tau << "wb)...\n";

    State state(Vec3::Zero(), Vec3(cfg.ux0, 0.0, cfg.uz0));

    // Standard simulation output (for Blender pipeline)
    auto output = initOutput(cfg.wings, sim_kin, nsteps);

    // Controller-specific time histories
    std::vector<double> phi_amp_out, psi_mean_out;
    phi_amp_out.reserve(nsteps + 1);
    psi_mean_out.reserve(nsteps + 1);

    std::vector<SingleWingVectors> scratch(wings.size());
    std::vector<SingleWingVectors> wing_data(wings.size());

    double t = 0.0;
    storeTimestep(output, t, state, wings, wing_data);
    phi_amp_out.push_back(phi_amp_cmd);
    psi_mean_out.push_back(psi_mean_cmd);

    // Rolling average accumulator
    double ux_accum = 0.0, uz_accum = 0.0;
    int accum_count = 0;

    // Delay ring buffer: stores averaged velocities waiting to be applied.
    // Entry 0 is the oldest (next to be consumed), back is the newest.
    std::deque<std::pair<double, double>> delay_buf;
    // Pre-fill with initial velocity so the controller has something before
    // the first real average is ready.
    for (int d = 0; d < delay_ticks + 1; ++d) {
        delay_buf.push_back({cfg.ux0, cfg.uz0});
    }

    // Control targets
    double phi_amp_target  = phi_amp_eq;
    double psi_mean_target = psi_mean_eq;

    for (int i = 0; i < nsteps; ++i) {
        // Accumulate velocity
        ux_accum += state.vel(0);
        uz_accum += state.vel(2);
        accum_count++;

        // End of averaging window: push average into delay buffer, pop oldest
        if (accum_count == avg_steps) {
            double ux_avg = ux_accum / accum_count;
            double uz_avg = uz_accum / accum_count;
            ux_accum = 0.0;
            uz_accum = 0.0;
            accum_count = 0;

            delay_buf.push_back({ux_avg, uz_avg});

            // Pop the delayed value and compute new targets
            auto [ux_delayed, uz_delayed] = delay_buf.front();
            delay_buf.pop_front();

            phi_amp_target  = phi_amp_eq  - cfg.Kp_z * uz_delayed;
            psi_mean_target = psi_mean_eq + cfg.Kp_x * ux_delayed;

            phi_amp_target  = std::max(cfg.phi_amp_min, std::min(cfg.phi_amp_max, phi_amp_target));
            psi_mean_target = std::max(cfg.psi_mean_min, std::min(cfg.psi_mean_max, psi_mean_target));
        }

        // Neuromuscular lag: smooth approach to target
        phi_amp_cmd  += lag_alpha * (phi_amp_target  - phi_amp_cmd);
        psi_mean_cmd += lag_alpha * (psi_mean_target - psi_mean_cmd);

        state = stepRK4(t, dt, state, wings, scratch);
        t += dt;
        storeTimestep(output, t, state, wings, wing_data);
        phi_amp_out.push_back(phi_amp_cmd);
        psi_mean_out.push_back(psi_mean_cmd);
    }

    std::cout << "  Final: ux = " << state.vel(0) << ", uz = " << state.vel(2)
              << ", x = " << state.pos(0) << ", z = " << state.pos(2) << "\n";

    // ---- Phase 3: Write HDF5 ----
    // Write standard simulation output (readable by Blender pipeline)
    std::cout << "Writing " << cfg.output << "...\n";
    writeHDF5(cfg.output, output, wings);

    // Append controller-specific datasets
    {
        HighFive::File file(cfg.output, HighFive::File::ReadWrite);
        file.createGroup("/hover_control");
        file.createDataSet("/hover_control/phi_amp", phi_amp_out);
        file.createDataSet("/hover_control/psi_mean", psi_mean_out);
        file.createDataSet("/hover_control/phi_amp_eq", phi_amp_eq);
        file.createDataSet("/hover_control/psi_mean_eq", psi_mean_eq);
        file.createDataSet("/hover_control/power_eq", sol.power);
        file.createDataSet("/hover_control/Kp_z", cfg.Kp_z);
        file.createDataSet("/hover_control/Kp_x", cfg.Kp_x);
        file.createDataSet("/hover_control/ux0", cfg.ux0);
        file.createDataSet("/hover_control/uz0", cfg.uz0);
    }

    std::cout << "Done.\n";
    return 0;
}

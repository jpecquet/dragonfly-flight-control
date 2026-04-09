#include "cmd_pursuit.hpp"
#include "integrator.hpp"
#include "optimize.hpp"
#include "sim_setup.hpp"
#include "trajectory.hpp"

#include <highfive/H5File.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <deque>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

namespace {

enum class FlightMode : int8_t { HOVER = 0, PURSUIT = 1 };

struct PursuitConfig {
    // Morphology
    double omega;
    std::vector<WingConfig> wings;

    // Fixed wing parameters (shared across modes)
    double gamma;       // Hover stroke plane angle
    double psi_amp;     // Pitch amplitude
    double psi_phase;   // Pitch phase

    // Sensing model
    double avg_window;  // Velocity averaging window (wingbeats)
    double sense_delay; // Neural processing delay (wingbeats)
    double muscle_tau;  // Neuromuscular lag time constant (wingbeats)

    // Hover mode
    double Kp_z;        // phi_amp gain on -u_z
    double Kp_x;        // psi_mean gain on -u_x
    double phi_amp_min, phi_amp_max;
    double psi_mean_min, psi_mean_max;

    // Pursuit mode
    double Kp_gamma;           // Gamma proportional gain
    double pursuit_phi_amp;    // Fixed phi_amp during pursuit
    double pursuit_psi_mean;   // Fixed psi_mean during pursuit
    double pursuit_psi_amp;    // Fixed psi_amp during pursuit
    double gamma_pursuit_base; // Neutral pursuit gamma (level flight)
    double gamma_min;          // Min gamma during pursuit
    double gamma_max;          // Max gamma during pursuit

    // Detection / transitions
    double fov_half_angle;  // Half-cone angle (rad)
    Vec3 fov_axis;          // Cone axis direction
    double intercept_distance; // Body lengths
    int post_intercept_wingbeats;

    // Target
    std::string trajectory_spec;

    // Simulation
    int max_wingbeats;
    int steps_per_wingbeat;

    // Equilibrium finder
    int n_samples;
    int max_eval;
    double equilibrium_tol;

    // Output
    std::string output;
};

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

PursuitConfig parsePursuitConfig(const std::string& path) {
    YAML::Node yaml = YAML::LoadFile(path);
    PursuitConfig cfg;

    cfg.omega = yaml["morphology"]["omega"].as<double>();
    for (const auto& w : yaml["morphology"]["wings"]) {
        cfg.wings.push_back(parseWingYAML(w));
    }

    auto ctrl = yaml["control"];
    cfg.gamma     = ctrl["gamma"].as<double>();
    cfg.psi_amp   = ctrl["psi_amp"].as<double>();
    cfg.psi_phase = ctrl["psi_phase"].as<double>();
    cfg.avg_window  = ctrl["avg_window"].as<double>(0.5);
    cfg.sense_delay = ctrl["sense_delay"].as<double>(0.25);
    cfg.muscle_tau  = ctrl["muscle_tau"].as<double>(0.5);

    auto hover = ctrl["hover"];
    cfg.Kp_z         = hover["Kp_z"].as<double>(0.7);
    cfg.Kp_x         = hover["Kp_x"].as<double>(1.2);
    cfg.phi_amp_min  = hover["phi_amp_min"].as<double>(0.20);
    cfg.phi_amp_max  = hover["phi_amp_max"].as<double>(0.90);
    cfg.psi_mean_min = hover["psi_mean_min"].as<double>(-0.10);
    cfg.psi_mean_max = hover["psi_mean_max"].as<double>(0.80);

    auto pursuit = ctrl["pursuit"];
    cfg.Kp_gamma           = pursuit["Kp_gamma"].as<double>(1.0);
    cfg.pursuit_phi_amp    = pursuit["phi_amp"].as<double>(1.2217);
    cfg.pursuit_psi_mean   = pursuit["psi_mean"].as<double>(0.0);
    cfg.pursuit_psi_amp    = pursuit["psi_amp"].as<double>(cfg.psi_amp);
    cfg.gamma_pursuit_base = pursuit["gamma_base"].as<double>(cfg.gamma);
    cfg.gamma_min          = pursuit["gamma_min"].as<double>(0.0);
    cfg.gamma_max          = pursuit["gamma_max"].as<double>(M_PI / 2.0);

    auto det = yaml["detection"];
    cfg.fov_half_angle = det["fov_half_angle"].as<double>(M_PI / 3.0);
    auto axis_node = det["fov_axis"];
    cfg.fov_axis = Vec3(axis_node[0].as<double>(),
                        axis_node[1].as<double>(),
                        axis_node[2].as<double>()).normalized();
    cfg.intercept_distance = det["intercept_distance"].as<double>(0.1);
    cfg.post_intercept_wingbeats = det["post_intercept_wingbeats"].as<int>(30);

    cfg.trajectory_spec = yaml["target"]["trajectory"].as<std::string>();

    auto sim = yaml["simulation"];
    cfg.max_wingbeats      = sim["max_wingbeats"].as<int>(200);
    cfg.steps_per_wingbeat = sim["steps_per_wingbeat"].as<int>(200);

    auto alg = yaml["algorithm"];
    cfg.n_samples       = alg["n_samples"].as<int>(500);
    cfg.max_eval        = alg["max_eval"].as<int>(500);
    cfg.equilibrium_tol = alg["equilibrium_tol"].as<double>(1e-4);

    cfg.output = yaml["output"].as<std::string>("pursuit.h5");
    {
        namespace fs = std::filesystem;
        fs::path out(cfg.output);
        if (out.is_relative())
            cfg.output = (fs::path(path).parent_path() / out).string();
    }

    return cfg;
}

}  // namespace

int runPursuit(const std::string& cfg_path) {
    PursuitConfig cfg = parsePursuitConfig(cfg_path);
    TrajectoryFunc trajectory = trajectories::parse(cfg.trajectory_spec);

    // ---- Phase 0: Find hover equilibrium ----
    std::cout << "Phase 0: Finding hover equilibrium at gamma = "
              << cfg.gamma * 180.0 / M_PI << " deg...\n";

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

    // ---- Phase 1: Build wings with controlled angle functions ----
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

    HarmonicSeries cone_hs;

    // Mutable control variables (all four are now actively controlled)
    double gamma_mean_cmd = cfg.gamma;
    double psi_mean_cmd   = psi_mean_eq;
    double phi_amp_cmd    = phi_amp_eq;
    double psi_amp_cmd    = cfg.psi_amp;  // Starts at hover value

    SimKinematicParams sim_kin;
    sim_kin.omega = cfg.omega;
    sim_kin.harmonic_period_wingbeats = 1.0;
    sim_kin.n_harmonics = 1;
    sim_kin.gamma = gamma_hs;
    sim_kin.phi = phi_hs;
    sim_kin.psi = psi_hs;

    for (auto& wc : cfg.wings) {
        wc.omega = cfg.omega;
        wc.harmonic_period_wingbeats = 1.0;
        wc.gamma = gamma_hs;
        wc.phi = phi_hs;
        wc.psi = psi_hs;
        wc.has_custom_motion = true;
    }

    auto wings = createWings(cfg.wings, sim_kin);

    for (size_t w = 0; w < wings.size(); ++w) {
        auto angleFunc = makeControlledAngleFunc(
            gamma_hs, phi_hs, psi_hs,
            cfg.wings[w].phase_offset,
            cfg.omega,
            1.0,
            cfg.gamma,       // gamma_mean_base
            psi_mean_eq,     // psi_mean_base
            phi_amp_eq,      // phi_amp_base
            phi_amp_eq,      // wing_phi_amp_base
            gamma_mean_cmd,  // mutable ref
            psi_mean_cmd,    // mutable ref
            phi_amp_cmd,     // mutable ref
            1e-12,
            cone_hs,
            PhiWaveform::Fourier, 0.0,
            PsiWaveform::Fourier, 0.0,
            &psi_amp_cmd     // mutable psi_amp
        );
        wings[w].setAngleFunc(std::move(angleFunc));
    }

    // ---- Phase 2: Dual-mode time integration ----
    const double Twb = 2.0 * M_PI / cfg.omega;
    const double dt  = Twb / cfg.steps_per_wingbeat;
    const int max_steps = cfg.max_wingbeats * cfg.steps_per_wingbeat;
    const double lag_alpha = 1.0 - std::exp(-dt / (cfg.muscle_tau * Twb));

    const int avg_steps = std::max(1, static_cast<int>(std::round(
        cfg.avg_window * cfg.steps_per_wingbeat)));
    const int delay_ticks = std::max(0, static_cast<int>(std::round(
        cfg.sense_delay / cfg.avg_window)));

    std::cout << "Phase 1: Running dual-mode simulation (max " << cfg.max_wingbeats
              << " wingbeats, avg=" << cfg.avg_window << "wb, delay=" << cfg.sense_delay
              << "wb, tau=" << cfg.muscle_tau << "wb)...\n";

    State state(Vec3::Zero(), Vec3::Zero());  // Start at hover equilibrium

    auto output = initOutput(cfg.wings, sim_kin, max_steps);

    // Controller time histories
    std::vector<int8_t> mode_out;
    std::vector<double> gamma_mean_out, phi_amp_out, psi_mean_out;
    std::vector<double> target_x_out, target_y_out, target_z_out;
    std::vector<double> distance_out;
    mode_out.reserve(max_steps + 1);
    gamma_mean_out.reserve(max_steps + 1);
    phi_amp_out.reserve(max_steps + 1);
    psi_mean_out.reserve(max_steps + 1);
    target_x_out.reserve(max_steps + 1);
    target_y_out.reserve(max_steps + 1);
    target_z_out.reserve(max_steps + 1);
    distance_out.reserve(max_steps + 1);

    std::vector<SingleWingVectors> scratch(wings.size());
    std::vector<SingleWingVectors> wing_data(wings.size());

    double ux_delayed = 0.0, uz_delayed = 0.0;  // delayed velocity for gamma control

    double t = 0.0;
    storeTimestep(output, t, state, wings, wing_data);

    auto storeControllerState = [&](FlightMode mode, const Vec3& target_pos, double dist) {
        mode_out.push_back(static_cast<int8_t>(mode));
        gamma_mean_out.push_back(gamma_mean_cmd);
        phi_amp_out.push_back(phi_amp_cmd);
        psi_mean_out.push_back(psi_mean_cmd);
        target_x_out.push_back(target_pos(0));
        target_y_out.push_back(target_pos(1));
        target_z_out.push_back(target_pos(2));
        distance_out.push_back(dist);
    };

    // Initial controller state
    TrajectoryPoint tp0 = trajectory(0.0);
    storeControllerState(FlightMode::HOVER, tp0.position, (tp0.position - state.pos).norm());

    // Sensing pipeline
    double ux_accum = 0.0, uz_accum = 0.0;
    int accum_count = 0;
    std::deque<std::pair<double, double>> delay_buf;
    for (int d = 0; d < delay_ticks + 1; ++d) {
        delay_buf.push_back({0.0, 0.0});  // Start at rest
    }

    // Control targets (initialized to hover equilibrium)
    double gamma_target    = cfg.gamma;
    double phi_amp_target  = phi_amp_eq;
    double psi_mean_target = psi_mean_eq;
    double psi_amp_target  = cfg.psi_amp;

    // Mode state
    FlightMode mode = FlightMode::HOVER;
    double intercept_time = -1.0;
    double detection_wingbeat = -1.0;
    double intercept_wingbeat = -1.0;

    for (int i = 0; i < max_steps; ++i) {
        // --- Velocity accumulation ---
        ux_accum += state.vel(0);
        uz_accum += state.vel(2);
        accum_count++;

        if (accum_count == avg_steps) {
            double ux_avg = ux_accum / accum_count;
            double uz_avg = uz_accum / accum_count;
            ux_accum = 0.0;
            uz_accum = 0.0;
            accum_count = 0;

            delay_buf.push_back({ux_avg, uz_avg});
            auto front = delay_buf.front();
            ux_delayed = front.first;
            uz_delayed = front.second;
            delay_buf.pop_front();

            // --- Mode-specific target computation ---
            if (mode == FlightMode::HOVER) {
                gamma_target    = cfg.gamma;
                phi_amp_target  = phi_amp_eq  - cfg.Kp_z * uz_delayed;
                psi_mean_target = psi_mean_eq + cfg.Kp_x * ux_delayed;
                psi_amp_target  = cfg.psi_amp;
                phi_amp_target  = std::clamp(phi_amp_target,  cfg.phi_amp_min,  cfg.phi_amp_max);
                psi_mean_target = std::clamp(psi_mean_target, cfg.psi_mean_min, cfg.psi_mean_max);
            } else {
                // Pursuit: fixed kinematics, gamma is updated per-step below
                phi_amp_target  = cfg.pursuit_phi_amp;
                psi_mean_target = cfg.pursuit_psi_mean;
                psi_amp_target  = cfg.pursuit_psi_amp;
            }
        }

        // --- Mode transitions ---
        TrajectoryPoint tp = trajectory(t);
        Vec3 los = tp.position - state.pos;
        double dist = los.norm();

        if (mode == FlightMode::HOVER && intercept_time < 0.0) {
            if (dist > 1e-12) {
                Vec3 los_hat = los / dist;
                double angle = std::acos(std::clamp(
                    static_cast<double>(los_hat.dot(cfg.fov_axis)), -1.0, 1.0));
                if (angle < cfg.fov_half_angle) {
                    mode = FlightMode::PURSUIT;
                    detection_wingbeat = t / Twb;
                    std::cout << "  PURSUIT at wingbeat " << detection_wingbeat << "\n";
                }
            }
        } else if (mode == FlightMode::PURSUIT) {
            if (dist < cfg.intercept_distance) {
                mode = FlightMode::HOVER;
                intercept_time = t;
                intercept_wingbeat = t / Twb;
                std::cout << "  INTERCEPT at wingbeat " << intercept_wingbeat
                          << " (distance = " << dist << ")\n";
            }
        }

        // --- End condition ---
        if (intercept_time > 0.0) {
            double wingbeats_since = (t - intercept_time) / Twb;
            if (wingbeats_since >= cfg.post_intercept_wingbeats) {
                break;
            }
        }

        // --- Pursuit gamma target: z-error controller ---
        // gamma = gamma_base - Kp * (z_target - z_dragonfly)
        // gamma_base (~52.5 deg): gamma when target is at same altitude.
        // Calibrated so that at detection (z_error = z_target ~ 3-5 bl), gamma ≈ 45-48 deg.
        // Target above (z_error > 0): gamma decreases → more vertical force → dragonfly climbs.
        // Target below (z_error < 0): gamma increases → less vertical force → dragonfly levels.
        if (mode == FlightMode::PURSUIT) {
            double z_error = los(2);  // z_target - z_dragonfly (positive = target above)
            gamma_target = std::clamp(cfg.gamma_pursuit_base - cfg.Kp_gamma * z_error,
                                      cfg.gamma_min, cfg.gamma_max);
        }

        // --- Neuromuscular lag on all four parameters ---
        gamma_mean_cmd += lag_alpha * (gamma_target   - gamma_mean_cmd);
        phi_amp_cmd    += lag_alpha * (phi_amp_target  - phi_amp_cmd);
        psi_mean_cmd   += lag_alpha * (psi_mean_target - psi_mean_cmd);
        psi_amp_cmd    += lag_alpha * (psi_amp_target  - psi_amp_cmd);

        // --- Physics ---
        state = stepRK4(t, dt, state, wings, scratch);
        t += dt;

        storeTimestep(output, t, state, wings, wing_data);
        storeControllerState(mode, tp.position, dist);
    }

    // Report
    std::cout << "  Final: ux = " << state.vel(0) << ", uz = " << state.vel(2)
              << ", x = " << state.pos(0) << ", z = " << state.pos(2) << "\n";
    if (intercept_time < 0.0) {
        std::cout << "  WARNING: No interception occurred within " << cfg.max_wingbeats
                  << " wingbeats.\n";
    }

    // ---- Phase 3: Write HDF5 ----
    std::cout << "Writing " << cfg.output << "...\n";
    writeHDF5(cfg.output, output, wings);

    // Append controller-specific datasets
    {
        HighFive::File file(cfg.output, HighFive::File::ReadWrite);
        file.createGroup("/pursuit_control");
        file.createDataSet("/pursuit_control/mode", mode_out);
        file.createDataSet("/pursuit_control/gamma_mean", gamma_mean_out);
        file.createDataSet("/pursuit_control/phi_amp", phi_amp_out);
        file.createDataSet("/pursuit_control/psi_mean", psi_mean_out);
        file.createDataSet("/pursuit_control/target_x", target_x_out);
        file.createDataSet("/pursuit_control/target_y", target_y_out);
        file.createDataSet("/pursuit_control/target_z", target_z_out);
        file.createDataSet("/pursuit_control/distance", distance_out);
        file.createDataSet("/pursuit_control/phi_amp_eq", phi_amp_eq);
        file.createDataSet("/pursuit_control/psi_mean_eq", psi_mean_eq);
        file.createDataSet("/pursuit_control/detection_wingbeat", detection_wingbeat);
        file.createDataSet("/pursuit_control/intercept_wingbeat", intercept_wingbeat);
    }

    std::cout << "Done.\n";
    return 0;
}

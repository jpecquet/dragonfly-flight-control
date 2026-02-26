#include "sim_setup.hpp"

#include <highfive/H5Easy.hpp>
#include <highfive/H5File.hpp>
#include <algorithm>
#include <limits>
#include <stdexcept>

namespace {

// Convert a vector of Vec3 to an Nx3 Eigen matrix for HDF5 writing
Eigen::MatrixXd vec3sToMatrix(const std::vector<Vec3>& vecs) {
    Eigen::MatrixXd mat(vecs.size(), 3);
    for (size_t i = 0; i < vecs.size(); ++i) {
        mat.row(i) = vecs[i].transpose();
    }
    return mat;
}

// Extract a specific Vec3 member from per-timestep wing data into an Nx3 matrix
Eigen::MatrixXd toMatrix(
    const std::vector<std::vector<SingleWingVectors>>& wing_data,
    size_t wing_index,
    Vec3 SingleWingVectors::* vec_member
) {
    size_t n = wing_data.size();
    Eigen::MatrixXd result = Eigen::MatrixXd::Zero(static_cast<Eigen::Index>(n), 3);

    for (size_t i = 0; i < n; ++i) {
        if (wing_index < wing_data[i].size()) {
            result.row(i) = (wing_data[i][wing_index].*vec_member).transpose();
        }
    }
    return result;
}

std::vector<double> toScalarSeries(
    const std::vector<std::vector<SingleWingVectors>>& wing_data,
    size_t wing_index,
    double SingleWingVectors::* scalar_member
) {
    const size_t n = wing_data.size();
    std::vector<double> result(n, 0.0);
    for (size_t i = 0; i < n; ++i) {
        if (wing_index < wing_data[i].size()) {
            result[i] = wing_data[i][wing_index].*scalar_member;
        }
    }
    return result;
}

size_t bladeCountForWing(
    const std::vector<std::vector<SingleWingVectors>>& wing_data,
    size_t wing_index,
    const std::vector<Vec3> SingleWingVectors::* vec_member
) {
    size_t count = 0;
    for (const auto& step : wing_data) {
        if (wing_index >= step.size()) {
            continue;
        }
        const size_t n = (step[wing_index].*vec_member).size();
        if (count == 0) {
            count = n;
        } else if (n != count) {
            throw std::runtime_error("Inconsistent per-blade vector count across timesteps");
        }
    }
    return count;
}

size_t bladeCountForWing(
    const std::vector<std::vector<SingleWingVectors>>& wing_data,
    size_t wing_index,
    const std::vector<double> SingleWingVectors::* scalar_member
) {
    size_t count = 0;
    for (const auto& step : wing_data) {
        if (wing_index >= step.size()) {
            continue;
        }
        const size_t n = (step[wing_index].*scalar_member).size();
        if (count == 0) {
            count = n;
        } else if (n != count) {
            throw std::runtime_error("Inconsistent per-blade scalar count across timesteps");
        }
    }
    return count;
}

Eigen::MatrixXd toBladeVecMatrixFlattened(
    const std::vector<std::vector<SingleWingVectors>>& wing_data,
    size_t wing_index,
    const std::vector<Vec3> SingleWingVectors::* vec_member,
    size_t blade_count
) {
    const size_t n_steps = wing_data.size();
    Eigen::MatrixXd result = Eigen::MatrixXd::Constant(
        static_cast<Eigen::Index>(n_steps),
        static_cast<Eigen::Index>(blade_count * 3),
        std::numeric_limits<double>::quiet_NaN()
    );
    for (size_t t = 0; t < n_steps; ++t) {
        if (wing_index >= wing_data[t].size()) {
            continue;
        }
        const auto& vecs = wing_data[t][wing_index].*vec_member;
        if (vecs.size() != blade_count) {
            throw std::runtime_error("Per-blade vector count mismatch while serializing HDF5");
        }
        for (size_t b = 0; b < blade_count; ++b) {
            const Eigen::Index col = static_cast<Eigen::Index>(3 * b);
            result(static_cast<Eigen::Index>(t), col + 0) = vecs[b](0);
            result(static_cast<Eigen::Index>(t), col + 1) = vecs[b](1);
            result(static_cast<Eigen::Index>(t), col + 2) = vecs[b](2);
        }
    }
    return result;
}

Eigen::MatrixXd toBladeScalarMatrix(
    const std::vector<std::vector<SingleWingVectors>>& wing_data,
    size_t wing_index,
    const std::vector<double> SingleWingVectors::* scalar_member,
    size_t blade_count
) {
    const size_t n_steps = wing_data.size();
    Eigen::MatrixXd result = Eigen::MatrixXd::Constant(
        static_cast<Eigen::Index>(n_steps),
        static_cast<Eigen::Index>(blade_count),
        std::numeric_limits<double>::quiet_NaN()
    );
    for (size_t t = 0; t < n_steps; ++t) {
        if (wing_index >= wing_data[t].size()) {
            continue;
        }
        const auto& vals = wing_data[t][wing_index].*scalar_member;
        if (vals.size() != blade_count) {
            throw std::runtime_error("Per-blade scalar count mismatch while serializing HDF5");
        }
        for (size_t b = 0; b < blade_count; ++b) {
            result(static_cast<Eigen::Index>(t), static_cast<Eigen::Index>(b)) = vals[b];
        }
    }
    return result;
}

std::string wingGroupName(const Wing& wing) {
    std::string suffix = (wing.side() == WingSide::Left) ? "_left" : "_right";
    return wing.name() + suffix;
}

void writeMatrixRow(Eigen::MatrixXd& matrix, Eigen::Index row,
                    const std::vector<double>& values) {
    const Eigen::Index ncols = matrix.cols();
    if (values.size() != static_cast<size_t>(ncols)) {
        throw std::runtime_error("writeMatrixRow: coefficient size does not match matrix columns");
    }
    for (Eigen::Index col = 0; col < ncols; ++col) {
        matrix(row, col) = values[static_cast<size_t>(col)];
    }
}

// Write a HarmonicSeries set for each angle across all wings
void writeAngleHarmonics(HighFive::File& file, const std::string& prefix,
                         const std::vector<WingConfig>& wingConfigs,
                         HarmonicSeries WingConfig::* angle, size_t n_harmonics) {
    const size_t n = wingConfigs.size();
    std::vector<double> mean_vals;
    mean_vals.reserve(n);
    Eigen::MatrixXd amp_mat(n, n_harmonics);
    Eigen::MatrixXd phase_mat(n, n_harmonics);

    for (size_t i = 0; i < n; ++i) {
        const auto& series = wingConfigs[i].*angle;
        mean_vals.push_back(series.mean);
        writeMatrixRow(amp_mat, static_cast<Eigen::Index>(i), series.amplitude_coeff);
        writeMatrixRow(phase_mat, static_cast<Eigen::Index>(i), series.phase_coeff);
    }

    H5Easy::dump(file, "/parameters/wings/" + prefix + "_mean", mean_vals);
    file.createDataSet("/parameters/wings/" + prefix + "_amp", amp_mat);
    file.createDataSet("/parameters/wings/" + prefix + "_phase", phase_mat);
}

} // namespace

void writeHDF5(const std::string& filename, const SimulationOutput& output,
               const std::vector<Wing>& wings) {
    HighFive::File file(filename, HighFive::File::Overwrite);

    // Write kinematic parameters
    file.createGroup("/parameters");
    const auto& k = output.kin;
    H5Easy::dump(file, "/parameters/n_harmonics", k.n_harmonics);
    const std::pair<const char*, double> scalar_params[] = {
        {"omega", k.omega},
        {"harmonic_period_wingbeats", k.harmonic_period_wingbeats},
        {"gamma_mean", k.gamma.mean},
        {"phi_mean", k.phi.mean},
        {"psi_mean", k.psi.mean},
        {"cone_mean", k.cone.mean},
    };
    for (auto& [name, val] : scalar_params) {
        H5Easy::dump(file, std::string("/parameters/") + name, val);
    }
    H5Easy::dump(file, "/parameters/gamma_amp", k.gamma.amplitude_coeff);
    H5Easy::dump(file, "/parameters/gamma_phase", k.gamma.phase_coeff);
    H5Easy::dump(file, "/parameters/phi_amp", k.phi.amplitude_coeff);
    H5Easy::dump(file, "/parameters/phi_phase", k.phi.phase_coeff);
    H5Easy::dump(file, "/parameters/psi_amp", k.psi.amplitude_coeff);
    H5Easy::dump(file, "/parameters/psi_phase", k.psi.phase_coeff);
    H5Easy::dump(file, "/parameters/cone_amp", k.cone.amplitude_coeff);
    H5Easy::dump(file, "/parameters/cone_phase", k.cone.phase_coeff);

    // Write wing configurations
    size_t num_configs = output.wingConfigs.size();
    file.createGroup("/parameters/wings");
    H5Easy::dump(file, "/parameters/wings/count", static_cast<int>(num_configs));

    // Store wing config arrays
    std::vector<std::string> names;
    std::vector<int> sides;
    std::vector<double> mu0_vals, lb0_vals, Cd_min_vals, Cd_max_vals, Cd_alpha_neutral_vals, Cl0_vals, phase_vals, cone_vals;
    std::vector<std::string> drag_model_vals, lift_model_vals;
    std::vector<double> cl_alpha_slope_vals, cl_alpha_neutral_vals, cl_min_vals, cl_max_vals;
    std::vector<int> n_blade_elements_vals;
    std::vector<int> has_psi_twist_h1_vals;
    std::vector<double> psi_twist_h1_root_vals, psi_twist_ref_eta_vals;

    for (const auto& wc : output.wingConfigs) {
        names.push_back(wc.name + (wc.side == WingSide::Left ? "_left" : "_right"));
        sides.push_back(wc.side == WingSide::Left ? 0 : 1);
        mu0_vals.push_back(wc.mu0);
        lb0_vals.push_back(wc.lb0);
        drag_model_vals.push_back(toString(wc.drag_model));
        lift_model_vals.push_back(toString(wc.lift_model));
        Cd_min_vals.push_back(wc.Cd_min);
        Cd_max_vals.push_back(wc.Cd_max);
        Cd_alpha_neutral_vals.push_back(wc.Cd_alpha_neutral);
        Cl0_vals.push_back(wc.Cl0);
        cl_alpha_slope_vals.push_back(wc.Cl_alpha_slope);
        cl_alpha_neutral_vals.push_back(wc.Cl_alpha_neutral);
        cl_min_vals.push_back(wc.Cl_min);
        cl_max_vals.push_back(wc.Cl_max);
        phase_vals.push_back(wc.phase_offset);
        cone_vals.push_back(wc.cone_angle);
        n_blade_elements_vals.push_back(wc.n_blade_elements);
        has_psi_twist_h1_vals.push_back(wc.has_psi_twist_h1 ? 1 : 0);
        psi_twist_h1_root_vals.push_back(wc.psi_twist_h1_root);
        psi_twist_ref_eta_vals.push_back(wc.psi_twist_ref_eta);
    }

    H5Easy::dump(file, "/parameters/wings/names", names);
    H5Easy::dump(file, "/parameters/wings/sides", sides);
    H5Easy::dump(file, "/parameters/wings/mu0", mu0_vals);
    H5Easy::dump(file, "/parameters/wings/lb0", lb0_vals);
    H5Easy::dump(file, "/parameters/wings/drag_model", drag_model_vals);
    H5Easy::dump(file, "/parameters/wings/lift_model", lift_model_vals);
    H5Easy::dump(file, "/parameters/wings/Cd_min", Cd_min_vals);
    // Backward-compatibility alias for older postprocessing code.
    H5Easy::dump(file, "/parameters/wings/Cd0", Cd_min_vals);
    H5Easy::dump(file, "/parameters/wings/Cd_max", Cd_max_vals);
    H5Easy::dump(file, "/parameters/wings/Cd_alpha_neutral", Cd_alpha_neutral_vals);
    H5Easy::dump(file, "/parameters/wings/Cl0", Cl0_vals);
    H5Easy::dump(file, "/parameters/wings/Cl_alpha_slope", cl_alpha_slope_vals);
    H5Easy::dump(file, "/parameters/wings/Cl_alpha_neutral", cl_alpha_neutral_vals);
    H5Easy::dump(file, "/parameters/wings/Cl_min", cl_min_vals);
    H5Easy::dump(file, "/parameters/wings/Cl_max", cl_max_vals);
    H5Easy::dump(file, "/parameters/wings/phase_offset", phase_vals);
    H5Easy::dump(file, "/parameters/wings/cone_angle", cone_vals);
    H5Easy::dump(file, "/parameters/wings/n_blade_elements", n_blade_elements_vals);
    H5Easy::dump(file, "/parameters/wings/has_psi_twist_h1", has_psi_twist_h1_vals);
    H5Easy::dump(file, "/parameters/wings/psi_twist_h1_root", psi_twist_h1_root_vals);
    H5Easy::dump(file, "/parameters/wings/psi_twist_ref_eta", psi_twist_ref_eta_vals);

    // Optional per-wing kinematic parameters (resolved values actually used in simulation).
    const size_t n_harmonics = static_cast<size_t>(std::max(0, output.kin.n_harmonics));
    bool have_wing_motion = n_harmonics > 0;
    if (have_wing_motion) {
        for (const auto& wc : output.wingConfigs) {
            for (const auto* s : {&wc.gamma, &wc.phi, &wc.psi, &wc.cone}) {
                if (s->amplitude_coeff.size() != n_harmonics || s->phase_coeff.size() != n_harmonics) {
                    have_wing_motion = false;
                    break;
                }
            }
            if (!have_wing_motion) break;
        }
    }

    if (have_wing_motion) {
        std::vector<int> has_custom_motion;
        std::vector<double> omega_vals, harmonic_period_vals;
        has_custom_motion.reserve(num_configs);
        omega_vals.reserve(num_configs);
        harmonic_period_vals.reserve(num_configs);

        for (const auto& wc : output.wingConfigs) {
            has_custom_motion.push_back(wc.has_custom_motion ? 1 : 0);
            omega_vals.push_back(wc.omega);
            harmonic_period_vals.push_back(wc.harmonic_period_wingbeats);
        }

        H5Easy::dump(file, "/parameters/wings/has_custom_motion", has_custom_motion);
        H5Easy::dump(file, "/parameters/wings/omega", omega_vals);
        H5Easy::dump(file, "/parameters/wings/harmonic_period_wingbeats", harmonic_period_vals);

        writeAngleHarmonics(file, "gamma", output.wingConfigs, &WingConfig::gamma, n_harmonics);
        writeAngleHarmonics(file, "phi", output.wingConfigs, &WingConfig::phi, n_harmonics);
        writeAngleHarmonics(file, "psi", output.wingConfigs, &WingConfig::psi, n_harmonics);
        writeAngleHarmonics(file, "cone", output.wingConfigs, &WingConfig::cone, n_harmonics);
    }

    // Write time
    file.createDataSet("/time", output.time);

    // Write states as contiguous Nx6 matrix [pos, vel]
    size_t n_states = output.states.size();
    Eigen::MatrixXd states_matrix(n_states, 6);
    for (size_t i = 0; i < n_states; ++i) {
        states_matrix.row(i) << output.states[i].pos.transpose(),
                                output.states[i].vel.transpose();
    }
    file.createDataSet("/state", states_matrix);

    // Write wing data (variable number of wings)
    file.createGroup("/wings");

    size_t num_wings = wings.size();
    H5Easy::dump(file, "/wings/num_wings", static_cast<int>(num_wings));

    for (size_t i = 0; i < num_wings; ++i) {
        std::string group = "/wings/" + wingGroupName(wings[i]);
        file.createGroup(group);
        file.createDataSet(group + "/e_s", toMatrix(output.wing_data, i, &SingleWingVectors::e_s));
        file.createDataSet(group + "/e_r", toMatrix(output.wing_data, i, &SingleWingVectors::e_r));
        file.createDataSet(group + "/e_c", toMatrix(output.wing_data, i, &SingleWingVectors::e_c));
        file.createDataSet(group + "/lift", toMatrix(output.wing_data, i, &SingleWingVectors::lift));
        file.createDataSet(group + "/drag", toMatrix(output.wing_data, i, &SingleWingVectors::drag));
        H5Easy::dump(file, group + "/alpha", toScalarSeries(output.wing_data, i, &SingleWingVectors::alpha));

        const size_t blade_count = wings[i].bladeEta().size();
        file.createGroup(group + "/blade");
        H5Easy::dump(file, group + "/blade/eta", wings[i].bladeEta());
        if (blade_count > 0) {
            if (bladeCountForWing(output.wing_data, i, &SingleWingVectors::blade_e_c) != blade_count) {
                throw std::runtime_error("Per-blade output count does not match wing bladeEta size");
            }
            if (bladeCountForWing(output.wing_data, i, &SingleWingVectors::blade_alpha) != blade_count) {
                throw std::runtime_error("Per-blade alpha count does not match wing bladeEta size");
            }
            file.createDataSet(
                group + "/blade/e_s",
                toBladeVecMatrixFlattened(output.wing_data, i, &SingleWingVectors::blade_e_s, blade_count)
            );
            file.createDataSet(
                group + "/blade/e_r",
                toBladeVecMatrixFlattened(output.wing_data, i, &SingleWingVectors::blade_e_r, blade_count)
            );
            file.createDataSet(
                group + "/blade/e_c",
                toBladeVecMatrixFlattened(output.wing_data, i, &SingleWingVectors::blade_e_c, blade_count)
            );
            file.createDataSet(
                group + "/blade/lift",
                toBladeVecMatrixFlattened(output.wing_data, i, &SingleWingVectors::blade_lift, blade_count)
            );
            file.createDataSet(
                group + "/blade/drag",
                toBladeVecMatrixFlattened(output.wing_data, i, &SingleWingVectors::blade_drag, blade_count)
            );
            file.createDataSet(
                group + "/blade/alpha",
                toBladeScalarMatrix(output.wing_data, i, &SingleWingVectors::blade_alpha, blade_count)
            );
        }
    }

    // Write controller data if tracking was active
    if (output.controller_active) {
        if (output.target_positions.size() != output.position_errors.size()) {
            throw std::runtime_error("writeHDF5: target_positions and position_errors size mismatch");
        }

        file.createGroup("/controller");
        H5Easy::dump(file, "/controller/active", 1);

        // Target positions and tracking errors (Nx3 matrices)
        file.createDataSet("/controller/target_position", vec3sToMatrix(output.target_positions));
        file.createDataSet("/controller/position_error", vec3sToMatrix(output.position_errors));

        // Parameter time histories
        file.createDataSet("/controller/gamma_mean", output.param_gamma_mean);
        file.createDataSet("/controller/psi_mean", output.param_psi_mean);
        file.createDataSet("/controller/phi_amp", output.param_phi_amp);
    }
}

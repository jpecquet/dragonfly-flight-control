#include "output.hpp"

#include <highfive/H5Easy.hpp>
#include <highfive/H5File.hpp>
#include <algorithm>

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
    Eigen::MatrixXd result(n, 3);

    for (size_t i = 0; i < n; ++i) {
        if (wing_index < wing_data[i].size()) {
            result.row(i) = (wing_data[i][wing_index].*vec_member).transpose();
        }
    }
    return result;
}

std::string wingGroupName(const Wing& wing) {
    std::string suffix = (wing.side() == WingSide::Left) ? "_left" : "_right";
    return wing.name() + suffix;
}

} // namespace

void writeHDF5(const std::string& filename, const SimulationOutput& output,
               const std::vector<Wing>& wings) {
    HighFive::File file(filename, HighFive::File::Overwrite);

    // Write kinematic parameters
    file.createGroup("/parameters");
    const auto& k = output.kin;
    H5Easy::dump(file, "/parameters/n_harmonics", k.n_harmonics);
    const std::pair<const char*, double> kin_params[] = {
        {"omega", k.omega}, {"gamma_mean", k.gamma_mean},
        {"phi_mean", k.phi_mean}, {"psi_mean", k.psi_mean},
    };
    for (auto& [name, val] : kin_params) {
        H5Easy::dump(file, std::string("/parameters/") + name, val);
    }
    H5Easy::dump(file, "/parameters/gamma_cos", k.gamma_cos);
    H5Easy::dump(file, "/parameters/gamma_sin", k.gamma_sin);
    H5Easy::dump(file, "/parameters/phi_cos", k.phi_cos);
    H5Easy::dump(file, "/parameters/phi_sin", k.phi_sin);
    H5Easy::dump(file, "/parameters/psi_cos", k.psi_cos);
    H5Easy::dump(file, "/parameters/psi_sin", k.psi_sin);

    // Write wing configurations
    size_t num_configs = output.wingConfigs.size();
    file.createGroup("/parameters/wings");
    H5Easy::dump(file, "/parameters/wings/count", static_cast<int>(num_configs));

    // Store wing config arrays
    std::vector<std::string> names;
    std::vector<int> sides;
    std::vector<double> mu0_vals, lb0_vals, Cd0_vals, Cl0_vals, phase_vals;

    for (const auto& wc : output.wingConfigs) {
        names.push_back(wc.name + (wc.side == WingSide::Left ? "_left" : "_right"));
        sides.push_back(wc.side == WingSide::Left ? 0 : 1);
        mu0_vals.push_back(wc.mu0);
        lb0_vals.push_back(wc.lb0);
        Cd0_vals.push_back(wc.Cd0);
        Cl0_vals.push_back(wc.Cl0);
        phase_vals.push_back(wc.phaseOffset);
    }

    H5Easy::dump(file, "/parameters/wings/names", names);
    H5Easy::dump(file, "/parameters/wings/sides", sides);
    H5Easy::dump(file, "/parameters/wings/mu0", mu0_vals);
    H5Easy::dump(file, "/parameters/wings/lb0", lb0_vals);
    H5Easy::dump(file, "/parameters/wings/Cd0", Cd0_vals);
    H5Easy::dump(file, "/parameters/wings/Cl0", Cl0_vals);
    H5Easy::dump(file, "/parameters/wings/phase_offset", phase_vals);

    // Optional per-wing kinematic parameters (resolved values actually used in simulation).
    const size_t n_harmonics = static_cast<size_t>(std::max(0, output.kin.n_harmonics));
    bool have_wing_motion = n_harmonics > 0;
    if (have_wing_motion) {
        for (const auto& wc : output.wingConfigs) {
            if (wc.gamma_cos.size() != n_harmonics || wc.gamma_sin.size() != n_harmonics ||
                wc.phi_cos.size() != n_harmonics || wc.phi_sin.size() != n_harmonics ||
                wc.psi_cos.size() != n_harmonics || wc.psi_sin.size() != n_harmonics) {
                have_wing_motion = false;
                break;
            }
        }
    }

    if (have_wing_motion) {
        std::vector<int> has_custom_motion;
        std::vector<double> omega_vals, gamma_mean_vals, phi_mean_vals, psi_mean_vals;
        has_custom_motion.reserve(num_configs);
        omega_vals.reserve(num_configs);
        gamma_mean_vals.reserve(num_configs);
        phi_mean_vals.reserve(num_configs);
        psi_mean_vals.reserve(num_configs);

        Eigen::MatrixXd gamma_cos_mat(num_configs, n_harmonics);
        Eigen::MatrixXd gamma_sin_mat(num_configs, n_harmonics);
        Eigen::MatrixXd phi_cos_mat(num_configs, n_harmonics);
        Eigen::MatrixXd phi_sin_mat(num_configs, n_harmonics);
        Eigen::MatrixXd psi_cos_mat(num_configs, n_harmonics);
        Eigen::MatrixXd psi_sin_mat(num_configs, n_harmonics);

        for (size_t i = 0; i < num_configs; ++i) {
            const auto& wc = output.wingConfigs[i];
            has_custom_motion.push_back(wc.hasCustomMotion ? 1 : 0);
            omega_vals.push_back(wc.omega);
            gamma_mean_vals.push_back(wc.gamma_mean);
            phi_mean_vals.push_back(wc.phi_mean);
            psi_mean_vals.push_back(wc.psi_mean);
            for (size_t k = 0; k < n_harmonics; ++k) {
                gamma_cos_mat(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(k)) = wc.gamma_cos[k];
                gamma_sin_mat(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(k)) = wc.gamma_sin[k];
                phi_cos_mat(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(k)) = wc.phi_cos[k];
                phi_sin_mat(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(k)) = wc.phi_sin[k];
                psi_cos_mat(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(k)) = wc.psi_cos[k];
                psi_sin_mat(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(k)) = wc.psi_sin[k];
            }
        }

        H5Easy::dump(file, "/parameters/wings/has_custom_motion", has_custom_motion);
        H5Easy::dump(file, "/parameters/wings/omega", omega_vals);
        H5Easy::dump(file, "/parameters/wings/gamma_mean", gamma_mean_vals);
        H5Easy::dump(file, "/parameters/wings/phi_mean", phi_mean_vals);
        H5Easy::dump(file, "/parameters/wings/psi_mean", psi_mean_vals);
        file.createDataSet("/parameters/wings/gamma_cos", gamma_cos_mat);
        file.createDataSet("/parameters/wings/gamma_sin", gamma_sin_mat);
        file.createDataSet("/parameters/wings/phi_cos", phi_cos_mat);
        file.createDataSet("/parameters/wings/phi_sin", phi_sin_mat);
        file.createDataSet("/parameters/wings/psi_cos", psi_cos_mat);
        file.createDataSet("/parameters/wings/psi_sin", psi_sin_mat);
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

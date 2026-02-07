#include "output.hpp"

#include <highfive/H5Easy.hpp>
#include <highfive/H5File.hpp>

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
    const std::pair<const char*, double> kin_params[] = {
        {"omega", k.omega}, {"gamma_mean", k.gamma_mean},
        {"gamma_amp", k.gamma_amp}, {"gamma_phase", k.gamma_phase},
        {"phi_amp", k.phi_amp}, {"psi_mean", k.psi_mean},
        {"psi_amp", k.psi_amp}, {"psi_phase", k.psi_phase},
    };
    for (auto& [name, val] : kin_params) {
        H5Easy::dump(file, std::string("/parameters/") + name, val);
    }

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

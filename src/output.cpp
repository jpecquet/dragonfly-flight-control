#include "output.hpp"

#include <highfive/H5Easy.hpp>
#include <highfive/H5File.hpp>

namespace {

// Helper to extract Vec3 data into a pre-allocated contiguous Eigen matrix
// Returns Nx3 matrix for efficient HDF5 writing
Eigen::MatrixXd toMatrix(
    const std::vector<std::vector<SingleWingVectors>>& wing_data,
    size_t wing_index,
    Vec3 SingleWingVectors::* vec_member
) {
    size_t n = wing_data.size();
    Eigen::MatrixXd result(n, 3);

    for (size_t i = 0; i < n; ++i) {
        if (wing_index < wing_data[i].size()) {
            const Vec3& v = wing_data[i][wing_index].*vec_member;
            result(i, 0) = v.x();
            result(i, 1) = v.y();
            result(i, 2) = v.z();
        }
    }
    return result;
}

} // namespace

std::string wingGroupName(const Wing& wing) {
    std::string suffix = (wing.side() == WingSide::Left) ? "_left" : "_right";
    return wing.name() + suffix;
}

void writeHDF5(const std::string& filename, const SimulationOutput& output,
               const std::vector<Wing>& wings) {
    HighFive::File file(filename, HighFive::File::Overwrite);

    // Write kinematic parameters
    file.createGroup("/parameters");
    H5Easy::dump(file, "/parameters/omega", output.omega);
    H5Easy::dump(file, "/parameters/gamma_mean", output.gamma_mean);
    H5Easy::dump(file, "/parameters/gamma_amp", output.gamma_amp);
    H5Easy::dump(file, "/parameters/gamma_phase", output.gamma_phase);
    H5Easy::dump(file, "/parameters/phi_amp", output.phi_amp);
    H5Easy::dump(file, "/parameters/psi_mean", output.psi_mean);
    H5Easy::dump(file, "/parameters/psi_amp", output.psi_amp);
    H5Easy::dump(file, "/parameters/psi_phase", output.psi_phase);

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

    // Write states as contiguous Nx6 matrix
    size_t n_states = output.states.size();
    Eigen::MatrixXd states_matrix(n_states, 6);
    for (size_t i = 0; i < n_states; ++i) {
        const State& s = output.states[i];
        states_matrix(i, 0) = s.pos.x();
        states_matrix(i, 1) = s.pos.y();
        states_matrix(i, 2) = s.pos.z();
        states_matrix(i, 3) = s.vel.x();
        states_matrix(i, 4) = s.vel.y();
        states_matrix(i, 5) = s.vel.z();
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
        file.createGroup("/controller");
        H5Easy::dump(file, "/controller/active", 1);

        // Target positions (Nx3 matrix)
        size_t n_ctrl = output.target_positions.size();
        Eigen::MatrixXd targets(n_ctrl, 3);
        Eigen::MatrixXd errors(n_ctrl, 3);
        for (size_t i = 0; i < n_ctrl; ++i) {
            targets(i, 0) = output.target_positions[i].x();
            targets(i, 1) = output.target_positions[i].y();
            targets(i, 2) = output.target_positions[i].z();
            errors(i, 0) = output.position_errors[i].x();
            errors(i, 1) = output.position_errors[i].y();
            errors(i, 2) = output.position_errors[i].z();
        }
        file.createDataSet("/controller/target_position", targets);
        file.createDataSet("/controller/position_error", errors);

        // Parameter time histories
        file.createDataSet("/controller/gamma_mean", output.param_gamma_mean);
        file.createDataSet("/controller/psi_mean", output.param_psi_mean);
        file.createDataSet("/controller/phi_amp", output.param_phi_amp);
    }
}

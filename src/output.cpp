#include "output.hpp"

#include <highfive/H5Easy.hpp>
#include <highfive/H5File.hpp>

namespace {

// Helper to extract a Vec3 member from wing data at a specific wing index
std::vector<std::vector<double>> toArray(
    const std::vector<std::vector<SingleWingVectors>>& wing_data,
    size_t wing_index,
    Vec3 SingleWingVectors::* vec_member
) {
    std::vector<std::vector<double>> result;
    result.reserve(wing_data.size());
    for (const auto& timestep : wing_data) {
        if (wing_index < timestep.size()) {
            const Vec3& v = timestep[wing_index].*vec_member;
            result.push_back({v.x(), v.y(), v.z()});
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

    // Write parameters
    file.createGroup("/parameters");
    H5Easy::dump(file, "/parameters/lb0_f", output.lb0_f);
    H5Easy::dump(file, "/parameters/lb0_h", output.lb0_h);
    H5Easy::dump(file, "/parameters/mu0_f", output.mu0_f);
    H5Easy::dump(file, "/parameters/mu0_h", output.mu0_h);
    H5Easy::dump(file, "/parameters/Cd0", output.Cd0);
    H5Easy::dump(file, "/parameters/Cl0", output.Cl0);
    H5Easy::dump(file, "/parameters/omg0", output.omg0);
    H5Easy::dump(file, "/parameters/gam0", output.gam0);
    H5Easy::dump(file, "/parameters/phi0", output.phi0);
    H5Easy::dump(file, "/parameters/psim", output.psim);
    H5Easy::dump(file, "/parameters/dpsi", output.dpsi);
    H5Easy::dump(file, "/parameters/sig0", output.sig0);
    H5Easy::dump(file, "/parameters/dlt0", output.dlt0);

    // Write time
    file.createDataSet("/time", output.time);

    // Write states as 2D array
    std::vector<std::vector<double>> states_2d;
    states_2d.reserve(output.states.size());
    for (const auto& s : output.states) {
        states_2d.push_back({s[0], s[1], s[2], s[3], s[4], s[5]});
    }
    file.createDataSet("/state", states_2d);

    // Write wing data (variable number of wings)
    file.createGroup("/wings");

    size_t num_wings = wings.size();
    H5Easy::dump(file, "/wings/num_wings", static_cast<int>(num_wings));

    for (size_t i = 0; i < num_wings; ++i) {
        std::string group = "/wings/" + wingGroupName(wings[i]);
        file.createGroup(group);
        file.createDataSet(group + "/e_s", toArray(output.wing_data, i, &SingleWingVectors::e_s));
        file.createDataSet(group + "/e_r", toArray(output.wing_data, i, &SingleWingVectors::e_r));
        file.createDataSet(group + "/e_c", toArray(output.wing_data, i, &SingleWingVectors::e_c));
        file.createDataSet(group + "/lift", toArray(output.wing_data, i, &SingleWingVectors::lift));
        file.createDataSet(group + "/drag", toArray(output.wing_data, i, &SingleWingVectors::drag));
    }
}

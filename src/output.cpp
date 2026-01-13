#include "output.hpp"

#include <highfive/H5Easy.hpp>
#include <highfive/H5File.hpp>

namespace {

// Helper to convert Vec3 array to 2D vector for HDF5
std::vector<std::vector<double>> toArray(
    const std::vector<WingVectors>& wing_data,
    Vec3 SingleWingVectors::* vec_member,
    SingleWingVectors WingVectors::* wing_member
) {
    std::vector<std::vector<double>> result;
    result.reserve(wing_data.size());
    for (const auto& w : wing_data) {
        const Vec3& v = (w.*wing_member).*vec_member;
        result.push_back({v.x(), v.y(), v.z()});
    }
    return result;
}

} // namespace

void writeHDF5(const std::string& filename, const SimulationOutput& output) {
    HighFive::File file(filename, HighFive::File::Overwrite);

    // Write parameters
    file.createGroup("/parameters");
    H5Easy::dump(file, "/parameters/lb0_f", output.params.lb0_f);
    H5Easy::dump(file, "/parameters/lb0_h", output.params.lb0_h);
    H5Easy::dump(file, "/parameters/mu0_f", output.params.mu0_f);
    H5Easy::dump(file, "/parameters/mu0_h", output.params.mu0_h);
    H5Easy::dump(file, "/parameters/Cd0", output.params.Cd0);
    H5Easy::dump(file, "/parameters/Cl0", output.params.Cl0);
    H5Easy::dump(file, "/parameters/omg0", output.params.omg0);
    H5Easy::dump(file, "/parameters/gam0", output.params.gam0);
    H5Easy::dump(file, "/parameters/phi0", output.params.phi0);
    H5Easy::dump(file, "/parameters/psim", output.params.psim);
    H5Easy::dump(file, "/parameters/dpsi", output.params.dpsi);
    H5Easy::dump(file, "/parameters/sig0", output.params.sig0);
    H5Easy::dump(file, "/parameters/dlt0", output.params.dlt0);

    // Write time
    file.createDataSet("/time", output.time);

    // Write states as 2D array
    std::vector<std::vector<double>> states_2d;
    states_2d.reserve(output.states.size());
    for (const auto& s : output.states) {
        states_2d.push_back({s[0], s[1], s[2], s[3], s[4], s[5]});
    }
    file.createDataSet("/state", states_2d);

    // Write wing data
    file.createGroup("/wings");

    // Forewing left
    file.createGroup("/wings/fl");
    file.createDataSet("/wings/fl/e_s", toArray(output.wing_data, &SingleWingVectors::e_s, &WingVectors::fl));
    file.createDataSet("/wings/fl/e_r", toArray(output.wing_data, &SingleWingVectors::e_r, &WingVectors::fl));
    file.createDataSet("/wings/fl/e_c", toArray(output.wing_data, &SingleWingVectors::e_c, &WingVectors::fl));
    file.createDataSet("/wings/fl/lift", toArray(output.wing_data, &SingleWingVectors::lift, &WingVectors::fl));
    file.createDataSet("/wings/fl/drag", toArray(output.wing_data, &SingleWingVectors::drag, &WingVectors::fl));

    // Forewing right
    file.createGroup("/wings/fr");
    file.createDataSet("/wings/fr/e_s", toArray(output.wing_data, &SingleWingVectors::e_s, &WingVectors::fr));
    file.createDataSet("/wings/fr/e_r", toArray(output.wing_data, &SingleWingVectors::e_r, &WingVectors::fr));
    file.createDataSet("/wings/fr/e_c", toArray(output.wing_data, &SingleWingVectors::e_c, &WingVectors::fr));
    file.createDataSet("/wings/fr/lift", toArray(output.wing_data, &SingleWingVectors::lift, &WingVectors::fr));
    file.createDataSet("/wings/fr/drag", toArray(output.wing_data, &SingleWingVectors::drag, &WingVectors::fr));

    // Hindwing left
    file.createGroup("/wings/hl");
    file.createDataSet("/wings/hl/e_s", toArray(output.wing_data, &SingleWingVectors::e_s, &WingVectors::hl));
    file.createDataSet("/wings/hl/e_r", toArray(output.wing_data, &SingleWingVectors::e_r, &WingVectors::hl));
    file.createDataSet("/wings/hl/e_c", toArray(output.wing_data, &SingleWingVectors::e_c, &WingVectors::hl));
    file.createDataSet("/wings/hl/lift", toArray(output.wing_data, &SingleWingVectors::lift, &WingVectors::hl));
    file.createDataSet("/wings/hl/drag", toArray(output.wing_data, &SingleWingVectors::drag, &WingVectors::hl));

    // Hindwing right
    file.createGroup("/wings/hr");
    file.createDataSet("/wings/hr/e_s", toArray(output.wing_data, &SingleWingVectors::e_s, &WingVectors::hr));
    file.createDataSet("/wings/hr/e_r", toArray(output.wing_data, &SingleWingVectors::e_r, &WingVectors::hr));
    file.createDataSet("/wings/hr/e_c", toArray(output.wing_data, &SingleWingVectors::e_c, &WingVectors::hr));
    file.createDataSet("/wings/hr/lift", toArray(output.wing_data, &SingleWingVectors::lift, &WingVectors::hr));
    file.createDataSet("/wings/hr/drag", toArray(output.wing_data, &SingleWingVectors::drag, &WingVectors::hr));
}

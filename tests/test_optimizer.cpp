#include "optimize.hpp"
#include "wing.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

// Helper to create default physical params (4-wing dragonfly)
PhysicalParams defaultPhysicalParams() {
    PhysicalParams params;
    // Forewing pair
    params.wings.emplace_back("fore", WingSide::Left, 0.075, 0.75, 0.4, 1.2, 0.0);
    params.wings.emplace_back("fore", WingSide::Right, 0.075, 0.75, 0.4, 1.2, 0.0);
    // Hindwing pair (phase offset = pi)
    params.wings.emplace_back("hind", WingSide::Left, 0.075, 0.75, 0.4, 1.2, M_PI);
    params.wings.emplace_back("hind", WingSide::Right, 0.075, 0.75, 0.4, 1.2, M_PI);
    return params;
}

// Helper to create kinematic params with specific values
KinematicParams makeKinematicParams(double omg0, double gam0, double phi0, double psim,
                                     double dpsi, double dlt0) {
    KinematicParams k;
    k.omg0 = {"omg0", false, omg0, omg0, omg0};
    k.gam0 = {"gam0", false, gam0, gam0, gam0};
    k.phi0 = {"phi0", false, phi0, phi0, phi0};
    k.psim = {"psim", false, psim, psim, psim};
    k.dpsi = {"dpsi", false, dpsi, dpsi, dpsi};
    k.dlt0 = {"dlt0", false, dlt0, dlt0, dlt0};
    return k;
}

// Test wingBeatAccel is non-negative and finite
bool testWingBeatAccel() {
    std::cout << "Testing wingBeatAccel..." << std::endl;

    PhysicalParams phys = defaultPhysicalParams();
    KinematicParams kin = makeKinematicParams(
        8.0 * M_PI,   // omg0 (default)
        M_PI / 2.0,   // gam0 = 90 deg
        M_PI / 8.0,   // phi0 (default)
        M_PI / 4.0,   // psim = 45 deg
        M_PI / 6.0,   // dpsi = 30 deg
        M_PI / 2.0    // dlt0 (default)
    );

    double accel_sq = wingBeatAccel(kin, phys, 0.0, 0.0);  // hover

    std::cout << "  Mean acceleration squared: " << accel_sq << std::endl;
    std::cout << "  Mean acceleration magnitude: " << std::sqrt(accel_sq) << std::endl;

    bool passed = true;

    if (accel_sq < 0) {
        std::cout << "  FAILED: squared acceleration should be non-negative" << std::endl;
        passed = false;
    }

    if (!std::isfinite(accel_sq)) {
        std::cout << "  FAILED: non-finite result" << std::endl;
        passed = false;
    }

    if (passed) {
        std::cout << "  PASSED" << std::endl;
    }
    return passed;
}

// Test that different N values give similar results (convergence test)
bool testConvergence() {
    std::cout << "Testing integration convergence..." << std::endl;

    PhysicalParams phys = defaultPhysicalParams();
    KinematicParams kin = makeKinematicParams(
        8.0 * M_PI, M_PI / 2.0, M_PI / 8.0, M_PI / 4.0,
        M_PI / 6.0, M_PI / 2.0
    );

    double accel_N20 = wingBeatAccel(kin, phys, 1.0, 0.0, 20);  // forward flight
    double accel_N40 = wingBeatAccel(kin, phys, 1.0, 0.0, 40);
    double accel_N80 = wingBeatAccel(kin, phys, 1.0, 0.0, 80);

    std::cout << "  N=20: " << std::sqrt(accel_N20) << std::endl;
    std::cout << "  N=40: " << std::sqrt(accel_N40) << std::endl;
    std::cout << "  N=80: " << std::sqrt(accel_N80) << std::endl;

    bool passed = true;

    // Check that all values are in a reasonable range of each other (within 5%)
    double mean_val = (std::sqrt(accel_N20) + std::sqrt(accel_N40) + std::sqrt(accel_N80)) / 3.0;
    double max_deviation = std::max({
        std::abs(std::sqrt(accel_N20) - mean_val),
        std::abs(std::sqrt(accel_N40) - mean_val),
        std::abs(std::sqrt(accel_N80) - mean_val)
    });
    double rel_deviation = max_deviation / (mean_val + 1e-10);

    std::cout << "  Mean value: " << mean_val << std::endl;
    std::cout << "  Max relative deviation: " << rel_deviation * 100 << "%" << std::endl;

    if (rel_deviation > 0.05) {
        std::cout << "  FAILED: integration results vary too much with N" << std::endl;
        passed = false;
    }

    if (passed) {
        std::cout << "  PASSED" << std::endl;
    }
    return passed;
}

// Test symmetry: changing sign of dpsi should give finite results
bool testSymmetry() {
    std::cout << "Testing symmetry..." << std::endl;

    PhysicalParams phys = defaultPhysicalParams();
    KinematicParams kin_pos = makeKinematicParams(
        8.0 * M_PI, M_PI / 2.0, M_PI / 8.0, M_PI / 4.0,
        M_PI / 6.0, M_PI / 2.0
    );
    KinematicParams kin_neg = makeKinematicParams(
        8.0 * M_PI, M_PI / 2.0, M_PI / 8.0, M_PI / 4.0,
        -M_PI / 6.0, M_PI / 2.0
    );

    double accel_pos = wingBeatAccel(kin_pos, phys, 0.0, 0.0);
    double accel_neg = wingBeatAccel(kin_neg, phys, 0.0, 0.0);

    std::cout << "  dpsi > 0: " << std::sqrt(accel_pos) << std::endl;
    std::cout << "  dpsi < 0: " << std::sqrt(accel_neg) << std::endl;

    bool passed = true;

    if (!std::isfinite(accel_pos) || !std::isfinite(accel_neg)) {
        std::cout << "  FAILED: non-finite results" << std::endl;
        passed = false;
    }

    if (passed) {
        std::cout << "  PASSED" << std::endl;
    }
    return passed;
}

// Test that variable parameters can be set and retrieved correctly
bool testVariableParams() {
    std::cout << "Testing variable parameter handling..." << std::endl;

    KinematicParams kin;
    kin.omg0 = {"omg0", false, 8.0 * M_PI, 8.0 * M_PI, 8.0 * M_PI};
    kin.gam0 = {"gam0", false, M_PI / 2.0, M_PI / 2.0, M_PI / 2.0};
    kin.phi0 = {"phi0", false, M_PI / 8.0, M_PI / 8.0, M_PI / 8.0};
    kin.psim = {"psim", true, 0.5, 0.0, M_PI / 2.0};  // variable
    kin.dpsi = {"dpsi", true, 0.3, -M_PI / 2.0, M_PI / 2.0};  // variable
    kin.dlt0 = {"dlt0", false, M_PI / 2.0, M_PI / 2.0, M_PI / 2.0};

    bool passed = true;

    // Check numVariable
    if (kin.numVariable() != 2) {
        std::cout << "  FAILED: expected 2 variable params, got " << kin.numVariable() << std::endl;
        passed = false;
    }

    // Check variableNames
    auto names = kin.variableNames();
    if (names.size() != 2 || names[0] != "psim" || names[1] != "dpsi") {
        std::cout << "  FAILED: unexpected variable names" << std::endl;
        passed = false;
    }

    // Check set/get round trip
    std::vector<double> new_vals = {1.0, 0.5};
    kin.setVariableValues(new_vals);
    auto retrieved = kin.variableValues();
    if (std::abs(retrieved[0] - 1.0) > 1e-10 || std::abs(retrieved[1] - 0.5) > 1e-10) {
        std::cout << "  FAILED: set/get round trip failed" << std::endl;
        passed = false;
    }

    if (passed) {
        std::cout << "  PASSED" << std::endl;
    }
    return passed;
}

int main() {
    std::cout << "Optimizer Unit Tests" << std::endl;
    std::cout << "====================" << std::endl << std::endl;

    int num_passed = 0;
    int num_tests = 4;

    if (testWingBeatAccel()) num_passed++;
    std::cout << std::endl;

    if (testConvergence()) num_passed++;
    std::cout << std::endl;

    if (testSymmetry()) num_passed++;
    std::cout << std::endl;

    if (testVariableParams()) num_passed++;
    std::cout << std::endl;

    std::cout << "====================" << std::endl;
    std::cout << num_passed << "/" << num_tests << " tests passed" << std::endl;

    return (num_passed == num_tests) ? 0 : 1;
}

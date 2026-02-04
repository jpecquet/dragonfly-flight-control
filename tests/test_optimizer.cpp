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
KinematicParams makeKinematicParams(double omega, double gamma_mean, double phi_amp, double psi_mean,
                                     double psi_amp, double psi_phase) {
    KinematicParams k;
    k.omega = {"omega", false, omega, omega, omega};
    k.gamma_mean = {"gamma_mean", false, gamma_mean, gamma_mean, gamma_mean};
    k.gamma_amp = {"gamma_amp", false, 0.0, 0.0, 0.0};
    k.gamma_phase = {"gamma_phase", false, 0.0, 0.0, 0.0};
    k.phi_amp = {"phi_amp", false, phi_amp, phi_amp, phi_amp};
    k.psi_mean = {"psi_mean", false, psi_mean, psi_mean, psi_mean};
    k.psi_amp = {"psi_amp", false, psi_amp, psi_amp, psi_amp};
    k.psi_phase = {"psi_phase", false, psi_phase, psi_phase, psi_phase};
    return k;
}

// Test wingBeatAccel is non-negative and finite
bool testWingBeatAccel() {
    std::cout << "Testing wingBeatAccel..." << std::endl;

    PhysicalParams phys = defaultPhysicalParams();
    KinematicParams kin = makeKinematicParams(
        8.0 * M_PI,   // omega (default)
        M_PI / 2.0,   // gamma_mean = 90 deg
        M_PI / 8.0,   // phi_amp (default)
        M_PI / 4.0,   // psi_mean = 45 deg
        M_PI / 6.0,   // psi_amp = 30 deg
        M_PI / 2.0    // psi_phase (default)
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

// Test that variable parameters can be set and retrieved correctly
bool testVariableParams() {
    std::cout << "Testing variable parameter handling..." << std::endl;

    KinematicParams kin;
    kin.omega = {"omega", false, 8.0 * M_PI, 8.0 * M_PI, 8.0 * M_PI};
    kin.gamma_mean = {"gamma_mean", false, M_PI / 2.0, M_PI / 2.0, M_PI / 2.0};
    kin.gamma_amp = {"gamma_amp", false, 0.0, 0.0, 0.0};
    kin.gamma_phase = {"gamma_phase", false, 0.0, 0.0, 0.0};
    kin.phi_amp = {"phi_amp", false, M_PI / 8.0, M_PI / 8.0, M_PI / 8.0};
    kin.psi_mean = {"psi_mean", true, 0.5, 0.0, M_PI / 2.0};  // variable
    kin.psi_amp = {"psi_amp", true, 0.3, -M_PI / 2.0, M_PI / 2.0};  // variable
    kin.psi_phase = {"psi_phase", false, M_PI / 2.0, M_PI / 2.0, M_PI / 2.0};

    bool passed = true;

    // Check numVariable
    if (kin.numVariable() != 2) {
        std::cout << "  FAILED: expected 2 variable params, got " << kin.numVariable() << std::endl;
        passed = false;
    }

    // Check variableNames
    auto names = kin.variableNames();
    if (names.size() != 2 || names[0] != "psi_mean" || names[1] != "psi_amp") {
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
    int num_tests = 3;

    if (testWingBeatAccel()) num_passed++;
    std::cout << std::endl;

    if (testConvergence()) num_passed++;
    std::cout << std::endl;

    if (testVariableParams()) num_passed++;
    std::cout << std::endl;

    std::cout << "====================" << std::endl;
    std::cout << num_passed << "/" << num_tests << " tests passed" << std::endl;

    return (num_passed == num_tests) ? 0 : 1;
}

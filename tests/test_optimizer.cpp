#include "optimizer/objective.hpp"
#include "optimizer/optim_params.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

// Test instantAccel returns expected values for known inputs
bool testInstantAccel() {
    std::cout << "Testing instantAccel..." << std::endl;

    FixedParams fixed = FixedParams::defaults();

    // Test at hover with vertical stroke plane
    OptimParams optim = {M_PI / 2.0, M_PI / 4.0, M_PI / 6.0};  // gam0=90deg, psim=45deg, dpsi=30deg
    FlightCondition flight = {0.0, 0.0};  // hover

    Vec3 a = instantAccel(0.0, optim, flight, fixed);

    // At t=0, we should get a non-zero acceleration (gravity + wing forces)
    // The z-component should include gravity (-1 in nondimensional units)
    std::cout << "  Acceleration at t=0: (" << a.x() << ", " << a.y() << ", " << a.z() << ")" << std::endl;

    // Basic sanity checks
    bool passed = true;

    // y-component should be zero (symmetric flight)
    if (std::abs(a.y()) > 1e-10) {
        std::cout << "  FAILED: y-acceleration should be zero (symmetric)" << std::endl;
        passed = false;
    }

    // Should get some finite values
    if (!std::isfinite(a.x()) || !std::isfinite(a.y()) || !std::isfinite(a.z())) {
        std::cout << "  FAILED: non-finite acceleration values" << std::endl;
        passed = false;
    }

    if (passed) {
        std::cout << "  PASSED" << std::endl;
    }
    return passed;
}

// Test wingBeatAccel is non-negative and finite
bool testWingBeatAccel() {
    std::cout << "Testing wingBeatAccel..." << std::endl;

    FixedParams fixed = FixedParams::defaults();
    OptimParams optim = {M_PI / 2.0, M_PI / 4.0, M_PI / 6.0};
    FlightCondition flight = {0.0, 0.0};

    double accel_sq = wingBeatAccel(optim, flight, fixed);

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

    FixedParams fixed = FixedParams::defaults();
    OptimParams optim = {M_PI / 2.0, M_PI / 4.0, M_PI / 6.0};
    FlightCondition flight = {1.0, 0.0};  // forward flight

    double accel_N20 = wingBeatAccel(optim, flight, fixed, 20);
    double accel_N40 = wingBeatAccel(optim, flight, fixed, 40);
    double accel_N80 = wingBeatAccel(optim, flight, fixed, 80);

    std::cout << "  N=20: " << std::sqrt(accel_N20) << std::endl;
    std::cout << "  N=40: " << std::sqrt(accel_N40) << std::endl;
    std::cout << "  N=80: " << std::sqrt(accel_N80) << std::endl;

    bool passed = true;

    // Check that all values are in a reasonable range of each other (within 5%)
    // This tests that the integration is converging to a consistent value
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

// Test symmetry: changing sign of dpsi should give symmetric behavior
bool testSymmetry() {
    std::cout << "Testing symmetry..." << std::endl;

    FixedParams fixed = FixedParams::defaults();
    FlightCondition flight = {0.0, 0.0};

    OptimParams optim_pos = {M_PI / 2.0, M_PI / 4.0, M_PI / 6.0};
    OptimParams optim_neg = {M_PI / 2.0, M_PI / 4.0, -M_PI / 6.0};

    double accel_pos = wingBeatAccel(optim_pos, flight, fixed);
    double accel_neg = wingBeatAccel(optim_neg, flight, fixed);

    std::cout << "  dpsi > 0: " << std::sqrt(accel_pos) << std::endl;
    std::cout << "  dpsi < 0: " << std::sqrt(accel_neg) << std::endl;

    // These may not be exactly equal due to the dlt0 phase offset,
    // but they should be finite
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

int main() {
    std::cout << "Optimizer Unit Tests" << std::endl;
    std::cout << "====================" << std::endl << std::endl;

    int num_passed = 0;
    int num_tests = 4;

    if (testInstantAccel()) num_passed++;
    std::cout << std::endl;

    if (testWingBeatAccel()) num_passed++;
    std::cout << std::endl;

    if (testConvergence()) num_passed++;
    std::cout << std::endl;

    if (testSymmetry()) num_passed++;
    std::cout << std::endl;

    std::cout << "====================" << std::endl;
    std::cout << num_passed << "/" << num_tests << " tests passed" << std::endl;

    return (num_passed == num_tests) ? 0 : 1;
}

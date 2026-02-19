#include "config.hpp"
#include "optimize.hpp"
#include "wing.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

namespace fs = std::filesystem;

namespace {

fs::path writeTempConfig(const std::string& contents) {
    static int counter = 0;
    auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    fs::path path = fs::temp_directory_path() /
                    ("dragonfly_optimizer_test_" + std::to_string(stamp) + "_" +
                     std::to_string(counter++) + ".cfg");
    std::ofstream out(path);
    out << contents;
    out.close();
    return path;
}

}  // namespace

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
KinematicParams makeKinematicParams(double omega, double gamma_mean, double phi_cos, double psi_mean,
                                     double psi_cos, double psi_sin) {
    KinematicParams k;
    k.omega = {"omega", false, omega, omega, omega};
    k.gamma_mean = {"gamma_mean", false, gamma_mean, gamma_mean, gamma_mean};
    k.gamma_cos = {"gamma_cos", false, 0.0, 0.0, 0.0};
    k.gamma_sin = {"gamma_sin", false, 0.0, 0.0, 0.0};
    k.phi_mean = {"phi_mean", false, 0.0, 0.0, 0.0};
    k.phi_cos = {"phi_cos", false, phi_cos, phi_cos, phi_cos};
    k.phi_sin = {"phi_sin", false, 0.0, 0.0, 0.0};
    k.psi_mean = {"psi_mean", false, psi_mean, psi_mean, psi_mean};
    k.psi_cos = {"psi_cos", false, psi_cos, psi_cos, psi_cos};
    k.psi_sin = {"psi_sin", false, psi_sin, psi_sin, psi_sin};
    return k;
}

// Test wingBeatAccel is non-negative and finite
bool testWingBeatAccel() {
    std::cout << "Testing wingBeatAccel..." << std::endl;

    PhysicalParams phys = defaultPhysicalParams();
    KinematicParams kin = makeKinematicParams(
        8.0 * M_PI,   // omega (default)
        M_PI / 2.0,   // gamma_mean = 90 deg
        M_PI / 8.0,   // phi_cos (default)
        M_PI / 4.0,   // psi_mean = 45 deg
        0.0,          // psi_cos
        -M_PI / 6.0   // psi_sin
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
        0.0, -M_PI / 6.0
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
    kin.gamma_cos = {"gamma_cos", false, 0.0, 0.0, 0.0};
    kin.gamma_sin = {"gamma_sin", false, 0.0, 0.0, 0.0};
    kin.phi_mean = {"phi_mean", false, 0.0, 0.0, 0.0};
    kin.phi_cos = {"phi_cos", false, M_PI / 8.0, M_PI / 8.0, M_PI / 8.0};
    kin.phi_sin = {"phi_sin", false, 0.0, 0.0, 0.0};
    kin.psi_mean = {"psi_mean", true, 0.5, 0.0, M_PI / 2.0};  // variable
    kin.psi_cos = {"psi_cos", false, 0.0, 0.0, 0.0};
    kin.psi_sin = {"psi_sin", true, -0.3, -M_PI / 2.0, M_PI / 2.0};  // variable

    bool passed = true;

    // Check numVariable
    if (kin.numVariable() != 2) {
        std::cout << "  FAILED: expected 2 variable params, got " << kin.numVariable() << std::endl;
        passed = false;
    }

    // Check variableNames
    auto names = kin.variableNames();
    if (names.size() != 2 || names[0] != "psi_mean" || names[1] != "psi_sin") {
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

bool testPhysicalParamsFromConfig() {
    std::cout << "Testing PhysicalParams::fromConfig wing option consistency..." << std::endl;

    const std::string cfg_text =
        "n_blade_elements = 6\n"
        "\n"
        "[[wing]]\n"
        "name = fore\n"
        "side = left\n"
        "mu0 = 0.075\n"
        "lb0 = 0.75\n"
        "Cd_min = 0.4\n"
        "Cl0 = 1.2\n"
        "phase = 0.0\n"
        "psi_twist_h1_root_deg = 9.0\n"
        "psi_twist_ref_eta = 0.8\n"
        "\n"
        "[[wing]]\n"
        "name = hind\n"
        "side = right\n"
        "mu0 = 0.075\n"
        "lb0 = 0.75\n"
        "Cd_min = 0.4\n"
        "Cl0 = 1.2\n"
        "phase = 3.14159265359\n"
        "n_blade_elements = 10\n";

    const fs::path path = writeTempConfig(cfg_text);
    bool passed = true;
    try {
        Config cfg = Config::load(path.string());
        PhysicalParams phys = PhysicalParams::fromConfig(cfg);
        if (phys.wings.size() != 2) {
            passed = false;
            std::cout << "  FAILED: expected 2 wings, got " << phys.wings.size() << std::endl;
        } else {
            const WingConfig& fore = phys.wings[0];
            const WingConfig& hind = phys.wings[1];
            if (fore.n_blade_elements != 6 || hind.n_blade_elements != 10) {
                passed = false;
                std::cout << "  FAILED: n_blade_elements inheritance/override mismatch" << std::endl;
            }
            if (!fore.has_psi_twist_h1 || std::abs(fore.psi_twist_h1_root - (9.0 * M_PI / 180.0)) > 1e-12) {
                passed = false;
                std::cout << "  FAILED: forewing twist config mismatch" << std::endl;
            }
            if (std::abs(fore.psi_twist_ref_eta - 0.8) > 1e-12) {
                passed = false;
                std::cout << "  FAILED: forewing twist ref eta mismatch" << std::endl;
            }
            if (hind.has_psi_twist_h1) {
                passed = false;
                std::cout << "  FAILED: hindwing should not have twist enabled" << std::endl;
            }
        }
    } catch (const std::exception& e) {
        passed = false;
        std::cout << "  FAILED: unexpected exception: " << e.what() << std::endl;
    }
    fs::remove(path);

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

    if (testVariableParams()) num_passed++;
    std::cout << std::endl;

    if (testPhysicalParamsFromConfig()) num_passed++;
    std::cout << std::endl;

    std::cout << "====================" << std::endl;
    std::cout << num_passed << "/" << num_tests << " tests passed" << std::endl;

    return (num_passed == num_tests) ? 0 : 1;
}

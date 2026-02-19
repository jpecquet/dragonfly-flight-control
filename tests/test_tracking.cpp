#include "cmd_track.hpp"
#include "config.hpp"

#include <highfive/H5Easy.hpp>
#include <highfive/H5File.hpp>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace {

fs::path makeTempPath(const std::string& stem, const std::string& ext) {
    static int counter = 0;
    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    return fs::temp_directory_path() /
           (stem + "_" + std::to_string(stamp) + "_" + std::to_string(counter++) + ext);
}

bool nearlyEqual(double a, double b, double tol = 1e-10) {
    return std::abs(a - b) <= tol;
}

bool testRunTrackControllerStartupAndStorage() {
    std::cout << "Test: runTrack controller startup behavior and storage count\n";

    const fs::path output_path = makeTempPath("dragonfly_track_output", ".h5");
    const fs::path config_path = makeTempPath("dragonfly_track_config", ".cfg");

    const std::string cfg_text =
        "output = " + output_path.string() + "\n"
        "omega = 6.283185307179586\n"
        "n_harmonics = 1\n"
        "n_wingbeats = 1\n"
        "steps_per_wingbeat = 10\n"
        "trajectory = hover 1.0 0.0 0.0\n"
        "gamma_mean = 0.0\n"
        "gamma_cos = 0.0\n"
        "gamma_sin = 0.0\n"
        "phi_mean = 0.0\n"
        "phi_cos = 0.5\n"
        "phi_sin = 0.0\n"
        "psi_mean = 0.0\n"
        "psi_cos = 0.0\n"
        "psi_sin = 0.0\n"
        "pid_x_kp = 1.0\n"
        "pid_x_ki = 1.0\n"
        "pid_x_kd = 0.0\n"
        "pid_x_imax = 100.0\n"
        "pid_y_kp = 0.0\n"
        "pid_y_ki = 0.0\n"
        "pid_y_kd = 0.0\n"
        "pid_y_imax = 0.0\n"
        "pid_z_kp = 0.0\n"
        "pid_z_ki = 0.0\n"
        "pid_z_kd = 0.0\n"
        "pid_z_imax = 0.0\n"
        "mix_gamma_x = 1.0\n"
        "mix_gamma_y = 0.0\n"
        "mix_gamma_z = 0.0\n"
        "mix_psi_x = 0.0\n"
        "mix_psi_z = 0.0\n"
        "mix_phi_x = 0.0\n"
        "mix_phi_z = 0.0\n"
        "gamma_mean_min = -10.0\n"
        "gamma_mean_max = 10.0\n"
        "\n"
        "[[wing]]\n"
        "name = fore\n"
        "side = left\n"
        "mu0 = 0.075\n"
        "lb0 = 0.75\n"
        "Cd_min = 0.4\n"
        "Cl0 = 1.2\n"
        "phase = 0.0\n";

    {
        std::ofstream out(config_path);
        out << cfg_text;
    }

    bool passed = true;
    try {
        Config cfg = Config::load(config_path.string());
        const int rc = runTrack(cfg);
        if (rc != 0) {
            std::cout << "  FAILED: runTrack returned " << rc << "\n";
            passed = false;
        }

        HighFive::File file(output_path.string(), HighFive::File::ReadOnly);
        const auto time = H5Easy::load<std::vector<double>>(file, "/time");
        const auto gamma = H5Easy::load<std::vector<double>>(file, "/controller/gamma_mean");
        const auto psi = H5Easy::load<std::vector<double>>(file, "/controller/psi_mean");
        const auto phi = H5Easy::load<std::vector<double>>(file, "/controller/phi_amp");
        const auto target_dims =
            file.getDataSet("/controller/target_position").getSpace().getDimensions();

        if (time.size() < 2) {
            std::cout << "  FAILED: expected at least 2 time samples, got " << time.size() << "\n";
            passed = false;
        }

        const size_t expected_samples = time.size();
        if (gamma.size() != expected_samples || psi.size() != expected_samples || phi.size() != expected_samples) {
            std::cout << "  FAILED: controller scalar histories do not match /time sample count\n";
            passed = false;
        }
        if (target_dims.size() != 2 || target_dims[0] != expected_samples || target_dims[1] != 3) {
            std::cout << "  FAILED: /controller/target_position shape mismatch\n";
            passed = false;
        }

        if (passed) {
            const double dt = time[1] - time[0];
            const double expected_gamma0 = 1.0 + dt;  // Kp*e + Ki*(e*dt), with e=1

            if (!nearlyEqual(gamma[0], expected_gamma0)) {
                std::cout << "  FAILED: gamma_mean[0] = " << gamma[0]
                          << ", expected " << expected_gamma0 << "\n";
                passed = false;
            }
            if (!nearlyEqual(psi[0], 0.0)) {
                std::cout << "  FAILED: psi_mean[0] = " << psi[0] << ", expected 0\n";
                passed = false;
            }
            if (!nearlyEqual(phi[0], 0.5)) {
                std::cout << "  FAILED: phi_amp[0] = " << phi[0] << ", expected 0.5\n";
                passed = false;
            }

            // Regression guard for duplicate startup compute:
            // first step should use the same command computed at t=0.
            if (!nearlyEqual(gamma[1], gamma[0])) {
                std::cout << "  FAILED: gamma_mean[1] = " << gamma[1]
                          << " differs from gamma_mean[0] = " << gamma[0]
                          << " (startup duplicate compute regression)\n";
                passed = false;
            }
        }
    } catch (const std::exception& e) {
        std::cout << "  FAILED: unexpected exception: " << e.what() << "\n";
        passed = false;
    }

    fs::remove(config_path);
    fs::remove(output_path);

    std::cout << "  " << (passed ? "PASSED" : "FAILED") << "\n\n";
    return passed;
}

}  // namespace

int main() {
    std::cout << "Tracking Tests\n";
    std::cout << "==============\n\n";

    int passed = 0;
    const int total = 1;

    if (testRunTrackControllerStartupAndStorage()) passed++;

    std::cout << "Summary: " << passed << "/" << total << " tests passed\n";
    return (passed == total) ? 0 : 1;
}

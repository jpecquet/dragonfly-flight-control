#include "config.hpp"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <string>

namespace fs = std::filesystem;

namespace {

fs::path writeTempConfig(const std::string& contents) {
    static int counter = 0;
    auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    fs::path path = fs::temp_directory_path() /
                    ("dragonfly_config_test_" + std::to_string(stamp) + "_" +
                     std::to_string(counter++) + ".cfg");
    std::ofstream out(path);
    out << contents;
    out.close();
    return path;
}

bool expectThrow(const std::function<void()>& fn) {
    try {
        fn();
        return false;
    } catch (const std::exception&) {
        return true;
    }
}

bool testInlineComments() {
    std::cout << "Test: Inline comments in global and wing values\n";

    const std::string cfg_text =
        "omega = 25.1327412287  # 8*pi\n"
        "steps_per_wingbeat = 50 # integration resolution\n"
        "\n"
        "[[wing]]\n"
        "name = fore # forewing\n"
        "side = left # left side\n"
        "mu0 = 0.075 # mass parameter\n"
        "lb0 = 0.75 # span\n"
        "Cd0 = 0.4 # drag\n"
        "Cl0 = 1.2 # lift\n"
        "phase = 0.0 # phase offset\n";

    fs::path path = writeTempConfig(cfg_text);
    bool passed = true;
    try {
        Config cfg = Config::load(path.string());

        if (std::abs(cfg.getDouble("omega") - 25.1327412287) > 1e-12) {
            passed = false;
            std::cout << "  FAILED: omega was not parsed correctly\n";
        }
        if (cfg.getInt("steps_per_wingbeat") != 50) {
            passed = false;
            std::cout << "  FAILED: steps_per_wingbeat was not parsed correctly\n";
        }

        const auto& wings = cfg.getWingEntries();
        if (wings.size() != 1) {
            passed = false;
            std::cout << "  FAILED: expected 1 wing section, got " << wings.size() << "\n";
        } else {
            if (wings[0].name != "fore" || wings[0].side != "left") {
                passed = false;
                std::cout << "  FAILED: wing string fields parsed incorrectly\n";
            }
            if (std::abs(wings[0].mu0 - 0.075) > 1e-12 ||
                std::abs(wings[0].lb0 - 0.75) > 1e-12 ||
                std::abs(wings[0].Cd0 - 0.4) > 1e-12 ||
                std::abs(wings[0].Cl0 - 1.2) > 1e-12 ||
                std::abs(wings[0].phase - 0.0) > 1e-12) {
                passed = false;
                std::cout << "  FAILED: wing numeric fields parsed incorrectly\n";
            }
        }
    } catch (const std::exception& e) {
        passed = false;
        std::cout << "  FAILED: unexpected exception: " << e.what() << "\n";
    }

    fs::remove(path);
    std::cout << "  " << (passed ? "PASSED" : "FAILED") << "\n\n";
    return passed;
}

bool testStrictGlobalNumeric() {
    std::cout << "Test: Strict numeric parsing for global keys\n";

    const std::string cfg_text =
        "omega = 25.1327412287oops\n"
        "\n"
        "[[wing]]\n"
        "name = fore\n"
        "side = left\n"
        "mu0 = 0.075\n"
        "lb0 = 0.75\n"
        "Cd0 = 0.4\n"
        "Cl0 = 1.2\n"
        "phase = 0.0\n";

    fs::path path = writeTempConfig(cfg_text);
    bool passed = true;
    try {
        Config cfg = Config::load(path.string());
        passed = expectThrow([&]() {
            (void)cfg.getDouble("omega");
        });
        if (!passed) {
            std::cout << "  FAILED: expected getDouble(\"omega\") to throw\n";
        }
    } catch (const std::exception& e) {
        passed = false;
        std::cout << "  FAILED: unexpected exception during load: " << e.what() << "\n";
    }

    fs::remove(path);
    std::cout << "  " << (passed ? "PASSED" : "FAILED") << "\n\n";
    return passed;
}

bool testStrictWingNumeric() {
    std::cout << "Test: Strict numeric parsing for wing keys\n";

    const std::string cfg_text =
        "[[wing]]\n"
        "name = fore\n"
        "side = left\n"
        "mu0 = 0.075oops\n"
        "lb0 = 0.75\n"
        "Cd0 = 0.4\n"
        "Cl0 = 1.2\n"
        "phase = 0.0\n";

    fs::path path = writeTempConfig(cfg_text);
    bool passed = expectThrow([&]() {
        (void)Config::load(path.string());
    });
    fs::remove(path);

    if (!passed) {
        std::cout << "  FAILED: expected Config::load to reject invalid wing numeric token\n";
    }
    std::cout << "  " << (passed ? "PASSED" : "FAILED") << "\n\n";
    return passed;
}

bool testMissingRequiredWingFields() {
    std::cout << "Test: Missing required wing fields are rejected\n";

    const std::string cfg_text =
        "[[wing]]\n"
        "name = fore\n"
        "side = left\n"
        "mu0 = 0.075\n"
        "lb0 = 0.75\n"
        "Cd0 = 0.4\n"
        "phase = 0.0\n";

    fs::path path = writeTempConfig(cfg_text);
    bool passed = expectThrow([&]() {
        (void)Config::load(path.string());
    });
    fs::remove(path);

    if (!passed) {
        std::cout << "  FAILED: expected Config::load to reject missing required wing fields\n";
    }
    std::cout << "  " << (passed ? "PASSED" : "FAILED") << "\n\n";
    return passed;
}

bool testDoubleListParsing() {
    std::cout << "Test: getDoubleList parses comma-separated and bracketed lists\n";

    const std::string cfg_text =
        "gamma_cos = 0.1, -0.2, 0.3\n"
        "psi_sin = [0.0, 1.0, -1.0]\n"
        "\n"
        "[[wing]]\n"
        "name = fore\n"
        "side = left\n"
        "mu0 = 0.075\n"
        "lb0 = 0.75\n"
        "Cd0 = 0.4\n"
        "Cl0 = 1.2\n"
        "phase = 0.0\n";

    fs::path path = writeTempConfig(cfg_text);
    bool passed = true;
    try {
        Config cfg = Config::load(path.string());
        std::vector<double> gamma = cfg.getDoubleList("gamma_cos");
        std::vector<double> psi = cfg.getDoubleList("psi_sin");

        if (gamma.size() != 3 || std::abs(gamma[0] - 0.1) > 1e-12 ||
            std::abs(gamma[1] + 0.2) > 1e-12 || std::abs(gamma[2] - 0.3) > 1e-12) {
            passed = false;
            std::cout << "  FAILED: gamma_cos parsed incorrectly\n";
        }
        if (psi.size() != 3 || std::abs(psi[0] - 0.0) > 1e-12 ||
            std::abs(psi[1] - 1.0) > 1e-12 || std::abs(psi[2] + 1.0) > 1e-12) {
            passed = false;
            std::cout << "  FAILED: psi_sin parsed incorrectly\n";
        }
    } catch (const std::exception& e) {
        passed = false;
        std::cout << "  FAILED: unexpected exception: " << e.what() << "\n";
    }

    fs::remove(path);
    std::cout << "  " << (passed ? "PASSED" : "FAILED") << "\n\n";
    return passed;
}

bool testDoubleListStrictness() {
    std::cout << "Test: getDoubleList rejects malformed lists\n";

    const std::string cfg_text =
        "gamma_cos = 0.1,,0.3\n"
        "\n"
        "[[wing]]\n"
        "name = fore\n"
        "side = left\n"
        "mu0 = 0.075\n"
        "lb0 = 0.75\n"
        "Cd0 = 0.4\n"
        "Cl0 = 1.2\n"
        "phase = 0.0\n";

    fs::path path = writeTempConfig(cfg_text);
    bool passed = true;
    try {
        Config cfg = Config::load(path.string());
        passed = expectThrow([&]() {
            (void)cfg.getDoubleList("gamma_cos");
        });
        if (!passed) {
            std::cout << "  FAILED: expected malformed list to throw\n";
        }
    } catch (const std::exception& e) {
        passed = false;
        std::cout << "  FAILED: unexpected exception during load: " << e.what() << "\n";
    }

    fs::remove(path);
    std::cout << "  " << (passed ? "PASSED" : "FAILED") << "\n\n";
    return passed;
}

bool testWingMotionOverrides() {
    std::cout << "Test: [[wing]] accepts per-wing motion override keys\n";

    const std::string cfg_text =
        "omega = 10.0\n"
        "phi_cos = 0.2\n"
        "phi_sin = 0.0\n"
        "gamma_mean = 1.0\n"
        "psi_mean = 0.5\n"
        "psi_cos = 0.3\n"
        "psi_sin = 0.0\n"
        "\n"
        "[[wing]]\n"
        "name = fore\n"
        "side = left\n"
        "mu0 = 0.075\n"
        "lb0 = 0.75\n"
        "Cd0 = 0.4\n"
        "Cl0 = 1.2\n"
        "phase = 0.0\n"
        "phi_cos = 0.3, 0.1\n"
        "psi_mean = 0.7\n";

    fs::path path = writeTempConfig(cfg_text);
    bool passed = true;
    try {
        Config cfg = Config::load(path.string());
        const auto& wings = cfg.getWingEntries();
        if (wings.size() != 1) {
            passed = false;
            std::cout << "  FAILED: expected one wing\n";
        } else {
            const auto& overrides = wings[0].motion_overrides;
            auto it_phi = overrides.find("phi_cos");
            auto it_psi = overrides.find("psi_mean");
            if (it_phi == overrides.end() || it_phi->second != "0.3, 0.1") {
                passed = false;
                std::cout << "  FAILED: missing or incorrect phi_cos override\n";
            }
            if (it_psi == overrides.end() || it_psi->second != "0.7") {
                passed = false;
                std::cout << "  FAILED: missing or incorrect psi_mean override\n";
            }
        }
    } catch (const std::exception& e) {
        passed = false;
        std::cout << "  FAILED: unexpected exception: " << e.what() << "\n";
    }

    fs::remove(path);
    std::cout << "  " << (passed ? "PASSED" : "FAILED") << "\n\n";
    return passed;
}

bool testUnknownWingMotionKeysRejected() {
    std::cout << "Test: [[wing]] rejects unknown motion keys\n";

    const std::string cfg_text =
        "omega = 10.0\n"
        "phi_cos = 0.2\n"
        "phi_sin = 0.0\n"
        "\n"
        "[[wing]]\n"
        "name = fore\n"
        "side = left\n"
        "mu0 = 0.075\n"
        "lb0 = 0.75\n"
        "Cd0 = 0.4\n"
        "Cl0 = 1.2\n"
        "phase = 0.0\n"
        "psi_scale = 0.3\n";

    fs::path path = writeTempConfig(cfg_text);
    bool passed = expectThrow([&]() {
        (void)Config::load(path.string());
    });
    fs::remove(path);

    if (!passed) {
        std::cout << "  FAILED: expected unknown wing key to be rejected\n";
    }
    std::cout << "  " << (passed ? "PASSED" : "FAILED") << "\n\n";
    return passed;
}

}  // namespace

int main() {
    std::cout << "Config Parser Tests\n";
    std::cout << "===================\n\n";

    int passed = 0;
    const int total = 8;

    if (testInlineComments()) passed++;
    if (testStrictGlobalNumeric()) passed++;
    if (testStrictWingNumeric()) passed++;
    if (testMissingRequiredWingFields()) passed++;
    if (testDoubleListParsing()) passed++;
    if (testDoubleListStrictness()) passed++;
    if (testWingMotionOverrides()) passed++;
    if (testUnknownWingMotionKeysRejected()) passed++;

    std::cout << "Summary: " << passed << "/" << total << " tests passed\n";
    return (passed == total) ? 0 : 1;
}

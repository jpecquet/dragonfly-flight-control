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

}  // namespace

int main() {
    std::cout << "Config Parser Tests\n";
    std::cout << "===================\n\n";

    int passed = 0;
    const int total = 4;

    if (testInlineComments()) passed++;
    if (testStrictGlobalNumeric()) passed++;
    if (testStrictWingNumeric()) passed++;
    if (testMissingRequiredWingFields()) passed++;

    std::cout << "Summary: " << passed << "/" << total << " tests passed\n";
    return (passed == total) ? 0 : 1;
}

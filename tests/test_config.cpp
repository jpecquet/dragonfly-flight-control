#include "config.hpp"
#include "sim_setup.hpp"

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
    std::cout << "Test: Inline comments in global and wing values (legacy Cd0 alias)\n";

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
                std::abs(wings[0].Cd_min - 0.4) > 1e-12 ||
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
        "Cd_min = 0.4\n"
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
        "Cd_min = 0.4\n"
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
        "Cd_min = 0.4\n"
        "Cl0 = 1.2\n"
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

bool testAeroPresetParsing() {
    std::cout << "Test: wang2004 aerodynamic preset applies expected sinusoidal coefficients\n";

    const std::string cfg_text =
        "omega = 10.0\n"
        "gamma_mean = 1.0\n"
        "phi_mean = 0.0\n"
        "psi_mean = 0.0\n"
        "\n"
        "[[wing]]\n"
        "name = fore\n"
        "side = left\n"
        "mu0 = 0.075\n"
        "lb0 = 0.75\n"
        "drag_model = sinusoidal\n"
        "drag_coeff_set = wang2004\n"
        "lift_model = sinusoidal\n"
        "lift_coeff_set = wang2004\n"
        "phase = 0.0\n";

    fs::path path = writeTempConfig(cfg_text);
    bool passed = true;
    try {
        Config cfg = Config::load(path.string());
        SimKinematicParams kin = readKinematicParams(cfg);
        auto wings = buildWingConfigs(cfg, kin);
        if (wings.size() != 1) {
            passed = false;
            std::cout << "  FAILED: expected one wing\n";
        } else {
            if (wings[0].drag_model != DragCoefficientModel::Sinusoidal ||
                wings[0].lift_model != LiftCoefficientModel::Sinusoidal) {
                passed = false;
                std::cout << "  FAILED: expected sinusoidal drag/lift models\n";
            }
            if (std::abs(wings[0].Cd_min - 0.4) > 1e-12 || std::abs(wings[0].Cl0 - 1.2) > 1e-12) {
                passed = false;
                std::cout << "  FAILED: wang2004 preset coefficients not applied\n";
            }
            if (std::abs(wings[0].Cd_max - 2.4) > 1e-12) {
                passed = false;
                std::cout << "  FAILED: wang2004 preset Cd_max should be 2.4\n";
            }
            if (std::abs(wings[0].Cd_alpha_neutral) > 1e-12 ||
                std::abs(wings[0].Cl_alpha_neutral) > 1e-12) {
                passed = false;
                std::cout << "  FAILED: wang2004 preset neutral angles should be zero\n";
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

bool testLinearLiftParsingAndValidation() {
    std::cout << "Test: linear lift model parse + validation\n";

    const std::string good_cfg =
        "omega = 10.0\n"
        "gamma_mean = 1.0\n"
        "phi_mean = 0.0\n"
        "psi_mean = 0.0\n"
        "\n"
        "[[wing]]\n"
        "name = fore\n"
        "side = left\n"
        "mu0 = 0.075\n"
        "lb0 = 0.75\n"
        "Cd_min = 0.4\n"
        "Cd_alpha_neutral = -0.2\n"
        "lift_model = linear\n"
        "Cl_alpha_slope = 2.0\n"
        "Cl_alpha_neutral = 0.1\n"
        "Cl_min = -0.4\n"
        "Cl_max = 0.8\n"
        "phase = 0.0\n";

    const std::string good_sinusoidal_shift_cfg =
        "omega = 10.0\n"
        "gamma_mean = 1.0\n"
        "phi_mean = 0.0\n"
        "psi_mean = 0.0\n"
        "\n"
        "[[wing]]\n"
        "name = fore\n"
        "side = left\n"
        "mu0 = 0.075\n"
        "lb0 = 0.75\n"
        "Cd_min = 0.4\n"
        "Cd_alpha_neutral = -0.2\n"
        "Cl0 = 1.2\n"
        "Cl_alpha_neutral = 0.05\n"
        "phase = 0.0\n";

    const std::string good_linear_azuma_cfg =
        "omega = 10.0\n"
        "gamma_mean = 1.0\n"
        "phi_mean = 0.0\n"
        "psi_mean = 0.0\n"
        "\n"
        "[[wing]]\n"
        "name = fore\n"
        "side = left\n"
        "mu0 = 0.075\n"
        "lb0 = 0.75\n"
        "Cd_min = 0.4\n"
        "lift_model = linear\n"
        "lift_coeff_set = azuma1988\n"
        "phase = 0.0\n";

    const std::string good_drag_azuma_cfg =
        "omega = 10.0\n"
        "gamma_mean = 1.0\n"
        "phi_mean = 0.0\n"
        "psi_mean = 0.0\n"
        "\n"
        "[[wing]]\n"
        "name = fore\n"
        "side = left\n"
        "mu0 = 0.075\n"
        "lb0 = 0.75\n"
        "drag_coeff_set = azuma1988\n"
        "Cl0 = 1.2\n"
        "phase = 0.0\n";

    const std::string bad_missing_param =
        "omega = 10.0\n"
        "gamma_mean = 1.0\n"
        "phi_mean = 0.0\n"
        "psi_mean = 0.0\n"
        "\n"
        "[[wing]]\n"
        "name = fore\n"
        "side = left\n"
        "mu0 = 0.075\n"
        "lb0 = 0.75\n"
        "Cd_min = 0.4\n"
        "lift_model = linear\n"
        "Cl_alpha_slope = 2.0\n"
        "phase = 0.0\n";

    const std::string bad_preset_mix =
        "omega = 10.0\n"
        "gamma_mean = 1.0\n"
        "phi_mean = 0.0\n"
        "psi_mean = 0.0\n"
        "\n"
        "[[wing]]\n"
        "name = fore\n"
        "side = left\n"
        "mu0 = 0.075\n"
        "lb0 = 0.75\n"
        "drag_coeff_set = wang2004\n"
        "Cd_alpha_neutral = 0.1\n"
        "lift_coeff_set = wang2004\n"
        "Cl_alpha_neutral = 0.2\n"
        "phase = 0.0\n";

    const std::string bad_azuma_override =
        "omega = 10.0\n"
        "gamma_mean = 1.0\n"
        "phi_mean = 0.0\n"
        "psi_mean = 0.0\n"
        "\n"
        "[[wing]]\n"
        "name = fore\n"
        "side = left\n"
        "mu0 = 0.075\n"
        "lb0 = 0.75\n"
        "Cd_min = 0.4\n"
        "lift_model = linear\n"
        "lift_coeff_set = azuma1988\n"
        "Cl_max = 2.0\n"
        "phase = 0.0\n";

    const std::string bad_drag_azuma_override =
        "omega = 10.0\n"
        "gamma_mean = 1.0\n"
        "phi_mean = 0.0\n"
        "psi_mean = 0.0\n"
        "\n"
        "[[wing]]\n"
        "name = fore\n"
        "side = left\n"
        "mu0 = 0.075\n"
        "lb0 = 0.75\n"
        "drag_coeff_set = azuma1988\n"
        "Cd_min = 0.2\n"
        "Cl0 = 1.2\n"
        "phase = 0.0\n";

    fs::path p_good = writeTempConfig(good_cfg);
    fs::path p_good_shift = writeTempConfig(good_sinusoidal_shift_cfg);
    fs::path p_good_azuma = writeTempConfig(good_linear_azuma_cfg);
    fs::path p_good_drag_azuma = writeTempConfig(good_drag_azuma_cfg);
    fs::path p_bad1 = writeTempConfig(bad_missing_param);
    fs::path p_bad2 = writeTempConfig(bad_preset_mix);
    fs::path p_bad3 = writeTempConfig(bad_azuma_override);
    fs::path p_bad4 = writeTempConfig(bad_drag_azuma_override);

    bool passed = true;
    try {
        Config cfg = Config::load(p_good.string());
        SimKinematicParams kin = readKinematicParams(cfg);
        auto wings = buildWingConfigs(cfg, kin);
        if (wings.size() != 1) {
            passed = false;
            std::cout << "  FAILED: expected one wing in good config\n";
        } else {
            const auto& w = wings[0];
            if (w.lift_model != LiftCoefficientModel::Linear) {
                passed = false;
                std::cout << "  FAILED: lift model should be linear\n";
            }
            if (std::abs(w.Cd_alpha_neutral + 0.2) > 1e-12 ||
                std::abs(w.Cl_alpha_slope - 2.0) > 1e-12 ||
                std::abs(w.Cl_alpha_neutral - 0.1) > 1e-12 ||
                std::abs(w.Cl_min + 0.4) > 1e-12 ||
                std::abs(w.Cl_max - 0.8) > 1e-12) {
                passed = false;
                std::cout << "  FAILED: linear lift / sinusoidal drag neutral parameters parsed incorrectly\n";
            }
        }
    } catch (const std::exception& e) {
        passed = false;
        std::cout << "  FAILED: unexpected exception in good config: " << e.what() << "\n";
    }

    try {
        Config cfg_shift = Config::load(p_good_shift.string());
        SimKinematicParams kin_shift = readKinematicParams(cfg_shift);
        auto wings_shift = buildWingConfigs(cfg_shift, kin_shift);
        if (wings_shift.size() != 1) {
            passed = false;
            std::cout << "  FAILED: expected one wing in shifted sinusoidal config\n";
        } else {
            const auto& w = wings_shift[0];
            if (std::abs(w.Cd_alpha_neutral + 0.2) > 1e-12 ||
                std::abs(w.Cl_alpha_neutral - 0.05) > 1e-12) {
                passed = false;
                std::cout << "  FAILED: sinusoidal neutral angles parsed incorrectly\n";
            }
        }
    } catch (const std::exception& e) {
        passed = false;
        std::cout << "  FAILED: unexpected exception in shifted sinusoidal config: "
                  << e.what() << "\n";
    }

    try {
        Config cfg_azuma = Config::load(p_good_azuma.string());
        SimKinematicParams kin_azuma = readKinematicParams(cfg_azuma);
        auto wings_azuma = buildWingConfigs(cfg_azuma, kin_azuma);
        if (wings_azuma.size() != 1) {
            passed = false;
            std::cout << "  FAILED: expected one wing in azuma1988 linear preset config\n";
        } else {
            const auto& w = wings_azuma[0];
            const double expected_slope = 0.052 * (180.0 / M_PI);
            const double expected_neutral = -7.0 * (M_PI / 180.0);
            if (std::abs(w.Cl_alpha_slope - expected_slope) > 1e-12 ||
                std::abs(w.Cl_alpha_neutral - expected_neutral) > 1e-12 ||
                std::abs(w.Cl_min + 1.2) > 1e-12 ||
                std::abs(w.Cl_max - 1.2) > 1e-12) {
                passed = false;
                std::cout << "  FAILED: azuma1988 linear preset values incorrect\n";
            }
        }
    } catch (const std::exception& e) {
        passed = false;
        std::cout << "  FAILED: unexpected exception in azuma1988 linear preset config: "
                  << e.what() << "\n";
    }

    try {
        Config cfg_drag_azuma = Config::load(p_good_drag_azuma.string());
        SimKinematicParams kin_drag_azuma = readKinematicParams(cfg_drag_azuma);
        auto wings_drag_azuma = buildWingConfigs(cfg_drag_azuma, kin_drag_azuma);
        if (wings_drag_azuma.size() != 1) {
            passed = false;
            std::cout << "  FAILED: expected one wing in azuma1988 drag preset config\n";
        } else {
            const auto& w = wings_drag_azuma[0];
            const double expected_neutral = 7.0 * (M_PI / 180.0);
            if (std::abs(w.Cd_min - 0.07) > 1e-12 ||
                std::abs(w.Cd_max - 2.0) > 1e-12 ||
                std::abs(w.Cd_alpha_neutral - expected_neutral) > 1e-12) {
                passed = false;
                std::cout << "  FAILED: azuma1988 drag preset values incorrect\n";
            }
        }
    } catch (const std::exception& e) {
        passed = false;
        std::cout << "  FAILED: unexpected exception in azuma1988 drag preset config: "
                  << e.what() << "\n";
    }

    try {
        Config cfg1 = Config::load(p_bad1.string());
        SimKinematicParams kin1 = readKinematicParams(cfg1);
        if (!expectThrow([&]() { (void)buildWingConfigs(cfg1, kin1); })) {
            passed = false;
            std::cout << "  FAILED: expected linear lift missing parameter to fail\n";
        }
    } catch (const std::exception& e) {
        passed = false;
        std::cout << "  FAILED: unexpected exception while validating missing-parameter config: "
                  << e.what() << "\n";
    }

    try {
        Config cfg2 = Config::load(p_bad2.string());
        SimKinematicParams kin2 = readKinematicParams(cfg2);
        if (!expectThrow([&]() { (void)buildWingConfigs(cfg2, kin2); })) {
            passed = false;
            std::cout << "  FAILED: expected nonzero neutral angles with wang2004 preset to fail\n";
        }
    } catch (const std::exception& e) {
        passed = false;
        std::cout << "  FAILED: unexpected exception while validating preset conflict config: "
                  << e.what() << "\n";
    }

    try {
        Config cfg3 = Config::load(p_bad3.string());
        SimKinematicParams kin3 = readKinematicParams(cfg3);
        if (!expectThrow([&]() { (void)buildWingConfigs(cfg3, kin3); })) {
            passed = false;
            std::cout << "  FAILED: expected azuma1988 preset + override mix to fail\n";
        }
    } catch (const std::exception& e) {
        passed = false;
        std::cout << "  FAILED: unexpected exception while validating azuma1988 override conflict: "
                  << e.what() << "\n";
    }

    try {
        Config cfg4 = Config::load(p_bad4.string());
        SimKinematicParams kin4 = readKinematicParams(cfg4);
        if (!expectThrow([&]() { (void)buildWingConfigs(cfg4, kin4); })) {
            passed = false;
            std::cout << "  FAILED: expected azuma1988 drag preset + override mix to fail\n";
        }
    } catch (const std::exception& e) {
        passed = false;
        std::cout << "  FAILED: unexpected exception while validating azuma1988 drag override conflict: "
                  << e.what() << "\n";
    }

    fs::remove(p_good);
    fs::remove(p_good_shift);
    fs::remove(p_good_azuma);
    fs::remove(p_good_drag_azuma);
    fs::remove(p_bad1);
    fs::remove(p_bad2);
    fs::remove(p_bad3);
    fs::remove(p_bad4);
    std::cout << "  " << (passed ? "PASSED" : "FAILED") << "\n\n";
    return passed;
}

bool testDoubleListParsing() {
    std::cout << "Test: getDoubleList parses comma-separated and bracketed lists\n";

    const std::string cfg_text =
        "gamma_amp = 0.1, -0.2, 0.3\n"
        "psi_phase = [0.0, 1.0, -1.0]\n"
        "\n"
        "[[wing]]\n"
        "name = fore\n"
        "side = left\n"
        "mu0 = 0.075\n"
        "lb0 = 0.75\n"
        "Cd_min = 0.4\n"
        "Cl0 = 1.2\n"
        "phase = 0.0\n";

    fs::path path = writeTempConfig(cfg_text);
    bool passed = true;
    try {
        Config cfg = Config::load(path.string());
        std::vector<double> gamma = cfg.getDoubleList("gamma_amp");
        std::vector<double> psi = cfg.getDoubleList("psi_phase");

        if (gamma.size() != 3 || std::abs(gamma[0] - 0.1) > 1e-12 ||
            std::abs(gamma[1] + 0.2) > 1e-12 || std::abs(gamma[2] - 0.3) > 1e-12) {
            passed = false;
            std::cout << "  FAILED: gamma_amp parsed incorrectly\n";
        }
        if (psi.size() != 3 || std::abs(psi[0] - 0.0) > 1e-12 ||
            std::abs(psi[1] - 1.0) > 1e-12 || std::abs(psi[2] + 1.0) > 1e-12) {
            passed = false;
            std::cout << "  FAILED: psi_phase parsed incorrectly\n";
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
        "gamma_amp = 0.1,,0.3\n"
        "\n"
        "[[wing]]\n"
        "name = fore\n"
        "side = left\n"
        "mu0 = 0.075\n"
        "lb0 = 0.75\n"
        "Cd_min = 0.4\n"
        "Cl0 = 1.2\n"
        "phase = 0.0\n";

    fs::path path = writeTempConfig(cfg_text);
    bool passed = true;
    try {
        Config cfg = Config::load(path.string());
        passed = expectThrow([&]() {
            (void)cfg.getDoubleList("gamma_amp");
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
        "phi_amp = 0.2\n"
        "phi_phase = 0.0\n"
        "gamma_mean = 1.0\n"
        "psi_mean = 0.5\n"
        "psi_amp = 0.3\n"
        "psi_phase = 0.0\n"
        "\n"
        "[[wing]]\n"
        "name = fore\n"
        "side = left\n"
        "mu0 = 0.075\n"
        "lb0 = 0.75\n"
        "Cd_min = 0.4\n"
        "Cl0 = 1.2\n"
        "phase = 0.0\n"
        "phi_amp = 0.3, 0.1\n"
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
            auto it_phi = overrides.find("phi_amp");
            auto it_psi = overrides.find("psi_mean");
            if (it_phi == overrides.end() || it_phi->second != "0.3, 0.1") {
                passed = false;
                std::cout << "  FAILED: missing or incorrect phi_amp override\n";
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

bool testAmpPhaseKinematicsParsing() {
    std::cout << "Test: kinematics parse from *_amp/*_phase keys\n";

    const std::string cfg_text =
        "omega = 10.0\n"
        "n_harmonics = 2\n"
        "gamma_mean = 1.0\n"
        "gamma_amp = 0.3, 0.0\n"
        "gamma_phase = 0.1, 0.0\n"
        "phi_mean = 0.0\n"
        "phi_amp = 0.4, 0.2\n"
        "phi_phase = 0.0, -0.3\n"
        "psi_mean = 0.5\n"
        "psi_amp = 0.7, 0.0\n"
        "psi_phase = 0.2, 0.0\n"
        "\n"
        "[[wing]]\n"
        "name = fore\n"
        "side = left\n"
        "mu0 = 0.075\n"
        "lb0 = 0.75\n"
        "Cd_min = 0.4\n"
        "Cl0 = 1.2\n"
        "phase = 0.0\n"
        "phi_amp = 0.45, 0.2\n";

    fs::path path = writeTempConfig(cfg_text);
    bool passed = true;
    try {
        Config cfg = Config::load(path.string());
        SimKinematicParams kin = readKinematicParams(cfg);
        auto wings = buildWingConfigs(cfg, kin);

        if (kin.phi.amplitude_coeff.size() != 2 || kin.phi.phase_coeff.size() != 2) {
            passed = false;
            std::cout << "  FAILED: global phi harmonic sizes are incorrect\n";
        } else {
            if (std::abs(kin.phi.amplitude_coeff[0] - 0.4) > 1e-12 ||
                std::abs(kin.phi.amplitude_coeff[1] - 0.2) > 1e-12 ||
                std::abs(kin.phi.phase_coeff[0] - 0.0) > 1e-12 ||
                std::abs(kin.phi.phase_coeff[1] + 0.3) > 1e-12) {
                passed = false;
                std::cout << "  FAILED: global phi amp/phase values are incorrect\n";
            }
        }

        if (wings.size() != 1) {
            passed = false;
            std::cout << "  FAILED: expected one wing\n";
        } else if (std::abs(wings[0].phi.amplitude_coeff[0] - 0.45) > 1e-12) {
            passed = false;
            std::cout << "  FAILED: per-wing phi_amp override was not applied\n";
        }
    } catch (const std::exception& e) {
        passed = false;
        std::cout << "  FAILED: unexpected exception: " << e.what() << "\n";
    }

    fs::remove(path);
    std::cout << "  " << (passed ? "PASSED" : "FAILED") << "\n\n";
    return passed;
}

bool testPartialAmpPhaseInputs() {
    std::cout << "Test: partial *_amp/*_phase inputs default missing counterparts to zero\n";

    const std::string global_partial_cfg =
        "omega = 10.0\n"
        "n_harmonics = 2\n"
        "phi_mean = 0.0\n"
        "phi_amp = 0.2, 0.1\n"
        "psi_mean = 0.0\n"
        "gamma_mean = 0.0\n"
        "\n"
        "[[wing]]\n"
        "name = fore\n"
        "side = left\n"
        "mu0 = 0.075\n"
        "lb0 = 0.75\n"
        "Cd_min = 0.4\n"
        "Cl0 = 1.2\n"
        "phase = 0.0\n";

    const std::string wing_partial_cfg =
        "omega = 10.0\n"
        "n_harmonics = 2\n"
        "phi_mean = 0.0\n"
        "phi_amp = 0.2, 0.1\n"
        "phi_phase = 0.0, 0.0\n"
        "psi_mean = 0.0\n"
        "gamma_mean = 0.0\n"
        "\n"
        "[[wing]]\n"
        "name = fore\n"
        "side = left\n"
        "mu0 = 0.075\n"
        "lb0 = 0.75\n"
        "Cd_min = 0.4\n"
        "Cl0 = 1.2\n"
        "phase = 0.0\n"
        "phi_phase = 0.3, 0.1\n";

    fs::path p_global = writeTempConfig(global_partial_cfg);
    fs::path p_wing = writeTempConfig(wing_partial_cfg);

    bool passed = true;
    try {
        Config cfg = Config::load(p_global.string());
        SimKinematicParams kin = readKinematicParams(cfg);
        if (kin.phi.phase_coeff.size() != 2 ||
            std::abs(kin.phi.phase_coeff[0]) > 1e-12 ||
            std::abs(kin.phi.phase_coeff[1]) > 1e-12) {
            passed = false;
            std::cout << "  FAILED: missing global phi_phase should default to zeros\n";
        }
    } catch (const std::exception& e) {
        passed = false;
        std::cout << "  FAILED: unexpected exception in global partial test: " << e.what() << "\n";
    }

    try {
        Config cfg = Config::load(p_wing.string());
        SimKinematicParams kin = readKinematicParams(cfg);
        auto wings = buildWingConfigs(cfg, kin);
        if (wings.size() != 1 ||
            wings[0].phi.phase_coeff.size() != 2 ||
            std::abs(wings[0].phi.phase_coeff[0] - 0.3) > 1e-12 ||
            std::abs(wings[0].phi.phase_coeff[1] - 0.1) > 1e-12) {
            passed = false;
            std::cout << "  FAILED: per-wing phi_phase override was not applied\n";
        }
    } catch (const std::exception& e) {
        passed = false;
        std::cout << "  FAILED: unexpected exception in wing partial test: " << e.what() << "\n";
    }

    fs::remove(p_global);
    fs::remove(p_wing);
    std::cout << "  " << (passed ? "PASSED" : "FAILED") << "\n\n";
    return passed;
}

bool testUnknownWingMotionKeysRejected() {
    std::cout << "Test: [[wing]] rejects unknown motion keys\n";

    const std::string cfg_text =
        "omega = 10.0\n"
        "phi_amp = 0.2\n"
        "phi_phase = 0.0\n"
        "\n"
        "[[wing]]\n"
        "name = fore\n"
        "side = left\n"
        "mu0 = 0.075\n"
        "lb0 = 0.75\n"
        "Cd_min = 0.4\n"
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

bool testWingMotionOverrideFlagSemantics() {
    std::cout << "Test: hasWingMotionSeries reflects explicit per-wing overrides\n";

    const std::string cfg_text =
        "omega = 10.0\n"
        "gamma_mean = 1.0\n"
        "phi_mean = 0.0\n"
        "psi_mean = 0.0\n"
        "\n"
        "[[wing]]\n"
        "name = fore\n"
        "side = left\n"
        "mu0 = 0.075\n"
        "lb0 = 0.75\n"
        "Cd_min = 0.4\n"
        "Cl0 = 1.2\n"
        "phase = 0.0\n"
        "\n"
        "[[wing]]\n"
        "name = hind\n"
        "side = left\n"
        "mu0 = 0.075\n"
        "lb0 = 0.75\n"
        "Cd_min = 0.4\n"
        "Cl0 = 1.2\n"
        "phase = 0.0\n"
        "phi_mean = 0.1\n";

    fs::path path = writeTempConfig(cfg_text);
    bool passed = true;
    try {
        Config cfg = Config::load(path.string());
        SimKinematicParams kin = readKinematicParams(cfg);
        auto wings = buildWingConfigs(cfg, kin);
        if (wings.size() != 2) {
            passed = false;
            std::cout << "  FAILED: expected 2 wings, got " << wings.size() << "\n";
        } else {
            if (hasWingMotionSeries(wings[0])) {
                passed = false;
                std::cout << "  FAILED: wing without overrides reported custom motion\n";
            }
            if (!hasWingMotionSeries(wings[1])) {
                passed = false;
                std::cout << "  FAILED: wing with overrides did not report custom motion\n";
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

bool testBladeElementCountParsing() {
    std::cout << "Test: n_blade_elements supports global default + per-wing override\n";

    const std::string cfg_text =
        "omega = 10.0\n"
        "gamma_mean = 1.0\n"
        "phi_mean = 0.0\n"
        "psi_mean = 0.0\n"
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
        "\n"
        "[[wing]]\n"
        "name = hind\n"
        "side = right\n"
        "mu0 = 0.075\n"
        "lb0 = 0.75\n"
        "Cd_min = 0.4\n"
        "Cl0 = 1.2\n"
        "phase = 0.0\n"
        "n_blade_elements = 10\n";

    fs::path path = writeTempConfig(cfg_text);
    bool passed = true;
    try {
        Config cfg = Config::load(path.string());
        SimKinematicParams kin = readKinematicParams(cfg);
        auto wings = buildWingConfigs(cfg, kin);
        if (wings.size() != 2) {
            passed = false;
            std::cout << "  FAILED: expected 2 wings, got " << wings.size() << "\n";
        } else {
            if (wings[0].n_blade_elements != 6) {
                passed = false;
                std::cout << "  FAILED: first wing should inherit global n_blade_elements=6\n";
            }
            if (wings[1].n_blade_elements != 10) {
                passed = false;
                std::cout << "  FAILED: second wing should use per-wing n_blade_elements=10\n";
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

bool testBladeElementCountValidation() {
    std::cout << "Test: n_blade_elements rejects invalid values\n";

    const std::string per_wing_bad =
        "[[wing]]\n"
        "name = fore\n"
        "side = left\n"
        "mu0 = 0.075\n"
        "lb0 = 0.75\n"
        "Cd_min = 0.4\n"
        "Cl0 = 1.2\n"
        "phase = 0.0\n"
        "n_blade_elements = 0\n";

    const std::string global_bad =
        "omega = 10.0\n"
        "gamma_mean = 1.0\n"
        "phi_mean = 0.0\n"
        "psi_mean = 0.0\n"
        "n_blade_elements = 0\n"
        "\n"
        "[[wing]]\n"
        "name = fore\n"
        "side = left\n"
        "mu0 = 0.075\n"
        "lb0 = 0.75\n"
        "Cd_min = 0.4\n"
        "Cl0 = 1.2\n"
        "phase = 0.0\n";

    fs::path p1 = writeTempConfig(per_wing_bad);
    fs::path p2 = writeTempConfig(global_bad);

    bool passed = true;
    if (!expectThrow([&]() { (void)Config::load(p1.string()); })) {
        passed = false;
        std::cout << "  FAILED: per-wing n_blade_elements=0 should be rejected at parse time\n";
    }

    try {
        Config cfg = Config::load(p2.string());
        SimKinematicParams kin = readKinematicParams(cfg);
        if (!expectThrow([&]() { (void)buildWingConfigs(cfg, kin); })) {
            passed = false;
            std::cout << "  FAILED: global n_blade_elements=0 should be rejected\n";
        }
    } catch (const std::exception& e) {
        passed = false;
        std::cout << "  FAILED: unexpected exception while validating global key: " << e.what() << "\n";
    }

    fs::remove(p1);
    fs::remove(p2);
    std::cout << "  " << (passed ? "PASSED" : "FAILED") << "\n\n";
    return passed;
}

bool testPitchTwistParsing() {
    std::cout << "Test: psi_twist_h1_root_deg and psi_twist_ref_eta parse into wing config\n";

    const std::string cfg_text =
        "omega = 10.0\n"
        "gamma_mean = 1.0\n"
        "phi_mean = 0.0\n"
        "psi_mean = 0.0\n"
        "psi_amp = 0.5\n"
        "psi_phase = 0.0\n"
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
        "psi_twist_ref_eta = 0.75\n";

    fs::path path = writeTempConfig(cfg_text);
    bool passed = true;
    try {
        Config cfg = Config::load(path.string());
        SimKinematicParams kin = readKinematicParams(cfg);
        auto wings = buildWingConfigs(cfg, kin);
        if (wings.size() != 1) {
            passed = false;
            std::cout << "  FAILED: expected one wing\n";
        } else {
            if (!wings[0].has_psi_twist_h1) {
                passed = false;
                std::cout << "  FAILED: expected has_psi_twist_h1=true\n";
            }
            if (std::abs(wings[0].psi_twist_h1_root - (9.0 * M_PI / 180.0)) > 1e-12) {
                passed = false;
                std::cout << "  FAILED: psi_twist_h1_root radians conversion incorrect\n";
            }
            if (std::abs(wings[0].psi_twist_ref_eta - 0.75) > 1e-12) {
                passed = false;
                std::cout << "  FAILED: psi_twist_ref_eta incorrect\n";
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

bool testPitchTwistValidation() {
    std::cout << "Test: psi_twist_ref_eta validation\n";

    const std::string cfg_text =
        "omega = 10.0\n"
        "gamma_mean = 1.0\n"
        "phi_mean = 0.0\n"
        "psi_mean = 0.0\n"
        "psi_amp = 0.5\n"
        "psi_phase = 0.0\n"
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
        "psi_twist_ref_eta = 0.0\n";

    fs::path path = writeTempConfig(cfg_text);
    bool passed = true;
    try {
        Config cfg = Config::load(path.string());
        SimKinematicParams kin = readKinematicParams(cfg);
        passed = expectThrow([&]() { (void)buildWingConfigs(cfg, kin); });
        if (!passed) {
            std::cout << "  FAILED: expected buildWingConfigs to reject psi_twist_ref_eta <= 0\n";
        }
    } catch (const std::exception& e) {
        passed = false;
        std::cout << "  FAILED: unexpected exception while loading config: " << e.what() << "\n";
    }

    fs::remove(path);
    std::cout << "  " << (passed ? "PASSED" : "FAILED") << "\n\n";
    return passed;
}

}  // namespace

int main() {
    std::cout << "Config Parser Tests\n";
    std::cout << "===================\n\n";

    int passed = 0;
    const int total = 17;

    if (testInlineComments()) passed++;
    if (testStrictGlobalNumeric()) passed++;
    if (testStrictWingNumeric()) passed++;
    if (testMissingRequiredWingFields()) passed++;
    if (testAeroPresetParsing()) passed++;
    if (testLinearLiftParsingAndValidation()) passed++;
    if (testDoubleListParsing()) passed++;
    if (testDoubleListStrictness()) passed++;
    if (testWingMotionOverrides()) passed++;
    if (testAmpPhaseKinematicsParsing()) passed++;
    if (testPartialAmpPhaseInputs()) passed++;
    if (testUnknownWingMotionKeysRejected()) passed++;
    if (testWingMotionOverrideFlagSemantics()) passed++;
    if (testBladeElementCountParsing()) passed++;
    if (testBladeElementCountValidation()) passed++;
    if (testPitchTwistParsing()) passed++;
    if (testPitchTwistValidation()) passed++;

    std::cout << "Summary: " << passed << "/" << total << " tests passed\n";
    return (passed == total) ? 0 : 1;
}

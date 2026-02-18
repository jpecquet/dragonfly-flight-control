#include "cmd_wingtest.hpp"
#include "cmd_args.hpp"
#include "parse_utils.hpp"
#include "rotation.hpp"

#include <highfive/H5Easy.hpp>
#include <highfive/H5File.hpp>

#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

struct AngleRange {
    double start = 0.0;
    double end = 0.0;
};

struct TestConfig {
    AngleRange gam{0.0, 90.0};
    AngleRange phi{0.0, 25.0};
    AngleRange psi{0.0, 45.0};
    std::vector<std::string> sequence{"gam", "phi", "psi"};  // rotation order
    int frames_per_phase = 50;
    bool is_left = true;
    std::string output_file = "wingtest.h5";
};

void printUsage(const char* prog) {
    std::cerr << "Usage: " << prog << " wingtest [options]\n";
    std::cerr << "\n";
    std::cerr << "Generate wing rotation test data for visualization.\n";
    std::cerr << "\n";
    std::cerr << "Options:\n";
    std::cerr << "  --gam START:END   Stroke plane angle range in degrees (default: 0:90)\n";
    std::cerr << "  --phi START:END   Flapping angle range in degrees (default: 0:25)\n";
    std::cerr << "  --psi START:END   Pitch angle range in degrees (default: 0:45)\n";
    std::cerr << "  --seq A,B,C       Sweep sequence of gam/phi/psi (default: gam,phi,psi)\n";
    std::cerr << "  --frames N        Frames per phase (default: 50)\n";
    std::cerr << "  --right           Use right wing (default: left)\n";
    std::cerr << "  -o FILE           Output HDF5 file (default: wingtest.h5)\n";
}

bool parseRange(const std::string& arg, AngleRange& range) {
    const size_t sep = arg.find(':');
    if (sep == std::string::npos || sep == 0 || sep + 1 >= arg.size()) {
        return false;
    }
    try {
        range.start = parseutil::parseDoubleStrict(arg.substr(0, sep), "range start");
        range.end = parseutil::parseDoubleStrict(arg.substr(sep + 1), "range end");
    } catch (const std::exception&) {
        return false;
    }
    return true;
}

bool parseSequence(const std::string& arg, std::vector<std::string>& seq) {
    seq.clear();
    std::stringstream ss(arg);
    std::string token;
    while (std::getline(ss, token, ',')) {
        token = parseutil::trimCopy(token);
        if (token != "gam" && token != "phi" && token != "psi") {
            return false;
        }
        seq.push_back(token);
    }
    return seq.size() == 3;
}

TestConfig parseTestArgs(int argc, char* argv[]) {
    TestConfig cfg;

    for (int i = 2; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--gam") {
            if (!parseRange(cliarg::requireOptionValue(argc, argv, i, "--gam"), cfg.gam)) {
                throw std::runtime_error("Invalid --gam format. Use START:END");
            }
        } else if (arg == "--phi") {
            if (!parseRange(cliarg::requireOptionValue(argc, argv, i, "--phi"), cfg.phi)) {
                throw std::runtime_error("Invalid --phi format. Use START:END");
            }
        } else if (arg == "--psi") {
            if (!parseRange(cliarg::requireOptionValue(argc, argv, i, "--psi"), cfg.psi)) {
                throw std::runtime_error("Invalid --psi format. Use START:END");
            }
        } else if (arg == "--seq") {
            if (!parseSequence(cliarg::requireOptionValue(argc, argv, i, "--seq"), cfg.sequence)) {
                throw std::runtime_error("Invalid --seq format. Use comma-separated list of gam,phi,psi (e.g. --seq phi,gam,psi)");
            }
        } else if (arg == "--frames") {
            cfg.frames_per_phase = cliarg::parseInt(
                cliarg::requireOptionValue(argc, argv, i, "--frames"),
                "--frames"
            );
            if (cfg.frames_per_phase < 2) {
                throw std::runtime_error("Frames must be >= 2");
            }
        } else if (arg == "--right") {
            cfg.is_left = false;
        } else if (arg == "-o") {
            cfg.output_file = cliarg::requireOptionValue(argc, argv, i, "-o");
        } else if (cliarg::isHelpFlag(arg)) {
            printUsage(argv[0]);
            throw std::runtime_error("__help__");
        } else if (!arg.empty() && arg[0] != '-') {
            cfg.output_file = arg;
        } else {
            throw std::runtime_error(std::string("Unknown option: ") + arg);
        }
    }

    return cfg;
}

constexpr double deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

// Helper to get angle name for display
std::string angleName(const std::string& key) {
    if (key == "gam") return "Stroke plane (gam)";
    if (key == "phi") return "Flapping (phi)";
    if (key == "psi") return "Pitch (psi)";
    return key;
}

// Helper to get angle range by key
const AngleRange& getRange(const TestConfig& cfg, const std::string& key) {
    if (key == "gam") return cfg.gam;
    if (key == "phi") return cfg.phi;
    return cfg.psi;
}

}  // namespace

int runWingtest(int argc, char* argv[]) {
    TestConfig cfg;
    try {
        cfg = parseTestArgs(argc, argv);
    } catch (const std::runtime_error& e) {
        if (std::string(e.what()) == "__help__") {
            return 0;
        }
        std::cerr << e.what() << "\n";
        printUsage(argv[0]);
        return 1;
    }

    int total_frames = 3 * cfg.frames_per_phase;

    std::cout << "Wing rotation test (" << (cfg.is_left ? "left" : "right") << " wing)\n";
    std::cout << "==================\n";
    std::cout << "Rotation sequence: " << cfg.sequence[0] << " -> " << cfg.sequence[1] << " -> " << cfg.sequence[2] << "\n";
    for (int p = 0; p < 3; ++p) {
        const auto& range = getRange(cfg, cfg.sequence[p]);
        std::cout << "Phase " << (p + 1) << ": " << angleName(cfg.sequence[p])
                  << " " << range.start << " -> " << range.end << " deg\n";
    }
    std::cout << "Frames per phase: " << cfg.frames_per_phase << "\n\n";

    // Pre-allocate storage
    std::vector<int> frame_vec;
    std::vector<double> gam_vec, phi_vec, psi_vec;
    std::vector<std::vector<double>> e_s_vec, e_r_vec, e_c_vec;

    frame_vec.reserve(total_frames);
    gam_vec.reserve(total_frames);
    phi_vec.reserve(total_frames);
    psi_vec.reserve(total_frames);
    e_s_vec.reserve(total_frames);
    e_r_vec.reserve(total_frames);
    e_c_vec.reserve(total_frames);

    // Phase boundaries
    std::vector<int> phase_boundaries = {0, cfg.frames_per_phase, 2 * cfg.frames_per_phase};

    int frame = 0;
    int n = cfg.frames_per_phase;

    // Current angle values (start at initial values)
    double gam_deg = cfg.gam.start;
    double phi_deg = cfg.phi.start;
    double psi_deg = cfg.psi.start;

    // Process each phase according to sequence
    for (int phase = 0; phase < 3; ++phase) {
        const std::string& sweep_key = cfg.sequence[phase];
        const AngleRange& range = getRange(cfg, sweep_key);

        std::cout << "Phase " << (phase + 1) << ": " << angleName(sweep_key) << " sweep\n";

        for (int i = 0; i < n; ++i, ++frame) {
            double t = static_cast<double>(i) / (n - 1);
            double sweep_val = range.start + t * (range.end - range.start);

            // Update the sweeping angle
            if (sweep_key == "gam") gam_deg = sweep_val;
            else if (sweep_key == "phi") phi_deg = sweep_val;
            else if (sweep_key == "psi") psi_deg = sweep_val;

            auto orient = computeWingOrientation(
                deg2rad(gam_deg), deg2rad(phi_deg), deg2rad(psi_deg), 0.0, cfg.is_left);

            frame_vec.push_back(frame);
            gam_vec.push_back(gam_deg);
            phi_vec.push_back(phi_deg);
            psi_vec.push_back(psi_deg);
            e_s_vec.push_back({orient.e_s.x(), orient.e_s.y(), orient.e_s.z()});
            e_r_vec.push_back({orient.e_r.x(), orient.e_r.y(), orient.e_r.z()});
            e_c_vec.push_back({orient.e_c.x(), orient.e_c.y(), orient.e_c.z()});

            if (i == 0 || i == n - 1) {
                std::cout << "  " << sweep_key << "=" << sweep_val << " deg\n";
            }
        }

        std::cout << "\n";
    }

    // Write HDF5
    std::cout << "Writing to " << cfg.output_file << "...\n";

    HighFive::File file(cfg.output_file, HighFive::File::Overwrite);

    file.createGroup("/parameters");
    H5Easy::dump(file, "/parameters/frames_per_phase", cfg.frames_per_phase);
    H5Easy::dump(file, "/parameters/total_frames", total_frames);
    H5Easy::dump(file, "/parameters/is_left", cfg.is_left ? 1 : 0);
    H5Easy::dump(file, "/parameters/gam_start", cfg.gam.start);
    H5Easy::dump(file, "/parameters/gam_end", cfg.gam.end);
    H5Easy::dump(file, "/parameters/phi_start", cfg.phi.start);
    H5Easy::dump(file, "/parameters/phi_end", cfg.phi.end);
    H5Easy::dump(file, "/parameters/psi_start", cfg.psi.start);
    H5Easy::dump(file, "/parameters/psi_end", cfg.psi.end);
    H5Easy::dump(file, "/parameters/phase_boundaries", phase_boundaries);
    H5Easy::dump(file, "/parameters/sequence", cfg.sequence);

    file.createDataSet("/frame", frame_vec);
    file.createGroup("/angles");
    file.createDataSet("/angles/gam", gam_vec);
    file.createDataSet("/angles/phi", phi_vec);
    file.createDataSet("/angles/psi", psi_vec);

    file.createGroup("/wing");
    file.createDataSet("/wing/e_s", e_s_vec);
    file.createDataSet("/wing/e_r", e_r_vec);
    file.createDataSet("/wing/e_c", e_c_vec);

    std::cout << "Done. Total frames: " << total_frames << "\n";

    return 0;
}

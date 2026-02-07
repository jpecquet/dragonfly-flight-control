#include "cmd_wingtest.hpp"
#include "rotation.hpp"

#include <highfive/H5Easy.hpp>
#include <highfive/H5File.hpp>

#include <cstring>
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

bool parseRange(const char* arg, AngleRange& range) {
    char* end;
    range.start = std::strtod(arg, &end);
    if (*end != ':') return false;
    range.end = std::strtod(end + 1, &end);
    return *end == '\0';
}

bool parseSequence(const char* arg, std::vector<std::string>& seq) {
    seq.clear();
    std::string s(arg);
    std::stringstream ss(s);
    std::string token;
    while (std::getline(ss, token, ',')) {
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
        if (std::strcmp(argv[i], "--gam") == 0 && i + 1 < argc) {
            if (!parseRange(argv[++i], cfg.gam)) {
                throw std::runtime_error("Invalid --gam format. Use START:END");
            }
        } else if (std::strcmp(argv[i], "--phi") == 0 && i + 1 < argc) {
            if (!parseRange(argv[++i], cfg.phi)) {
                throw std::runtime_error("Invalid --phi format. Use START:END");
            }
        } else if (std::strcmp(argv[i], "--psi") == 0 && i + 1 < argc) {
            if (!parseRange(argv[++i], cfg.psi)) {
                throw std::runtime_error("Invalid --psi format. Use START:END");
            }
        } else if (std::strcmp(argv[i], "--seq") == 0 && i + 1 < argc) {
            if (!parseSequence(argv[++i], cfg.sequence)) {
                throw std::runtime_error("Invalid --seq format. Use comma-separated list of gam,phi,psi (e.g. --seq phi,gam,psi)");
            }
        } else if (std::strcmp(argv[i], "--frames") == 0 && i + 1 < argc) {
            cfg.frames_per_phase = std::atoi(argv[++i]);
            if (cfg.frames_per_phase < 2) {
                throw std::runtime_error("Frames must be >= 2");
            }
        } else if (std::strcmp(argv[i], "--right") == 0) {
            cfg.is_left = false;
        } else if (std::strcmp(argv[i], "-o") == 0 && i + 1 < argc) {
            cfg.output_file = argv[++i];
        } else if (argv[i][0] != '-') {
            cfg.output_file = argv[i];
        } else {
            throw std::runtime_error(std::string("Unknown option: ") + argv[i]);
        }
    }

    return cfg;
}

constexpr double deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

}  // namespace

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

int runWingtest(int argc, char* argv[]) {
    TestConfig cfg = parseTestArgs(argc, argv);

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
                deg2rad(gam_deg), deg2rad(phi_deg), deg2rad(psi_deg), cfg.is_left);

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

        // Fix the swept angle at its end value for subsequent phases
        if (sweep_key == "gam") gam_deg = range.end;
        else if (sweep_key == "phi") phi_deg = range.end;
        else if (sweep_key == "psi") psi_deg = range.end;

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

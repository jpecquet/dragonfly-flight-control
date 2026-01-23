#include "cmd_test.hpp"
#include "rotation.hpp"

#include <highfive/H5Easy.hpp>
#include <highfive/H5File.hpp>

#include <cstdlib>
#include <cstring>
#include <iostream>
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
    int frames_per_phase = 50;
    bool is_left = true;
    std::string output_file = "wing_rotation.h5";
};

bool parseRange(const char* arg, AngleRange& range) {
    char* end;
    range.start = std::strtod(arg, &end);
    if (*end != ':') return false;
    range.end = std::strtod(end + 1, &end);
    return *end == '\0';
}

TestConfig parseTestArgs(int argc, char* argv[]) {
    TestConfig cfg;

    for (int i = 2; i < argc; ++i) {
        if (std::strcmp(argv[i], "--gam") == 0 && i + 1 < argc) {
            if (!parseRange(argv[++i], cfg.gam)) {
                std::cerr << "Invalid --gam format. Use START:END\n";
                std::exit(1);
            }
        } else if (std::strcmp(argv[i], "--phi") == 0 && i + 1 < argc) {
            if (!parseRange(argv[++i], cfg.phi)) {
                std::cerr << "Invalid --phi format. Use START:END\n";
                std::exit(1);
            }
        } else if (std::strcmp(argv[i], "--psi") == 0 && i + 1 < argc) {
            if (!parseRange(argv[++i], cfg.psi)) {
                std::cerr << "Invalid --psi format. Use START:END\n";
                std::exit(1);
            }
        } else if (std::strcmp(argv[i], "--frames") == 0 && i + 1 < argc) {
            cfg.frames_per_phase = std::atoi(argv[++i]);
            if (cfg.frames_per_phase < 2) {
                std::cerr << "Frames must be >= 2\n";
                std::exit(1);
            }
        } else if (std::strcmp(argv[i], "--right") == 0) {
            cfg.is_left = false;
        } else if (std::strcmp(argv[i], "-o") == 0 && i + 1 < argc) {
            cfg.output_file = argv[++i];
        } else if (argv[i][0] != '-') {
            cfg.output_file = argv[i];
        } else {
            std::cerr << "Unknown option: " << argv[i] << "\n";
            std::exit(1);
        }
    }

    return cfg;
}

constexpr double deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

}  // namespace

int runTest(int argc, char* argv[]) {
    TestConfig cfg = parseTestArgs(argc, argv);

    int total_frames = 3 * cfg.frames_per_phase;

    std::cout << "Wing rotation test (" << (cfg.is_left ? "left" : "right") << " wing)\n";
    std::cout << "==================\n";
    std::cout << "Phase 1: Stroke plane (gam) " << cfg.gam.start << " -> " << cfg.gam.end << " deg\n";
    std::cout << "Phase 2: Flapping (phi) " << cfg.phi.start << " -> " << cfg.phi.end << " deg\n";
    std::cout << "Phase 3: Pitch (psi) " << cfg.psi.start << " -> " << cfg.psi.end << " deg\n";
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

    // Phase 1: Sweep stroke plane angle
    std::cout << "Phase 1: Stroke plane sweep\n";
    for (int i = 0; i < n; ++i, ++frame) {
        double t = static_cast<double>(i) / (n - 1);
        double gam_deg = cfg.gam.start + t * (cfg.gam.end - cfg.gam.start);
        double phi_deg = cfg.phi.start;
        double psi_deg = cfg.psi.start;

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
            std::cout << "  gam=" << gam_deg << " deg: e_s=[" << orient.e_s.transpose() << "]\n";
        }
    }

    // Phase 2: Sweep flapping angle
    std::cout << "\nPhase 2: Flapping sweep\n";
    double gam_fixed = cfg.gam.end;
    for (int i = 0; i < n; ++i, ++frame) {
        double t = static_cast<double>(i) / (n - 1);
        double gam_deg = gam_fixed;
        double phi_deg = cfg.phi.start + t * (cfg.phi.end - cfg.phi.start);
        double psi_deg = cfg.psi.start;

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
            std::cout << "  phi=" << phi_deg << " deg: e_s=[" << orient.e_s.transpose() << "]\n";
        }
    }

    // Phase 3: Sweep pitch angle
    std::cout << "\nPhase 3: Pitch sweep\n";
    double phi_fixed = cfg.phi.end;
    for (int i = 0; i < n; ++i, ++frame) {
        double t = static_cast<double>(i) / (n - 1);
        double gam_deg = gam_fixed;
        double phi_deg = phi_fixed;
        double psi_deg = cfg.psi.start + t * (cfg.psi.end - cfg.psi.start);

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
            std::cout << "  psi=" << psi_deg << " deg: e_c=[" << orient.e_c.transpose() << "]\n";
        }
    }

    // Write HDF5
    std::cout << "\nWriting to " << cfg.output_file << "...\n";

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

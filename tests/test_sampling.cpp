#include "sampling.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

// Test that samples are within [0, 1]
bool testUnitRange() {
    std::cout << "Testing unit range [0,1]^d..." << std::endl;

    bool passed = true;

    for (size_t dim = 1; dim <= 6; ++dim) {
        SobolSequence seq(dim);

        for (int i = 0; i < 1000; ++i) {
            auto pt = seq.next();

            for (size_t d = 0; d < dim; ++d) {
                if (pt[d] < 0.0 || pt[d] >= 1.0) {
                    std::cout << "  FAILED: dim=" << dim << ", i=" << i
                              << ", d=" << d << ", value=" << pt[d] << std::endl;
                    passed = false;
                }
            }
        }
    }

    if (passed) {
        std::cout << "  PASSED" << std::endl;
    }
    return passed;
}

// Test that samples respect bounds
bool testBounds() {
    std::cout << "Testing bounded samples..." << std::endl;

    std::vector<double> lb = {-1.0, 0.5, 10.0};
    std::vector<double> ub = {1.0, 2.5, 20.0};

    SobolSequence seq(3);
    bool passed = true;

    for (int i = 0; i < 500; ++i) {
        auto pt = seq.next(lb, ub);

        for (size_t d = 0; d < 3; ++d) {
            if (pt[d] < lb[d] || pt[d] > ub[d]) {
                std::cout << "  FAILED: sample out of bounds at i=" << i
                          << ", d=" << d << ", value=" << pt[d]
                          << " not in [" << lb[d] << ", " << ub[d] << "]" << std::endl;
                passed = false;
            }
        }
    }

    if (passed) {
        std::cout << "  PASSED" << std::endl;
    }
    return passed;
}

// Test that reset produces identical sequence
bool testDeterminism() {
    std::cout << "Testing determinism (reset)..." << std::endl;

    SobolSequence seq(3);
    std::vector<std::vector<double>> first_run;

    for (int i = 0; i < 100; ++i) {
        first_run.push_back(seq.next());
    }

    seq.reset();

    bool passed = true;
    for (int i = 0; i < 100; ++i) {
        auto pt = seq.next();
        for (size_t d = 0; d < 3; ++d) {
            if (std::abs(pt[d] - first_run[i][d]) > 1e-15) {
                std::cout << "  FAILED: sequences differ at i=" << i
                          << ", d=" << d << std::endl;
                passed = false;
            }
        }
    }

    if (passed) {
        std::cout << "  PASSED" << std::endl;
    }
    return passed;
}

// Test uniformity: verify distribution is reasonably spread out
// Sobol sequences should have much better coverage than random
bool testUniformity() {
    std::cout << "Testing uniformity (coverage check)..." << std::endl;

    const int N_SAMPLES = 256;  // Use 2^8 for clean Sobol properties
    const int N_BINS = 4;  // 4x4 grid = 16 bins

    SobolSequence seq(2);
    std::vector<std::vector<int>> bins(N_BINS, std::vector<int>(N_BINS, 0));

    // Skip first point (origin)
    seq.next();

    for (int i = 0; i < N_SAMPLES; ++i) {
        auto pt = seq.next();
        int bx = static_cast<int>(pt[0] * N_BINS);
        int by = static_cast<int>(pt[1] * N_BINS);
        bx = std::min(bx, N_BINS - 1);
        by = std::min(by, N_BINS - 1);
        bins[bx][by]++;
    }

    // Expected count per bin
    double expected = static_cast<double>(N_SAMPLES) / (N_BINS * N_BINS);

    // Find min and max bin counts
    int min_count = N_SAMPLES;
    int max_count = 0;
    int empty_bins = 0;

    for (int i = 0; i < N_BINS; ++i) {
        for (int j = 0; j < N_BINS; ++j) {
            if (bins[i][j] < min_count) min_count = bins[i][j];
            if (bins[i][j] > max_count) max_count = bins[i][j];
            if (bins[i][j] == 0) empty_bins++;
        }
    }

    std::cout << "  Expected per bin: " << expected << std::endl;
    std::cout << "  Min bin count: " << min_count << std::endl;
    std::cout << "  Max bin count: " << max_count << std::endl;
    std::cout << "  Empty bins: " << empty_bins << std::endl;

    // Pass criteria:
    // 1. No empty bins
    // 2. No bin has more than 3x expected (badly clustered)
    // 3. No bin has less than expected/3 (sparse region)
    bool passed = true;

    if (empty_bins > 0) {
        std::cout << "  FAILED: has empty bins (poor coverage)" << std::endl;
        passed = false;
    }

    if (max_count > expected * 3) {
        std::cout << "  FAILED: max bin too full (clustering)" << std::endl;
        passed = false;
    }

    if (min_count < expected / 3 && min_count > 0) {
        std::cout << "  FAILED: min bin too sparse" << std::endl;
        passed = false;
    }

    if (passed) {
        std::cout << "  PASSED" << std::endl;
    }
    return passed;
}

// Test convenience function
bool testGenerateSobolSamples() {
    std::cout << "Testing generateSobolSamples()..." << std::endl;

    std::vector<double> lb = {0.0, -1.0};
    std::vector<double> ub = {1.0, 1.0};

    auto samples = generateSobolSamples(50, lb, ub);

    bool passed = true;

    if (samples.size() != 50) {
        std::cout << "  FAILED: expected 50 samples, got " << samples.size() << std::endl;
        passed = false;
    }

    for (size_t i = 0; i < samples.size(); ++i) {
        if (samples[i].size() != 2) {
            std::cout << "  FAILED: expected 2D samples" << std::endl;
            passed = false;
            break;
        }
        for (size_t d = 0; d < 2; ++d) {
            if (samples[i][d] < lb[d] || samples[i][d] > ub[d]) {
                std::cout << "  FAILED: sample " << i << " out of bounds" << std::endl;
                passed = false;
            }
        }
    }

    if (passed) {
        std::cout << "  PASSED" << std::endl;
    }
    return passed;
}

// Test index tracking
bool testIndex() {
    std::cout << "Testing index tracking..." << std::endl;

    SobolSequence seq(2);
    bool passed = true;

    if (seq.index() != 0) {
        std::cout << "  FAILED: initial index should be 0" << std::endl;
        passed = false;
    }

    for (int i = 0; i < 10; ++i) {
        seq.next();
        if (seq.index() != static_cast<size_t>(i + 1)) {
            std::cout << "  FAILED: index after " << (i + 1) << " calls should be "
                      << (i + 1) << ", got " << seq.index() << std::endl;
            passed = false;
        }
    }

    seq.reset();
    if (seq.index() != 0) {
        std::cout << "  FAILED: index after reset should be 0" << std::endl;
        passed = false;
    }

    if (passed) {
        std::cout << "  PASSED" << std::endl;
    }
    return passed;
}

int main() {
    std::cout << "Sampling Unit Tests" << std::endl;
    std::cout << "===================" << std::endl << std::endl;

    int num_passed = 0;
    int num_tests = 6;

    if (testUnitRange()) num_passed++;
    std::cout << std::endl;

    if (testBounds()) num_passed++;
    std::cout << std::endl;

    if (testDeterminism()) num_passed++;
    std::cout << std::endl;

    if (testUniformity()) num_passed++;
    std::cout << std::endl;

    if (testGenerateSobolSamples()) num_passed++;
    std::cout << std::endl;

    if (testIndex()) num_passed++;
    std::cout << std::endl;

    std::cout << "===================" << std::endl;
    std::cout << num_passed << "/" << num_tests << " tests passed" << std::endl;

    return (num_passed == num_tests) ? 0 : 1;
}

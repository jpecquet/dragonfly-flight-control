#pragma once

#include <cstdint>
#include <vector>

// Sobol quasi-random sequence generator
// Provides better coverage of parameter space than uniform random sampling
// Uses Bratley & Fox direction numbers with Gray code optimization
class SobolSequence {
public:
    // Create generator for given dimension (1-6)
    explicit SobolSequence(size_t dim);

    // Generate next point in [0,1]^dim
    std::vector<double> next();

    // Generate next point in [lb, ub]
    std::vector<double> next(const std::vector<double>& lb,
                             const std::vector<double>& ub);

    // Reset sequence to beginning
    void reset();

    // Current index in sequence (0-based)
    size_t index() const;

    // Dimension of the sequence
    size_t dimension() const { return dim_; }

private:
    size_t dim_;
    size_t index_;
    std::vector<uint32_t> x_;                       // Current state
    std::vector<std::vector<uint32_t>> directions_; // Direction numbers

    void initDirections();
};

// Convenience function: generate n Sobol samples in [lb, ub]
std::vector<std::vector<double>> generateSobolSamples(
    size_t n, const std::vector<double>& lb, const std::vector<double>& ub);

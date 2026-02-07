#include "sampling.hpp"

#include <stdexcept>

// Maximum number of bits for Sobol sequence
static constexpr size_t MAX_BITS = 32;
static constexpr double SOBOL_SCALE = 1.0 / static_cast<double>(1ULL << MAX_BITS);

// Initialize direction numbers from initial m-values and a recurrence relation.
// Each Sobol dimension beyond 2 is defined by a primitive polynomial whose degree
// determines the number of initial m-values, and a recurrence that generates the rest.
template <size_t N, typename Recurrence>
static void initDimFromRecurrence(std::vector<uint32_t>& dirs,
                                  const uint32_t (&init_m)[N],
                                  Recurrence recurrence) {
    uint32_t m[MAX_BITS];
    for (size_t i = 0; i < N; ++i) m[i] = init_m[i];
    for (size_t i = N; i < MAX_BITS; ++i) {
        m[i] = recurrence(m, i);
        m[i] &= (1U << (i + 1)) - 1;  // keep only i+1 bits
        m[i] |= 1;                      // ensure odd
    }
    for (size_t i = 0; i < MAX_BITS; ++i) {
        dirs[i] = m[i] << (31 - i);
    }
}

SobolSequence::SobolSequence(size_t dim) : dim_(dim), index_(0), x_(dim, 0) {
    if (dim < 1 || dim > 6) {
        throw std::runtime_error("SobolSequence: dimension must be 1-6, got " + std::to_string(dim));
    }
    directions_.resize(dim);
    initDirections();
}

void SobolSequence::initDirections() {
    for (size_t d = 0; d < dim_; ++d) {
        directions_[d].resize(MAX_BITS);
    }

    // Dimension 1: V_i = 2^(32-i)
    // This generates the sequence 0, 0.5, 0.25, 0.75, 0.125, 0.625, 0.375, 0.875, ...
    for (size_t i = 0; i < MAX_BITS; ++i) {
        directions_[0][i] = 1U << (31 - i);
    }

    if (dim_ < 2) return;

    // Dimension 2: Using polynomial x + 1 (s=1, a=0)
    // m values: 1, 3, 5, 15, 17, 51, 85, 255, ...
    // Direct computation: V_i = V_{i-1} XOR (V_{i-1} >> 1)
    // Start with m_1 = 1
    {
        uint32_t m = 1;
        for (size_t i = 0; i < MAX_BITS; ++i) {
            directions_[1][i] = m << (31 - i);
            m = (m << 1) ^ m;  // next m = 2*m XOR m = 3*m, but we only need odd bits
            // Keep m odd by masking to i+1 bits
            if (i < 31) {
                // m should have i+2 bits max, and be odd
                m &= (1U << (i + 2)) - 1;
                m |= 1;  // ensure odd
            }
        }
    }

    // Dimensions 3-6: each defined by a primitive polynomial with initial m-values
    // and a recurrence relation that generates direction numbers.
    if (dim_ >= 3) {  // x^2 + x + 1
        uint32_t init[] = {1, 1};
        initDimFromRecurrence(directions_[2], init,
            [](const uint32_t* m, size_t i) { return (m[i-1]<<1) ^ m[i-2] ^ (m[i-2]>>2); });
    }
    if (dim_ >= 4) {  // x^3 + x + 1
        uint32_t init[] = {1, 3, 1};
        initDimFromRecurrence(directions_[3], init,
            [](const uint32_t* m, size_t i) { return (m[i-1]<<1) ^ m[i-3] ^ (m[i-3]>>3); });
    }
    if (dim_ >= 5) {  // x^3 + x^2 + 1
        uint32_t init[] = {1, 1, 1};
        initDimFromRecurrence(directions_[4], init,
            [](const uint32_t* m, size_t i) { return (m[i-2]<<1) ^ m[i-3] ^ (m[i-3]>>3); });
    }
    if (dim_ >= 6) {  // x^4 + x + 1
        uint32_t init[] = {1, 1, 3, 3};
        initDimFromRecurrence(directions_[5], init,
            [](const uint32_t* m, size_t i) { return (m[i-1]<<1) ^ m[i-4] ^ (m[i-4]>>4); });
    }
}

std::vector<double> SobolSequence::next() {
    std::vector<double> result(dim_);

    if (index_ == 0) {
        // First point is the origin
        for (size_t d = 0; d < dim_; ++d) {
            result[d] = 0.0;
        }
    } else {
        // Find rightmost zero bit of (index - 1)
        // This gives us the direction number index to use (Gray code optimization)
        uint32_t c = 0;
        uint32_t value = static_cast<uint32_t>(index_ - 1);
        while ((value & 1U) != 0) {
            value >>= 1;
            c++;
        }

        // XOR with direction number for each dimension
        for (size_t d = 0; d < dim_; ++d) {
            x_[d] ^= directions_[d][c];
            // Convert to [0,1) by dividing by 2^32
            result[d] = static_cast<double>(x_[d]) * SOBOL_SCALE;
        }
    }

    index_++;
    return result;
}

std::vector<double> SobolSequence::next(const std::vector<double>& lb,
                                         const std::vector<double>& ub) {
    if (lb.size() != dim_ || ub.size() != dim_) {
        throw std::runtime_error("SobolSequence: bounds dimension mismatch");
    }

    std::vector<double> unit = next();
    for (size_t d = 0; d < dim_; ++d) {
        unit[d] = lb[d] + unit[d] * (ub[d] - lb[d]);
    }
    return unit;
}

void SobolSequence::reset() {
    index_ = 0;
    std::fill(x_.begin(), x_.end(), 0);
}

size_t SobolSequence::index() const {
    return index_;
}

std::vector<std::vector<double>> generateSobolSamples(
    size_t n, const std::vector<double>& lb, const std::vector<double>& ub) {
    if (lb.size() != ub.size()) {
        throw std::runtime_error("generateSobolSamples: lb and ub must have same size");
    }
    if (lb.empty()) {
        throw std::runtime_error("generateSobolSamples: dimension must be at least 1");
    }

    SobolSequence seq(lb.size());
    std::vector<std::vector<double>> samples;
    samples.reserve(n);

    // Skip first point (origin) for better coverage
    seq.next();

    for (size_t i = 0; i < n; ++i) {
        samples.push_back(seq.next(lb, ub));
    }

    return samples;
}

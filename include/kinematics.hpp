#pragma once

#include "eom.hpp"

#include <cmath>
#include <string>
#include <stdexcept>
#include <vector>

struct HarmonicSeries {
    double mean = 0.0;
    std::vector<double> cos_coeff;
    std::vector<double> sin_coeff;
};

inline void validateHarmonicSeries(const HarmonicSeries& series, const char* name) {
    if (series.cos_coeff.size() != series.sin_coeff.size()) {
        throw std::runtime_error(std::string("Harmonic series '") + name +
                                 "' has mismatched cos/sin coefficient lengths");
    }
}

inline double evaluateHarmonicValue(const HarmonicSeries& series, double phase, double harmonic_scale = 1.0) {
    double value = series.mean;
    for (size_t i = 0; i < series.cos_coeff.size(); ++i) {
        const double n_phase = static_cast<double>(i + 1) * phase;
        value += harmonic_scale * (series.cos_coeff[i] * std::cos(n_phase) +
                                   series.sin_coeff[i] * std::sin(n_phase));
    }
    return value;
}

inline double evaluateHarmonicRate(const HarmonicSeries& series, double phase,
                                   double omega, double harmonic_scale = 1.0) {
    double rate = 0.0;
    for (size_t i = 0; i < series.cos_coeff.size(); ++i) {
        const double n = static_cast<double>(i + 1);
        const double n_phase = n * phase;
        rate += harmonic_scale * n * omega *
                (-series.cos_coeff[i] * std::sin(n_phase) +
                 series.sin_coeff[i] * std::cos(n_phase));
    }
    return rate;
}

inline AngleFunc makeAngleFunc(const HarmonicSeries& gamma,
                               const HarmonicSeries& phi,
                               const HarmonicSeries& psi,
                               double phaseOffset, double omega) {
    validateHarmonicSeries(gamma, "gamma");
    validateHarmonicSeries(phi, "phi");
    validateHarmonicSeries(psi, "psi");

    return [=](double t) -> WingAngles {
        const double phase = omega * t + phaseOffset;
        return {
            evaluateHarmonicValue(gamma, phase),
            evaluateHarmonicRate(gamma, phase, omega),
            evaluateHarmonicValue(phi, phase),
            evaluateHarmonicRate(phi, phase, omega),
            evaluateHarmonicValue(psi, phase)
        };
    };
}

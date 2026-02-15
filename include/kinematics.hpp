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

struct MotionHarmonicSeries {
    HarmonicSeries gamma;
    HarmonicSeries phi;
    HarmonicSeries psi;
};

struct MotionParams {
    double omega = 0.0;
    double harmonic_period_wingbeats = 1.0;
    double gamma_mean = 0.0;
    double phi_mean = 0.0;
    double psi_mean = 0.0;
    std::vector<double> gamma_cos;
    std::vector<double> gamma_sin;
    std::vector<double> phi_cos;
    std::vector<double> phi_sin;
    std::vector<double> psi_cos;
    std::vector<double> psi_sin;

    MotionHarmonicSeries toHarmonicSeries() const {
        return {
            {gamma_mean, gamma_cos, gamma_sin},
            {phi_mean, phi_cos, phi_sin},
            {psi_mean, psi_cos, psi_sin}
        };
    }
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

inline AngleFunc makeControlledAngleFunc(
    const HarmonicSeries& gamma_base,
    const HarmonicSeries& phi_base,
    const HarmonicSeries& psi_base,
    double phase_offset,
    double omega,
    double harmonic_period_wingbeats,
    double gamma_mean_base,
    double psi_mean_base,
    double phi_amp_base,
    double wing_phi_amp_base,
    const double& gamma_mean_cmd,
    const double& psi_mean_cmd,
    const double& phi_amp_cmd,
    double eps = 1e-12
) {
    const double* gamma_mean_cmd_ptr = &gamma_mean_cmd;
    const double* psi_mean_cmd_ptr = &psi_mean_cmd;
    const double* phi_amp_cmd_ptr = &phi_amp_cmd;
    if (harmonic_period_wingbeats <= 0.0) {
        throw std::runtime_error("harmonic_period_wingbeats must be > 0");
    }
    const double basis_omega = omega / harmonic_period_wingbeats;

    return [gamma_mean_cmd_ptr,
            psi_mean_cmd_ptr,
            phi_amp_cmd_ptr,
            basis_omega,
            gamma_mean_base,
            psi_mean_base,
            phi_amp_base,
            wing_phi_amp_base,
            gamma_base,
            phi_base,
            psi_base,
            eps,
            phase_offset](double t) -> WingAngles {
        const double phase = basis_omega * t + phase_offset;
        const double gamma_shift = *gamma_mean_cmd_ptr - gamma_mean_base;
        const double psi_shift = *psi_mean_cmd_ptr - psi_mean_base;

        const double phi_ref_amp = (std::abs(phi_amp_base) > eps) ? phi_amp_base : wing_phi_amp_base;
        double phi_scale = 1.0;
        if (std::abs(phi_ref_amp) > eps) {
            phi_scale = *phi_amp_cmd_ptr / phi_ref_amp;
        }

        double gam = evaluateHarmonicValue(gamma_base, phase) + gamma_shift;
        double gam_dot = evaluateHarmonicRate(gamma_base, phase, basis_omega);
        double phi = evaluateHarmonicValue(phi_base, phase, phi_scale);
        double phi_dot = evaluateHarmonicRate(phi_base, phase, basis_omega, phi_scale);

        // If both baseline references are zero, allow controller to inject a first harmonic.
        if (std::abs(phi_ref_amp) <= eps && std::abs(*phi_amp_cmd_ptr) > eps) {
            phi += *phi_amp_cmd_ptr * std::cos(phase);
            phi_dot += -*phi_amp_cmd_ptr * basis_omega * std::sin(phase);
        }

        const double psi = evaluateHarmonicValue(psi_base, phase) + psi_shift;
        return {
            gam,
            gam_dot,
            phi,
            phi_dot,
            psi
        };
    };
}

inline AngleFunc makeAngleFunc(const HarmonicSeries& gamma,
                               const HarmonicSeries& phi,
                               const HarmonicSeries& psi,
                               double phaseOffset, double omega,
                               double harmonic_period_wingbeats = 1.0) {
    validateHarmonicSeries(gamma, "gamma");
    validateHarmonicSeries(phi, "phi");
    validateHarmonicSeries(psi, "psi");
    if (harmonic_period_wingbeats <= 0.0) {
        throw std::runtime_error("harmonic_period_wingbeats must be > 0");
    }
    const double basis_omega = omega / harmonic_period_wingbeats;

    return [=](double t) -> WingAngles {
        const double phase = basis_omega * t + phaseOffset;
        return {
            evaluateHarmonicValue(gamma, phase),
            evaluateHarmonicRate(gamma, phase, basis_omega),
            evaluateHarmonicValue(phi, phase),
            evaluateHarmonicRate(phi, phase, basis_omega),
            evaluateHarmonicValue(psi, phase)
        };
    };
}

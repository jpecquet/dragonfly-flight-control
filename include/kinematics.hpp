#pragma once

#include <cmath>
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

// Wing angles at a given time
struct WingAngles {
    double gam = 0.0;      // Stroke plane angle
    double gam_dot = 0.0;  // Stroke plane angular velocity
    double phi = 0.0;      // Stroke angle
    double phi_dot = 0.0;  // Stroke angular velocity
    double psi = 0.0;      // Pitch angle
    double cone = 0.0;     // Dynamic coning offset added to wing config cone angle
    double cone_dot = 0.0; // Dynamic coning angular velocity
};

// Function type for computing wing angles from time
using AngleFunc = std::function<WingAngles(double t)>;

struct HarmonicSeries {
    double mean = 0.0;
    std::vector<double> amplitude_coeff;
    std::vector<double> phase_coeff;
};

struct MotionParams {
    double omega = 0.0;
    double harmonic_period_wingbeats = 1.0;
    HarmonicSeries gamma;
    HarmonicSeries phi;
    HarmonicSeries psi;
    HarmonicSeries cone;  // Dynamic coning angle beta(t) [rad], added to wing config cone_angle
};

inline void validateHarmonicSeries(const HarmonicSeries& series, const char* name) {
    if (series.amplitude_coeff.size() != series.phase_coeff.size()) {
        throw std::runtime_error(std::string("Harmonic series '") + name +
                                 "' has mismatched amplitude/phase coefficient lengths");
    }
}

inline double evaluateHarmonicValue(const HarmonicSeries& series, double phase, double harmonic_scale = 1.0) {
    double value = series.mean;
    for (size_t i = 0; i < series.amplitude_coeff.size(); ++i) {
        const double n_phase = static_cast<double>(i + 1) * phase;
        value += harmonic_scale * series.amplitude_coeff[i] *
                 std::cos(n_phase + series.phase_coeff[i]);
    }
    return value;
}

inline double evaluateHarmonicRate(const HarmonicSeries& series, double phase,
                                   double omega, double harmonic_scale = 1.0) {
    double rate = 0.0;
    for (size_t i = 0; i < series.amplitude_coeff.size(); ++i) {
        const double n = static_cast<double>(i + 1);
        const double n_phase = n * phase;
        rate += -harmonic_scale * n * omega * series.amplitude_coeff[i] *
                std::sin(n_phase + series.phase_coeff[i]);
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
    double eps = 1e-12,
    const HarmonicSeries& cone_base = HarmonicSeries()
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
            cone_base,
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
        const double cone = evaluateHarmonicValue(cone_base, phase);
        const double cone_dot = evaluateHarmonicRate(cone_base, phase, basis_omega);
        return {
            gam,
            gam_dot,
            phi,
            phi_dot,
            psi,
            cone,
            cone_dot
        };
    };
}

inline AngleFunc makeAngleFunc(const HarmonicSeries& gamma,
                               const HarmonicSeries& phi,
                               const HarmonicSeries& psi,
                               double phaseOffset, double omega,
                               double harmonic_period_wingbeats = 1.0,
                               const HarmonicSeries& cone = HarmonicSeries()) {
    validateHarmonicSeries(gamma, "gamma");
    validateHarmonicSeries(phi, "phi");
    validateHarmonicSeries(psi, "psi");
    validateHarmonicSeries(cone, "cone");
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
            evaluateHarmonicValue(psi, phase),
            evaluateHarmonicValue(cone, phase),
            evaluateHarmonicRate(cone, phase, basis_omega)
        };
    };
}

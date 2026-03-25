#pragma once

#include <cmath>
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

enum class PhiWaveform { Fourier, Berman };  // Berman: smoothed triangular (Eq. 2.10)
enum class PsiWaveform { Fourier, Berman };  // Berman: hyperbolic tangent (Eq. 2.12)

// Wing angles at a given time
struct WingAngles {
    double gam = 0.0;      // Stroke plane angle
    double gam_dot = 0.0;  // Stroke plane angular velocity
    double phi = 0.0;      // Stroke angle
    double phi_dot = 0.0;  // Stroke angular velocity
    double psi = 0.0;      // Pitch angle
    double psi_dot = 0.0;  // Pitch angular velocity
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
    PhiWaveform phi_waveform = PhiWaveform::Fourier;
    double phi_k = 0.0;  // Berman K in (0, 1): 0→sinusoidal, 1→triangular
    PsiWaveform psi_waveform = PsiWaveform::Fourier;
    double psi_k = 0.0;  // Berman C_η > 0: 0→sinusoidal, ∞→step function
};

// Berman (2007) Eq. 2.10: smoothed triangular waveform for flapping angle.
// phi(t) = phi_mean + (phi_m / asin(k)) * asin(k * cos(theta))
// Outputs value (relative to mean) and its time derivative. Uses cos for phase convention.
inline void evalBermanPhi(double theta, double phi_m, double k, double basis_omega,
                          double& val, double& rate) {
    const double k_cos = k * std::cos(theta);
    val = (phi_m / std::asin(k)) * std::asin(k_cos);
    const double denom = std::sqrt(std::max(0.0, 1.0 - k_cos * k_cos));
    rate = (phi_m / std::asin(k)) * k * (-std::sin(theta)) / std::max(denom, 1e-12) * basis_omega;
}

// Berman (2007) Eq. 2.12: hyperbolic tangent waveform for pitch angle.
// psi(t) = psi_mean + (psi_m / tanh(k)) * tanh(k * cos(theta))
// Returns value relative to mean.
inline double evalBermanPsi(double theta, double psi_m, double k) {
    return (psi_m / std::tanh(k)) * std::tanh(k * std::cos(theta));
}

// Time derivative of Berman psi waveform.
// d/dt[tanh(k cos θ)] = -k sin θ · sech²(k cos θ) · dθ/dt
inline double evalBermanPsiRate(double theta, double psi_m, double k, double basis_omega) {
    const double kc = k * std::cos(theta);
    const double sech = 1.0 / std::cosh(kc);
    return (psi_m / std::tanh(k)) * (-k * std::sin(theta)) * sech * sech * basis_omega;
}

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
    const HarmonicSeries& cone_base = HarmonicSeries(),
    PhiWaveform phi_waveform = PhiWaveform::Fourier,
    double phi_k = 0.0,
    PsiWaveform psi_waveform = PsiWaveform::Fourier,
    double psi_k = 0.0,
    const double* psi_amp_cmd_ptr_ext = nullptr
) {
    const double* gamma_mean_cmd_ptr = &gamma_mean_cmd;
    const double* psi_mean_cmd_ptr = &psi_mean_cmd;
    const double* phi_amp_cmd_ptr = &phi_amp_cmd;
    if (harmonic_period_wingbeats <= 0.0) {
        throw std::runtime_error("harmonic_period_wingbeats must be > 0");
    }
    const double basis_omega = omega / harmonic_period_wingbeats;

    // Compute psi_amp reference for scaling (same pattern as phi_amp)
    const double psi_amp_ref = psi_base.amplitude_coeff.empty() ? 0.0 : psi_base.amplitude_coeff[0];

    return [gamma_mean_cmd_ptr,
            psi_mean_cmd_ptr,
            phi_amp_cmd_ptr,
            psi_amp_cmd_ptr_ext,
            psi_amp_ref,
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
            phase_offset,
            phi_waveform,
            phi_k,
            psi_waveform,
            psi_k](double t) -> WingAngles {
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

        double phi_val, phi_dot_val;
        if (phi_waveform == PhiWaveform::Berman) {
            const double phi_m = (phi_base.amplitude_coeff.empty() ? 0.0 : phi_base.amplitude_coeff[0]) * phi_scale;
            const double phi_phase = phi_base.phase_coeff.empty() ? 0.0 : phi_base.phase_coeff[0];
            evalBermanPhi(phase + phi_phase, phi_m, phi_k, basis_omega, phi_val, phi_dot_val);
            phi_val += phi_base.mean;
        } else {
            phi_val = evaluateHarmonicValue(phi_base, phase, phi_scale);
            phi_dot_val = evaluateHarmonicRate(phi_base, phase, basis_omega, phi_scale);
            // If both baseline references are zero, allow controller to inject a first harmonic.
            if (std::abs(phi_ref_amp) <= eps && std::abs(*phi_amp_cmd_ptr) > eps) {
                phi_val += *phi_amp_cmd_ptr * std::cos(phase);
                phi_dot_val += -*phi_amp_cmd_ptr * basis_omega * std::sin(phase);
            }
        }

        // Compute psi_amp scale factor (1.0 if no external command pointer)
        double psi_scale = 1.0;
        if (psi_amp_cmd_ptr_ext && std::abs(psi_amp_ref) > eps) {
            psi_scale = *psi_amp_cmd_ptr_ext / psi_amp_ref;
        }

        double psi_val, psi_dot_val;
        if (psi_waveform == PsiWaveform::Berman) {
            const double psi_m = (psi_base.amplitude_coeff.empty() ? 0.0 : psi_base.amplitude_coeff[0]) * psi_scale;
            const double psi_phase = psi_base.phase_coeff.empty() ? 0.0 : psi_base.phase_coeff[0];
            psi_val = psi_base.mean + evalBermanPsi(phase + psi_phase, psi_m, psi_k) + psi_shift;
            psi_dot_val = evalBermanPsiRate(phase + psi_phase, psi_m, psi_k, basis_omega);
        } else {
            psi_val = evaluateHarmonicValue(psi_base, phase, psi_scale) + psi_shift;
            psi_dot_val = evaluateHarmonicRate(psi_base, phase, basis_omega, psi_scale);
        }

        const double cone = evaluateHarmonicValue(cone_base, phase);
        const double cone_dot = evaluateHarmonicRate(cone_base, phase, basis_omega);
        return {gam, gam_dot, phi_val, phi_dot_val, psi_val, psi_dot_val, cone, cone_dot};
    };
}

inline AngleFunc makeAngleFunc(const HarmonicSeries& gamma,
                               const HarmonicSeries& phi,
                               const HarmonicSeries& psi,
                               double phaseOffset, double omega,
                               double harmonic_period_wingbeats = 1.0,
                               const HarmonicSeries& cone = HarmonicSeries(),
                               PhiWaveform phi_waveform = PhiWaveform::Fourier,
                               double phi_k = 0.0,
                               PsiWaveform psi_waveform = PsiWaveform::Fourier,
                               double psi_k = 0.0) {
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

        double phi_val, phi_dot_val;
        if (phi_waveform == PhiWaveform::Berman) {
            const double phi_m = phi.amplitude_coeff.empty() ? 0.0 : phi.amplitude_coeff[0];
            const double phi_phase = phi.phase_coeff.empty() ? 0.0 : phi.phase_coeff[0];
            evalBermanPhi(phase + phi_phase, phi_m, phi_k, basis_omega, phi_val, phi_dot_val);
            phi_val += phi.mean;
        } else {
            phi_val = evaluateHarmonicValue(phi, phase);
            phi_dot_val = evaluateHarmonicRate(phi, phase, basis_omega);
        }

        double psi_val, psi_dot_val;
        if (psi_waveform == PsiWaveform::Berman) {
            const double psi_m = psi.amplitude_coeff.empty() ? 0.0 : psi.amplitude_coeff[0];
            const double psi_phase = psi.phase_coeff.empty() ? 0.0 : psi.phase_coeff[0];
            psi_val = psi.mean + evalBermanPsi(phase + psi_phase, psi_m, psi_k);
            psi_dot_val = evalBermanPsiRate(phase + psi_phase, psi_m, psi_k, basis_omega);
        } else {
            psi_val = evaluateHarmonicValue(psi, phase);
            psi_dot_val = evaluateHarmonicRate(psi, phase, basis_omega);
        }

        return {
            evaluateHarmonicValue(gamma, phase),
            evaluateHarmonicRate(gamma, phase, basis_omega),
            phi_val,
            phi_dot_val,
            psi_val,
            psi_dot_val,
            evaluateHarmonicValue(cone, phase),
            evaluateHarmonicRate(cone, phase, basis_omega)
        };
    };
}

#include "wing.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace {
constexpr double SPAN_EVAL_POINT = 2.0 / 3.0;
constexpr double EPS = 1e-12;

double clampUnitInterval(double x) {
    return std::max(0.0, std::min(1.0, x));
}

double clampSymmetricUnitInterval(double x) {
    return std::max(-1.0, std::min(1.0, x));
}

// Antiderivative of sqrt(1 - x^2) over x in [-1, 1].
double semiCircleIntegral(double x) {
    const double xc = clampSymmetricUnitInterval(x);
    const double rem = std::max(0.0, 1.0 - xc * xc);
    return 0.5 * (xc * std::sqrt(rem) + std::asin(xc));
}

// Antiderivative of x * sqrt(1 - x^2) over x in [-1, 1].
double xSemiCircleIntegral(double x) {
    const double xc = clampSymmetricUnitInterval(x);
    const double rem = std::max(0.0, 1.0 - xc * xc);
    return -std::pow(rem, 1.5) / 3.0;
}

// Composite full-ellipse chord profile along normalized span station eta in [0,1]:
// chord(eta) ∝ sqrt(1 - (2*eta - 1)^2).
double fullEllipseChordAreaBetween(double a, double b) {
    const double aa = clampUnitInterval(a);
    const double bb = clampUnitInterval(b);
    const double xa = 2.0 * aa - 1.0;
    const double xb = 2.0 * bb - 1.0;
    return 0.5 * (semiCircleIntegral(xb) - semiCircleIntegral(xa));
}

double fullEllipseEtaChordMomentBetween(double a, double b) {
    const double aa = clampUnitInterval(a);
    const double bb = clampUnitInterval(b);
    const double xa = 2.0 * aa - 1.0;
    const double xb = 2.0 * bb - 1.0;
    const double a_part = semiCircleIntegral(xb) - semiCircleIntegral(xa);
    const double b_part = xSemiCircleIntegral(xb) - xSemiCircleIntegral(xa);
    return 0.25 * (a_part + b_part);
}

void buildCompositeEllipseBladeGrid(int n_blades, std::vector<double>& eta,
                                    std::vector<double>& area_weights) {
    eta.clear();
    area_weights.clear();

    if (n_blades <= 1) {
        eta.push_back(SPAN_EVAL_POINT);
        area_weights.push_back(1.0);
        return;
    }

    eta.reserve(static_cast<size_t>(n_blades));
    area_weights.reserve(static_cast<size_t>(n_blades));

    constexpr double total_area_factor = M_PI / 4.0;  // ∫_0^1 sqrt(1-eta^2) d eta
    for (int i = 0; i < n_blades; ++i) {
        const double a = static_cast<double>(i) / static_cast<double>(n_blades);
        const double b = static_cast<double>(i + 1) / static_cast<double>(n_blades);

        const double area_bin = fullEllipseChordAreaBetween(a, b);
        const double eta_moment_bin = fullEllipseEtaChordMomentBetween(a, b);

        const double eta_center = (area_bin > 1e-15) ? (eta_moment_bin / area_bin)
                                                     : (0.5 * (a + b));
        const double area_weight = area_bin / total_area_factor;

        eta.push_back(clampUnitInterval(eta_center));
        area_weights.push_back(std::max(0.0, area_weight));
    }

    double sum_w = 0.0;
    for (double w : area_weights) sum_w += w;
    if (sum_w <= 0.0) {
        throw std::runtime_error("Invalid blade element area weights (non-positive sum)");
    }
    for (double& w : area_weights) w /= sum_w;
}

void buildTwistH1Scales(const PitchTwistH1Model& twist_model,
                        const std::vector<double>& blade_eta,
                        std::vector<double>& scales) {
    scales.clear();
    scales.resize(blade_eta.size(), 1.0);
    if (!twist_model.enabled) {
        return;
    }
    if (!std::isfinite(twist_model.ref_eta) || twist_model.ref_eta <= 0.0) {
        throw std::runtime_error("pitch twist ref_eta must be finite and > 0");
    }

    const double ref_coeff = std::hypot(twist_model.c1, twist_model.s1);
    if (ref_coeff <= EPS) {
        throw std::runtime_error(
            "pitch twist requires non-zero first harmonic pitch coefficient at reference span station"
        );
    }

    for (size_t i = 0; i < blade_eta.size(); ++i) {
        const double eta = blade_eta[i];
        const double coeff_eta =
            ((ref_coeff - twist_model.root_coeff) * (eta / twist_model.ref_eta)) + twist_model.root_coeff;
        scales[i] = coeff_eta / ref_coeff;
    }
}
}  // namespace

Wing::Wing(const std::string& name, double mu0, double lb0, WingSide side,
           const BladeElementAeroParams& aero_params, double cone_angle, AngleFunc angleFunc,
           int n_blade_elements, const PitchTwistH1Model& pitch_twist_h1)
    : name_(name), mu0_(mu0), lb0_(lb0), side_(side), cone_angle_(cone_angle),
      n_blade_elements_(std::max(1, n_blade_elements)),
      blade_(aero_params), pitch_twist_h1_(pitch_twist_h1), angleFunc_(std::move(angleFunc)) {
    buildCompositeEllipseBladeGrid(n_blade_elements_, blade_eta_, blade_area_weights_);
    buildTwistH1Scales(pitch_twist_h1_, blade_eta_, twist_h1_scales_);
}

Wing::Wing(const std::string& name, double mu0, double lb0, WingSide side,
           double Cd_min, double Cl0, double cone_angle, AngleFunc angleFunc,
           int n_blade_elements, const PitchTwistH1Model& pitch_twist_h1)
    : Wing(name, mu0, lb0, side,
           BladeElementAeroParams::sinusoidal(Cd_min, Cl0),
           cone_angle, std::move(angleFunc), n_blade_elements, pitch_twist_h1) {}

Vec3 Wing::computeForce(
    double t,
    const Vec3& ub,
    SingleWingVectors& vecs
) const {
    // Get angles from the angle function
    WingAngles angles = angleFunc_(t);
    // Simulator convention update: interpreted stroke-plane angle is pi - gamma.
    const double gam = M_PI - angles.gam;
    const double gam_dot = -angles.gam_dot;

    // Compute wing orientation at reference span station pitch (input Fourier coefficients).
    WingOrientation orient_ref = computeWingOrientation(
        gam, angles.phi, angles.psi, cone_angle_, side_ == WingSide::Left);

    // Store orientation vectors
    vecs.e_s = orient_ref.e_s;
    vecs.e_r = orient_ref.e_r;
    vecs.e_c = orient_ref.e_c;

    // Contribution from stroke plane rotation (gam_dot): omega_gam x r
    // omega_gam = sign * gam_dot * ey (sign depends on wing side due to rotation convention)
    double gam_sign = (side_ == WingSide::Left) ? -1.0 : 1.0;
    Vec3 ey(0, 1, 0);
    Vec3 omega_gam = gam_sign * gam_dot * ey;
    // For conical flapping, spanwise orbit radius about the cone axis is lb0*cos(cone).
    const double flap_speed_scale = std::cos(cone_angle_);
    const double force_coefficient = 0.5 * mu0_ / lb0_;
    const double psi_h1_value = pitch_twist_h1_.enabled
        ? (pitch_twist_h1_.c1 * std::cos(pitch_twist_h1_.basis_omega * t + pitch_twist_h1_.phase_offset) +
           pitch_twist_h1_.s1 * std::sin(pitch_twist_h1_.basis_omega * t + pitch_twist_h1_.phase_offset))
        : 0.0;

    vecs.lift = Vec3::Zero();
    vecs.drag = Vec3::Zero();
    Vec3 total_force = Vec3::Zero();

    for (size_t i = 0; i < blade_eta_.size(); ++i) {
        const double eta = blade_eta_[i];
        const double coeff_i = force_coefficient * blade_area_weights_[i];
        const double psi_eta = pitch_twist_h1_.enabled
            ? (angles.psi + (twist_h1_scales_[i] - 1.0) * psi_h1_value)
            : angles.psi;
        const WingOrientation orient_eta = pitch_twist_h1_.enabled
            ? computeWingOrientation(gam, angles.phi, psi_eta, cone_angle_, side_ == WingSide::Left)
            : orient_ref;

        // Wing velocity at span station eta.
        Vec3 v_phi = eta * lb0_ * angles.phi_dot * flap_speed_scale * orient_eta.e_s;
        Vec3 r = eta * lb0_ * orient_eta.e_r;
        Vec3 v_gam = omega_gam.cross(r);
        Vec3 uw = ub + v_phi + v_gam;

        // Wing velocity projected on plane normal to spanwise direction.
        Vec3 u = uw - uw.dot(orient_eta.e_r) * orient_eta.e_r;

        Vec3 lift_i, drag_i;
        total_force += blade_.computeForce(
            u, orient_eta.e_r, orient_eta.e_c, coeff_i, lift_i, drag_i);
        vecs.lift += lift_i;
        vecs.drag += drag_i;
    }

    return total_force;
}

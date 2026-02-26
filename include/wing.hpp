#pragma once

#include "blade_element.hpp"
#include "eom.hpp"
#include "kinematics.hpp"
#include "rotation.hpp"

#include <stdexcept>
#include <string>
#include <vector>

enum class WingSide { Left, Right };

// Parse "left"/"right" string to WingSide enum
inline WingSide parseSide(const std::string& s) {
    if (s == "left") return WingSide::Left;
    if (s == "right") return WingSide::Right;
    throw std::runtime_error("Invalid wing side: '" + s + "' (expected 'left' or 'right')");
}

// Configuration for a single wing (used for variable wing count support)
struct WingConfig : MotionParams {
    std::string name;       // Wing identifier (e.g., "fore", "hind", "aux")
    WingSide side;          // Left or Right
    double mu0 = 0.0;             // Mass parameter
    double lb0 = 0.0;             // Span length
    DragCoefficientModel drag_model = DragCoefficientModel::Sinusoidal;
    LiftCoefficientModel lift_model = LiftCoefficientModel::Sinusoidal;
    double Cd_min = 0.0;          // Minimum drag coefficient
    double Cd_max = 0.0;          // Max drag coefficient for sinusoidal model
    double Cd_alpha_neutral = 0.0; // Neutral AoA (rad) for sinusoidal drag
    std::vector<double> Cd_fourier; // Fourier drag model coefficients
    double Cl0 = 0.0;             // Lift coefficient
    double Cl_alpha_slope = 0.0;   // Linear model slope (Cl/rad)
    double Cl_alpha_neutral = 0.0; // Neutral AoA (rad) for lift models
    double Cl_min = -1.0e12;       // Linear model lower saturation
    double Cl_max = 1.0e12;        // Linear model upper saturation
    double phase_offset = 0.0;    // Phase offset from reference (radians)
    double cone_angle = 0.0;      // Beta angle from stroke plane (radians): sets cone half-angle (pi/2 - beta)
    int n_blade_elements = 1;     // Number of spanwise blade elements for force integration
    bool has_psi_twist_h1 = false;   // Enable linear spanwise twist based on 1st pitch harmonic amplitude
    double psi_twist_h1_root = 0.0;  // Root coefficient (radians) for first pitch harmonic
    double psi_twist_ref_eta = 0.75; // Reference eta where input first harmonic coefficient is defined

    // Optional per-wing kinematic parameters (if empty, global simulation parameters are used).
    bool has_custom_motion = false;

    WingConfig() = default;
    WingConfig(const std::string& name_, WingSide side_, double mu0_, double lb0_,
               double Cd_min_, double Cl0_, double phase_offset_ = 0.0, double cone_angle_ = 0.0,
               int n_blade_elements_ = 1, bool has_psi_twist_h1_ = false,
               double psi_twist_h1_root_ = 0.0, double psi_twist_ref_eta_ = 0.75)
        : name(name_), side(side_), mu0(mu0_), lb0(lb0_),
          Cd_min(Cd_min_), Cd_max(Cd_min_ + 2.0), Cl0(Cl0_), phase_offset(phase_offset_), cone_angle(cone_angle_),
          n_blade_elements(n_blade_elements_), has_psi_twist_h1(has_psi_twist_h1_),
          psi_twist_h1_root(psi_twist_h1_root_), psi_twist_ref_eta(psi_twist_ref_eta_) {}
};

struct PitchTwistH1Model {
    bool enabled = false;
    double root_coeff = 0.0;     // Root first-harmonic pitch coefficient (radians)
    double ref_eta = 0.75;       // Span station where input Fourier coefficients are defined
    double c1 = 0.0;             // Base first-harmonic cosine coefficient
    double s1 = 0.0;             // Base first-harmonic sine coefficient
    double basis_omega = 0.0;    // Harmonic basis angular rate
    double phase_offset = 0.0;   // Per-wing phase offset
};

class Wing {
public:
    Wing(const std::string& name, double mu0, double lb0, WingSide side,
         const BladeElementAeroParams& aero_params, double cone_angle, AngleFunc angleFunc,
         int n_blade_elements = 1,
         const PitchTwistH1Model& pitch_twist_h1 = PitchTwistH1Model());

    Wing(const std::string& name, double mu0, double lb0, WingSide side,
         double Cd_min, double Cl0, double cone_angle, AngleFunc angleFunc,
         int n_blade_elements = 1,
         const PitchTwistH1Model& pitch_twist_h1 = PitchTwistH1Model());

    // Compute force at time t given body velocity
    Vec3 computeForce(
        double t,                 // Time
        const Vec3& ub,           // Body velocity
        SingleWingVectors& vecs   // Output: orientation and force vectors
    ) const;

    // Update the angle function (for reusing Wing objects with new kinematics)
    void setAngleFunc(AngleFunc func) { angleFunc_ = std::move(func); }

    // Accessors
    const std::string& name() const { return name_; }
    double mu0() const { return mu0_; }
    double lb0() const { return lb0_; }
    WingSide side() const { return side_; }
    int nBladeElements() const { return n_blade_elements_; }
    const std::vector<double>& bladeEta() const { return blade_eta_; }
    bool hasPitchTwistH1() const { return pitch_twist_h1_.enabled; }
    double pitchTwistH1Root() const { return pitch_twist_h1_.root_coeff; }
    double pitchTwistH1RefEta() const { return pitch_twist_h1_.ref_eta; }

private:
    std::string name_;     // Wing identifier
    double mu0_;           // Wing mass parameter
    double lb0_;           // Wing length
    WingSide side_;        // Left or Right
    double cone_angle_;    // Coning angle (radians)
    int n_blade_elements_; // Number of spanwise blade elements
    BladeElement blade_;   // Blade element model (owns Cd_min, Cl0)
    std::vector<double> blade_eta_;          // Span stations (0..1, root->tip)
    std::vector<double> blade_area_weights_; // Area fractions, sum to 1
    PitchTwistH1Model pitch_twist_h1_;
    std::vector<double> twist_h1_scales_;    // Per-blade scale on first pitch harmonic
    AngleFunc angleFunc_;  // Function to compute angles from time
};

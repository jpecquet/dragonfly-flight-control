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
    double Cd0 = 0.0;             // Drag coefficient
    double Cl0 = 0.0;             // Lift coefficient
    double phase_offset = 0.0;    // Phase offset from reference (radians)
    double cone_angle = 0.0;      // Coning angle (radians): tilts the flapping plane about the stroke direction

    // Optional per-wing kinematic parameters (if empty, global simulation parameters are used).
    bool has_custom_motion = false;

    WingConfig() = default;
    WingConfig(const std::string& name_, WingSide side_, double mu0_, double lb0_,
               double Cd0_, double Cl0_, double phase_offset_ = 0.0, double cone_angle_ = 0.0)
        : name(name_), side(side_), mu0(mu0_), lb0(lb0_),
          Cd0(Cd0_), Cl0(Cl0_), phase_offset(phase_offset_), cone_angle(cone_angle_) {}
};

class Wing {
public:
    Wing(const std::string& name, double mu0, double lb0, WingSide side,
         double Cd0, double Cl0, double cone_angle, AngleFunc angleFunc);

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

private:
    std::string name_;     // Wing identifier
    double mu0_;           // Wing mass parameter
    double lb0_;           // Wing length
    WingSide side_;        // Left or Right
    double cone_angle_;    // Coning angle (radians)
    BladeElement blade_;   // Blade element model (owns Cd0, Cl0)
    AngleFunc angleFunc_;  // Function to compute angles from time
};

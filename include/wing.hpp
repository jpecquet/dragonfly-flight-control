#pragma once

#include "linalg.hpp"
#include "blade_element.hpp"
#include "rotation.hpp"
#include "eom.hpp"

#include <string>

enum class WingSide { Left, Right };

// Configuration for a single wing (used for variable wing count support)
struct WingConfig {
    std::string name;       // Wing identifier (e.g., "fore", "hind", "aux")
    WingSide side;          // Left or Right
    double mu0;             // Mass parameter
    double lb0;             // Span length
    double Cd0;             // Drag coefficient
    double Cl0;             // Lift coefficient
    double phaseOffset;     // Phase offset from reference (radians)

    WingConfig() = default;
    WingConfig(const std::string& name_, WingSide side_, double mu0_, double lb0_,
               double Cd0_, double Cl0_, double phaseOffset_ = 0.0)
        : name(name_), side(side_), mu0(mu0_), lb0(lb0_),
          Cd0(Cd0_), Cl0(Cl0_), phaseOffset(phaseOffset_) {}
};

class Wing {
public:
    Wing(const std::string& name, double mu0, double lb0, WingSide side,
         double Cd0, double Cl0, AngleFunc angleFunc);

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
    BladeElement blade_;   // Blade element model (owns Cd0, Cl0)
    AngleFunc angleFunc_;  // Function to compute angles from time
};

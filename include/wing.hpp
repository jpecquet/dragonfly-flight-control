#pragma once

#include "linalg.hpp"
#include "blade_element.hpp"
#include "rotation.hpp"
#include "eom.hpp"

#include <string>

enum class WingSide { Left, Right };

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

#pragma once

#include "config.hpp"
#include "wing.hpp"

#include <string>
#include <vector>

// Kinematic parameter that can be fixed or variable
struct KinematicParam {
    std::string name;
    bool is_variable;
    double value;      // Current/fixed value
    double min_bound;  // Only used if variable
    double max_bound;  // Only used if variable
};

// All kinematic parameters for optimization
// Note: Phase offsets are now per-wing in WingConfig, not global
struct KinematicParams {
    KinematicParam omg0;  // Wing beat frequency
    KinematicParam gam0;  // Stroke plane angle
    KinematicParam phi0;  // Stroke amplitude
    KinematicParam psim;  // Mean pitch angle
    KinematicParam dpsi;  // Pitch amplitude
    KinematicParam dlt0;  // Pitch phase offset

    // Get list of variable parameter names
    std::vector<std::string> variableNames() const;

    // Get list of all parameter names (for output header)
    std::vector<std::string> allNames() const;

    // Get current values of all parameters
    std::vector<double> allValues() const;

    // Get values of variable parameters only
    std::vector<double> variableValues() const;

    // Set values of variable parameters from optimization vector
    void setVariableValues(const std::vector<double>& x);

    // Get bounds for variable parameters
    void getVariableBounds(std::vector<double>& lb, std::vector<double>& ub) const;

    // Number of variable parameters
    size_t numVariable() const;

    // Load from config file
    static KinematicParams fromConfig(const Config& cfg);
};

// Fixed physical parameters (not optimized)
struct PhysicalParams {
    std::vector<WingConfig> wings;  // Wing configurations

    static PhysicalParams fromConfig(const Config& cfg);
};

// Pre-allocated buffers for optimization (avoids allocation in hot path)
struct OptimBuffers {
    std::vector<Wing> wings;
    std::vector<SingleWingVectors> scratch1;
    std::vector<SingleWingVectors> scratch2;

    // Initialize buffers for given wing count
    void init(const KinematicParams& kin, const PhysicalParams& phys);
};

// Compute mean squared acceleration with pre-allocated buffers (use in hot paths)
double wingBeatAccel(const KinematicParams& kin, const PhysicalParams& phys,
                         OptimBuffers& buffers, double ux, double uz, int N = 40);

// Convenience overload that allocates internally (for one-off calls)
double wingBeatAccel(const KinematicParams& kin, const PhysicalParams& phys,
                         double ux, double uz, int N = 40);

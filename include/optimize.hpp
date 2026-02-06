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
    KinematicParam omega;       // Wing beat frequency
    KinematicParam gamma_mean;  // Mean stroke plane angle
    KinematicParam gamma_amp;   // Stroke plane oscillation amplitude
    KinematicParam gamma_phase; // Stroke plane phase offset
    KinematicParam phi_amp;     // Stroke amplitude
    KinematicParam psi_mean;    // Mean pitch angle
    KinematicParam psi_amp;     // Pitch amplitude
    KinematicParam psi_phase;   // Pitch phase offset

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

private:
    template<typename Func>
    void forEachParam(Func&& f) {
        f("omega", omega); f("gamma_mean", gamma_mean);
        f("gamma_amp", gamma_amp); f("gamma_phase", gamma_phase);
        f("phi_amp", phi_amp); f("psi_mean", psi_mean);
        f("psi_amp", psi_amp); f("psi_phase", psi_phase);
    }
    template<typename Func>
    void forEachParam(Func&& f) const {
        f("omega", omega); f("gamma_mean", gamma_mean);
        f("gamma_amp", gamma_amp); f("gamma_phase", gamma_phase);
        f("phi_amp", phi_amp); f("psi_mean", psi_mean);
        f("psi_amp", psi_amp); f("psi_phase", psi_phase);
    }
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

// Optimization algorithm selection
enum class OptimAlgorithm {
    COBYLA,     // Local (current default, grid search + local)
    DIRECT,     // Global: dividing rectangles
    MLSL,       // Global: multi-level single-linkage
    CRS2,       // Global: controlled random search
    MULTISTART  // Multi-start local optimization with Sobol sampling
};

// Parse algorithm name from string
OptimAlgorithm parseAlgorithm(const std::string& name);

// Optimizer configuration
struct OptimConfig {
    OptimAlgorithm algorithm = OptimAlgorithm::COBYLA;
    int n_samples = 100;           // Sobol samples for MULTISTART algorithm
    int n_grid = 5;                // Grid resolution for COBYLA grid search
    int max_eval = 200;            // Max evaluations per local optimization
    double equilibrium_tol = 1e-6; // Threshold for equilibrium detection
    bool map_landscape = false;    // Generate landscape HDF5 file
    int map_resolution = 50;       // Resolution for landscape mapping

    static OptimConfig fromConfig(const Config& cfg);
};

// Landscape data for visualization
struct LandscapeData {
    std::vector<std::string> param_names;  // Names of varying parameters
    std::vector<double> param1_values;     // First parameter values
    std::vector<double> param2_values;     // Second parameter values (if 2D)
    std::vector<std::vector<double>> objective_values;  // Objective at each point
    double ux;  // Forward velocity

    void writeHDF5(const std::string& filename) const;
};

// Compute objective function landscape for visualization
LandscapeData computeLandscape(
    const KinematicParams& kin_template,
    const PhysicalParams& phys,
    double ux, int resolution);

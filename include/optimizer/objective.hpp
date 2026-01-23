#pragma once

#include "optim_params.hpp"
#include "../linalg.hpp"

// Compute instantaneous acceleration from gravity and aerodynamic forces
// Returns the acceleration vector [ax, ay, az]
Vec3 instantAccel(double t, const OptimParams& optim, const FlightCondition& flight,
                  const FixedParams& fixed);

// Compute mean squared acceleration over one wingbeat
// Uses trapezoidal integration with N intervals
// Returns |a_mean|^2 (scalar objective for optimization)
double wingBeatAccel(const OptimParams& optim, const FlightCondition& flight,
                     const FixedParams& fixed, int N = 40);

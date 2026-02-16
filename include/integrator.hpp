#pragma once

#include "eom.hpp"

// Forward declaration
class Wing;

// Convenience wrapper that allocates scratch internally (not for tight loops)
State stepRK4(double t, double h, const State& y,
              const std::vector<Wing>& wings);

// Version with pre-allocated scratch buffer (use in tight loops to avoid allocation)
State stepRK4(double t, double h, const State& y,
              const std::vector<Wing>& wings,
              std::vector<SingleWingVectors>& scratch);

#pragma once

#include "eom.hpp"

// Forward declaration
class Wing;

// Convenience wrappers that allocate scratch internally (not for tight loops)
State stepEuler(double t, double h, const State& y,
                const std::vector<Wing>& wings);

State stepRK4(double t, double h, const State& y,
              const std::vector<Wing>& wings);

// Versions with pre-allocated scratch buffer (use in tight loops to avoid allocation)
State stepEuler(double t, double h, const State& y,
                const std::vector<Wing>& wings,
                std::vector<SingleWingVectors>& scratch);

State stepRK4(double t, double h, const State& y,
              const std::vector<Wing>& wings,
              std::vector<SingleWingVectors>& scratch);

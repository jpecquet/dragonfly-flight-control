#pragma once

#include "eom.hpp"

// Forward declaration
class Wing;

// Euler method (first-order, for initialization)
State stepEuler(double t, double h, const State& y,
                const std::vector<Wing>& wings);

// Runge-Kutta 4th order method
State stepRK4(double t, double h, const State& y,
              const std::vector<Wing>& wings);

// Versions with pre-allocated scratch buffer (avoids repeated allocation in loops)
State stepEuler(double t, double h, const State& y,
                const std::vector<Wing>& wings,
                std::vector<SingleWingVectors>& scratch);

State stepRK4(double t, double h, const State& y,
              const std::vector<Wing>& wings,
              std::vector<SingleWingVectors>& scratch);

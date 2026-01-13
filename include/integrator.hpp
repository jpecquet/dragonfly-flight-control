#pragma once

#include "eom.hpp"

// Euler method (first-order, for initialization)
State stepEuler(double t, double h, const State& y, const Parameters& params);

// Runge-Kutta 4th order method
State stepRK4(double t, double h, const State& y, const Parameters& params);

#include "controller.hpp"

#include <algorithm>
#include <cmath>

PIDController::PIDController(const PIDGains& gains)
    : gains_(gains), integral_(0.0) {}

double PIDController::compute(double dt, double error, double error_dot) {
    // Proportional term
    double p_term = gains_.Kp * error;

    // Integral term with anti-windup
    integral_ += error * dt;
    // Clamp integral to prevent windup
    integral_ = std::clamp(integral_, -gains_.i_max, gains_.i_max);
    double i_term = gains_.Ki * integral_;

    // Derivative term (using provided error derivative)
    double d_term = gains_.Kd * error_dot;

    return p_term + i_term + d_term;
}

void PIDController::reset() {
    integral_ = 0.0;
}

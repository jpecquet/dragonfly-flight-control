#pragma once

// PID gains structure
struct PIDGains {
    double Kp = 0.0;    // Proportional gain
    double Ki = 0.0;    // Integral gain
    double Kd = 0.0;    // Derivative gain
    double i_max = 1.0; // Anti-windup: max integral magnitude

    PIDGains() = default;
    PIDGains(double kp, double ki, double kd, double imax = 1.0)
        : Kp(kp), Ki(ki), Kd(kd), i_max(imax) {}
};

// PID controller with anti-windup
class PIDController {
public:
    PIDController() = default;
    explicit PIDController(const PIDGains& gains);

    // Compute control output
    // dt: timestep
    // error: current error (setpoint - measurement)
    // error_dot: error derivative (can be computed from velocity error)
    double compute(double dt, double error, double error_dot);

    // Reset integral state
    void reset();

    // Accessors
    const PIDGains& gains() const { return gains_; }
    void setGains(const PIDGains& gains) { gains_ = gains; }
    double integral() const { return integral_; }

private:
    PIDGains gains_;
    double integral_ = 0.0;
};

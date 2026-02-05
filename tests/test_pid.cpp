#include "controller.hpp"

#include <cmath>
#include <iostream>

// Test 1: Zero error should produce zero output
bool testZeroError() {
    std::cout << "Test: Zero error -> zero output\n";

    PIDGains gains(1.0, 0.5, 0.2, 1.0);
    PIDController pid(gains);

    double dt = 0.01;
    double error = 0.0;
    double error_dot = 0.0;

    double output = pid.compute(dt, error, error_dot);

    bool passed = std::abs(output) < 1e-10;
    std::cout << "  Output: " << output << " (expected ~0)\n";
    std::cout << "  " << (passed ? "PASSED" : "FAILED") << "\n\n";
    return passed;
}

// Test 2: Proportional term only
bool testProportional() {
    std::cout << "Test: Proportional term\n";

    PIDGains gains(2.0, 0.0, 0.0, 1.0);  // Kp=2, Ki=0, Kd=0
    PIDController pid(gains);

    double dt = 0.01;
    double error = 5.0;
    double error_dot = 0.0;

    double output = pid.compute(dt, error, error_dot);
    double expected = 2.0 * 5.0;  // Kp * error

    bool passed = std::abs(output - expected) < 1e-10;
    std::cout << "  Output: " << output << " (expected " << expected << ")\n";
    std::cout << "  " << (passed ? "PASSED" : "FAILED") << "\n\n";
    return passed;
}

// Test 3: Derivative term only
bool testDerivative() {
    std::cout << "Test: Derivative term\n";

    PIDGains gains(0.0, 0.0, 3.0, 1.0);  // Kp=0, Ki=0, Kd=3
    PIDController pid(gains);

    double dt = 0.01;
    double error = 0.0;
    double error_dot = 2.0;

    double output = pid.compute(dt, error, error_dot);
    double expected = 3.0 * 2.0;  // Kd * error_dot

    bool passed = std::abs(output - expected) < 1e-10;
    std::cout << "  Output: " << output << " (expected " << expected << ")\n";
    std::cout << "  " << (passed ? "PASSED" : "FAILED") << "\n\n";
    return passed;
}

// Test 4: Integral accumulation
bool testIntegral() {
    std::cout << "Test: Integral accumulation\n";

    PIDGains gains(0.0, 1.0, 0.0, 10.0);  // Ki=1, large i_max
    PIDController pid(gains);

    double dt = 0.1;
    double error = 2.0;
    double error_dot = 0.0;

    // First step: integral = 2.0 * 0.1 = 0.2
    double output1 = pid.compute(dt, error, error_dot);
    double expected1 = 1.0 * 0.2;  // Ki * integral

    // Second step: integral = 0.2 + 2.0 * 0.1 = 0.4
    double output2 = pid.compute(dt, error, error_dot);
    double expected2 = 1.0 * 0.4;

    bool passed = std::abs(output1 - expected1) < 1e-10 &&
                  std::abs(output2 - expected2) < 1e-10;

    std::cout << "  Step 1: " << output1 << " (expected " << expected1 << ")\n";
    std::cout << "  Step 2: " << output2 << " (expected " << expected2 << ")\n";
    std::cout << "  " << (passed ? "PASSED" : "FAILED") << "\n\n";
    return passed;
}

// Test 5: Anti-windup limits
bool testAntiWindup() {
    std::cout << "Test: Anti-windup limits\n";

    PIDGains gains(0.0, 1.0, 0.0, 0.5);  // Ki=1, i_max=0.5
    PIDController pid(gains);

    double dt = 1.0;
    double error = 10.0;  // Large error
    double error_dot = 0.0;

    // After one step, integral would be 10.0 but clamped to 0.5
    double output = pid.compute(dt, error, error_dot);
    double expected = 1.0 * 0.5;  // Ki * clamped_integral

    bool passed = std::abs(output - expected) < 1e-10;
    std::cout << "  Output: " << output << " (expected " << expected << ", clamped)\n";

    // Check that integral is actually clamped
    bool integral_clamped = std::abs(pid.integral() - 0.5) < 1e-10;
    std::cout << "  Integral: " << pid.integral() << " (expected 0.5)\n";

    passed = passed && integral_clamped;
    std::cout << "  " << (passed ? "PASSED" : "FAILED") << "\n\n";
    return passed;
}

// Test 6: Negative anti-windup
bool testAntiWindupNegative() {
    std::cout << "Test: Anti-windup (negative)\n";

    PIDGains gains(0.0, 1.0, 0.0, 0.5);  // Ki=1, i_max=0.5
    PIDController pid(gains);

    double dt = 1.0;
    double error = -10.0;  // Large negative error
    double error_dot = 0.0;

    double output = pid.compute(dt, error, error_dot);
    double expected = 1.0 * (-0.5);  // Ki * clamped_integral

    bool passed = std::abs(output - expected) < 1e-10;
    std::cout << "  Output: " << output << " (expected " << expected << ", clamped)\n";

    bool integral_clamped = std::abs(pid.integral() - (-0.5)) < 1e-10;
    std::cout << "  Integral: " << pid.integral() << " (expected -0.5)\n";

    passed = passed && integral_clamped;
    std::cout << "  " << (passed ? "PASSED" : "FAILED") << "\n\n";
    return passed;
}

// Test 7: Reset clears integral
bool testReset() {
    std::cout << "Test: Reset clears integral\n";

    PIDGains gains(0.0, 1.0, 0.0, 10.0);
    PIDController pid(gains);

    // Accumulate some integral
    pid.compute(1.0, 5.0, 0.0);
    bool has_integral = std::abs(pid.integral()) > 1e-10;

    // Reset
    pid.reset();
    bool cleared = std::abs(pid.integral()) < 1e-10;

    bool passed = has_integral && cleared;
    std::cout << "  Had integral: " << (has_integral ? "yes" : "no") << "\n";
    std::cout << "  After reset: " << pid.integral() << "\n";
    std::cout << "  " << (passed ? "PASSED" : "FAILED") << "\n\n";
    return passed;
}

// Test 8: Combined PID response
bool testCombined() {
    std::cout << "Test: Combined P+I+D response\n";

    PIDGains gains(2.0, 0.5, 1.0, 10.0);  // Kp=2, Ki=0.5, Kd=1
    PIDController pid(gains);

    double dt = 0.1;
    double error = 3.0;
    double error_dot = -1.0;

    double output = pid.compute(dt, error, error_dot);

    // P = 2.0 * 3.0 = 6.0
    // I = 0.5 * (3.0 * 0.1) = 0.15
    // D = 1.0 * (-1.0) = -1.0
    // Total = 6.0 + 0.15 - 1.0 = 5.15
    double expected = 5.15;

    bool passed = std::abs(output - expected) < 1e-10;
    std::cout << "  Output: " << output << " (expected " << expected << ")\n";
    std::cout << "  " << (passed ? "PASSED" : "FAILED") << "\n\n";
    return passed;
}

int main() {
    std::cout << "PID Controller Tests\n";
    std::cout << "====================\n\n";

    int passed = 0;
    int total = 8;

    if (testZeroError()) passed++;
    if (testProportional()) passed++;
    if (testDerivative()) passed++;
    if (testIntegral()) passed++;
    if (testAntiWindup()) passed++;
    if (testAntiWindupNegative()) passed++;
    if (testReset()) passed++;
    if (testCombined()) passed++;

    std::cout << "Summary: " << passed << "/" << total << " tests passed\n";

    return (passed == total) ? 0 : 1;
}

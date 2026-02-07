#include "rotation.hpp"

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

// Check if a matrix is approximately identity
bool isIdentity(const Mat3& M, double tol) {
    Mat3 I = Mat3::Identity();
    return (M - I).norm() < tol;
}

// Check if two vectors are approximately equal
bool vecNear(const Vec3& a, const Vec3& b, double tol) {
    return (a - b).norm() < tol;
}

// Test orthogonality: R^T * R = I
bool testOrthogonality(const std::string& name, Mat3 (*rotFunc)(double),
                       const std::vector<double>& angles, double tol) {
    std::cout << "  " << name << " orthogonality: ";

    double max_error = 0.0;
    for (double angle : angles) {
        Mat3 R = rotFunc(angle);
        Mat3 RtR = R.transpose() * R;
        double error = (RtR - Mat3::Identity()).norm();
        max_error = std::max(max_error, error);
    }

    bool passed = max_error < tol;
    std::cout << (passed ? "PASS" : "FAIL")
              << " (max error: " << max_error << ")\n";
    return passed;
}

// Test determinant = 1
bool testDeterminant(const std::string& name, Mat3 (*rotFunc)(double),
                     const std::vector<double>& angles, double tol) {
    std::cout << "  " << name << " determinant=1: ";

    double max_error = 0.0;
    for (double angle : angles) {
        Mat3 R = rotFunc(angle);
        double error = std::abs(R.determinant() - 1.0);
        max_error = std::max(max_error, error);
    }

    bool passed = max_error < tol;
    std::cout << (passed ? "PASS" : "FAIL")
              << " (max error: " << max_error << ")\n";
    return passed;
}

int main() {
    std::cout << "Rotation Matrix Test: Orthogonality and 90° rotations\n";
    std::cout << "======================================================\n\n";

    double tol = 1e-14;
    bool all_passed = true;

    // Test angles including edge cases
    std::vector<double> angles = {
        0.0,
        M_PI / 6,    // 30°
        M_PI / 4,    // 45°
        M_PI / 3,    // 60°
        M_PI / 2,    // 90°
        M_PI,        // 180°
        3 * M_PI / 2, // 270°
        2 * M_PI,    // 360°
        -M_PI / 4,   // -45°
        0.123456     // arbitrary angle
    };

    // ========== Orthogonality tests ==========
    std::cout << "Orthogonality tests (R^T * R = I):\n";
    all_passed &= testOrthogonality("rotX", rotX, angles, tol);
    all_passed &= testOrthogonality("rotY", rotY, angles, tol);
    all_passed &= testOrthogonality("rotZ", rotZ, angles, tol);

    // ========== Determinant tests ==========
    std::cout << "\nDeterminant tests (det(R) = 1):\n";
    all_passed &= testDeterminant("rotX", rotX, angles, tol);
    all_passed &= testDeterminant("rotY", rotY, angles, tol);
    all_passed &= testDeterminant("rotZ", rotZ, angles, tol);

    // ========== Known-value rotation tests ==========

    Vec3 ex(1, 0, 0);
    Vec3 ey(0, 1, 0);
    Vec3 ez(0, 0, 1);

    struct VecTest {
        const char* label;
        Mat3 (*func)(double);
        double angle;
        Vec3 input, expected;
    };

    VecTest vec_tests[] = {
        // 90° rotations
        {"rotX(90°): X->X ",  rotX, M_PI/2, ex,  ex},
        {"rotX(90°): Y->Z ",  rotX, M_PI/2, ey,  ez},
        {"rotX(90°): Z->-Y",  rotX, M_PI/2, ez, -ey},
        {"rotY(90°): Y->Y ",  rotY, M_PI/2, ey,  ey},
        {"rotY(90°): Z->X ",  rotY, M_PI/2, ez,  ex},
        {"rotY(90°): X->-Z",  rotY, M_PI/2, ex, -ez},
        {"rotZ(90°): Z->Z ",  rotZ, M_PI/2, ez,  ez},
        {"rotZ(90°): X->Y ",  rotZ, M_PI/2, ex,  ey},
        {"rotZ(90°): Y->-X",  rotZ, M_PI/2, ey, -ex},
        // 180° rotations
        {"rotX(180°): X->X ",  rotX, M_PI, ex,  ex},
        {"rotX(180°): Y->-Y",  rotX, M_PI, ey, -ey},
        {"rotX(180°): Z->-Z",  rotX, M_PI, ez, -ez},
        {"rotY(180°): Y->Y ",  rotY, M_PI, ey,  ey},
        {"rotY(180°): X->-X",  rotY, M_PI, ex, -ex},
        {"rotY(180°): Z->-Z",  rotY, M_PI, ez, -ez},
        {"rotZ(180°): Z->Z ",  rotZ, M_PI, ez,  ez},
        {"rotZ(180°): X->-X",  rotZ, M_PI, ex, -ex},
        {"rotZ(180°): Y->-Y",  rotZ, M_PI, ey, -ey},
    };

    std::cout << "\nKnown-value rotation tests (90° and 180°):\n";
    for (const auto& t : vec_tests) {
        Mat3 R = t.func(t.angle);
        bool ok = vecNear(R * t.input, t.expected, tol);
        std::cout << "  " << t.label << ": " << (ok ? "PASS" : "FAIL") << "\n";
        all_passed &= ok;
    }

    // ========== Summary ==========
    std::cout << "\n======================================================\n";
    if (all_passed) {
        std::cout << "PASSED: All rotation matrix tests passed\n";
        return 0;
    } else {
        std::cout << "FAILED: Some tests failed\n";
        return 1;
    }
}

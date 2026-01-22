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
bool vecEqual(const Vec3& a, const Vec3& b, double tol) {
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

    // ========== 90° rotation tests ==========
    std::cout << "\n90° rotation tests (exact known values):\n";

    Vec3 ex(1, 0, 0);
    Vec3 ey(0, 1, 0);
    Vec3 ez(0, 0, 1);
    double pi2 = M_PI / 2;

    // rotX(90°): Y -> Z, Z -> -Y, X unchanged
    std::cout << "  rotX(90°):\n";
    {
        Mat3 Rx = rotX(pi2);

        bool t1 = vecEqual(Rx * ex, ex, tol);
        std::cout << "    X -> X:  " << (t1 ? "PASS" : "FAIL") << "\n";
        all_passed &= t1;

        bool t2 = vecEqual(Rx * ey, ez, tol);
        std::cout << "    Y -> Z:  " << (t2 ? "PASS" : "FAIL") << "\n";
        all_passed &= t2;

        bool t3 = vecEqual(Rx * ez, -ey, tol);
        std::cout << "    Z -> -Y: " << (t3 ? "PASS" : "FAIL") << "\n";
        all_passed &= t3;
    }

    // rotY(90°): Z -> X, X -> -Z, Y unchanged
    std::cout << "  rotY(90°):\n";
    {
        Mat3 Ry = rotY(pi2);

        bool t1 = vecEqual(Ry * ey, ey, tol);
        std::cout << "    Y -> Y:  " << (t1 ? "PASS" : "FAIL") << "\n";
        all_passed &= t1;

        bool t2 = vecEqual(Ry * ez, ex, tol);
        std::cout << "    Z -> X:  " << (t2 ? "PASS" : "FAIL") << "\n";
        all_passed &= t2;

        bool t3 = vecEqual(Ry * ex, -ez, tol);
        std::cout << "    X -> -Z: " << (t3 ? "PASS" : "FAIL") << "\n";
        all_passed &= t3;
    }

    // rotZ(90°): X -> Y, Y -> -X, Z unchanged
    std::cout << "  rotZ(90°):\n";
    {
        Mat3 Rz = rotZ(pi2);

        bool t1 = vecEqual(Rz * ez, ez, tol);
        std::cout << "    Z -> Z:  " << (t1 ? "PASS" : "FAIL") << "\n";
        all_passed &= t1;

        bool t2 = vecEqual(Rz * ex, ey, tol);
        std::cout << "    X -> Y:  " << (t2 ? "PASS" : "FAIL") << "\n";
        all_passed &= t2;

        bool t3 = vecEqual(Rz * ey, -ex, tol);
        std::cout << "    Y -> -X: " << (t3 ? "PASS" : "FAIL") << "\n";
        all_passed &= t3;
    }

    // ========== 180° rotation tests ==========
    std::cout << "\n180° rotation tests:\n";
    double pi = M_PI;

    {
        Mat3 Rx = rotX(pi);
        bool t1 = vecEqual(Rx * ex, ex, tol);
        bool t2 = vecEqual(Rx * ey, -ey, tol);
        bool t3 = vecEqual(Rx * ez, -ez, tol);
        std::cout << "  rotX(180°): X->X, Y->-Y, Z->-Z: "
                  << ((t1 && t2 && t3) ? "PASS" : "FAIL") << "\n";
        all_passed &= t1 && t2 && t3;
    }
    {
        Mat3 Ry = rotY(pi);
        bool t1 = vecEqual(Ry * ey, ey, tol);
        bool t2 = vecEqual(Ry * ex, -ex, tol);
        bool t3 = vecEqual(Ry * ez, -ez, tol);
        std::cout << "  rotY(180°): Y->Y, X->-X, Z->-Z: "
                  << ((t1 && t2 && t3) ? "PASS" : "FAIL") << "\n";
        all_passed &= t1 && t2 && t3;
    }
    {
        Mat3 Rz = rotZ(pi);
        bool t1 = vecEqual(Rz * ez, ez, tol);
        bool t2 = vecEqual(Rz * ex, -ex, tol);
        bool t3 = vecEqual(Rz * ey, -ey, tol);
        std::cout << "  rotZ(180°): Z->Z, X->-X, Y->-Y: "
                  << ((t1 && t2 && t3) ? "PASS" : "FAIL") << "\n";
        all_passed &= t1 && t2 && t3;
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

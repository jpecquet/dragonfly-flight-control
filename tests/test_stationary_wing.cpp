#include "wing.hpp"
#include "eom.hpp"

#include <cmath>
#include <iostream>

// Test a stationary wing (no flapping) in uniform flow.
// This verifies the full wing pipeline: angles → rotation → blade element → force

bool vecNear(const Vec3& a, const Vec3& b, double tol) {
    return (a - b).norm() < tol;
}

int main() {
    std::cout << "Stationary Wing Test: Wing in uniform flow\n";
    std::cout << "==========================================\n\n";

    // Wing parameters
    double mu0 = 0.1;
    double lb0 = 1.0;
    double Cd0 = 0.4;
    double Cl0 = 1.2;

    // Stationary wing: all angles constant, no angular velocities
    auto stationaryAngles = [](double t) -> WingAngles {
        return {
            0.0,  // gam: stroke plane angle
            0.0,  // gam_dot: stroke plane angular velocity
            0.0,  // phi: stroke angle
            0.0,  // phi_dot: stroke angular velocity (key: no flapping!)
            0.0   // psi: pitch angle
        };
    };

    Wing wing("test", mu0, lb0, WingSide::Left, Cd0, Cl0, stationaryAngles);

    double tol = 1e-10;
    bool all_passed = true;

    // With all angles = 0 for a left wing:
    //   Rs = rotY(0) * rotZ(0) = I
    //   e_s = (1, 0, 0)  - stroke direction
    //   e_r = (0, 1, 0)  - radial/span direction
    //   Rp = I * rotY(0) = I
    //   e_c = -ez = (0, 0, -1)  - chord direction (negated for left wing)

    Vec3 expected_e_s(1, 0, 0);
    Vec3 expected_e_r(0, 1, 0);
    Vec3 expected_e_c(0, 0, -1);

    // ========== Test 1: Orientation vectors ==========
    std::cout << "Test 1: Orientation vectors with zero angles\n";
    {
        SingleWingVectors vecs;
        Vec3 ub(0, 0, 0);
        wing.computeForce(0.0, ub, vecs);

        bool t1 = vecNear(vecs.e_s, expected_e_s, tol);
        bool t2 = vecNear(vecs.e_r, expected_e_r, tol);
        bool t3 = vecNear(vecs.e_c, expected_e_c, tol);

        std::cout << "  e_s = (" << vecs.e_s.x() << ", " << vecs.e_s.y() << ", " << vecs.e_s.z() << ")";
        std::cout << " expected (1, 0, 0): " << (t1 ? "PASS" : "FAIL") << "\n";
        std::cout << "  e_r = (" << vecs.e_r.x() << ", " << vecs.e_r.y() << ", " << vecs.e_r.z() << ")";
        std::cout << " expected (0, 1, 0): " << (t2 ? "PASS" : "FAIL") << "\n";
        std::cout << "  e_c = (" << vecs.e_c.x() << ", " << vecs.e_c.y() << ", " << vecs.e_c.z() << ")";
        std::cout << " expected (0, 0, -1): " << (t3 ? "PASS" : "FAIL") << "\n";

        all_passed &= t1 && t2 && t3;
    }

    // ========== Test 2: Flow along X (perpendicular to chord) ==========
    // Flow along X-axis: u = (U, 0, 0)
    // Chord is e_c = (0, 0, -1)
    // Angle of attack α = 90° (flow perpendicular to chord)
    //
    // At α = 90°:
    //   Cd = Cd0 + 2*sin²(90°) = Cd0 + 2 = 2.4
    //   Cl = Cl0*sin(180°) = 0
    //
    // Drag direction: e_d = -u/|u| = (-1, 0, 0)
    // force_coefficient = 0.5 * mu0 / lb0 = 0.05
    // |drag| = 0.05 * 2.4 * U²
    std::cout << "\nTest 2: Flow along X-axis (α = 90°, pure drag)\n";
    {
        double U = 2.0;
        Vec3 ub(U, 0, 0);
        SingleWingVectors vecs;
        Vec3 force = wing.computeForce(0.0, ub, vecs);

        double force_coef = 0.5 * mu0 / lb0;  // 0.05
        double Cd = Cd0 + 2.0;                 // 2.4
        double expected_drag_mag = force_coef * Cd * U * U;  // 0.48
        Vec3 expected_drag = expected_drag_mag * Vec3(-1, 0, 0);
        Vec3 expected_lift = Vec3::Zero();
        Vec3 expected_force = expected_drag;

        bool t1 = vecNear(vecs.drag, expected_drag, tol);
        bool t2 = vecNear(vecs.lift, expected_lift, tol);
        bool t3 = vecNear(force, expected_force, tol);

        std::cout << "  Drag = (" << vecs.drag.x() << ", " << vecs.drag.y() << ", " << vecs.drag.z() << ")";
        std::cout << " expected (" << expected_drag.x() << ", 0, 0): " << (t1 ? "PASS" : "FAIL") << "\n";
        std::cout << "  Lift = (" << vecs.lift.x() << ", " << vecs.lift.y() << ", " << vecs.lift.z() << ")";
        std::cout << " expected (0, 0, 0): " << (t2 ? "PASS" : "FAIL") << "\n";
        std::cout << "  Total force: " << (t3 ? "PASS" : "FAIL") << "\n";

        all_passed &= t1 && t2 && t3;
    }

    // ========== Test 3: Flow along -Z (aligned with chord) ==========
    // Flow along -Z: u = (0, 0, -U)
    // Chord is e_c = (0, 0, -1), so flow is aligned with chord
    // Angle of attack α = 0°
    //
    // At α = 0°:
    //   c_alpha = cos(0) = 1
    //   s_alpha = U*sin(0) = 0
    //   Cd = Cd0 + 2*sin²(0) = Cd0 = 0.4
    //   Cl = 0
    //
    // Drag direction: e_d = -u/|u| = (0, 0, 1)
    std::cout << "\nTest 3: Flow along -Z (α = 0°, minimum drag)\n";
    {
        double U = 2.0;
        Vec3 ub(0, 0, -U);
        SingleWingVectors vecs;
        Vec3 force = wing.computeForce(0.0, ub, vecs);

        double force_coef = 0.5 * mu0 / lb0;
        double Cd = Cd0;  // 0.4
        double expected_drag_mag = force_coef * Cd * U * U;  // 0.08
        Vec3 expected_drag = expected_drag_mag * Vec3(0, 0, 1);
        Vec3 expected_lift = Vec3::Zero();
        Vec3 expected_force = expected_drag;

        bool t1 = vecNear(vecs.drag, expected_drag, tol);
        bool t2 = vecNear(vecs.lift, expected_lift, tol);
        bool t3 = vecNear(force, expected_force, tol);

        std::cout << "  Drag = (" << vecs.drag.x() << ", " << vecs.drag.y() << ", " << vecs.drag.z() << ")";
        std::cout << " expected (0, 0, " << expected_drag.z() << "): " << (t1 ? "PASS" : "FAIL") << "\n";
        std::cout << "  Lift = (" << vecs.lift.x() << ", " << vecs.lift.y() << ", " << vecs.lift.z() << ")";
        std::cout << " expected (0, 0, 0): " << (t2 ? "PASS" : "FAIL") << "\n";
        std::cout << "  Total force: " << (t3 ? "PASS" : "FAIL") << "\n";

        all_passed &= t1 && t2 && t3;
    }

    // ========== Test 4: Flow at 45° to chord (maximum lift) ==========
    // Flow direction: u = U/√2 * (e_c + e_perp) where e_perp ⊥ e_c and e_r
    // With e_c = (0,0,-1) and e_r = (0,1,0), e_perp = (1,0,0)
    // So u = U/√2 * ((0,0,-1) + (1,0,0)) = U/√2 * (1, 0, -1)
    //
    // At α = 45°:
    //   c_alpha = cos(45°) = 1/√2
    //   Cd = Cd0 + 2*sin²(45°) = Cd0 + 1 = 1.4
    //   Cl = Cl0*sin(90°) = Cl0 = 1.2  (but scaled by U in the code)
    std::cout << "\nTest 4: Flow at 45° to chord (maximum lift)\n";
    {
        double U = 2.0;
        double s2 = 1.0 / std::sqrt(2.0);
        Vec3 ub = U * Vec3(s2, 0, -s2);  // 45° between X and -Z
        SingleWingVectors vecs;
        Vec3 force = wing.computeForce(0.0, ub, vecs);

        double force_coef = 0.5 * mu0 / lb0;

        // Analytical values
        double c_alpha = s2;  // cos(45°)
        double s_alpha = U * s2;  // U * sin(45°) - note: code doesn't normalize this
        double Cd = Cd0 + 2.0 * (1.0 - c_alpha * c_alpha);  // Cd0 + 2*sin²(45°) = 1.4
        double Cl = 2.0 * Cl0 * s_alpha * c_alpha;  // 2 * 1.2 * U/√2 * 1/√2 = 1.2*U

        // Drag direction: opposite to velocity
        Vec3 e_d = -ub.normalized();
        // Lift direction: e_d × e_r
        Vec3 e_l = e_d.cross(expected_e_r);

        Vec3 expected_drag = force_coef * Cd * U * U * e_d;
        Vec3 expected_lift = force_coef * Cl * U * U * e_l;
        Vec3 expected_force = expected_drag + expected_lift;

        bool t1 = vecNear(vecs.drag, expected_drag, tol);
        bool t2 = vecNear(vecs.lift, expected_lift, tol);
        bool t3 = vecNear(force, expected_force, tol);

        std::cout << "  Cd = " << Cd << ", Cl (with U factor) = " << Cl << "\n";
        std::cout << "  |Drag| = " << vecs.drag.norm() << " expected " << expected_drag.norm();
        std::cout << ": " << (t1 ? "PASS" : "FAIL") << "\n";
        std::cout << "  |Lift| = " << vecs.lift.norm() << " expected " << expected_lift.norm();
        std::cout << ": " << (t2 ? "PASS" : "FAIL") << "\n";
        std::cout << "  Total force: " << (t3 ? "PASS" : "FAIL") << "\n";

        all_passed &= t1 && t2 && t3;
    }

    // ========== Test 5: Flow with spanwise component (should be projected out) ==========
    // Add a Y-component to velocity - should be removed by projection onto plane ⊥ to e_r
    std::cout << "\nTest 5: Flow with spanwise component (projection test)\n";
    {
        double U = 2.0;
        Vec3 ub(U, 3.0, 0);  // X + Y components; Y should be projected out
        SingleWingVectors vecs;
        Vec3 force = wing.computeForce(0.0, ub, vecs);

        // After projection: u = (U, 0, 0) (Y removed)
        // Same as Test 2
        double force_coef = 0.5 * mu0 / lb0;
        double Cd = Cd0 + 2.0;
        double expected_drag_mag = force_coef * Cd * U * U;
        Vec3 expected_drag = expected_drag_mag * Vec3(-1, 0, 0);
        Vec3 expected_lift = Vec3::Zero();

        bool t1 = vecNear(vecs.drag, expected_drag, tol);
        bool t2 = vecNear(vecs.lift, expected_lift, tol);

        std::cout << "  Spanwise component (Y=" << ub.y() << ") projected out\n";
        std::cout << "  Drag matches Test 2: " << (t1 ? "PASS" : "FAIL") << "\n";
        std::cout << "  Lift = 0: " << (t2 ? "PASS" : "FAIL") << "\n";

        all_passed &= t1 && t2;
    }

    // ========== Summary ==========
    std::cout << "\n==========================================\n";
    if (all_passed) {
        std::cout << "PASSED: All stationary wing tests passed\n";
        return 0;
    } else {
        std::cout << "FAILED: Some tests failed\n";
        return 1;
    }
}

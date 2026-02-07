#pragma once

#include <cmath>

// Analytical solution for terminal velocity of a falling wing with fixed pitch angle.
//
// At steady state, the wing glides at angle θ from vertical with speed U.
// Force balance gives:
//   L/D = tan(θ)
//   D = cos(θ)
//   U = sqrt(2·lb0·cos(θ) / (μ0·Cd))
//
// Geometry (with γ = 90°, stroke plane vertical):
//   - Chord e_c points at angle ψ above horizontal (ψ=0 → horizontal, ψ=π/2 → vertical up)
//   - Flow u points at angle θ from vertical downward
//   - Aerodynamic angle of attack α = π/2 - ψ - θ
//
// Note: The geometric angle between flow and chord vectors is (π - α), but the
// aerodynamic α is defined to match the blade element sign convention where
// Cl = Cl0·sin(2α) gives positive lift for the expected drift direction.
//
// We solve: tan(θ) = Cl(α) / Cd(α) for θ.

struct TerminalVelocitySolution {
    double theta = 0.0;      // Glide angle from vertical (radians)
    double alpha = 0.0;      // Angle of attack (radians)
    double speed = 0.0;      // Total speed U
    double ux = 0.0;         // Horizontal velocity component
    double uz = 0.0;         // Vertical velocity component (negative = falling)
    double Cd = 0.0;         // Drag coefficient at equilibrium
    double Cl = 0.0;         // Lift coefficient at equilibrium
    bool converged = false;  // Whether the solver converged
};

// Compute Cd and Cl for given angle of attack
// Cd(α) = Cd0 + 2·sin²(α)
// Cl(α) = Cl0·sin(2α)
inline void aeroCoeffs(double alpha, double Cd0, double Cl0, double& Cd, double& Cl) {
    double sin_alpha = std::sin(alpha);
    Cd = Cd0 + 2.0 * sin_alpha * sin_alpha;
    Cl = Cl0 * std::sin(2.0 * alpha);
}

// Solve for terminal velocity given pitch angle ψ using Newton-Raphson
TerminalVelocitySolution solveTerminalVelocity(double psi, double mu0, double lb0,
                                                double Cd0, double Cl0);

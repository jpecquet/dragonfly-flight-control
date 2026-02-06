#include "terminal_velocity.hpp"

TerminalVelocitySolution solveTerminalVelocity(double psi, double mu0, double lb0,
                                                double Cd0, double Cl0) {
    TerminalVelocitySolution sol;
    sol.converged = false;

    // Normalize psi to [0, pi/2]
    // With gam = 90°, psi = 0 means horizontal chord, psi = π/2 means vertical chord
    double psi_norm = std::fmod(std::abs(psi), M_PI);
    if (psi_norm > M_PI / 2.0) {
        psi_norm = M_PI - psi_norm;
    }

    // Special case: psi ≈ 0 (horizontal chord)
    // At α = π/2, Cl = Cl0·sin(π) = 0, so no drift, θ = 0
    if (psi_norm < 1e-6) {
        sol.converged = true;
        sol.theta = 0.0;
        sol.alpha = M_PI / 2.0;
        sol.Cd = Cd0 + 2.0;  // sin²(π/2) = 1
        sol.Cl = 0.0;
        sol.speed = std::sqrt(2.0 * lb0 / (mu0 * sol.Cd));
        sol.ux = 0.0;
        sol.uz = -sol.speed;
        return sol;
    }

    // Special case: psi ≈ π/2 (vertical chord)
    // At α = 0, Cl = Cl0·sin(0) = 0, so no drift, θ = 0
    if (std::abs(psi_norm - M_PI / 2.0) < 1e-6) {
        sol.converged = true;
        sol.theta = 0.0;
        sol.alpha = 0.0;
        sol.Cd = Cd0;  // sin²(0) = 0
        sol.Cl = 0.0;
        sol.speed = std::sqrt(2.0 * lb0 / (mu0 * sol.Cd));
        sol.ux = 0.0;
        sol.uz = -sol.speed;
        return sol;
    }

    // Initial guess: θ ≈ (π/2 - ψ)/2
    double theta = (M_PI / 2.0 - psi_norm) / 2.0;
    if (theta < 0) theta = 0.1;

    // Newton-Raphson iteration
    const int max_iter = 50;
    const double tol = 1e-12;

    for (int iter = 0; iter < max_iter; ++iter) {
        // With gam = 90°: α = π/2 - ψ - θ
        double alpha = M_PI / 2.0 - psi_norm - theta;

        // Compute Cd, Cl at current alpha
        double Cd, Cl;
        aeroCoeffs(alpha, Cd0, Cl0, Cd, Cl);

        // f(θ) = tan(θ) - Cl/Cd
        double tan_theta = std::tan(theta);
        double f = tan_theta - Cl / Cd;

        // Check convergence
        if (std::abs(f) < tol) {
            sol.converged = true;
            sol.theta = theta;
            sol.alpha = alpha;
            sol.Cd = Cd;
            sol.Cl = Cl;

            // Compute speed from D = cos(θ)
            // D = 0.5 * (mu0/lb0) * Cd * U² = cos(θ)
            // U² = 2 * lb0 * cos(θ) / (mu0 * Cd)
            double cos_theta = std::cos(theta);
            sol.speed = std::sqrt(2.0 * lb0 * cos_theta / (mu0 * Cd));
            sol.ux = -sol.speed * std::sin(theta);
            sol.uz = -sol.speed * cos_theta;

            return sol;
        }

        // Compute derivative f'(θ) for Newton-Raphson
        // f'(θ) = sec²(θ) - d/dθ[Cl/Cd]
        // where d/dθ[Cl/Cd] = d/dα[Cl/Cd] * dα/dθ = -d/dα[Cl/Cd]
        //
        // d/dα[Cl] = Cl0 * 2 * cos(2α)
        // d/dα[Cd] = 2 * 2 * sin(α) * cos(α) = 2 * sin(2α)
        // d/dα[Cl/Cd] = (Cd * dCl/dα - Cl * dCd/dα) / Cd²

        double dCl_dalpha = Cl0 * 2.0 * std::cos(2.0 * alpha);
        double dCd_dalpha = 2.0 * std::sin(2.0 * alpha);
        double d_ratio_dalpha = (Cd * dCl_dalpha - Cl * dCd_dalpha) / (Cd * Cd);

        double sec_theta = 1.0 / std::cos(theta);
        double df = sec_theta * sec_theta + d_ratio_dalpha;  // Note: +, because dα/dθ = -1

        // Newton step
        double dtheta = -f / df;

        // Damping for stability
        if (std::abs(dtheta) > 0.1) {
            dtheta = 0.1 * (dtheta > 0 ? 1.0 : -1.0);
        }

        theta += dtheta;

        // Keep θ in valid range [0, π/2 - ψ] (α must stay positive)
        double max_theta = M_PI / 2.0 - psi_norm;
        if (theta < 0) theta = 0;
        if (theta > max_theta) theta = max_theta * 0.99;
    }

    // If we didn't converge, return best estimate
    sol.theta = theta;
    sol.alpha = M_PI / 2.0 - psi_norm - theta;
    aeroCoeffs(sol.alpha, Cd0, Cl0, sol.Cd, sol.Cl);
    double cos_theta = std::cos(theta);
    sol.speed = std::sqrt(2.0 * lb0 * cos_theta / (mu0 * sol.Cd));
    sol.ux = -sol.speed * std::sin(theta);
    sol.uz = -sol.speed * cos_theta;

    return sol;
}

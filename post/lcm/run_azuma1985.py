"""Run the LCM for the Azuma 1985 case and compare AoA results.

Usage:
    python -m post.lcm.run_azuma1985
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

from .kinematics import fourier_series


# --- Azuma 1985 parameters (paper convention) ---

FREQUENCY_HZ = 41.5
OMEGA = 2.0 * np.pi * FREQUENCY_HZ  # rad/s
T_WINGBEAT = 1.0 / FREQUENCY_HZ     # seconds

V_FLIGHT = 0.54  # m/s, nearly normal to stroke plane

# Morphology
R_FORE = 0.0335   # half-span (m)
R_HIND = 0.0325
S_FORE = 221e-6   # wing area (m^2)
S_HIND = 272e-6
BODY_MASS = 0.260e-3  # kg

RHO = 1.225  # kg/m^3

# Stroke plane angles (degrees)
GAMMA_FORE = 37.0
GAMMA_HIND = 40.0

# Span station for AoA evaluation
ETA = 0.75

# Wing kinematics in PROJECT convention (from case.yaml)
# phi: flapping angle, psi: pitch angle
WINGS = {
    "fore": {
        "span": R_FORE,
        "area": S_FORE,
        "gamma_deg": GAMMA_FORE,
        "phi_mean": 3.0,
        "phi_harmonics": [(43.0, 0.0)],
        # Paper convention: theta_paper = psi_project + 90
        # theta_f = 98 - 77cos(wt - 49) - 3cos(2wt + 67) - 8cos(3wt + 29)
        # So psi_project = theta_paper - 90:
        # psi_f = 8 - 77cos(wt - 49) - 3cos(2wt + 67) - 8cos(3wt + 29)
        "psi_mean": 8.0,
        "psi_harmonics": [(-77.0, -49.0), (-3.0, 67.0), (-8.0, 29.0)],
    },
    "hind": {
        "span": R_HIND,
        "area": S_HIND,
        "gamma_deg": GAMMA_HIND,
        "phi_mean": -2.0,
        "phi_harmonics": [(47.0, 77.0)],
        # theta_h = 93 - 65cos(wt + 18) + 8cos(2wt + 74) + 8cos(3wt + 28)
        # psi_h = 3 - 65cos(wt + 18) + 8cos(2wt + 74) + 8cos(3wt + 28)
        "psi_mean": 3.0,
        "psi_harmonics": [(-65.0, 18.0), (8.0, 74.0), (8.0, 28.0)],
    },
}


def simplified_aoa(t, wing, v_induced=0.0):
    """Compute AoA using the simplified paper formula.

    alpha = psi - pi/2 - atan2(V + v_induced, -0.75*R*phi_dot)

    Note: the paper convention for pitch is theta = psi + 90, so
    alpha = theta - pi/2 - atan(...) = psi + pi/2 - pi/2 - atan(...)
          = psi - atan(...)

    Wait -- let's be careful. The simplified formula from the docs is:
        alpha ~= psi - pi/2 - atan(U / (-0.75*R*phi_dot))

    where psi is the PROJECT convention pitch angle. Let me verify against
    the existing code (docs_artifacts.py line 300):
        alpha_simplified = psi_rad + 0.5*pi - arctan2(speed, denom)
    where denom = 0.75 * span * (-phi_dot_rad_s)

    So:  alpha = psi + pi/2 - atan2(V, -0.75*R*phi_dot)

    With induced velocity added to the perpendicular component:
        alpha = psi + pi/2 - atan2(V + v_induced, -0.75*R*phi_dot)

    Parameters
    ----------
    t : ndarray
        Time array in wingbeat fraction [0, 1].
    wing : dict
        Wing parameters.
    v_induced : float or ndarray
        Induced velocity normal to stroke plane (m/s). Positive = downwash.

    Returns
    -------
    alpha_deg : ndarray
        Angle of attack in degrees.
    """
    t_sec = t * T_WINGBEAT
    R = wing["span"]

    psi, _ = fourier_series(t_sec, OMEGA, wing["psi_mean"], wing["psi_harmonics"])
    _, phi_dot = fourier_series(t_sec, OMEGA, wing["phi_mean"], wing["phi_harmonics"])

    denom = ETA * R * (-phi_dot)  # tangential velocity at 0.75R
    numer = V_FLIGHT + v_induced  # perpendicular velocity (with induced)

    alpha = psi + 0.5 * np.pi - np.arctan2(numer, denom)
    return np.degrees(alpha)


GRAVITY = 9.81


def momentum_theory_induced_velocity():
    """Compute constant induced velocity from simple momentum theory (paper Eq. 5).

    The swept area S_e for a pair of wings with half-span R and flapping
    amplitude Phi_1 (half peak-to-peak) is:
        S_e = 2 * (1/2) * R^2 * (2*Phi_1) = 2 * R^2 * Phi_1

    Thrust per wing pair: T = W / (2 * cos(gamma))

    Induced velocity (momentum theory, axial flight through a disc):
        v_f = -V/2 + sqrt(T_f / (2*rho*S_e_f) + (V/2)^2)
        v_h = -(V + C_hf*v_f)/2 + sqrt(T_h / (2*rho*S_e_h) + ((V + C_hf*v_f)/2)^2)

    Returns (v_f, v_h) in m/s.
    """
    W = BODY_MASS * GRAVITY

    # Flapping amplitudes (first harmonic, in radians)
    phi1_fore = np.radians(43.0)
    phi1_hind = np.radians(47.0)

    # Swept areas
    S_e_fore = 2.0 * R_FORE**2 * phi1_fore
    S_e_hind = 2.0 * R_HIND**2 * phi1_hind

    # Thrust per wing pair
    T_fore = W / (2.0 * np.cos(np.radians(GAMMA_FORE)))
    T_hind = W / (2.0 * np.cos(np.radians(GAMMA_HIND)))

    # Forewing induced velocity
    V = V_FLIGHT
    v_f = -V / 2.0 + np.sqrt(T_fore / (2.0 * RHO * S_e_fore) + (V / 2.0) ** 2)

    # Hindwing induced velocity (with fore-hind interference)
    C_hf = 0.3  # interference coefficient from paper's simple analysis
    V_eff = V + C_hf * v_f
    v_h = -V_eff / 2.0 + np.sqrt(T_hind / (2.0 * RHO * S_e_hind) + (V_eff / 2.0) ** 2)

    return v_f, v_h


def load_paper_aoa():
    """Load the paper's AoA data from CSV."""
    csv_path = Path(__file__).resolve().parents[2] / "cases" / "azuma1985" / "aoa.csv"
    df = pd.read_csv(csv_path, header=[0, 1])
    arr = df.to_numpy(dtype=float)
    return {
        "fore_t": np.mod(arr[:, 0], 1.0),
        "fore_aoa": arr[:, 1],
        "hind_t": np.mod(arr[:, 2], 1.0),
        "hind_aoa": arr[:, 3],
    }


def main():
    from .solver import LCMSolver

    N_LCM = 200  # time steps per wingbeat for LCM
    N_PLOT = 500  # points for smooth plotting curves

    t_wb_plot = np.linspace(0, 1, N_PLOT, endpoint=False)
    t_wb_lcm = np.linspace(0, 1, N_LCM, endpoint=False)

    # Phase 1: simplified AoA (no induced velocity)
    aoa_fore_simplified = simplified_aoa(t_wb_plot, WINGS["fore"])
    aoa_hind_simplified = simplified_aoa(t_wb_plot, WINGS["hind"])

    # Phase 2: momentum theory (constant induced velocity)
    v_f, v_h = momentum_theory_induced_velocity()
    print(f"Momentum theory induced velocity: v_f = {v_f:.4f} m/s, v_h = {v_h:.4f} m/s")
    aoa_fore_momentum = simplified_aoa(t_wb_plot, WINGS["fore"], v_induced=v_f)
    aoa_hind_momentum = simplified_aoa(t_wb_plot, WINGS["hind"], v_induced=v_h)

    # Phase 3: Full LCM
    solver = LCMSolver(
        WINGS["fore"], WINGS["hind"],
        frequency_hz=FREQUENCY_HZ,
        V_flight=V_FLIGHT,
        rho=RHO,
        body_mass=BODY_MASS,
        n_span=10,
    )

    print("\nRunning LCM (direct Biot-Savart)...")
    v_lcm_fore, v_lcm_hind, _ = solver.run(
        n_steps=N_LCM, n_wingbeats=10,
        n_trail_steps=150, n_past_wingbeats=2)

    print(f"\nLCM induced velocity (last wingbeat):")
    print(f"  fore: mean={np.mean(v_lcm_fore):.4f}, "
          f"range=[{v_lcm_fore.min():.4f}, {v_lcm_fore.max():.4f}]")
    print(f"  hind: mean={np.mean(v_lcm_hind):.4f}, "
          f"range=[{v_lcm_hind.min():.4f}, {v_lcm_hind.max():.4f}]")

    aoa_fore_lcm = simplified_aoa(t_wb_lcm, WINGS["fore"], v_induced=v_lcm_fore)
    aoa_hind_lcm = simplified_aoa(t_wb_lcm, WINGS["hind"], v_induced=v_lcm_hind)

    # Load paper data
    paper = load_paper_aoa()

    # Plot
    fig, ax = plt.subplots(figsize=(10, 4))

    # Paper data (scatter)
    ax.scatter(paper["fore_t"], paper["fore_aoa"], s=12, alpha=0.5,
               color="C0", edgecolors="none", label="Forewing (paper LCM)")
    ax.scatter(paper["hind_t"], paper["hind_aoa"], s=12, alpha=0.5,
               color="C1", edgecolors="none", label="Hindwing (paper LCM)")

    # Simplified (no induced velocity) — faded
    ax.plot(t_wb_plot, aoa_fore_simplified, "--", color="C0", linewidth=0.8,
            alpha=0.3, label="Forewing (v=0)")
    ax.plot(t_wb_plot, aoa_hind_simplified, "--", color="C1", linewidth=0.8,
            alpha=0.3, label="Hindwing (v=0)")

    # Momentum theory — dashed
    ax.plot(t_wb_plot, aoa_fore_momentum, "--", color="C0", linewidth=1.2,
            alpha=0.6, label="Forewing (momentum)")
    ax.plot(t_wb_plot, aoa_hind_momentum, "--", color="C1", linewidth=1.2,
            alpha=0.6, label="Hindwing (momentum)")

    # Full LCM — solid
    ax.plot(t_wb_lcm, aoa_fore_lcm, "-", color="C0", linewidth=1.8,
            label="Forewing (LCM)")
    ax.plot(t_wb_lcm, aoa_hind_lcm, "-", color="C1", linewidth=1.8,
            label="Hindwing (LCM)")

    ax.set_xlabel(r"$t / T_{wb}$")
    ax.set_ylabel("AoA (deg)")
    ax.set_xlim(0, 1)
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best", fontsize=7, ncol=2)
    ax.set_title("Azuma 1985 — Angle of Attack at r = 0.75R")
    fig.tight_layout()
    out = Path(__file__).resolve().parents[2] / "lcm_aoa_comparison.png"
    fig.savefig(str(out), dpi=150, bbox_inches="tight")
    print(f"\nSaved to {out}")
    plt.close(fig)


if __name__ == "__main__":
    main()

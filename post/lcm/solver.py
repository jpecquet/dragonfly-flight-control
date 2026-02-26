"""Full Local Circulation Method (LCM) solver for beating wings.

Implements the LCM from Azuma et al. (1985), adapted from rotary wing analysis.
The method computes time-varying induced velocity by decomposing the wing into
elliptical sub-wings, solving for circulation iteratively, and tracking wake
effects through attenuation and interference coefficients.
"""

import numpy as np
from .kinematics import fourier_series
from .aerodynamics import cl_azuma1985


def elliptical_chord(eta, R, S):
    """Elliptical chord distribution: c(r) = c_max * sqrt(1 - (r/R)^2).

    c_max = 4*S / (pi*R) so that integral_0^R c(r) dr = S.
    """
    c_max = 4.0 * S / (np.pi * R)
    return c_max * np.sqrt(np.maximum(1.0 - eta**2, 0.0))


def build_decomposition_matrix(eta_ctrl, eta_wings):
    """Build the matrix A such that Gamma[j] = sum_i A[j,i] * dGamma[i].

    A[j,i] = sqrt(1 - (eta_ctrl[j] / eta_wings[i])^2) if eta_ctrl[j] <= eta_wings[i]
             0 otherwise

    Parameters
    ----------
    eta_ctrl : (n,) array
        Normalized spanwise positions of control points.
    eta_wings : (n,) array
        Normalized half-spans of the elliptical sub-wings.

    Returns
    -------
    A : (n, n) array
    """
    n = len(eta_ctrl)
    A = np.zeros((n, n))
    for j in range(n):
        for i in range(n):
            if eta_ctrl[j] <= eta_wings[i] + 1e-12:
                ratio = eta_ctrl[j] / eta_wings[i]
                A[j, i] = np.sqrt(max(1.0 - ratio**2, 0.0))
    return A


def biot_savart_segment(P, A, B, Gamma, r_core=1e-4):
    """Induced velocity at P from a vortex segment A->B with circulation Gamma.

    Uses the finite-segment Biot-Savart formula with a Lamb-Oseen core
    regularization to avoid singularities.

    Parameters
    ----------
    P : (3,) point where velocity is evaluated
    A, B : (3,) endpoints of the vortex segment
    Gamma : float, circulation strength
    r_core : float, vortex core radius for regularization

    Returns
    -------
    v : (3,) induced velocity
    """
    r1 = P - A
    r2 = P - B
    dl = B - A
    cross = np.cross(r1, r2)
    cross_mag_sq = np.dot(cross, cross)

    r1_mag = np.linalg.norm(r1)
    r2_mag = np.linalg.norm(r2)

    if r1_mag < 1e-12 or r2_mag < 1e-12:
        return np.zeros(3)

    # Regularized denominator (Lamb-Oseen core)
    denom = cross_mag_sq + r_core**2 * np.dot(dl, dl)
    if denom < 1e-30:
        return np.zeros(3)

    # Finite segment Biot-Savart
    factor = (Gamma / (4.0 * np.pi)) * (
        np.dot(dl, r1 / r1_mag - r2 / r2_mag) / denom
    )
    return factor * cross


def trailing_vortex_filament(wing_params, omega, t_current, t_start, n_segments,
                             U_P_mean, root_pos=np.zeros(3), stroke_plane_normal=None,
                             stroke_plane_tangent=None):
    """Generate 3D positions of trailing tip vortex filament.

    The tip vortex traces the path of the wing tip in the stroke plane,
    advected downstream (normal to stroke plane) at speed U_P_mean.

    Parameters
    ----------
    wing_params : dict with phi_mean, phi_harmonics, span
    omega : float, angular frequency
    t_current : float, current time (seconds)
    t_start : float, earliest time for the vortex trail
    n_segments : int, number of segments to discretize the filament
    U_P_mean : float, mean perpendicular flow speed for wake advection
    root_pos : (3,) wing root position in lab frame
    stroke_plane_normal : (3,) unit normal to stroke plane (advection direction)
    stroke_plane_tangent : (3,) unit tangent in stroke plane (φ=0 direction)

    Returns
    -------
    points : (n_segments+1, 3) array of filament positions
    """
    gamma_rad = np.radians(wing_params["gamma_deg"])

    if stroke_plane_normal is None:
        # Default: stroke plane tilted by gamma from horizontal
        # Normal points roughly along the flight velocity direction
        stroke_plane_normal = np.array([np.sin(gamma_rad), 0.0, np.cos(gamma_rad)])
    if stroke_plane_tangent is None:
        # In-plane direction for φ = 0 (perpendicular to normal, in the XZ plane)
        stroke_plane_tangent = np.array([np.cos(gamma_rad), 0.0, -np.sin(gamma_rad)])

    # Cross product gives the Y-axis (lateral direction)
    lateral = np.cross(stroke_plane_normal, stroke_plane_tangent)

    R = wing_params["span"]
    t_trail = np.linspace(t_current, t_start, n_segments + 1)

    # Evaluate flapping angle at each trail time
    phi_vals, _ = fourier_series(t_trail, omega, wing_params["phi_mean"],
                                 wing_params["phi_harmonics"])

    points = np.zeros((n_segments + 1, 3))
    for k in range(n_segments + 1):
        # Tip position in stroke plane at time t_trail[k]
        in_plane = R * (np.cos(phi_vals[k]) * stroke_plane_tangent +
                        np.sin(phi_vals[k]) * lateral)
        # Advected downstream
        downstream = U_P_mean * (t_current - t_trail[k])
        points[k] = root_pos + in_plane + downstream * stroke_plane_normal

    return points


def compute_biot_savart_at_point(P, filament_points, Gamma=1.0, r_core=1e-3):
    """Compute total induced velocity at P from a vortex filament."""
    v = np.zeros(3)
    for k in range(len(filament_points) - 1):
        v += biot_savart_segment(P, filament_points[k], filament_points[k + 1],
                                  Gamma, r_core)
    return v


def compute_attenuation_coefficient(wing_params, omega, t, dt, U_P_mean,
                                     root_pos=np.zeros(3), n_trail_steps=200,
                                     n_past_wingbeats=2, T_wingbeat=None,
                                     r_core=1e-3):
    """Compute the direct attenuation coefficient C for a wing at time t.

    C(t) = |v_BS(P(t), wake_from_before_t)| / |v_BS(P(t-dt), wake_from_before_(t-dt))|

    The control point is at r = 0.75R (the aerodynamically active region).
    The wake starts from slightly before the current time (not at t itself,
    since the just-shed vortex is handled separately as v_n).
    """
    R = wing_params["span"]
    gamma_rad = np.radians(wing_params["gamma_deg"])
    sp_normal = np.array([np.sin(gamma_rad), 0.0, np.cos(gamma_rad)])
    sp_tangent = np.array([np.cos(gamma_rad), 0.0, -np.sin(gamma_rad)])
    lateral = np.cross(sp_normal, sp_tangent)

    t_start = t - n_past_wingbeats * T_wingbeat
    eta_ctrl = 0.75

    # Control point at 0.75R, current azimuth
    phi_now, _ = fourier_series(np.array([t]), omega, wing_params["phi_mean"],
                                 wing_params["phi_harmonics"])
    P_now = root_pos + eta_ctrl * R * (np.cos(phi_now[0]) * sp_tangent +
                                        np.sin(phi_now[0]) * lateral)

    # Control point at 0.75R, previous azimuth
    phi_prev, _ = fourier_series(np.array([t - dt]), omega, wing_params["phi_mean"],
                                  wing_params["phi_harmonics"])
    P_prev = root_pos + eta_ctrl * R * (np.cos(phi_prev[0]) * sp_tangent +
                                         np.sin(phi_prev[0]) * lateral)

    # Wake filament: starts from t - dt (previous step), not t (current)
    # This is the wake that was "left behind" and affects the current step
    filament_now = trailing_vortex_filament(
        wing_params, omega, t, t_start, n_trail_steps, U_P_mean,
        root_pos, sp_normal, sp_tangent)

    filament_prev = trailing_vortex_filament(
        wing_params, omega, t - dt, t_start, n_trail_steps, U_P_mean,
        root_pos, sp_normal, sp_tangent)

    v_now = compute_biot_savart_at_point(P_now, filament_now, Gamma=1.0,
                                          r_core=r_core)
    v_prev = compute_biot_savart_at_point(P_prev, filament_prev, Gamma=1.0,
                                           r_core=r_core)

    v_n_now = np.dot(v_now, sp_normal)
    v_n_prev = np.dot(v_prev, sp_normal)

    if abs(v_n_prev) < 1e-15:
        return 0.5  # default: moderate decay

    ratio = abs(v_n_now / v_n_prev)
    # The attenuation coefficient should generally be <= 1 (wake decays).
    # Values > 1 can occur transiently when the control point sweeps near
    # the vortex, but we clamp to prevent runaway growth.
    return min(ratio, 1.5)


def compute_interference_coefficient(wing_source, wing_target, omega, t,
                                      U_P_mean_source, U_P_mean_target,
                                      root_source, root_target,
                                      n_trail_steps=200, n_past_wingbeats=2,
                                      T_wingbeat=None, r_core=1e-3):
    """Compute interference coefficient: effect of source wing's wake on target.

    C_hf = v_BS_from_source(P_target) / v_BS_from_target(P_target)

    This is the ratio of induced velocity at the target wing's control point
    due to the source wing's trailing vortex vs. the target's own trailing vortex.
    """
    R_target = wing_target["span"]
    gamma_target = np.radians(wing_target["gamma_deg"])
    sp_normal_t = np.array([np.sin(gamma_target), 0.0, np.cos(gamma_target)])
    sp_tangent_t = np.array([np.cos(gamma_target), 0.0, -np.sin(gamma_target)])
    lateral_t = np.cross(sp_normal_t, sp_tangent_t)

    gamma_source = np.radians(wing_source["gamma_deg"])
    sp_normal_s = np.array([np.sin(gamma_source), 0.0, np.cos(gamma_source)])
    sp_tangent_s = np.array([np.cos(gamma_source), 0.0, -np.sin(gamma_source)])

    t_start = t - n_past_wingbeats * T_wingbeat
    eta_ctrl = 0.75

    # Control point on target wing at 0.75R, current azimuth
    phi_target, _ = fourier_series(np.array([t]), omega, wing_target["phi_mean"],
                                    wing_target["phi_harmonics"])
    P_target = root_target + eta_ctrl * R_target * (
        np.cos(phi_target[0]) * sp_tangent_t +
        np.sin(phi_target[0]) * lateral_t)

    # Trailing vortex from source wing
    filament_source = trailing_vortex_filament(
        wing_source, omega, t, t_start, n_trail_steps, U_P_mean_source,
        root_source, sp_normal_s, sp_tangent_s)

    # Trailing vortex from target wing (own wake)
    filament_target = trailing_vortex_filament(
        wing_target, omega, t, t_start, n_trail_steps, U_P_mean_target,
        root_target, sp_normal_t, sp_tangent_t)

    # Induced velocities at target control point
    v_from_source = compute_biot_savart_at_point(P_target, filament_source,
                                                   Gamma=1.0, r_core=r_core)
    v_from_target = compute_biot_savart_at_point(P_target, filament_target,
                                                   Gamma=1.0, r_core=r_core)

    # Ratio of normal components at target's stroke plane
    v_n_source = np.dot(v_from_source, sp_normal_t)
    v_n_target = np.dot(v_from_target, sp_normal_t)

    if abs(v_n_target) < 1e-15:
        return 0.0

    ratio = v_n_source / v_n_target
    # Clamp to reasonable range (paper shows C_hf varying 0-6, mean ~1.2)
    return np.clip(ratio, 0.0, 6.0)


class LCMSolver:
    """Local Circulation Method solver for a pair of beating wings."""

    def __init__(self, wing_fore, wing_hind, *,
                 frequency_hz, V_flight, rho, body_mass, gravity=9.81,
                 n_span=10, wing_root_distance=8.2e-3, body_tilt_deg=10.0):
        self.fore = wing_fore
        self.hind = wing_hind
        self.omega = 2.0 * np.pi * frequency_hz
        self.T_wb = 1.0 / frequency_hz
        self.V = V_flight
        self.rho = rho
        self.mass = body_mass
        self.g = gravity
        self.n_span = n_span

        # Spanwise discretization
        self.eta_ctrl = np.linspace(0.5 / n_span, 1.0 - 0.5 / n_span, n_span)
        self.eta_wings = np.linspace(1.0 / n_span, 1.0, n_span)

        # Decomposition matrix
        self.A_mat = build_decomposition_matrix(self.eta_ctrl, self.eta_wings)

        # Chord distributions (elliptical)
        self.c_fore = elliptical_chord(self.eta_ctrl, wing_fore["span"],
                                        wing_fore["area"])
        self.c_hind = elliptical_chord(self.eta_ctrl, wing_hind["span"],
                                        wing_hind["area"])

        # Mean perpendicular velocity (for wake advection)
        # Paper Eq. 31: U_P_mean includes induced velocity, not just flight speed.
        # Initialized here with flight-only; updated after each wingbeat in run().
        gamma_f = np.radians(wing_fore["gamma_deg"])
        gamma_h = np.radians(wing_hind["gamma_deg"])
        self.U_P_base_fore = V_flight * np.cos(gamma_f)
        self.U_P_base_hind = V_flight * np.cos(gamma_h)
        self.U_P_mean_fore = self.U_P_base_fore
        self.U_P_mean_hind = self.U_P_base_hind

        # Wing root positions in lab frame
        # Forewing root at origin, hindwing behind it
        body_tilt = np.radians(body_tilt_deg)
        self.root_fore = np.zeros(3)
        self.root_hind = np.array([
            wing_root_distance * np.cos(body_tilt),
            0.0,
            -wing_root_distance * np.sin(body_tilt),
        ])

        # Vortex core radius: use a fraction of the wing span for regularization.
        # Larger core → smoother wake, less prone to spikes when vortex passes
        # close to control point. ~5% of span is reasonable.
        mean_span = 0.5 * (wing_fore["span"] + wing_hind["span"])
        self.r_core = 0.05 * mean_span

        # Index of eta = 0.75 in control points
        self.idx_075 = np.argmin(np.abs(self.eta_ctrl - 0.75))

    def _velocity_components(self, wing, t_sec, eta_stations, v_n, v_t):
        """Compute U_T, U_P at each spanwise station.

        U_T = r * phi_dot + V * sin(gamma) * sin(phi) - v_t
        U_P = V * cos(gamma) + v_n
        """
        R = wing["span"]
        gamma = np.radians(wing["gamma_deg"])

        phi, phi_dot = fourier_series(np.array([t_sec]), self.omega,
                                       wing["phi_mean"], wing["phi_harmonics"])
        psi, _ = fourier_series(np.array([t_sec]), self.omega,
                                 wing["psi_mean"], wing["psi_harmonics"])

        phi_val = phi[0]
        phi_dot_val = phi_dot[0]
        psi_val = psi[0]

        r_stations = eta_stations * R
        U_T = r_stations * phi_dot_val + self.V * np.sin(gamma) * np.sin(phi_val) - v_t
        U_P = self.V * np.cos(gamma) + v_n

        return U_T, U_P, psi_val

    def _solve_inner(self, wing, chord, t_sec, v_n0, max_iter=30, tol=1e-4):
        """Inner iteration: solve for circulation and induced velocity.

        Given v_n0 (memory from previous step), iteratively find the
        self-consistent v_n, v_t, and circulation distribution.

        Returns
        -------
        v_n : (n_span,) normal induced velocity at control points
        v_t : (n_span,) tangential induced velocity at control points
        Gamma : (n_span,) circulation at control points
        alpha : (n_span,) angle of attack at control points
        """
        n = self.n_span
        R = wing["span"]
        r_wings = self.eta_wings * R

        v_n = np.zeros(n)
        v_t = np.zeros(n)

        for iteration in range(max_iter):
            U_T, U_P_base, psi_val = self._velocity_components(
                wing, t_sec, self.eta_ctrl, v_n0 + v_n, v_t)

            U_mag = np.sqrt(U_T**2 + U_P_base**2)
            phi_inflow = np.arctan2(U_P_base, U_T)

            # Angle of attack: alpha = theta - phi_inflow
            # theta_paper = psi_project + pi/2
            theta = psi_val + 0.5 * np.pi
            alpha = theta - phi_inflow

            # Lift coefficient
            Cl = cl_azuma1985(alpha)

            # Circulation at each station
            Gamma = 0.5 * chord * U_mag * Cl

            # Decompose into elliptical components
            # Solve A * dGamma = Gamma
            try:
                dGamma = np.linalg.solve(self.A_mat, Gamma)
            except np.linalg.LinAlgError:
                break

            # Compute new induced velocity from elliptical components
            # For elliptical wing i with half-span r_i and circulation dGamma_i,
            # the downwash at station j (within the wing) is:
            #   w_i = dGamma_i / (4 * r_i)
            # Decomposed into stroke-plane normal and tangential using
            # the inflow angle difference:
            v_n_new = np.zeros(n)
            v_t_new = np.zeros(n)
            for j in range(n):
                for i in range(n):
                    if self.eta_ctrl[j] <= self.eta_wings[i] + 1e-12:
                        w_i = dGamma[i] / (4.0 * r_wings[i])
                        # Inflow angle at the "center" of the i-th elliptical wing
                        # Use the inflow at the nearest control point
                        i_ctrl = min(i, n - 1)
                        dphi = phi_inflow[j] - phi_inflow[i_ctrl]
                        # Decompose: perpendicular to U → normal/tangential to stroke plane
                        v_n_new[j] += w_i * np.cos(dphi)
                        v_t_new[j] += w_i * np.sin(dphi)

            # Relaxation for stability
            relax = 0.5
            change = np.max(np.abs(v_n_new - v_n))
            v_n = (1.0 - relax) * v_n + relax * v_n_new
            v_t = (1.0 - relax) * v_t + relax * v_t_new

            if change < tol * (np.mean(np.abs(v_n)) + 1e-6):
                break

        return v_n, v_t, Gamma, alpha

    def precompute_coefficients(self, n_steps=200, n_trail_steps=150,
                                 n_past_wingbeats=2):
        """Pre-compute attenuation and interference coefficients for one wingbeat.

        These depend only on the wake geometry (not on circulation magnitude),
        so they can be computed once and reused.

        Returns
        -------
        C_f, C_h : (n_steps,) attenuation coefficients
        C_hf : (n_steps,) interference coefficient (fore→hind)
        """
        dt = self.T_wb / n_steps
        C_f = np.ones(n_steps)
        C_h = np.ones(n_steps)
        C_hf = np.zeros(n_steps)

        for step in range(n_steps):
            t = step * dt

            C_f[step] = compute_attenuation_coefficient(
                self.fore, self.omega, t, dt, self.U_P_mean_fore,
                self.root_fore, n_trail_steps, n_past_wingbeats,
                self.T_wb, self.r_core)

            C_h[step] = compute_attenuation_coefficient(
                self.hind, self.omega, t, dt, self.U_P_mean_hind,
                self.root_hind, n_trail_steps, n_past_wingbeats,
                self.T_wb, self.r_core)

            C_hf[step] = compute_interference_coefficient(
                self.fore, self.hind, self.omega, t,
                self.U_P_mean_fore, self.U_P_mean_hind,
                self.root_fore, self.root_hind,
                n_trail_steps, n_past_wingbeats,
                self.T_wb, self.r_core)

        return C_f, C_h, C_hf

    def _wake_induced_at_075(self, wing, t_sec, Gamma_history, t_history,
                             root_pos, U_P_mean):
        """Compute wake-induced velocity at eta=0.75 from Biot-Savart.

        Uses the stored circulation history and trailing vortex geometry
        to compute the induced velocity from the full wake.
        """
        if len(Gamma_history) < 2:
            return 0.0

        R = wing["span"]
        gamma_rad = np.radians(wing["gamma_deg"])
        sp_normal = np.array([np.sin(gamma_rad), 0.0, np.cos(gamma_rad)])
        sp_tangent = np.array([np.cos(gamma_rad), 0.0, -np.sin(gamma_rad)])
        lateral = np.cross(sp_normal, sp_tangent)

        # Control point at 0.75R, current azimuth
        phi_now, _ = fourier_series(np.array([t_sec]), self.omega,
                                     wing["phi_mean"], wing["phi_harmonics"])
        P = root_pos + 0.75 * R * (np.cos(phi_now[0]) * sp_tangent +
                                    np.sin(phi_now[0]) * lateral)

        # Build trailing vortex filament from circulation history
        # Each past time step contributes a vortex segment at the tip position
        v_total = np.zeros(3)
        for k in range(len(t_history) - 1):
            t_k = t_history[k]
            t_k1 = t_history[k + 1]
            Gamma_k = Gamma_history[k]

            phi_k, _ = fourier_series(np.array([t_k]), self.omega,
                                       wing["phi_mean"], wing["phi_harmonics"])
            phi_k1, _ = fourier_series(np.array([t_k1]), self.omega,
                                        wing["phi_mean"], wing["phi_harmonics"])

            # Tip positions, advected downstream
            z_k = U_P_mean * (t_sec - t_k)
            z_k1 = U_P_mean * (t_sec - t_k1)

            A = (root_pos + R * (np.cos(phi_k[0]) * sp_tangent +
                                  np.sin(phi_k[0]) * lateral) +
                 z_k * sp_normal)
            B = (root_pos + R * (np.cos(phi_k1[0]) * sp_tangent +
                                  np.sin(phi_k1[0]) * lateral) +
                 z_k1 * sp_normal)

            v_total += biot_savart_segment(P, A, B, Gamma_k, self.r_core)

        # Negate: BS gives velocity in the "upwash" direction relative to the
        # normal; we want positive = downwash (adds to U_P).
        return -np.dot(v_total, sp_normal)

    def run(self, n_steps=200, n_wingbeats=5, **kwargs):
        """Run the full LCM over multiple wingbeats.

        Uses direct Biot-Savart wake computation instead of attenuation
        coefficients for robustness.

        Returns
        -------
        v_induced_fore : (n_steps,) induced velocity at eta=0.75 for forewing (m/s)
        v_induced_hind : (n_steps,) induced velocity at eta=0.75 for hindwing (m/s)
        diagnostics : dict with additional info
        """
        dt = self.T_wb / n_steps
        idx = self.idx_075
        # Keep circulation history for wake computation (last 2 wingbeats)
        max_history = 2 * n_steps
        Gamma_hist_fore = []
        Gamma_hist_hind = []
        t_hist = []

        v_out_fore = np.zeros(n_steps * n_wingbeats)
        v_out_hind = np.zeros(n_steps * n_wingbeats)

        for wb in range(n_wingbeats):
            for step in range(n_steps):
                global_step = wb * n_steps + step
                t_sec = global_step * dt

                # Compute wake-induced velocity from trailing vortex history
                v_wake_fore = self._wake_induced_at_075(
                    self.fore, t_sec, Gamma_hist_fore, t_hist,
                    self.root_fore, self.U_P_mean_fore)
                v_wake_hind_own = self._wake_induced_at_075(
                    self.hind, t_sec, Gamma_hist_hind, t_hist,
                    self.root_hind, self.U_P_mean_hind)
                # Interference: forewing wake at hindwing control point
                v_wake_hind_from_fore = self._wake_induced_at_075_cross(
                    self.fore, self.hind, t_sec, Gamma_hist_fore, t_hist,
                    self.root_fore, self.root_hind,
                    self.U_P_mean_fore)

                # Total v_n₀ at 0.75R
                v_n0_fore_075 = v_wake_fore
                v_n0_hind_075 = v_wake_hind_own + v_wake_hind_from_fore

                # Apply v_n₀ uniformly across span (simplification)
                v_n0_fore = np.full(self.n_span, v_n0_fore_075)
                v_n0_hind = np.full(self.n_span, v_n0_hind_075)

                # Solve for current self-induced velocity
                v_n_fore, _, Gamma_f, _ = self._solve_inner(
                    self.fore, self.c_fore, t_sec, v_n0_fore)
                v_n_hind, _, Gamma_h, _ = self._solve_inner(
                    self.hind, self.c_hind, t_sec, v_n0_hind)

                # Store circulation at 0.75R for wake history
                Gamma_hist_fore.append(Gamma_f[idx])
                Gamma_hist_hind.append(Gamma_h[idx])
                t_hist.append(t_sec)

                # Trim history to last N steps
                if len(t_hist) > max_history:
                    Gamma_hist_fore = Gamma_hist_fore[-max_history:]
                    Gamma_hist_hind = Gamma_hist_hind[-max_history:]
                    t_hist = t_hist[-max_history:]

                # Total induced velocity at 0.75R
                v_out_fore[global_step] = v_n0_fore_075 + v_n_fore[idx]
                v_out_hind[global_step] = v_n0_hind_075 + v_n_hind[idx]

            # Update wake advection speed with mean induced velocity from
            # this wingbeat (paper Eq. 31: U_P_mean includes downwash).
            v_mean_fore = v_out_fore[wb*n_steps:(wb+1)*n_steps].mean()
            v_mean_hind = v_out_hind[wb*n_steps:(wb+1)*n_steps].mean()
            self.U_P_mean_fore = self.U_P_base_fore + v_mean_fore
            self.U_P_mean_hind = self.U_P_base_hind + v_mean_hind

            print(f"  Wingbeat {wb + 1}/{n_wingbeats}: "
                  f"v_fore={v_mean_fore:.4f}, "
                  f"v_hind={v_mean_hind:.4f}, "
                  f"U_P_fore={self.U_P_mean_fore:.4f}, "
                  f"U_P_hind={self.U_P_mean_hind:.4f}")

        # Return the last wingbeat
        return (v_out_fore[-n_steps:],
                v_out_hind[-n_steps:],
                {})

    def run_coefficients(self, n_steps=200, n_wingbeats=10,
                         n_trail_steps=150, n_past_wingbeats=2):
        """Run the LCM using the paper's attenuation coefficient recursion (Eq. 28).

        Instead of direct Biot-Savart integration of the full wake at every step,
        this method precomputes geometric attenuation and interference coefficients
        for one wingbeat, then uses the recursion:

            v_n0_f(t) = C_f(t) · [v_f(t-dt) + C_fh(t) · v_h(t-dt)]
            v_n0_h(t) = C_h(t) · [v_h(t-dt) + C_hf(t) · v_f(t-dt)]

        where v_f, v_h are the total induced velocities from the previous step.
        """
        dt = self.T_wb / n_steps
        idx = self.idx_075

        # Precompute coefficients (periodic — one wingbeat is enough)
        print("  Precomputing attenuation/interference coefficients...")
        C_f, C_h, C_hf = self.precompute_coefficients(
            n_steps, n_trail_steps, n_past_wingbeats)

        # Clamp self-attenuation to <= 1.0: wake can only decay, not amplify.
        # Values > 1 are numerical artifacts from BS core regularization.
        C_f = np.minimum(C_f, 1.0)
        C_h = np.minimum(C_h, 1.0)

        print(f"  C_f: mean={C_f.mean():.4f}, range=[{C_f.min():.4f}, {C_f.max():.4f}]")
        print(f"  C_h: mean={C_h.mean():.4f}, range=[{C_h.min():.4f}, {C_h.max():.4f}]")
        print(f"  C_hf: mean={C_hf.mean():.4f}, range=[{C_hf.min():.4f}, {C_hf.max():.4f}]")

        v_out_fore = np.zeros(n_steps * n_wingbeats)
        v_out_hind = np.zeros(n_steps * n_wingbeats)

        # Per-wingbeat attenuation: product of per-step C over one wingbeat.
        # This is the fraction of wake that persists after a full wingbeat.
        C_f_wb = np.prod(C_f)
        C_h_wb = np.prod(C_h)
        C_hf_wb = np.mean(C_hf)  # mean interference over a wingbeat
        print(f"  Per-wingbeat: C_f={C_f_wb:.4f}, C_h={C_h_wb:.4f}, "
              f"C_hf={C_hf_wb:.4f}")

        # Wake memory at wingbeat scale: constant offset added to v_n0
        # for all steps within a wingbeat. Updated between wingbeats.
        v_wake_fore = 0.0
        v_wake_hind = 0.0

        for wb in range(n_wingbeats):
            # Solve each step within this wingbeat using current wake memory
            v_n_fore_wb = np.zeros(n_steps)
            v_n_hind_wb = np.zeros(n_steps)

            for step in range(n_steps):
                global_step = wb * n_steps + step
                t_sec = global_step * dt

                v_n0_fore = np.full(self.n_span, v_wake_fore)
                v_n0_hind = np.full(self.n_span, v_wake_hind)

                v_n_fore, _, _, _ = self._solve_inner(
                    self.fore, self.c_fore, t_sec, v_n0_fore)
                v_n_hind, _, _, _ = self._solve_inner(
                    self.hind, self.c_hind, t_sec, v_n0_hind)

                v_n_fore_wb[step] = v_n_fore[idx]
                v_n_hind_wb[step] = v_n_hind[idx]

                v_out_fore[global_step] = v_wake_fore + v_n_fore[idx]
                v_out_hind[global_step] = v_wake_hind + v_n_hind[idx]

            # Update wake memory for next wingbeat (Eq. 28 at wingbeat scale)
            v_mean_fore = v_n_fore_wb.mean()
            v_mean_hind = v_n_hind_wb.mean()
            v_wake_fore = C_f_wb * (v_wake_fore + v_mean_fore)
            v_wake_hind = C_h_wb * (v_wake_hind + v_mean_hind
                                     + C_hf_wb * (v_wake_fore + v_mean_fore))

            v_mean_fore = v_out_fore[wb*n_steps:(wb+1)*n_steps].mean()
            v_mean_hind = v_out_hind[wb*n_steps:(wb+1)*n_steps].mean()
            print(f"  Wingbeat {wb + 1}/{n_wingbeats}: "
                  f"v_fore={v_mean_fore:.4f}, "
                  f"v_hind={v_mean_hind:.4f}")

        return (v_out_fore[-n_steps:],
                v_out_hind[-n_steps:],
                {"C_f": C_f, "C_h": C_h, "C_hf": C_hf})

    def _wake_induced_at_075_cross(self, wing_source, wing_target, t_sec,
                                    Gamma_history, t_history,
                                    root_source, root_target, U_P_mean_source):
        """Wake-induced velocity at target's 0.75R from source's trailing vortex."""
        if len(Gamma_history) < 2:
            return 0.0

        R_target = wing_target["span"]
        R_source = wing_source["span"]
        gamma_target = np.radians(wing_target["gamma_deg"])
        gamma_source = np.radians(wing_source["gamma_deg"])

        sp_normal_t = np.array([np.sin(gamma_target), 0.0, np.cos(gamma_target)])
        sp_tangent_t = np.array([np.cos(gamma_target), 0.0, -np.sin(gamma_target)])
        lateral_t = np.cross(sp_normal_t, sp_tangent_t)

        sp_normal_s = np.array([np.sin(gamma_source), 0.0, np.cos(gamma_source)])
        sp_tangent_s = np.array([np.cos(gamma_source), 0.0, -np.sin(gamma_source)])
        lateral_s = np.cross(sp_normal_s, sp_tangent_s)

        # Control point on target wing at 0.75R
        phi_target, _ = fourier_series(np.array([t_sec]), self.omega,
                                        wing_target["phi_mean"],
                                        wing_target["phi_harmonics"])
        P = root_target + 0.75 * R_target * (np.cos(phi_target[0]) * sp_tangent_t +
                                              np.sin(phi_target[0]) * lateral_t)

        # Source wing trailing vortex
        v_total = np.zeros(3)
        for k in range(len(t_history) - 1):
            t_k = t_history[k]
            t_k1 = t_history[k + 1]
            Gamma_k = Gamma_history[k]

            phi_k, _ = fourier_series(np.array([t_k]), self.omega,
                                       wing_source["phi_mean"],
                                       wing_source["phi_harmonics"])
            phi_k1, _ = fourier_series(np.array([t_k1]), self.omega,
                                        wing_source["phi_mean"],
                                        wing_source["phi_harmonics"])

            z_k = U_P_mean_source * (t_sec - t_k)
            z_k1 = U_P_mean_source * (t_sec - t_k1)

            A = (root_source + R_source * (np.cos(phi_k[0]) * sp_tangent_s +
                                            np.sin(phi_k[0]) * lateral_s) +
                 z_k * sp_normal_s)
            B = (root_source + R_source * (np.cos(phi_k1[0]) * sp_tangent_s +
                                            np.sin(phi_k1[0]) * lateral_s) +
                 z_k1 * sp_normal_s)

            v_total += biot_savart_segment(P, A, B, Gamma_k, self.r_core)

        # Project onto target's stroke plane normal.
        # Negate: BS gives velocity in the "upwash" direction relative to the
        # normal; we want positive = downwash (adds to U_P).
        return -np.dot(v_total, sp_normal_t)

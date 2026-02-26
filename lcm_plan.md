# Local Circulation Method (LCM) — Implementation Plan

## Goal

Implement the Local Circulation Method from Azuma 1985 as a standalone Python module, and compare its angle-of-attack predictions at r = 0.75R against the paper's results (the scatter points in the current AoA comparison plot, stored in `cases/azuma1985/aoa.csv`).

## Background: what the LCM adds

The current simulation computes AoA purely from kinematics — flapping velocity plus body velocity, with **no induced velocity** from the aerodynamic wake. The solid "model" curves in the AoA plot diverge significantly from the paper's LCM results (dots), especially in the mid-stroke region where induced velocity effects are strongest.

The LCM adds a self-consistent induced velocity field by modeling the wake as trailing tip vortices that cling to a cylinder downstream of each stroke plane. It was originally developed for helicopter rotors (Azuma & Kawachi 1979, Azuma et al. 1981) and adapted here for reciprocating (flapping) wings.

## Notation mapping

| Paper symbol | Paper meaning | Project symbol | Notes |
|---|---|---|---|
| ψ | flapping / azimuth angle | φ | |
| θ | pitch / feathering angle | ψ | θ_paper = ψ_project + 90° |
| γ | stroke plane tilt angle | γ | same |
| V | flight velocity | U | 0.54 m/s |
| ψ̇ = ω | flapping rate | φ̇ | |

## Phases

### Phase 1: Pure kinematic AoA (validation baseline)

Reproduce the current simulation's AoA at r = 0.75R in pure Python. This validates our coordinate transforms and velocity computation before adding any induced velocity.

**Algorithm:**
1. Evaluate φ(t), φ̇(t), θ(t) from Fourier harmonics
2. At each time step, compute tangential and perpendicular velocity components:
   - U_T = r·φ̇ + V·sin(γ)·sin(φ)
   - U_P = V·cos(γ)  (no induced velocity)
3. Compute inflow angle: ϕ = atan2(U_P, U_T)
4. Compute AoA: α = θ - ϕ

This should match the "simplified" dashed curves in the current plot.

### Phase 2: Simple analysis (momentum theory)

Add a constant induced velocity from momentum theory (paper Eq. 5):
- v_f = -(V/2) + √(T/(2ρS_e) + (V/2)²)
- v_h = -(V + C_hf·v_f)/2 + √(T/(2ρS_e) + ((V + C_hf·v_f)/2)²)

where S_e is the swept area and T = W/(2·cos γ). Use C_hf = 0.3 as found in the paper.

This constant induced velocity modifies U_P:
- U_P = V·cos(γ) + v

### Phase 3: Full LCM

The core of the method. At each time step, solve for a spanwise distribution of induced velocity that is self-consistent with the circulation.

#### 3a. Elliptical wing decomposition (paper Fig. 10, Eq. 25)

The wing (half-span R) is decomposed into n sub-wings of diminishing span r₁ > r₂ > ... > rₙ, each carrying an elliptical circulation distribution. The total circulation at any spanwise station r is:

    Γ(r) = Σᵢ ΔΓᵢ · √(1 - (r/rᵢ)²)    for r ≤ rᵢ

where ΔΓᵢ is the circulation increment of the i-th elliptical wing.

The induced velocity perpendicular and tangential components at a control point r_j from the i-th elliptical wing (Eq. 25):

    v_p,i(r_j) = ΔΓᵢ/(2rᵢ) · cos(ϕ(r') - ϕ(rᵢ))    (integrated form)
    v_q,i(r_j) = ΔΓᵢ/(2rᵢ) · sin(ϕ(r') - ϕ(rᵢ))

These are then decomposed into stroke-plane normal and tangential components (Eq. 26):
    v_t = v_p·sin(ϕ) + v_q·cos(ϕ)
    v_n = v_p·cos(ϕ) - v_q·sin(ϕ)

#### 3b. Solving for circulation at each time step

At time step j, for each wing pair:

1. **Known from previous step:** v_n₀ = attenuated induced velocity left by all preceding blades (Eq. 28)
2. **Iterative solve:**
   - Guess v_n at each spanwise station
   - Compute U_T = r·φ̇ + V·sin(γ)·sin(φ) - v_t
   - Compute U_P = V·cos(γ) + v_n₀ + v_n
   - Compute α = θ - atan2(U_P, U_T)
   - Look up Cl(α) from airfoil model
   - Compute Γ(r) = ½·c·U·Cl(α) at each station
   - Decompose Γ(r) into elliptical components ΔΓᵢ
   - Recompute v_n from ΔΓᵢ
   - Repeat until convergence

#### 3c. Attenuation and interference coefficients (Eq. 28-36)

The induced velocity left at control points from the previous time step:

    (v_n₀)_f = C_f · (v_f_prev + C_fh · v_h_prev)
    (v_n₀)_h = C_h · (v_h_prev + C_hf · v_f_prev)

The coefficients C_f, C_h, C_fh, C_hf are computed by modeling the trailing tip vortex as a helical filament on a wake cylinder:

1. **Vortex geometry** (Eq. 32-33): The tip vortex traces the path of the wing tip, advected downstream at the mean perpendicular flow speed Ū_P
2. **Biot-Savart integration** (Eq. 34): Compute the induced velocity at the control point P = (R, ψ_P) from vortex element Q = (R, ψ_Q, z_Q)
3. **Attenuation coefficient** = ratio of induced velocity at the control point at time t vs. at time t-Δt (how much the wake from the previous step still affects this step)
4. **Interference coefficient** = ratio of induced velocity at the control point due to the other wing pair's trailing vortex vs. the same wing pair's trailing vortex

The integration uses the full 3D Biot-Savart law with the vortex element dℓ and separation vector a = r_P - r_Q:

    dv = (Γ / 4π) · (dℓ × a) / |a|³

#### 3d. Multiple wingbeats

Run for several wingbeats (3-5) until the solution converges to a periodic state. The attenuation mechanism naturally damps out the initial transient.

### Phase 4: Comparison plot

Plot AoA at r = 0.75R over one wingbeat for:
- Current model (no induced velocity) — from simulation HDF5 output
- Momentum theory correction
- Full LCM
- Paper data (from aoa.csv scatter)

## Input data summary

All from the paper / case.yaml:

| Parameter | Forewing | Hindwing |
|-----------|----------|----------|
| Half-span R (mm) | 33.5 | 32.5 |
| Wing area S (mm²) | 221 | 272 |
| Mean chord c = S/R (mm) | 6.60 | 8.37 |
| Stroke plane γ (°) | 37 | 40 |
| Cone angle β (°) | 8 | -2 |
| φ₀ (°) | 3 | -2 |
| φ₁, δ₁ (°) | 43, 0 | 47, 77 |
| θ₀ (°) | 98 | 93 |
| θ₁, ε₁ (°) | -77, -49 | -65, 18 |
| θ₂, ε₂ (°) | -3, 67 | 8, 74 |
| θ₃, ε₃ (°) | -8, 29 | 8, 28 |
| Frequency (Hz) | 41.5 | 41.5 |
| Flight velocity V (m/s) | 0.54 | 0.54 |
| Body mass (mg) | 260 | 260 |
| Air density ρ (kg/m³) | 1.225 | 1.225 |

Note: θ values above are in paper convention (θ_paper), related to the project's ψ by θ_paper = ψ + 90°. So θ₀ = 98° means ψ₀ = 8°, etc.

## Design decisions

### Validation approach

We validate against the paper by modifying the **simplified AoA formula** — no C++ changes needed. The LCM produces an induced velocity v(t) at r = 0.75R, which we add to the perpendicular velocity component:

    α ≈ ψ - π/2 - atan((U + v) / (-0.75R·φ̇))

where v is the induced velocity normal to the stroke plane (output of the LCM).

### Chord distribution: elliptical

    c(r) = c_max · √(1 - (r/R)²)
    c_max = 4S / (πR)

This satisfies the area constraint ∫₀ᴿ c(r)dr = S.

### Aerodynamic coefficients: Wang 2004 sinusoidal (initial)

    Cl(α) = 1.2 · sin(2α)
    Cd(α) = Cd_min + (Cd_max - Cd_min) · sin²(α)

The paper uses Cl,max ≈ 1.8 — digitized data for the paper's model to be provided later for a true 1:1 comparison.

### Coning angle: ignored

β is set to zero in both the simplified formula and the LCM velocity computation. Its effect on AoA is negligible.

### Wing root separation distance

For the interference coefficients (C_fh, C_hf), we need the distance between the fore and hind wing roots. Table 1 gives "Distance of wing roots = 8.2 mm". This is the fore-hind separation along the body axis.

## Proposed file structure

```
post/
  lcm/
    __init__.py
    kinematics.py      # Fourier series evaluation for φ(t), θ(t)
    aerodynamics.py    # Cl(α), Cd(α) airfoil model
    elliptical.py      # Elliptical wing decomposition & induced velocity
    wake.py            # Trailing vortex geometry, Biot-Savart, attenuation/interference
    solver.py          # Main LCM time-stepping solver
    run_azuma1985.py   # Entry point: run LCM and produce comparison plot
```

## Status

### Completed
- **Phase 1**: Kinematic AoA (no induced velocity) — matches existing "simplified" curves
- **Phase 2**: Momentum theory — constant v_f=0.41, v_h=0.38 m/s
- **Phase 3**: Full LCM with direct Biot-Savart wake computation
  - Elliptical wing decomposition (10 spanwise stations)
  - Iterative solver for circulation ↔ induced velocity
  - Trailing tip vortex wake tracked over 2 wingbeats
  - Fore-hind interference via cross-wake Biot-Savart
  - Azuma 1985 Cl/Cd model (piecewise-linear, digitized from Fig. 8)
  - Wake advection includes induced velocity (paper Eq. 31)
  - Converges in ~8 wingbeats (10 used for safety)
  - Forewing: mean v=0.31 m/s — shape matches paper, amplitude slightly low
  - Hindwing: mean v=0.70 m/s — improved from 1.31 after wake advection fix,
    but still ~2x what the paper shows (~0.4 m/s)

### Key implementation decisions
- Used direct Biot-Savart wake integration instead of attenuation coefficients.
  The paper's attenuation-coefficient approach (Eq. 28) was tried but has a
  fundamental time-scale mismatch (see "Attenuation coefficient findings" below).
  Direct Biot-Savart avoids this by recomputing the full wake effect each step.
- Biot-Savart sign convention: the BS integral gives velocity in the "upwash"
  direction relative to the stroke-plane normal; we negate it so positive =
  downwash (adds to U_P in the simplified formula).
- Vortex core radius = 5% of mean span (Lamb-Oseen regularization). This is
  a numerical parameter introduced by the BS approach — the paper's coefficient
  method would not need it (see below).
- Azuma 1985 Cl/Cd model used (piecewise-linear from paper's Fig. 8).
- Wake advection speed (paper Eq. 31): U_P_mean = V·cos(γ) + v_induced_mean.
  Updated after each wingbeat using the mean induced velocity from that wingbeat.
  This creates a feedback loop (induced velocity → faster advection → wake pushed
  further from control points → weaker induced velocity) that converges in ~8
  wingbeats.
- Unlike the momentum theory, the LCM does not assume aerodynamic force = weight.
  The circulation (and induced velocity) is whatever the aero model predicts for
  the given kinematics.

### Core radius sweep findings
Ran the LCM at r_core = 5%, 8%, 10%, 15% of mean span:
- **Forewing is insensitive** to core radius — all values give similar results
  (mean v ≈ 0.42–0.47 m/s without wake advection fix)
- **Hindwing spreads more** but even at 15% the interference is still too strong
- Core radius shifts BOTH wings proportionally — cannot selectively fix hindwing
  interference without also weakening the (already reasonable) forewing self-induction
- Conclusion: core radius alone cannot resolve the hindwing over-correction

### Wake advection fix (paper Eq. 31)
Including induced velocity in the wake advection speed was the single biggest
improvement:
- Hindwing mean v dropped from 1.31 → 0.70 m/s (nearly halved)
- Forewing mean v dropped from 0.47 → 0.31 m/s (slightly too low now)
- Physically: the downwash pushes the wake cylinder further downstream, so
  trailing vortex filaments end up further from control points, especially
  weakening the fore→hind interference
- Introduces a feedback loop that requires more wingbeats to converge (8-10
  vs 3-5 previously)

### Attenuation coefficient findings
The paper's Eq. 28 recursion was implemented in `solver.run_coefficients()` but
has a fundamental time-scale problem:

**The issue:** The attenuation coefficient C(t) = |v_BS(P(t), wake)| / |v_BS(P(t-dt), wake)|
measures how the wake effect changes between consecutive time steps. With 200
steps per wingbeat (dt = T/200), the wake barely moves in one dt, so C ≈ 1.0
per step. Three failure modes were observed:

1. **Per-step recursion, C unclamped** (mean C_f=1.004): exponential blowup.
   Even slightly > 1.0 mean causes v to diverge within 1-2 wingbeats.

2. **Per-step recursion, C clamped ≤ 1.0** (mean C_f=0.96): still diverges.
   The interference coefficient C_hf (mean 1.56, range 0-6) injects forewing
   energy into the hindwing, which feeds back through circulation. The inner
   solver's v_n (~0.1 m/s) accumulates 200× per wingbeat via the recursion
   (geometric series: v_n / (1-C) ≈ 25 × v_n for C=0.96).

3. **Per-wingbeat recursion** (product of per-step C): C_wb = 0.96^200 ≈ 0.
   Wake memory zeroes out completely. Method degenerates to inner solver only
   with no wake effect (mean v ≈ -0.05 m/s).

**Root cause:** The coefficients are per-step ratios of nearly-identical BS
integrals. They don't represent meaningful physical decay at the dt scale.
The paper's method was likely designed for coarser azimuthal steps (one per
blade passage in helicopter analysis, not 200 per revolution).

**Possible resolution:** Compute attenuation at the half-stroke scale — C as
the ratio of wake-induced velocity between the start and end of one half-stroke.
This matches the physical picture where the wake from the previous half-stroke
decays as the wing sweeps through the current one. However, this interpretation
is speculative without access to the paper's original implementation details.

### Known limitations
- The actual file structure is simpler than proposed: kinematics.py, aerodynamics.py,
  solver.py, run_azuma1985.py (no separate elliptical.py or wake.py — everything
  wake-related is in solver.py).
- Wake-induced velocity computed at eta=0.75 only, applied uniformly across all
  spanwise stations (the paper computes it at each station).
- Only tip vortex tracked (paper may include root vortex contributions).
- The vortex core radius is a numerical artifact of the BS approach that the
  paper's coefficient method would avoid entirely (coefficients are ratios, so
  absolute vortex strength normalizes out).

### Current state of the code
- `solver.run()` — direct Biot-Savart method (working, default)
- `solver.run_coefficients()` — attenuation coefficient method (unstable, experimental)
- Both methods available, selected in `run_azuma1985.py`

### Next steps
- Resolve attenuation coefficient time-scale issue (half-stroke scale? paper details?)
- Further tune BS method: separate core radii for self vs interference?
- Clean up code (refactor solver)

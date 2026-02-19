# Physics Verification Tests

Unit tests that verify the simulation physics against analytical solutions.

## Running Tests

```bash
# From project root
cd build && ctest

# Or run individually
./build/bin/test_freefall
./build/bin/test_blade_element
./build/bin/test_rotation
./build/bin/test_stationary_wing
./build/bin/test_optimizer
./build/bin/test_sampling
./build/bin/test_terminal_velocity

# Verbose output
ctest --output-on-failure
```

## Test Summary

| Test | Component | Analytical Solution |
|------|-----------|---------------------|
| FreeFall | EOM + Integrator | Ballistic trajectory under gravity |
| BladeElement | Aerodynamics | Cd/Cl coefficient formulas |
| Rotation | Kinematics | Rotation matrix properties |
| StationaryWing | Full wing pipeline | Wing in uniform flow |
| Optimizer | Objective function | Mean acceleration convergence |
| Sampling | Sobol sequences | Quasi-random distribution properties |
| TerminalVelocity | End-to-end physics | Steady-state falling wing |

## Test Details

### 1. FreeFall (`test_freefall.cpp`)

**What it tests:** Equations of motion and RK4 integrator with no aerodynamic forces.

**Analytical solution:**
```
x(t) = x₀ + uₓ₀·t
y(t) = y₀ + uᵧ₀·t
z(t) = z₀ + u_z₀·t - ½gt²   (g = 1 in nondimensional units)
```

**Method:** Runs simulation with empty `wings` vector, compares trajectory against closed-form solution.

**Expected accuracy:** Machine precision (~10⁻¹³) since RK4 is exact for constant acceleration.

---

### 2. BladeElement (`test_blade_element.cpp`)

**What it tests:** Aerodynamic coefficient formulas in `blade_element.cpp`.

**Analytical solution:**
```
Cd(α) = Cd_min + 2·sin²(α)     — Drag coefficient
Cl(α) = Cl0·sin(2α)         — Lift coefficient (magnitude)
```

**Method:** Sweeps angle of attack from 0° to 90°, compares computed coefficients against formulas.

**Key values:**
- α = 0°: Cd = Cd_min (minimum drag), Cl = 0
- α = 45°: Cd = Cd_min + 1, Cl = Cl0 (maximum lift)
- α = 90°: Cd = Cd_min + 2 (maximum drag), Cl = 0

---

### 3. Rotation (`test_rotation.cpp`)

**What it tests:** Rotation matrix construction in `rotation.cpp`.

**Analytical properties:**
1. **Orthogonality:** R^T · R = I
2. **Determinant:** det(R) = 1
3. **90° rotations:** Known axis permutations
4. **180° rotations:** Known axis inversions

**Key identities:**
```
rotX(90°): Y → Z,  Z → -Y
rotY(90°): Z → X,  X → -Z
rotZ(90°): X → Y,  Y → -X
```

---

### 4. StationaryWing (`test_stationary_wing.cpp`)

**What it tests:** Full wing force computation pipeline with no flapping motion.

**Setup:** Wing with `phi_dot = 0` and `gam_dot = 0` (stationary) and all angles = 0, subject to uniform flow.

**Analytical solution:** With zero angles for a left wing:
- e_s = (1, 0, 0) — stroke direction
- e_r = (0, 1, 0) — radial/span direction
- e_c = (0, 0, -1) — chord direction

Force magnitude: `|F| = (μ₀/2lb₀) · C · U²` where C is Cd or Cl.

**Test cases:**

| Flow direction | α | Expected result |
|---------------|---|-----------------|
| Along X | 90° | Pure drag, Cd = Cd_min + 2 |
| Along -Z | 0° | Minimum drag, Cd = Cd_min |
| 45° (X and -Z) | 45° | Maximum lift, Cl = Cl0 |
| With Y component | — | Y projected out (span-normal) |

---

### 5. Optimizer (`test_optimizer.cpp`)

**What it tests:** Mean squared acceleration objective function and parameter handling.

**Test cases:**

1. **WingBeatAccel:** Verifies `wingBeatAccel()` returns finite, non-negative values for hover conditions.

2. **Convergence:** Tests that integration converges as N increases:
   - N=20, N=40, N=80 should give results within 5% of each other

3. **VariableParams:** Tests `KinematicParams` get/set round-trip:
   - `numVariable()` returns correct count
   - `variableNames()` returns expected names
   - `setVariableValues()` / `variableValues()` preserve values

---

### 6. Sampling (`test_sampling.cpp`)

**What it tests:** Sobol quasi-random sequence generator for optimization sampling.

**Test cases:**

1. **UnitRange:** All samples fall within [0, 1)^d
2. **Bounds:** Samples respect custom lower/upper bounds
3. **Determinism:** Reset produces identical sequence
4. **Uniformity:** Samples cover space evenly (no empty bins, no clustering)
5. **GenerateSobolSamples:** Convenience function returns correct count and bounds
6. **Index:** Sequence index tracks correctly through calls and reset

---

### 7. TerminalVelocity (`test_terminal_velocity.cpp`)

**What it tests:** End-to-end physics validation — a falling wing reaches the correct terminal velocity.

**Setup:** Wing with fixed pitch angle ψ (no flapping), falling from rest under gravity until steady state.

**Analytical solution:** At terminal velocity, aerodynamic drag balances gravity:

```
|u_z| = √(2λ₀ / (μ₀ · Cd))
```

where Cd depends on the angle of attack, which is determined by ψ.

**Test cases:**

| ψ | Chord orientation | α | Cd | Terminal velocity |
|---|-------------------|---|-----|-------------------|
| π/2 | Horizontal | ±π/2 | Cd₀ + 2 | √(2λ₀ / (μ₀(Cd₀ + 2))) |
| 0 | Vertical (down) | π | Cd₀ | √(2λ₀ / (μ₀·Cd₀)) |
| π | Vertical (up) | π | Cd₀ | √(2λ₀ / (μ₀·Cd₀)) |

**Method:** Integrates until velocity stabilizes, compares against analytical formula.

**Expected accuracy:** < 0.001% relative error.

**Note:** Only ψ ∈ {0, π/2, π, ...} permit pure vertical fall. Other angles cause horizontal drift due to lift.

---

## Adding New Tests

1. Create `tests/test_<name>.cpp`
2. Add to `CMakeLists.txt`:
   ```cmake
   add_executable(test_<name> tests/test_<name>.cpp)
   target_link_libraries(test_<name> PRIVATE dragonfly_physics)
   set_target_properties(test_<name> PROPERTIES
       RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
   )
   add_test(NAME <Name> COMMAND test_<name>)
   ```
3. Rebuild: `cd build && cmake .. && make`

## Test Design Philosophy

These tests verify physics against **analytical solutions** rather than reference data or regression values. This means:

- Tests are self-documenting (the expected values come from formulas)
- No external data files needed
- Failures indicate actual bugs, not just changes
- Easy to understand what's being validated

Each test isolates a specific component:
- **FreeFall:** Integrator correctness (no aerodynamics)
- **BladeElement:** Coefficient formulas (no geometry)
- **Rotation:** Matrix construction (no physics)
- **StationaryWing:** Full pipeline (no time-varying kinematics)
- **Optimizer:** Objective function and parameter handling
- **Sampling:** Quasi-random sequence generation
- **TerminalVelocity:** End-to-end steady-state physics

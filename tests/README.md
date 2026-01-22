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
Cd(α) = Cd0 + 2·sin²(α)     — Drag coefficient
Cl(α) = Cl0·sin(2α)         — Lift coefficient (magnitude)
```

**Method:** Sweeps angle of attack from 0° to 90°, compares computed coefficients against formulas.

**Key values:**
- α = 0°: Cd = Cd0 (minimum drag), Cl = 0
- α = 45°: Cd = Cd0 + 1, Cl = Cl0 (maximum lift)
- α = 90°: Cd = Cd0 + 2 (maximum drag), Cl = 0

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

**Setup:** Wing with `phi_dot = 0` (stationary) and all angles = 0, subject to uniform flow.

**Analytical solution:** With zero angles for a left wing:
- e_s = (1, 0, 0) — stroke direction
- e_r = (0, 1, 0) — radial/span direction
- e_c = (0, 0, -1) — chord direction

Force magnitude: `|F| = (μ₀/2lb₀) · C · U²` where C is Cd or Cl.

**Test cases:**

| Flow direction | α | Expected result |
|---------------|---|-----------------|
| Along X | 90° | Pure drag, Cd = Cd0 + 2 |
| Along -Z | 0° | Minimum drag, Cd = Cd0 |
| 45° (X and -Z) | 45° | Maximum lift, Cl = Cl0 |
| With Y component | — | Y projected out (span-normal) |

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

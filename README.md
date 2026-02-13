# Dragonfly Flight Control

<!-- TODO: one-paragraph project description -->

<p align="center">
  <img src="assets/media/hover.gif" alt="Dragonfly hover simulation" width="600">
</p>

## Building

### Dependencies

- C++17 compiler
- CMake 3.16+
- [Eigen3](https://eigen.tuxfamily.org/)
- [HDF5](https://www.hdfgroup.org/solutions/hdf5/) (C and C++)
- [NLopt](https://github.com/stevengj/nlopt)

[HighFive](https://github.com/BlueBrain/HighFive) is fetched automatically via CMake FetchContent.

### Build

```bash
mkdir -p build && cd build
cmake ..
make
```

### Test

```bash
cd build
ctest
```

## Documentation

Theory and API docs are built with Sphinx (MyST Markdown), with optional C++ API extraction via Doxygen/Breathe.

```bash
python -m pip install -r docs/requirements.txt
make -C docs html
```

Open `docs/_build/html/index.html` in a browser.

Hosted builds can be enabled with Read the Docs via `.readthedocs.yaml`.

## Usage

All simulation modes are accessed through a single `dragonfly` executable:

```
dragonfly <command> [options]

Commands:
  sim      Run flight simulation
  track    Run trajectory tracking simulation
  optim    Find equilibrium flight conditions
  plot     Generate visualization
  wingtest Generate wing rotation test data
  termvel  Compute terminal velocity
```

## Flight Simulation

<!-- TODO: description -->

```bash
dragonfly sim -c configs/sim_hover.cfg
```

<p align="center">
  <img src="assets/media/hover.png" alt="Hover simulation frame" width="600">
</p>

### Config files

| Config | Description |
|--------|-------------|
| `sim_hover.cfg` | Hover flight (4 wings, free flight) |
| `sim_horizontal.cfg` | Horizontal forward flight |
| `sim_inclined.cfg` | Inclined body attitude |
| `sim_figure_eight.cfg` | Figure-eight trajectory |
| `sim_misc.cfg` | General-purpose template |

## Trajectory Tracking

<!-- TODO: description -->

```bash
dragonfly track -c configs/sim_track_circle.cfg
```

<p align="center">
  <img src="assets/media/track_circle.gif" alt="Circle trajectory tracking" width="600">
</p>

<p align="center">
  <img src="assets/media/track_circle_summary.png" alt="Circle tracking: trajectory, error, and control parameters" width="800">
</p>

### Config files

| Config | Description |
|--------|-------------|
| `sim_track_hover.cfg` | PID control to hover at a fixed point |
| `sim_track_circle.cfg` | PID control to track a circular path |

## Equilibrium Optimization

<!-- TODO: description -->

```bash
dragonfly optim -c configs/optim.cfg
```

### Config files

| Config | Description |
|--------|-------------|
| `optim.cfg` | 2-parameter pitch optimization across velocity range |
| `optim_phase_amplitude.cfg` | 4-parameter phase and amplitude study |
| `optim_stroke_plane.cfg` | Stroke plane angle optimization |

### Algorithms

<!-- TODO: brief description of each -->

| Algorithm | Flag | Description |
|-----------|------|-------------|
| COBYLA | `cobyla` | Grid search + local optimizer |
| DIRECT | `direct` | Dividing rectangles global optimizer |
| MLSL | `mlsl` | Multi-level single-linkage |
| CRS2 | `crs2` | Controlled random search |
| Multistart | `multistart` | Sobol sampling + local optimization |

## Visualization

<!-- TODO: description -->

```bash
dragonfly plot -c configs/plot.cfg
```

Rendering uses Blender when available, with a matplotlib fallback. Requires `ffmpeg` for video assembly.

### Python dependencies

```bash
pip install -r requirements.txt
```

### Direct usage

The visualization scripts can also be called directly:

```bash
python -m post.plot_simulation output.h5 animation.mp4 [--no-blender]
python -m post.plot_tracking track.h5 tracking.mp4 [--no-blender]
```

## Wing Kinematics

<!-- TODO: description -->

```bash
dragonfly wingtest --gam 0:90 --phi 0:25 --psi 0:45 -o wingtest.h5
python -m post.plot_wing_rotation wingtest.h5 rotation.mp4
```

<p align="center">
  <img src="assets/media/wing_rotation.gif" alt="Wing rotation basis vectors" width="500">
</p>

## Wang 2007 Pipeline

Convenience pipeline from experimental wing motion data to simulation and postprocessing:

```bash
# Full pipeline (fit -> translate -> sim -> post)
python scripts/wang2007_pipeline.py all --no-blender

# Stage-by-stage (same --run-dir to reuse artifacts)
python scripts/wang2007_pipeline.py fit --run-dir runs/wang2007/demo
python scripts/wang2007_pipeline.py translate --run-dir runs/wang2007/demo
python scripts/wang2007_pipeline.py sim --run-dir runs/wang2007/demo
python scripts/wang2007_pipeline.py post --run-dir runs/wang2007/demo --no-blender
# Skip frames in 3D animation render
python scripts/wang2007_pipeline.py post --run-dir runs/wang2007/demo --frame-step 3 --no-blender
```

By default, the translator derives nondimensional parameters from physical inputs:
- `omega = 2*pi*f*sqrt(L/g)` (default `L=50 mm`, `f=33.4 Hz`)
- `lb0 = lambda = L_wing/L` (default `L_wing=40 mm`)
- `mu0 = mu = rho_air*S_wing*L_wing/m` (default `S_wing=400 mm^2`, `m=300 mg`)

Use `--body-length-mm`, `--wing-length-mm`, `--wing-area-mm2`, `--frequency-hz`,
`--body-mass-mg`, `--rho-air`, and `--gravity` to change scaling.

Outputs are written under `runs/wang2007/<run_id>/`:

- `fit/fit_params.json` + fit diagnostic PDFs
- `sim/sim_wang2007.cfg` + `sim/output.h5`
- `post/simulation.mp4` (+ `post/stroke_fore_left.mp4` unless `--skip-stick`)
- `manifest.json` (stage metadata and artifact paths)

## Terminal Velocity

<!-- TODO: description -->

```bash
dragonfly termvel --psi 45 -o termvel.h5
python -m post.plot_terminal_velocity --psi 45 animation.mp4
```

## Configuration

<!-- TODO: overview of config file format -->

### Kinematic parameters

| Parameter | Description |
|-----------|-------------|
| `omega` | Wing beat frequency (rad/s) |
| `n_harmonics` | Number of Fourier harmonics per angle (`>=1`, default `1`) |
| `gamma_mean` | Stroke plane angle |
| `gamma_amp` | Stroke plane oscillation amplitude |
| `gamma_phase` | Stroke plane phase offset |
| `gamma_cos`, `gamma_sin` | Harmonic cosine/sine coefficients for `gamma` (length `n_harmonics`) |
| `phi_mean` | Mean stroke angle (default `0`) |
| `phi_amp` | Flapping stroke amplitude |
| `phi_cos`, `phi_sin` | Harmonic cosine/sine coefficients for `phi` (length `n_harmonics`) |
| `psi_mean` | Mean pitch angle |
| `psi_amp` | Pitch oscillation amplitude |
| `psi_phase` | Pitch phase offset |
| `psi_cos`, `psi_sin` | Harmonic cosine/sine coefficients for `psi` (length `n_harmonics`) |

Legacy single-harmonic inputs (`*_amp`, `*_phase`) are still supported. When `*_cos`/`*_sin`
are provided, they take precedence for that angle.

Fourier form used by the simulator:

```
angle(t) = mean + sum_{k=1..N}[a_k cos(k*(omega*t + wing_phase)) + b_k sin(k*(omega*t + wing_phase))]
```

The same kinematic keys can be placed inside `[[wing]]` blocks to override motion per wing.

### Wing definition

Each wing is defined in a `[[wing]]` block:

```
[[wing]]
name = fore
side = left
mu0 = 0.075
lb0 = 0.75
Cd0 = 0.4
Cl0 = 1.2
phase = 0.0
```

Per-wing motion overrides are optional. If present, they override global kinematic inputs for that wing only:

```
[[wing]]
...
omega = 12.0
gamma_mean = 1.4
gamma_cos = 0.2, 0.05
gamma_sin = 0.0, -0.02
phi_mean = 0.0
phi_cos = 0.5, 0.1
phi_sin = 0.0, 0.0
psi_mean = 0.7
psi_cos = 0.25, -0.03
psi_sin = 0.0, 0.01
```

You can also use legacy per-wing first-harmonic keys (`gamma_amp`, `gamma_phase`, `phi_amp`, `psi_amp`, `psi_phase`).
When every wing defines its own motion, global `gamma/phi/psi` values can be left at defaults; keep `omega` (and `n_harmonics` if `N>1`) at global scope.

## Output Format

Simulation output is written to HDF5. See [docs/output_format.md](docs/output_format.md) for the full specification.

Coordinate system: right-handed, X = forward, Y = left, Z = up. All quantities nondimensional, angles in radians.

## Project Structure

```
dragonfly-flight-control/
├── apps/           CLI commands (sim, track, optim, plot, wingtest, termvel)
├── src/            Physics engine (aerodynamics, kinematics, integration, control)
├── include/        C++ headers
├── tests/          Unit tests
├── configs/        Simulation config files
├── post/           Python postprocessing and visualization
│   └── blender/    Blender rendering pipeline
├── assets/         Wing meshes (OBJ) and generated media
├── data/           Reference morphological data
└── docs/           Output format specification
```

## License

MIT

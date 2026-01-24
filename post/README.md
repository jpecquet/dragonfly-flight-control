# Postprocessing Scripts

Python tools for visualizing dragonfly simulation output.

## Requirements

```bash
pip install numpy matplotlib h5py
# For animations:
brew install ffmpeg  # macOS
```

## Scripts

### plot_simulation.py

Visualize simulation output (body + wings + force vectors).

```bash
# Static image
python -m post.plot_simulation output.h5 output.png

# Animation
python -m post.plot_simulation output.h5 output.mp4
```

### plot_landscape.py

Plot optimizer objective function landscape (1D or 2D).

```bash
python -m post.plot_landscape optim_landscape.h5 landscape.png
```

### plot_wing_rotation.py

Animate wing basis vectors through rotation phases.

```bash
python -m post.plot_wing_rotation wingtest.h5 rotation.mp4
```

## Module Structure

```
post/
├── __init__.py          # Package init, matplotlib config
├── io.py                # HDF5 readers
├── dragonfly.py         # 3D body/wing rendering
├── plot_simulation.py   # CLI: simulation visualization
├── plot_landscape.py    # CLI: optimizer landscape
└── plot_wing_rotation.py # CLI: wing rotation animation
```

## Library Usage

```python
from post.io import read_simulation, read_landscape, read_wing_rotation
from post.dragonfly import plot_dragonfly

# Read and plot simulation
params, time, states, wings = read_simulation("output.h5")
plot_dragonfly(states, wings, params, "output.png")

# Read landscape data
data = read_landscape("landscape.h5")
```

"""
Dragonfly postprocessing and visualization tools.

Scripts:
    plot_simulation.py   - Visualize simulation output (static or animated)
    plot_landscape.py    - Plot optimizer objective landscape
    plot_wing_rotation.py - Animate wing rotation test results
"""

# Matplotlib configuration for consistent styling
# Guarded so that importing post.* from Blender's Python (no matplotlib) works
try:
    from .style import apply_matplotlib_style

    apply_matplotlib_style()
except ImportError:
    pass
except Exception:
    # Keep package importable in stripped/mocked doc-build environments.
    pass

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
    import matplotlib.pyplot as plt

    plt.rcParams["font.family"] = "serif"
    plt.rcParams["font.serif"] = ["Times New Roman"]
    plt.rcParams["mathtext.fontset"] = "stix"
    plt.rcParams["font.size"] = 12
except ImportError:
    pass

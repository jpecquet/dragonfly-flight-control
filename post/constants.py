"""
Shared geometry constants for dragonfly visualization.

Body segment lengths, radii, wing attachment positions, and visualization parameters
used across the hybrid visualization pipeline.
"""

# Body segment lengths
Lh = 0.1
Lt = 0.25
La = 1 - Lh - Lt  # 0.65

# Body segment radii
Rh = 0.07
Rt = 0.05
Ra = 0.03

# Wing attachment positions
DW = 0.06
FW_X0 = DW / 2
HW_X0 = -DW / 2

# Body segment centers (derived from wing attachment)
A_XC = HW_X0 - La / 2
T_XC = A_XC + La / 2 + Lt / 2
H_XC = T_XC + Lt / 2 + Lh / 2

# Visualization parameters
FORCE_CENTER_FRACTION = 0.67
FORCE_SCALE = 0.05
FORCE_THRESHOLD = 1e-10
DEFAULT_LB0 = 0.75

# Wing Y offsets
RIGHT_WING_Y_OFFSET = -0.02
LEFT_WING_Y_OFFSET = 0.02


def get_wing_info(wing_vectors, wing_params):
    """
    Build wing info list from wing vectors and parameters.

    Args:
        wing_vectors: dict mapping wing names to vector data (first timestep)
        wing_params: dict mapping wing names to lb0 values

    Returns:
        List of (name, xoffset, yoffset, lb0) tuples
    """
    wing_info = []
    for wname in wing_vectors:
        lb0 = wing_params.get(wname, DEFAULT_LB0)
        xoffset = FW_X0 if 'fore' in wname else HW_X0
        yoffset = RIGHT_WING_Y_OFFSET if 'right' in wname else LEFT_WING_Y_OFFSET
        wing_info.append((wname, xoffset, yoffset, lb0))
    return wing_info

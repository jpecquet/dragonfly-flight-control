"""
Shared geometry constants for dragonfly visualization.

Body segment lengths, radii, wing attachment positions, and visualization parameters
used across the hybrid visualization pipeline.
"""

import math

# Body segment lengths
Lh = 0.1
Lt = 0.25
La = 1 - Lh - Lt  # 0.65

# Body segment radii
Rh = 0.07
Rt = 0.05
Ra = 0.03

# Wing attachment positions (matching stick_plot defaults)
WING_ROOT_DISTANCE = 0.1    # fore-to-hind distance along the tilted root axis
WING_ROOT_MIDPOINT_Z = 0.04 # vertical midpoint between fore and hind roots
WING_ROOT_TILT_DEG = 15.0   # tilt of root axis from horizontal

_half = 0.5 * WING_ROOT_DISTANCE
_tilt = math.radians(WING_ROOT_TILT_DEG)
FW_X0 = _half * math.cos(_tilt)
HW_X0 = -_half * math.cos(_tilt)
FW_Z0 = WING_ROOT_MIDPOINT_Z + _half * math.sin(_tilt)
HW_Z0 = WING_ROOT_MIDPOINT_Z - _half * math.sin(_tilt)

# Body segment centers (derived from hindwing attachment X)
A_XC = HW_X0 - La / 2
T_XC = A_XC + La / 2 + Lt / 2
H_XC = T_XC + Lt / 2 + Lh / 2

# Visualization parameters
FORCE_CENTER_FRACTION = 0.67
FORCE_SCALE = 0.05
FORCE_THRESHOLD = 1e-10
DEFAULT_LB0 = 0.75

# Wing lateral (Y) offsets
RIGHT_WING_Y_OFFSET = -0.04
LEFT_WING_Y_OFFSET = 0.04


def get_wing_offsets(wname):
    """Return (xoffset, yoffset, zoffset) for a wing name."""
    xoffset = FW_X0 if 'fore' in wname else HW_X0
    zoffset = FW_Z0 if 'fore' in wname else HW_Z0
    yoffset = RIGHT_WING_Y_OFFSET if 'right' in wname else LEFT_WING_Y_OFFSET
    return xoffset, yoffset, zoffset


def get_wing_info(wing_vectors, wing_params):
    """
    Build wing info list from wing vectors and parameters.

    Args:
        wing_vectors: dict mapping wing names to vector data (first timestep)
        wing_params: dict mapping wing names to lb0 values

    Returns:
        List of (name, xoffset, yoffset, zoffset, lb0) tuples
    """
    wing_info = []
    for wname in wing_vectors:
        lb0 = wing_params.get(wname, DEFAULT_LB0)
        xoffset, yoffset, zoffset = get_wing_offsets(wname)
        wing_info.append((wname, xoffset, yoffset, zoffset, lb0))
    return wing_info

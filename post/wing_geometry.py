"""
Shared wing geometry helpers for visualization.

Wing profile model: composite ellipse around the 1/4-chord pitch axis.
"""

from __future__ import annotations

import numpy as np

DEFAULT_ASPECT_RATIO = 5

def get_chord_ratio(AR):
    # For the default full-ellipse wing planform:
    #   S = (pi/4) * L * c_root
    # so aspect ratio AR = L^2/S = 4 / (pi * (c_root/L)).
    return 4 / (np.pi * AR)

DEFAULT_ROOT_CHORD_RATIO = get_chord_ratio(DEFAULT_ASPECT_RATIO)

def composite_ellipse_polygon_local(
    span: float,
    root_chord: float,
    n_span: int = 24,
    span_sign: float = 1.0,
) -> np.ndarray:
    """
    Build a closed local-space wing polygon.

    Local convention:
    - +X: chord direction (leading edge -> trailing edge)
    - +Y or -Y: span direction (controlled by span_sign)
    - Z = 0
    """
    n = max(2, int(n_span))
    eta = np.linspace(0.0, 1.0, n + 1, dtype=float)
    chord_scale = np.sqrt(np.clip(1.0 - (2.0 * eta - 1.0) ** 2, 0.0, None))

    # +X follows the simulation chord vector e_c. In this convention, leading edge
    # lies on +X and trailing edge on -X relative to the 1/4-chord pitch axis.
    x_le = 0.25 * float(root_chord) * chord_scale
    x_te = -0.75 * float(root_chord) * chord_scale
    y = float(span_sign) * float(span) * eta

    leading = np.column_stack([x_le, y, np.zeros_like(y)])
    trailing = np.column_stack([x_te[-2::-1], y[-2::-1], np.zeros_like(y[:-1])])
    return np.vstack([leading, trailing])


def composite_ellipse_polygon_world(
    root: np.ndarray,
    e_r: np.ndarray,
    e_c: np.ndarray,
    span: float,
    root_chord: float,
    n_span: int = 24,
) -> np.ndarray:
    """Map local composite-ellipse polygon to world coordinates."""
    poly_local = composite_ellipse_polygon_local(span, root_chord, n_span=n_span, span_sign=1.0)
    root = np.asarray(root, dtype=float)
    e_r = np.asarray(e_r, dtype=float)
    e_c = np.asarray(e_c, dtype=float)
    return root + np.outer(poly_local[:, 0], e_c) + np.outer(poly_local[:, 1], e_r)

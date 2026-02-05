"""
Shared configuration for hybrid Blender + Matplotlib visualization.

This module defines camera and style settings that are shared between
the Blender renderer and the matplotlib overlay.
"""

from dataclasses import dataclass, field
from typing import Optional, Tuple
import json
import numpy as np


@dataclass
class CameraConfig:
    """Orthographic camera configuration shared between Blender and matplotlib.

    The figure size is specified in inches (figsize_width, figsize_height) and
    the output resolution is computed as figsize * dpi.
    """
    elevation: float = 30.0  # degrees
    azimuth: float = -60.0   # degrees (isometric-style view)
    ortho_scale: float = 3.0  # world units visible
    figsize_width: float = 6.0   # inches
    figsize_height: float = 4.0  # inches
    dpi: int = 300  # dots per inch

    @property
    def figsize(self) -> Tuple[float, float]:
        """Figure size in inches (width, height)."""
        return (self.figsize_width, self.figsize_height)

    @property
    def resolution(self) -> Tuple[int, int]:
        """Output resolution in pixels (width, height)."""
        return (int(self.figsize_width * self.dpi),
                int(self.figsize_height * self.dpi))

    def to_dict(self) -> dict:
        return {
            'elevation': self.elevation,
            'azimuth': self.azimuth,
            'ortho_scale': self.ortho_scale,
            'figsize_width': self.figsize_width,
            'figsize_height': self.figsize_height,
            'dpi': self.dpi,
        }

    @classmethod
    def from_dict(cls, d: dict) -> 'CameraConfig':
        return cls(**{k: v for k, v in d.items() if k in cls.__dataclass_fields__})


@dataclass
class StyleConfig:
    """Visual style configuration for matplotlib overlay."""
    font_family: str = 'serif'
    font_serif: str = 'Times New Roman'
    mathtext_fontset: str = 'stix'
    font_size: int = 12

    # Colors
    trajectory_color: str = 'blue'
    target_color: str = 'green'
    error_color: str = 'red'
    lift_color: str = 'blue'
    drag_color: str = 'red'

    # Line widths
    trajectory_linewidth: float = 1.5
    force_linewidth: float = 2.0

    # Sizes
    marker_size: float = 50

    def to_dict(self) -> dict:
        return {
            'font_family': self.font_family,
            'font_serif': self.font_serif,
            'mathtext_fontset': self.mathtext_fontset,
            'font_size': self.font_size,
            'trajectory_color': self.trajectory_color,
            'target_color': self.target_color,
            'error_color': self.error_color,
            'lift_color': self.lift_color,
            'drag_color': self.drag_color,
            'trajectory_linewidth': self.trajectory_linewidth,
            'force_linewidth': self.force_linewidth,
            'marker_size': self.marker_size,
        }

    @classmethod
    def from_dict(cls, d: dict) -> 'StyleConfig':
        return cls(**{k: v for k, v in d.items() if k in cls.__dataclass_fields__})


@dataclass
class ViewportConfig:
    """Computed viewport bounds for consistent rendering."""
    center: np.ndarray = field(default_factory=lambda: np.zeros(3))
    extent: float = 2.0

    def to_dict(self) -> dict:
        return {
            'center': self.center.tolist(),
            'extent': self.extent,
        }

    @classmethod
    def from_dict(cls, d: dict) -> 'ViewportConfig':
        return cls(
            center=np.array(d['center']),
            extent=d['extent'],
        )


@dataclass
class HybridConfig:
    """Complete configuration for hybrid rendering."""
    camera: CameraConfig = field(default_factory=CameraConfig)
    style: StyleConfig = field(default_factory=StyleConfig)
    viewport: Optional[ViewportConfig] = None

    # Rendering options
    framerate: int = 30
    trail_length: int = 100
    show_forces: bool = True
    force_scale: float = 0.05

    def to_dict(self) -> dict:
        return {
            'camera': self.camera.to_dict(),
            'style': self.style.to_dict(),
            'viewport': self.viewport.to_dict() if self.viewport else None,
            'framerate': self.framerate,
            'trail_length': self.trail_length,
            'show_forces': self.show_forces,
            'force_scale': self.force_scale,
        }

    @classmethod
    def from_dict(cls, d: dict) -> 'HybridConfig':
        camera = CameraConfig.from_dict(d.get('camera', {}))
        style = StyleConfig.from_dict(d.get('style', {}))
        viewport = ViewportConfig.from_dict(d['viewport']) if d.get('viewport') else None
        return cls(
            camera=camera,
            style=style,
            viewport=viewport,
            framerate=d.get('framerate', 30),
            trail_length=d.get('trail_length', 100),
            show_forces=d.get('show_forces', True),
            force_scale=d.get('force_scale', 0.05),
        )

    def save(self, path: str) -> None:
        """Save configuration to JSON file."""
        with open(path, 'w') as f:
            json.dump(self.to_dict(), f, indent=2)

    @classmethod
    def load(cls, path: str) -> 'HybridConfig':
        """Load configuration from JSON file."""
        with open(path, 'r') as f:
            return cls.from_dict(json.load(f))


def compute_viewport(states, targets=None, padding=1.2) -> ViewportConfig:
    """
    Compute viewport bounds from trajectory data.

    Args:
        states: list of state arrays [x, y, z, ux, uy, uz] or Nx6 array
        targets: optional target positions (Nx3 array)
        padding: multiplier for extent (>1 adds margin)

    Returns:
        ViewportConfig with computed center and extent
    """
    positions = np.array([s[0:3] for s in states])

    if targets is not None:
        positions = np.vstack([positions, targets])

    center = (positions.min(axis=0) + positions.max(axis=0)) / 2
    extent = (positions.max(axis=0) - positions.min(axis=0)).max() * padding

    # Minimum extent to avoid degenerate viewports
    extent = max(extent, 1.0)

    return ViewportConfig(center=center, extent=extent)


def camera_position_from_config(camera: CameraConfig, viewport: ViewportConfig) -> Tuple[np.ndarray, np.ndarray]:
    """
    Compute camera position and focal point from configuration.

    Args:
        camera: Camera configuration
        viewport: Viewport bounds

    Returns:
        Tuple of (camera_position, focal_point) as numpy arrays
    """
    # Convert angles to radians
    elev = np.radians(camera.elevation)
    azim = np.radians(camera.azimuth)

    # Distance from center (for orthographic, this affects what's in frame)
    dist = viewport.extent * 2

    # Compute camera position (spherical coordinates)
    x = dist * np.cos(elev) * np.cos(azim)
    y = dist * np.cos(elev) * np.sin(azim)
    z = dist * np.sin(elev)

    camera_pos = viewport.center + np.array([x, y, z])
    focal_point = viewport.center.copy()

    return camera_pos, focal_point

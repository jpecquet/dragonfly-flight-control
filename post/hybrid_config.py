"""
Shared configuration for hybrid Blender + Matplotlib visualization.

This module defines camera and style settings that are shared between
the Blender renderer and the matplotlib overlay.
"""

from dataclasses import asdict, dataclass, field
from typing import Optional, Tuple
import json
import numpy as np

VIEWPORT_PADDING_UNITS = 0.5


@dataclass
class CameraConfig:
    """Orthographic camera configuration shared between Blender and matplotlib.

    The figure size is specified in inches (figsize_width, figsize_height) and
    the output resolution is computed as figsize * dpi.
    """
    elevation: float = 30.0  # degrees
    azimuth: float = -60.0   # degrees (isometric-style view)
    ortho_scale: float = 3.0  # world units visible
    figsize_width: float = 6.5   # inches
    figsize_height: float = 4.333333333333333  # inches
    dpi: int = 300  # dots per inch
    box_zoom: float = 1.0  # matplotlib 3D axes zoom (visual framing only; limits unchanged)

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
        return asdict(self)

    @classmethod
    def from_dict(cls, d: dict) -> 'CameraConfig':
        return cls(**{k: v for k, v in d.items() if k in cls.__dataclass_fields__})


@dataclass
class StyleConfig:
    """Visual style configuration for matplotlib overlay."""
    theme: str = 'light'

    font_family: str = 'serif'
    font_serif: str = 'Times New Roman'
    mathtext_fontset: str = 'stix'
    font_size: int = 12

    # Figure/axes colors
    figure_facecolor: str = '#ffffff'
    axes_facecolor: str = '#ffffff'
    axes_edge_color: str = '#303030'
    text_color: str = '#101010'
    muted_text_color: str = '#666666'
    grid_color: str = '#b0b0b0'

    # Colors
    trajectory_color: str = '#1f77b4'
    target_color: str = '#2ca02c'
    error_color: str = '#d62728'
    lift_color: str = '#1f77b4'
    drag_color: str = '#d62728'
    body_color: str = '#111111'
    wing_color: str = '#d3d3d3'
    wing_edge_color: str = '#2e2e2e'
    legend_facecolor: str = '#ffffff'
    legend_edge_color: str = '#cccccc'
    landscape_colormap: str = 'viridis'
    landscape_contour_color: str = '#ffffff'
    landscape_min_marker_color: str = '#d62728'

    # Line widths
    trajectory_linewidth: float = 1.5
    force_linewidth: float = 2.0

    # Sizes
    marker_size: float = 50

    @staticmethod
    def normalize_theme(theme: str) -> str:
        """Normalize theme name and fall back to light for unknown themes."""
        name = str(theme).strip().lower()
        if name in ('light', 'dark'):
            return name
        return 'light'

    @classmethod
    def themed(cls, theme: str = 'light') -> 'StyleConfig':
        """Create style defaults for a named theme."""
        name = cls.normalize_theme(theme)
        if name == 'dark':
            return cls(
                theme='dark',
                figure_facecolor='#131416',
                axes_facecolor='#131416',
                axes_edge_color='#c7d1db',
                text_color='#f1f5f9',
                muted_text_color='#9fb0c0',
                grid_color='#4a5563',
                trajectory_color='#4aa3ff',
                target_color='#56d68b',
                error_color='#ff6b6b',
                lift_color='#4aa3ff',
                drag_color='#ff8f6b',
                body_color='#f2f5f7',
                wing_color='#8f9aa6',
                wing_edge_color='#d9dee4',
                legend_facecolor='#1a222b',
                legend_edge_color='#4a5563',
                landscape_colormap='cividis',
                landscape_contour_color='#d9dee4',
                landscape_min_marker_color='#ff6b6b',
            )
        return cls(theme='light')

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, d: dict) -> 'StyleConfig':
        if not d:
            return cls.themed('light')
        theme = cls.normalize_theme(d.get('theme', 'light'))
        style = cls.themed(theme)
        for key, value in d.items():
            if key in cls.__dataclass_fields__:
                setattr(style, key, value)
        style.theme = cls.normalize_theme(style.theme)
        return style


@dataclass
class BlenderRenderConfig:
    """Configuration for Blender render.

    The scale_factor adjusts the apparent size of the dragonfly relative to
    the matplotlib axes. Increase to make dragonfly smaller, decrease to make larger.

    If computed_ortho_scale is set (by compute_blender_ortho_scale()), it will be
    used directly instead of computing ortho_scale = viewport_extent * scale_factor.

    center_offset is the pixel offset from render center to where the viewport
    center appears in matplotlib's projection. This is used to adjust the Blender
    camera position so that the same 3D point appears at the same pixel location
    in both renders.
    """
    scale_factor: float = 1.8    # Fallback multiplier if computed_ortho_scale not set
    computed_ortho_scale: Optional[float] = None  # Exact value from matplotlib projection
    center_offset_x: float = 0.0  # Pixel offset in X
    center_offset_y: float = 0.0  # Pixel offset in Y
    stroke_planes: Optional[dict] = None  # Optional Blender-only stroke-plane plane overlays
    stroke_cones: Optional[dict] = None  # Optional Blender-only wing-cone overlays

    def to_dict(self) -> dict:
        d = {
            'scale_factor': self.scale_factor,
            'center_offset_x': self.center_offset_x,
            'center_offset_y': self.center_offset_y,
        }
        if self.computed_ortho_scale is not None:
            d['computed_ortho_scale'] = self.computed_ortho_scale
        if self.stroke_planes is not None:
            d['stroke_planes'] = self.stroke_planes
        if self.stroke_cones is not None:
            d['stroke_cones'] = self.stroke_cones
        return d

    @classmethod
    def from_dict(cls, d: dict) -> 'BlenderRenderConfig':
        return cls(**{k: v for k, v in d.items() if k in cls.__dataclass_fields__})


@dataclass
class ViewportConfig:
    """Computed viewport bounds for consistent rendering."""
    center: np.ndarray = field(default_factory=lambda: np.zeros(3))
    extent_xyz: np.ndarray = field(default_factory=lambda: np.array([2.0, 2.0, 2.0]))

    @property
    def extent(self) -> float:
        """Legacy scalar extent (max axis span), kept for backward compatibility."""
        return float(np.max(self.extent_xyz))

    @property
    def half_extent_xyz(self) -> np.ndarray:
        return 0.5 * self.extent_xyz

    def to_dict(self) -> dict:
        return {
            'center': self.center.tolist(),
            'extent': self.extent,
            'extent_xyz': self.extent_xyz.tolist(),
        }

    @classmethod
    def from_dict(cls, d: dict) -> 'ViewportConfig':
        center = np.array(d['center'], dtype=float)
        extent_xyz_payload = d.get('extent_xyz')
        if extent_xyz_payload is not None:
            extent_xyz = np.array(extent_xyz_payload, dtype=float)
            if extent_xyz.shape != (3,):
                raise ValueError(f"viewport.extent_xyz must have shape (3,), got {extent_xyz.shape}")
        else:
            extent = float(d['extent'])
            extent_xyz = np.array([extent, extent, extent], dtype=float)
        extent_xyz = np.maximum(extent_xyz, 1.0)
        return cls(
            center=center,
            extent_xyz=extent_xyz,
        )


@dataclass
class HybridConfig:
    """Complete configuration for hybrid rendering."""
    camera: CameraConfig = field(default_factory=CameraConfig)
    style: StyleConfig = field(default_factory=StyleConfig)
    viewport: Optional[ViewportConfig] = None
    blender: BlenderRenderConfig = field(default_factory=BlenderRenderConfig)

    # Rendering options
    framerate: int = 30
    trail_length: int = -1  # <= 0 means draw full trajectory history
    show_axes: bool = True
    show_velocity_text: bool = True
    show_forces: bool = False
    force_scale: float = 0.05

    # Parallelization
    n_workers: int = 0  # 0 = auto (cpu_count)

    def to_dict(self) -> dict:
        return {
            'camera': self.camera.to_dict(),
            'style': self.style.to_dict(),
            'viewport': self.viewport.to_dict() if self.viewport else None,
            'blender': self.blender.to_dict(),
            'framerate': self.framerate,
            'trail_length': self.trail_length,
            'show_axes': self.show_axes,
            'show_velocity_text': self.show_velocity_text,
            'show_forces': self.show_forces,
            'force_scale': self.force_scale,
            'n_workers': self.n_workers,
        }

    @classmethod
    def from_dict(cls, d: dict) -> 'HybridConfig':
        camera = CameraConfig.from_dict(d.get('camera', {}))
        style = StyleConfig.from_dict(d.get('style', {}))
        viewport = ViewportConfig.from_dict(d['viewport']) if d.get('viewport') else None
        blender = BlenderRenderConfig.from_dict(d.get('blender', {}))
        return cls(
            camera=camera,
            style=style,
            viewport=viewport,
            blender=blender,
            framerate=d.get('framerate', 30),
            trail_length=d.get('trail_length', -1),
            show_axes=d.get('show_axes', True),
            show_velocity_text=d.get('show_velocity_text', True),
            show_forces=d.get('show_forces', False),
            force_scale=d.get('force_scale', 0.05),
            n_workers=d.get('n_workers', 0),
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


def compute_viewport(states, targets=None, padding=VIEWPORT_PADDING_UNITS) -> ViewportConfig:
    """
    Compute viewport bounds from trajectory data.

    Args:
        states: list of state arrays [x, y, z, ux, uy, uz] or Nx6 array
        targets: optional target positions (Nx3 array)
        padding: absolute margin in nondimensional length units, applied to each
            side of every axis extent.

    Returns:
        ViewportConfig with computed center and per-axis extents.
    """
    if padding < 0.0:
        raise ValueError(f"padding must be >= 0, got {padding}")

    positions = np.array([s[0:3] for s in states])

    if targets is not None:
        positions = np.vstack([positions, targets])

    mins = positions.min(axis=0) - float(padding)
    maxs = positions.max(axis=0) + float(padding)
    center = 0.5 * (mins + maxs)
    extent_xyz = maxs - mins

    # Minimum extent to avoid degenerate viewports.
    extent_xyz = np.maximum(extent_xyz, 1.0)

    return ViewportConfig(center=center, extent_xyz=extent_xyz)


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

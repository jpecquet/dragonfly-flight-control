"""
Matplotlib 3D axes overlay rendering for hybrid visualization.

Renders transparent PNG overlays with:
- 3D axes with proper styling
- Trajectory trail
- Target position (for tracking)
- Error vectors (for tracking)
- Force vectors (lift/drag)
- Optional simplified body/wing models (matplotlib-only fallback)
"""

import os
import tempfile
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import matplotlib
matplotlib.use('Agg')  # Non-interactive backend
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from mpl_toolkits.mplot3d.proj3d import proj_transform
import numpy as np
from matplotlib.colors import to_rgba

from .constants import (
    DEFAULT_LB0,
    Lh, Lt, La, Rh, Rt, Ra, H_XC, T_XC, A_XC,
    FORCE_CENTER_FRACTION, FORCE_THRESHOLD,
    get_wing_offsets,
)
from .hybrid_config import CameraConfig, StyleConfig, ViewportConfig, HybridConfig
from .progress import ProgressBar, pip_progress
from .style import apply_matplotlib_style
from .wing_geometry import DEFAULT_ROOT_CHORD_RATIO, composite_ellipse_polygon_world

_CHUNK_STATES = None
_CHUNK_WING_VECTORS = None
_CHUNK_PARAMS = None
_CHUNK_CONTROLLER = None
_CHUNK_CONFIG_DICT = None
_CHUNK_OUTPUT_PATH = None
_CHUNK_DRAW_MODELS = False
_CHUNK_BOTTOM_OVERLAY = None
MAX_PARALLEL_WORKERS = 4


def _major_tick_step(extent: float) -> float:
    return 0.5 if extent < 2.0 else 1.0


def _trail_points(states: np.ndarray, frame_idx: int, trail_length: int) -> Optional[np.ndarray]:
    """Return the trajectory segment to draw for this frame (<=0 means full trail)."""
    if frame_idx <= 0:
        return None
    if trail_length <= 0:
        return states[0:frame_idx + 1, 0:3]
    start = max(0, frame_idx - trail_length + 1)
    return states[start:frame_idx + 1, 0:3]


def compute_blender_ortho_scale(
    camera: CameraConfig,
    viewport: ViewportConfig,
    style: Optional[StyleConfig] = None,
    show_axes: bool = True,
) -> Tuple[float, Tuple[float, float]]:
    """
    Compute the exact ortho_scale for Blender to match matplotlib projection.

    Creates a reference matplotlib figure and measures the projection
    to determine the effective ortho_scale and center offset.

    Args:
        camera: Camera configuration
        viewport: Viewport configuration

    Returns:
        Tuple of (ortho_scale, (center_offset_x, center_offset_y)):
        - ortho_scale: World units visible in Blender's horizontal direction
        - center_offset: Pixel offset from render center to where viewport center
          appears in matplotlib (used to adjust Blender camera position)
    """
    import math

    resolved_style = style if style is not None else StyleConfig.themed('light')
    apply_matplotlib_style(resolved_style)
    fig, ax = create_overlay_figure(camera, resolved_style)
    setup_axes(ax, viewport, camera, resolved_style, show_axes=show_axes)
    fig.canvas.draw()  # Force projection matrix update

    proj = ax.get_proj()
    render_width, render_height = camera.resolution

    # Compute camera's horizontal direction in world space
    # The camera right vector lies in the XY plane, perpendicular to view direction
    azim_rad = math.radians(camera.azimuth)
    cam_right = np.array([
        math.sin(azim_rad),
        -math.cos(azim_rad),
        0.0
    ])

    # Project viewport center to display coordinates
    c = viewport.center
    cx, cy, _ = proj_transform(c[0], c[1], c[2], proj)
    center_disp = ax.transData.transform([(cx, cy)])[0]

    # Project a point 1 unit in camera-right direction
    p1 = c + cam_right
    px, py, _ = proj_transform(p1[0], p1[1], p1[2], proj)
    p1_disp = ax.transData.transform([(px, py)])[0]

    # Pixels per world unit in camera horizontal direction
    pixels_per_world = np.linalg.norm(p1_disp - center_disp)

    # Blender ortho_scale = render_width / pixels_per_world_unit
    ortho_scale = render_width / pixels_per_world

    # Compute center offset: where viewport center appears vs render center
    # Positive offset means matplotlib renders it to the right/up of center
    center_offset_x = center_disp[0] - render_width / 2
    center_offset_y = center_disp[1] - render_height / 2

    plt.close(fig)
    return ortho_scale, (center_offset_x, center_offset_y)


def create_overlay_figure(camera: CameraConfig, style: StyleConfig) -> Tuple[plt.Figure, Axes3D]:
    """
    Create matplotlib figure with transparent 3D axes.

    Args:
        camera: Camera configuration

    Returns:
        Tuple of (figure, axes)
    """
    # Create figure with configured size and dpi
    fig = plt.figure(
        figsize=camera.figsize,
        dpi=camera.dpi,
        facecolor=style.figure_facecolor,
    )

    # Create 3D axes
    ax = fig.add_subplot(111, projection='3d', facecolor=style.axes_facecolor)

    # Set view to match Blender camera
    ax.view_init(elev=camera.elevation, azim=camera.azimuth)

    # Use orthographic projection
    ax.set_proj_type('ortho')

    return fig, ax


def setup_axes(
    ax: Axes3D,
    viewport: ViewportConfig,
    camera: CameraConfig,
    style: StyleConfig,
    show_axes: bool = True,
):
    """
    Configure axes limits and appearance.

    Args:
        ax: Matplotlib 3D axes
        viewport: Viewport configuration
        camera: Camera configuration
    """
    # Set axis limits based on per-axis viewport extents.
    half = viewport.half_extent_xyz
    ax.set_xlim(viewport.center[0] - half[0], viewport.center[0] + half[0])
    ax.set_ylim(viewport.center[1] - half[1], viewport.center[1] + half[1])
    ax.set_zlim(viewport.center[2] - half[2], viewport.center[2] + half[2])

    # Keep equal data-unit scaling while allowing non-cubic extents.
    # `zoom` tightens/loosens framing without changing axis limits.
    box_zoom = float(getattr(camera, "box_zoom", 1.0))
    try:
        ax.set_box_aspect(viewport.extent_xyz.tolist(), zoom=box_zoom)
    except TypeError:
        # Backward compatibility with older matplotlib versions lacking `zoom`.
        ax.set_box_aspect(viewport.extent_xyz.tolist())

    if not show_axes:
        # Expand to full canvas and disable axis artists to avoid border artifacts.
        ax.set_position([0.0, 0.0, 1.0, 1.0])
        ax.grid(False)
        ax.set_axis_off()
        return

    # Labels — pad proportional to span so longer axes don't overlap tick labels.
    ex, ey, ez = (float(v) for v in viewport.extent_xyz)
    pad_per_unit = 6.0
    ax.set_xlabel(r'$\tilde{X}$', fontsize=10, color=style.text_color, labelpad=pad_per_unit * ex)
    ax.set_ylabel(r'$\tilde{Y}$', fontsize=10, color=style.text_color, labelpad=pad_per_unit * ey)
    ax.set_zlabel(r'$\tilde{Z}$', fontsize=10, color=style.text_color, labelpad=pad_per_unit * ez)

    # Axis ticks: denser for compact extents, coarser for larger extents.
    ax.xaxis.set_major_locator(MultipleLocator(_major_tick_step(float(viewport.extent_xyz[0]))))
    ax.yaxis.set_major_locator(MultipleLocator(_major_tick_step(float(viewport.extent_xyz[1]))))
    ax.zaxis.set_major_locator(MultipleLocator(_major_tick_step(float(viewport.extent_xyz[2]))))
    ax.tick_params(axis='x', which='major', pad=1)
    ax.tick_params(axis='y', which='major', pad=1)
    ax.tick_params(axis='z', which='major', pad=1)

    # Grid styling
    ax.xaxis.pane.fill = False
    ax.yaxis.pane.fill = False
    ax.zaxis.pane.fill = False
    pane_edge = to_rgba(style.axes_edge_color, alpha=0.8)
    ax.xaxis.pane.set_edgecolor(pane_edge)
    ax.yaxis.pane.set_edgecolor(pane_edge)
    ax.zaxis.pane.set_edgecolor(pane_edge)

    # Lighter grid
    grid_color = to_rgba(style.grid_color, alpha=0.45)
    ax.xaxis._axinfo['grid']['color'] = grid_color
    ax.yaxis._axinfo['grid']['color'] = grid_color
    ax.zaxis._axinfo['grid']['color'] = grid_color


def _wing_frame(wings, frame_idx):
    """Extract single-frame wing vectors from dict-of-arrays format."""
    return {
        wname: {k: v[frame_idx] for k, v in wdata.items()}
        for wname, wdata in wings.items()
    }


def _draw_forces(ax, xb, wing_vectors, wing_lb0, config):
    """Draw lift/drag force vectors for all wings."""
    style = config.style
    for wname, wdata in wing_vectors.items():
        lb0 = wing_lb0.get(wname, DEFAULT_LB0)
        xoffset, yoffset, zoffset = get_wing_offsets(wname)

        origin = xb + np.array([xoffset, yoffset, zoffset])
        cp = origin + FORCE_CENTER_FRACTION * lb0 * wdata['e_r']

        for force, color in [(wdata['lift'], style.lift_color),
                             (wdata['drag'], style.drag_color)]:
            if np.linalg.norm(force) > FORCE_THRESHOLD:
                end = cp + config.force_scale * force
                ax.plot([cp[0], end[0]], [cp[1], end[1]], [cp[2], end[2]],
                        color=color, linewidth=style.force_linewidth)


def _draw_body_and_wings(ax, xb, wing_vectors, wing_lb0, style: StyleConfig):
    """
    Draw simplified dragonfly body and wing geometry in matplotlib 3D.

    This is intended for matplotlib-only fallback rendering where Blender meshes
    are unavailable.
    """
    wing_color = style.wing_color
    wing_edge = style.wing_edge_color

    # Body as a black stick with a hollow head marker.
    tail = xb + np.array([A_XC - La / 2.0, 0.0, 0.0])
    head = xb + np.array([H_XC + Lh / 2.0, 0.0, 0.0])
    ax.plot(
        [tail[0], head[0]], [tail[1], head[1]], [tail[2], head[2]],
        color=style.body_color, linewidth=2.0, solid_capstyle='round', zorder=2,
    )
    ax.scatter(
        [head[0]], [head[1]], [head[2]],
        s=30,
        facecolors=style.axes_facecolor,
        edgecolors=style.body_color,
        linewidths=2.0,
        zorder=3,
    )

    # Wings as thin composite-ellipse plates around the 1/4-chord axis.
    for wname, wdata in wing_vectors.items():
        lb0 = wing_lb0.get(wname, DEFAULT_LB0)
        xoffset, yoffset, zoffset = get_wing_offsets(wname)
        root = xb + np.array([xoffset, yoffset, zoffset])

        e_r = np.asarray(wdata['e_r'], dtype=float)
        e_c = np.asarray(wdata['e_c'], dtype=float)
        nr = np.linalg.norm(e_r)
        nc = np.linalg.norm(e_c)
        if nr < 1e-12 or nc < 1e-12:
            continue
        e_r = e_r / nr
        e_c = e_c / nc

        span = float(lb0)
        root_chord = DEFAULT_ROOT_CHORD_RATIO * span
        verts = composite_ellipse_polygon_world(
            root=root,
            e_r=e_r,
            e_c=e_c,
            span=span,
            root_chord=root_chord,
            n_span=24,
        )

        wing_poly = Poly3DCollection(
            [verts],
            facecolors=wing_color,
            edgecolors=wing_edge,
            linewidths=0.6,
            alpha=0.75,
            zorder=1,
        )
        ax.add_collection3d(wing_poly)

        ax.plot(
            [root[0], (root + span * e_r)[0]],
            [root[1], (root + span * e_r)[1]],
            [root[2], (root + span * e_r)[2]],
            color=wing_edge,
            linewidth=1.2,
            alpha=0.95,
            zorder=2,
        )


def _draw_bottom_wingtip_paths(ax, xb: np.ndarray, payload: Dict) -> None:
    """Draw wingtip paths that belong on the bottom (mpl) layer."""
    wing_tip_offsets = payload.get("wing_tip_offsets", {})
    line_width = float(payload.get("line_width", 1.0))
    line_alpha = float(payload.get("line_alpha", 0.9))
    mono_color = str(payload.get("mono_color", "#ffffff"))
    for wname, tip_offsets in wing_tip_offsets.items():
        path = xb[None, :] + np.asarray(tip_offsets, dtype=float)
        ax.plot(
            path[:, 0],
            path[:, 1],
            path[:, 2],
            linestyle="--",
            linewidth=line_width,
            color=mono_color,
            alpha=line_alpha,
        )


def render_simulation_frame(
    frame_idx: int,
    states: np.ndarray,
    wing_vectors: Dict,
    params: Dict,
    config: HybridConfig,
    output_path: Path,
    draw_models: bool = False,
    render_context: Optional[Tuple[plt.Figure, Axes3D]] = None,
    bottom_overlay_payload: Optional[Dict] = None,
) -> str:
    """
    Render matplotlib overlay for a simulation frame.

    Args:
        frame_idx: Current frame index
        states: State array (N, 6)
        wing_vectors: Dict-of-arrays keyed by wing name
        params: Simulation parameters
        config: Hybrid configuration
        output_path: Output directory

    Returns:
        Path to rendered PNG file
    """
    style = config.style
    if render_context is None:
        apply_matplotlib_style(style)
        fig, ax = create_overlay_figure(config.camera, style)
    else:
        fig, ax = render_context
        ax.cla()
    setup_axes(ax, config.viewport, config.camera, style, show_axes=bool(config.show_axes))

    wing_lb0 = params.get('wing_lb0', {})

    # Current position
    xb = states[frame_idx][0:3]
    v = _wing_frame(wing_vectors, frame_idx)

    if draw_models:
        _draw_body_and_wings(ax, xb, v, wing_lb0, style)

    # Draw trajectory trail
    trail_points = _trail_points(states, frame_idx, int(config.trail_length))
    if trail_points is not None:
        ax.plot(trail_points[:, 0], trail_points[:, 1], trail_points[:, 2],
                color=style.trajectory_color,
                linewidth=style.trajectory_linewidth,
                alpha=0.7)

    if config.show_forces:
        _draw_forces(ax, xb, v, wing_lb0, config)

    # Add velocity text
    if config.show_velocity_text:
        ux, uz = states[frame_idx][3], states[frame_idx][5]
        ax.text2D(0.02, 0.98, f"$u_x$ = {ux:.2f}, $u_z$ = {uz:.2f}",
                  transform=ax.transAxes, fontsize=10, verticalalignment='top',
                  color=style.text_color)

    # Bottom-layer wingtip paths (drawn behind Blender mesh)
    if bottom_overlay_payload is not None:
        _draw_bottom_wingtip_paths(ax, xb, bottom_overlay_payload)

    # Save to file
    output_file = output_path / f"mpl_{frame_idx:06d}.png"
    fig.savefig(output_file, dpi=config.camera.dpi, pad_inches=0)
    if render_context is None:
        plt.close(fig)

    return str(output_file)


def render_tracking_frame(
    frame_idx: int,
    states: np.ndarray,
    wing_vectors: Dict,
    params: Dict,
    controller: Dict,
    config: HybridConfig,
    output_path: Path,
    draw_models: bool = False,
    render_context: Optional[Tuple[plt.Figure, Axes3D]] = None,
) -> str:
    """
    Render matplotlib overlay for a tracking frame.

    Includes trajectory, target, and error visualization.

    Args:
        frame_idx: Current frame index
        states: State array (N, 6)
        wing_vectors: Dict-of-arrays keyed by wing name
        params: Simulation parameters
        controller: Controller data dict
        config: Hybrid configuration
        output_path: Output directory

    Returns:
        Path to rendered PNG file
    """
    style = config.style
    if render_context is None:
        apply_matplotlib_style(style)
        fig, ax = create_overlay_figure(config.camera, style)
    else:
        fig, ax = render_context
        ax.cla()
    setup_axes(ax, config.viewport, config.camera, style, show_axes=bool(config.show_axes))

    wing_lb0 = params.get('wing_lb0', {})

    # Current state
    xb = states[frame_idx][0:3]
    v = _wing_frame(wing_vectors, frame_idx)
    if draw_models:
        _draw_body_and_wings(ax, xb, v, wing_lb0, style)

    target = controller['target_position'][frame_idx]
    error = controller['position_error'][frame_idx]
    error_mag = np.linalg.norm(error)

    # Draw full target trajectory (faded)
    target_positions = controller['target_position']
    unique_targets = np.unique(target_positions, axis=0)
    if len(unique_targets) > 1:
        ax.plot(target_positions[:, 0], target_positions[:, 1], target_positions[:, 2],
                color=style.target_color, linewidth=1.0, alpha=0.3, linestyle='--')

    # Draw actual trajectory trail
    trail_points = _trail_points(states, frame_idx, int(config.trail_length))
    if trail_points is not None:
        ax.plot(trail_points[:, 0], trail_points[:, 1], trail_points[:, 2],
                color=style.trajectory_color,
                linewidth=style.trajectory_linewidth,
                alpha=0.7)

    # Draw current target marker
    ax.scatter([target[0]], [target[1]], [target[2]],
               color=style.target_color, s=style.marker_size, marker='o', alpha=0.8)

    # Draw error line (current position to target)
    if error_mag > 0.01:
        ax.plot([xb[0], target[0]], [xb[1], target[1]], [xb[2], target[2]],
                color=style.error_color, linewidth=1.5, linestyle='--', alpha=0.7)

    if config.show_forces:
        _draw_forces(ax, xb, v, wing_lb0, config)

    # Add info text
    text_lines = [
        f"Error: {error_mag:.3f}",
        f"Target: ({target[0]:.2f}, {target[1]:.2f}, {target[2]:.2f})",
        f"Actual: ({xb[0]:.2f}, {xb[1]:.2f}, {xb[2]:.2f})",
    ]
    if config.show_velocity_text:
        ux, uz = states[frame_idx][3], states[frame_idx][5]
        text_lines.append(f"$u_x$={ux:.2f}, $u_z$={uz:.2f}")
    text = "\n".join(text_lines)
    ax.text2D(0.02, 0.98, text, transform=ax.transAxes, fontsize=9,
              verticalalignment='top', family='monospace',
              color=style.text_color)

    # Legend
    ax.text2D(0.85, 0.98, "Target", color=style.target_color,
              transform=ax.transAxes, fontsize=9, verticalalignment='top')
    ax.text2D(0.85, 0.94, "Actual", color=style.trajectory_color,
              transform=ax.transAxes, fontsize=9, verticalalignment='top')
    ax.text2D(0.85, 0.90, "Error", color=style.error_color,
              transform=ax.transAxes, fontsize=9, verticalalignment='top')

    # Frame counter
    n_frames = len(states)
    ax.text2D(0.98, 0.02, f"Frame {frame_idx + 1}/{n_frames}",
              transform=ax.transAxes, fontsize=8, color=style.muted_text_color,
              verticalalignment='bottom', horizontalalignment='right')

    # Save to file
    output_file = output_path / f"mpl_{frame_idx:06d}.png"
    fig.savefig(output_file, dpi=config.camera.dpi, pad_inches=0)
    if render_context is None:
        plt.close(fig)

    return str(output_file)


def render_all_frames(
    states: np.ndarray,
    wing_vectors: Dict,
    params: Dict,
    config: HybridConfig,
    output_path: Path,
    controller: Optional[Dict] = None,
    draw_models: bool = False,
    progress_callback=None
) -> List[str]:
    """
    Render all matplotlib overlay frames (simulation or tracking).

    Args:
        states: State array (N, 6)
        wing_vectors: Dict-of-arrays keyed by wing name
        params: Simulation parameters
        config: Hybrid configuration
        output_path: Output directory
        controller: Controller data dict (None for simulation mode)
        progress_callback: Optional callback(frame_idx, total) for progress

    Returns:
        List of paths to rendered PNG files
    """
    output_path = Path(output_path)
    output_path.mkdir(parents=True, exist_ok=True)

    n_frames = len(states)
    output_files = []

    if progress_callback:
        for i in range(n_frames):
            if controller is not None:
                filepath = render_tracking_frame(
                    i, states, wing_vectors, params, controller, config, output_path,
                    draw_models=draw_models,
                )
            else:
                filepath = render_simulation_frame(
                    i, states, wing_vectors, params, config, output_path,
                    draw_models=draw_models,
                )
            output_files.append(filepath)
            progress_callback(i, n_frames)
        return output_files

    with pip_progress(n_frames, "Matplotlib", unit="frame") as progress:
        for i in range(n_frames):
            if controller is not None:
                filepath = render_tracking_frame(
                    i, states, wing_vectors, params, controller, config, output_path,
                    draw_models=draw_models,
                )
            else:
                filepath = render_simulation_frame(
                    i, states, wing_vectors, params, config, output_path,
                    draw_models=draw_models,
                )
            output_files.append(filepath)
            progress.update(1)

    return output_files


def _split_frame_ranges(n_frames: int, n_workers: int) -> List[Tuple[int, int]]:
    """Split frame indices into contiguous chunks for worker-level reuse."""
    chunk_size = max(1, n_frames // max(1, n_workers * 4))
    ranges = []
    start = 0
    while start < n_frames:
        end = min(start + chunk_size, n_frames)
        ranges.append((start, end))
        start = end
    return ranges


def _save_array_npy(path: Path, array_like) -> str:
    """Write an array-like object to .npy and return the path."""
    np.save(path, np.asarray(array_like), allow_pickle=False)
    return str(path)


def _prepare_shared_payload(
    states: np.ndarray,
    wing_vectors: Dict,
    controller: Optional[Dict],
    shared_dir: Path,
) -> Dict:
    """
    Prepare file-backed array payload for worker processes.

    Large arrays are written once and loaded by workers using memmap.
    """
    payload = {
        "states_path": _save_array_npy(shared_dir / "states.npy", states),
        "wing_vectors": {},
        "controller": None,
    }

    for wname, wdata in wing_vectors.items():
        wing_payload = {}
        for key, value in wdata.items():
            wing_payload[key] = _save_array_npy(shared_dir / f"wing_{wname}_{key}.npy", value)
        payload["wing_vectors"][wname] = wing_payload

    if controller is not None:
        controller_payload = {}
        for key, value in controller.items():
            if isinstance(value, np.ndarray):
                controller_payload[key] = {
                    "type": "array",
                    "path": _save_array_npy(shared_dir / f"controller_{key}.npy", value),
                }
            else:
                controller_payload[key] = {"type": "literal", "value": value}
        payload["controller"] = controller_payload

    return payload


def _load_shared_payload(shared_payload: Dict) -> Tuple[np.ndarray, Dict, Optional[Dict]]:
    """Load file-backed arrays from shared payload using memmap."""
    states = np.load(shared_payload["states_path"], mmap_mode='r')

    wing_vectors = {}
    for wname, wdata in shared_payload["wing_vectors"].items():
        wing_vectors[wname] = {}
        for key, path in wdata.items():
            wing_vectors[wname][key] = np.load(path, mmap_mode='r')

    controller_payload = shared_payload.get("controller")
    controller = None
    if controller_payload is not None:
        controller = {}
        for key, entry in controller_payload.items():
            if entry["type"] == "array":
                controller[key] = np.load(entry["path"], mmap_mode='r')
            else:
                controller[key] = entry["value"]

    return states, wing_vectors, controller


def _render_frame_range(
    start_frame: int,
    end_frame: int,
    states: np.ndarray,
    wing_vectors: Dict,
    params: Dict,
    config: HybridConfig,
    output_path: Path,
    controller: Optional[Dict] = None,
    draw_models: bool = False,
    progress: Optional[ProgressBar] = None,
    bottom_overlay_payload: Optional[Dict] = None,
) -> List[str]:
    """Render a contiguous frame range, reusing one figure/axes context."""
    apply_matplotlib_style(config.style)
    fig, ax = create_overlay_figure(config.camera, config.style)
    render_context = (fig, ax)

    files = []
    try:
        for frame_idx in range(start_frame, end_frame):
            if controller is not None:
                files.append(
                    render_tracking_frame(
                        frame_idx, states, wing_vectors, params, controller, config, output_path,
                        draw_models=draw_models, render_context=render_context,
                    )
                )
            else:
                files.append(
                    render_simulation_frame(
                        frame_idx, states, wing_vectors, params, config, output_path,
                        draw_models=draw_models, render_context=render_context,
                        bottom_overlay_payload=bottom_overlay_payload,
                    )
                )
            if progress is not None:
                progress.update(1)
    finally:
        plt.close(fig)
    return files


def _chunk_worker_init(
    shared_payload: Dict,
    params: Dict,
    config_dict: Dict,
    output_path_str: str,
    draw_models: bool,
    bottom_overlay_payload: Optional[Dict] = None,
):
    """Initialize worker process globals once to avoid per-frame pickling."""
    global _CHUNK_STATES, _CHUNK_WING_VECTORS, _CHUNK_PARAMS
    global _CHUNK_CONTROLLER, _CHUNK_CONFIG_DICT, _CHUNK_OUTPUT_PATH, _CHUNK_DRAW_MODELS
    global _CHUNK_BOTTOM_OVERLAY
    _CHUNK_STATES, _CHUNK_WING_VECTORS, _CHUNK_CONTROLLER = _load_shared_payload(shared_payload)
    _CHUNK_PARAMS = params
    _CHUNK_CONFIG_DICT = config_dict
    _CHUNK_OUTPUT_PATH = output_path_str
    _CHUNK_DRAW_MODELS = draw_models
    _CHUNK_BOTTOM_OVERLAY = bottom_overlay_payload


def _chunk_worker(frame_range: Tuple[int, int]) -> Tuple[Tuple[int, int], List[str]]:
    """Render one chunk of frames in a worker process."""
    start, end = frame_range
    config = HybridConfig.from_dict(_CHUNK_CONFIG_DICT)
    output_path = Path(_CHUNK_OUTPUT_PATH)
    files = _render_frame_range(
        start, end,
        _CHUNK_STATES, _CHUNK_WING_VECTORS, _CHUNK_PARAMS, config, output_path,
        controller=_CHUNK_CONTROLLER, draw_models=_CHUNK_DRAW_MODELS,
        bottom_overlay_payload=_CHUNK_BOTTOM_OVERLAY,
    )
    return frame_range, files


def render_all_frames_parallel(
    states: np.ndarray,
    wing_vectors: Dict,
    params: Dict,
    config: HybridConfig,
    output_path: Path,
    controller: Optional[Dict] = None,
    draw_models: bool = False,
    n_workers: Optional[int] = None,
    bottom_overlay_payload: Optional[Dict] = None,
) -> List[str]:
    """
    Render all matplotlib overlay frames in parallel.

    Args:
        states: State array (N, 6)
        wing_vectors: Dict-of-arrays keyed by wing name
        params: Simulation parameters
        config: Hybrid configuration
        output_path: Output directory
        controller: Controller data dict (None for simulation mode)
        n_workers: Number of parallel workers (None = auto)
        bottom_overlay_payload: Optional wingtip-path payload for bottom layer

    Returns:
        List of paths to rendered PNG files
    """
    if n_workers is None or n_workers <= 0:
        n_workers = os.cpu_count() or MAX_PARALLEL_WORKERS
    n_workers = max(1, min(int(n_workers), MAX_PARALLEL_WORKERS))

    output_path = Path(output_path)
    output_path.mkdir(parents=True, exist_ok=True)

    n_frames = len(states)
    frame_ranges = _split_frame_ranges(n_frames, n_workers)
    config_dict = config.to_dict()

    # Fast path for serial rendering: still use chunk renderer to reuse figure context.
    if n_workers == 1:
        with pip_progress(n_frames, "Matplotlib", unit="frame") as progress:
            return _render_frame_range(
                0, n_frames, states, wing_vectors, params, config, output_path,
                controller=controller, draw_models=draw_models, progress=progress,
                bottom_overlay_payload=bottom_overlay_payload,
            )

    try:
        with tempfile.TemporaryDirectory(prefix="mpl_shared_") as shared_tmp:
            shared_payload = _prepare_shared_payload(
                states, wing_vectors, controller, Path(shared_tmp)
            )
            with ProcessPoolExecutor(
                max_workers=n_workers,
                initializer=_chunk_worker_init,
                initargs=(
                    shared_payload, params,
                    config_dict, str(output_path), draw_models,
                    bottom_overlay_payload,
                ),
            ) as executor:
                futures = {executor.submit(_chunk_worker, frame_range): frame_range for frame_range in frame_ranges}

                chunk_results: Dict[Tuple[int, int], List[str]] = {}
                with pip_progress(n_frames, "Matplotlib", unit="frame") as progress:
                    for future in as_completed(futures):
                        frame_range, files = future.result()
                        chunk_results[frame_range] = files
                        chunk_len = frame_range[1] - frame_range[0]
                        progress.update(chunk_len)
    except (OSError, PermissionError) as exc:
        # Restricted environments (e.g., some CI sandboxes) may block process pools.
        print(f"  Matplotlib: parallel render unavailable ({exc}); falling back to 1 worker")
        with pip_progress(n_frames, "Matplotlib", unit="frame") as progress:
            return _render_frame_range(
                0, n_frames, states, wing_vectors, params, config, output_path,
                controller=controller, draw_models=draw_models, progress=progress,
                bottom_overlay_payload=bottom_overlay_payload,
            )

    ordered = []
    for frame_range in sorted(chunk_results.keys()):
        ordered.extend(chunk_results[frame_range])
    return ordered

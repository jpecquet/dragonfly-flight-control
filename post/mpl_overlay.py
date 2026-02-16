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
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
from typing import Dict, List, Optional, Tuple

# Optional tqdm support for progress bars
try:
    from tqdm.contrib.concurrent import process_map
    TQDM_AVAILABLE = True
except ImportError:
    TQDM_AVAILABLE = False

import matplotlib
matplotlib.use('Agg')  # Non-interactive backend
import matplotlib.pyplot as plt
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
from .style import apply_matplotlib_style


def compute_blender_ortho_scale(
    camera: CameraConfig,
    viewport: ViewportConfig,
    style: Optional[StyleConfig] = None,
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
    setup_axes(ax, viewport, camera, resolved_style)
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


def setup_axes(ax: Axes3D, viewport: ViewportConfig, camera: CameraConfig, style: StyleConfig):
    """
    Configure axes limits and appearance.

    Args:
        ax: Matplotlib 3D axes
        viewport: Viewport configuration
        camera: Camera configuration
    """
    # Set axis limits based on viewport
    half_extent = viewport.extent / 2

    ax.set_xlim(viewport.center[0] - half_extent, viewport.center[0] + half_extent)
    ax.set_ylim(viewport.center[1] - half_extent, viewport.center[1] + half_extent)
    ax.set_zlim(viewport.center[2] - half_extent, viewport.center[2] + half_extent)

    # Labels
    ax.set_xlabel('X (forward)', fontsize=10, color=style.text_color)
    ax.set_ylabel('Y (left)', fontsize=10, color=style.text_color)
    ax.set_zlabel('Z (up)', fontsize=10, color=style.text_color)

    # Equal aspect ratio (as close as matplotlib allows)
    ax.set_box_aspect([1, 1, 1])

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
        xoffset, yoffset = get_wing_offsets(wname)

        origin = xb + np.array([xoffset, yoffset, 0])
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

    # Wings as thin oriented plates.
    for wname, wdata in wing_vectors.items():
        lb0 = wing_lb0.get(wname, DEFAULT_LB0)
        xoffset, yoffset = get_wing_offsets(wname)
        root = xb + np.array([xoffset, yoffset, 0.0])

        e_r = np.asarray(wdata['e_r'], dtype=float)
        e_c = np.asarray(wdata['e_c'], dtype=float)
        nr = np.linalg.norm(e_r)
        nc = np.linalg.norm(e_c)
        if nr < 1e-12 or nc < 1e-12:
            continue
        e_r = e_r / nr
        e_c = e_c / nc

        span = lb0
        chord = 0.24 * lb0
        root_le = root + 0.5 * chord * e_c
        root_te = root - 0.5 * chord * e_c
        tip_le = root_le + span * e_r
        tip_te = root_te + span * e_r

        wing_poly = Poly3DCollection(
            [[root_le, tip_le, tip_te, root_te]],
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


def render_simulation_frame(
    frame_idx: int,
    states: np.ndarray,
    wing_vectors: Dict,
    params: Dict,
    config: HybridConfig,
    output_path: Path,
    draw_models: bool = False,
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
    apply_matplotlib_style(config.style)
    style = config.style
    fig, ax = create_overlay_figure(config.camera, style)
    setup_axes(ax, config.viewport, config.camera, style)

    wing_lb0 = params.get('wing_lb0', {})

    # Current position
    xb = states[frame_idx][0:3]
    v = _wing_frame(wing_vectors, frame_idx)

    if draw_models:
        _draw_body_and_wings(ax, xb, v, wing_lb0, style)

    # Draw trajectory trail (full history)
    if frame_idx > 0:
        trail_points = states[0:frame_idx + 1, 0:3]
        ax.plot(trail_points[:, 0], trail_points[:, 1], trail_points[:, 2],
                color=style.trajectory_color,
                linewidth=style.trajectory_linewidth,
                alpha=0.7)

    if config.show_forces:
        _draw_forces(ax, xb, v, wing_lb0, config)

    # Add velocity text
    ux, uz = states[frame_idx][3], states[frame_idx][5]
    ax.text2D(0.02, 0.98, f"$u_x$ = {ux:.2f}, $u_z$ = {uz:.2f}",
              transform=ax.transAxes, fontsize=10, verticalalignment='top',
              color=style.text_color)

    # Save to file
    output_file = output_path / f"mpl_{frame_idx:06d}.png"
    fig.savefig(output_file, dpi=config.camera.dpi, pad_inches=0)
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
    apply_matplotlib_style(config.style)
    style = config.style
    fig, ax = create_overlay_figure(config.camera, style)
    setup_axes(ax, config.viewport, config.camera, style)

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

    # Draw actual trajectory trail (full history)
    if frame_idx > 0:
        trail_points = states[0:frame_idx + 1, 0:3]
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
    ux, uz = states[frame_idx][3], states[frame_idx][5]
    text = (f"Error: {error_mag:.3f}\n"
            f"Target: ({target[0]:.2f}, {target[1]:.2f}, {target[2]:.2f})\n"
            f"Actual: ({xb[0]:.2f}, {xb[1]:.2f}, {xb[2]:.2f})\n"
            f"$u_x$={ux:.2f}, $u_z$={uz:.2f}")
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

        if progress_callback:
            progress_callback(i, n_frames)
        elif i % 100 == 0:
            print(f"  Matplotlib: frame {i}/{n_frames}")

    return output_files


def _frame_worker(args):
    """Worker for parallel frame rendering."""
    frame_idx, states, wing_vectors, params, controller, config_dict, output_path_str, draw_models = args
    config = HybridConfig.from_dict(config_dict)
    output_path = Path(output_path_str)
    if controller is not None:
        return render_tracking_frame(
            frame_idx, states, wing_vectors, params, controller, config, output_path,
            draw_models=draw_models,
        )
    return render_simulation_frame(
        frame_idx, states, wing_vectors, params, config, output_path,
        draw_models=draw_models,
    )


def render_all_frames_parallel(
    states: np.ndarray,
    wing_vectors: Dict,
    params: Dict,
    config: HybridConfig,
    output_path: Path,
    controller: Optional[Dict] = None,
    draw_models: bool = False,
    n_workers: Optional[int] = None
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

    Returns:
        List of paths to rendered PNG files
    """
    if n_workers is None or n_workers <= 0:
        n_workers = os.cpu_count() or 4

    output_path = Path(output_path)
    output_path.mkdir(parents=True, exist_ok=True)

    if n_workers == 1:
        return render_all_frames(
            states, wing_vectors, params, config, output_path,
            controller=controller, draw_models=draw_models,
        )

    n_frames = len(states)
    config_dict = config.to_dict()
    output_path_str = str(output_path)

    work = [
        (i, states, wing_vectors, params, controller, config_dict, output_path_str, draw_models)
        for i in range(n_frames)
    ]

    if TQDM_AVAILABLE:
        results = process_map(
            _frame_worker, work,
            max_workers=n_workers, chunksize=10,
            desc="Matplotlib   ", ncols=60
        )
    else:
        print(f"  Matplotlib: rendering {n_frames} frames with {n_workers} workers...")
        with ProcessPoolExecutor(max_workers=n_workers) as executor:
            results = list(executor.map(_frame_worker, work, chunksize=10))
        print(f"  Matplotlib: done ({n_frames} frames)")

    return results

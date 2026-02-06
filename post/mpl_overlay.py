"""
Matplotlib 3D axes overlay rendering for hybrid visualization.

Renders transparent PNG overlays with:
- 3D axes with proper styling
- Trajectory trail
- Target position (for tracking)
- Error vectors (for tracking)
- Force vectors (lift/drag)
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
from mpl_toolkits.mplot3d.proj3d import proj_transform
import numpy as np

from .constants import (
    DW, FW_X0, HW_X0, DEFAULT_LB0,
    FORCE_CENTER_FRACTION, FORCE_THRESHOLD,
    RIGHT_WING_Y_OFFSET, LEFT_WING_Y_OFFSET,
)
from .hybrid_config import CameraConfig, StyleConfig, ViewportConfig, HybridConfig


def compute_blender_ortho_scale(
    camera: CameraConfig,
    viewport: ViewportConfig
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

    fig, ax = create_overlay_figure(camera)
    setup_axes(ax, viewport, camera)
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


def apply_style(style: StyleConfig):
    """Apply matplotlib style settings."""
    plt.rcParams["font.family"] = style.font_family
    plt.rcParams["font.serif"] = [style.font_serif]
    plt.rcParams["mathtext.fontset"] = style.mathtext_fontset
    plt.rcParams["font.size"] = style.font_size


def create_overlay_figure(camera: CameraConfig) -> Tuple[plt.Figure, Axes3D]:
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
        facecolor='white'
    )

    # Create 3D axes
    ax = fig.add_subplot(111, projection='3d', facecolor='white')

    # Set view to match Blender camera
    ax.view_init(elev=camera.elevation, azim=camera.azimuth)

    # Use orthographic projection
    ax.set_proj_type('ortho')

    return fig, ax


def setup_axes(ax: Axes3D, viewport: ViewportConfig, camera: CameraConfig):
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
    ax.set_xlabel('X (forward)', fontsize=10)
    ax.set_ylabel('Y (left)', fontsize=10)
    ax.set_zlabel('Z (up)', fontsize=10)

    # Equal aspect ratio (as close as matplotlib allows)
    ax.set_box_aspect([1, 1, 1])

    # Grid styling
    ax.xaxis.pane.fill = False
    ax.yaxis.pane.fill = False
    ax.zaxis.pane.fill = False
    ax.xaxis.pane.set_edgecolor('gray')
    ax.yaxis.pane.set_edgecolor('gray')
    ax.zaxis.pane.set_edgecolor('gray')

    # Lighter grid
    ax.xaxis._axinfo['grid']['color'] = (0.8, 0.8, 0.8, 0.5)
    ax.yaxis._axinfo['grid']['color'] = (0.8, 0.8, 0.8, 0.5)
    ax.zaxis._axinfo['grid']['color'] = (0.8, 0.8, 0.8, 0.5)


def render_simulation_frame(
    frame_idx: int,
    states: List[np.ndarray],
    wing_vectors: List[Dict],
    params: Dict,
    config: HybridConfig,
    output_path: Path
) -> str:
    """
    Render matplotlib overlay for a simulation frame.

    Args:
        frame_idx: Current frame index
        states: List of state arrays
        wing_vectors: List of wing vector dicts
        params: Simulation parameters
        config: Hybrid configuration
        output_path: Output directory

    Returns:
        Path to rendered PNG file
    """
    apply_style(config.style)
    fig, ax = create_overlay_figure(config.camera)
    setup_axes(ax, config.viewport, config.camera)

    style = config.style
    wing_lb0 = params.get('wing_lb0', {})

    # Current position
    xb = states[frame_idx][0:3]
    v = wing_vectors[frame_idx]

    # Draw trajectory trail (full history)
    if frame_idx > 0:
        trail_points = np.array([s[0:3] for s in states[0:frame_idx + 1]])
        ax.plot(trail_points[:, 0], trail_points[:, 1], trail_points[:, 2],
                color='black', linewidth=0.5, alpha=0.7)

    # Draw force vectors
    if config.show_forces:
        wing_names = list(v.keys())

        for wname in wing_names:
            lb0 = wing_lb0.get(wname, DEFAULT_LB0)
            xoffset = FW_X0 if 'fore' in wname else HW_X0
            yoffset = RIGHT_WING_Y_OFFSET if 'right' in wname else LEFT_WING_Y_OFFSET

            origin = xb + np.array([xoffset, yoffset, 0])
            cp = origin + FORCE_CENTER_FRACTION * lb0 * v[wname]['e_r']

            # Lift vector
            lift = v[wname]['lift']
            if np.linalg.norm(lift) > FORCE_THRESHOLD:
                end = cp + config.force_scale * lift
                ax.plot([cp[0], end[0]], [cp[1], end[1]], [cp[2], end[2]],
                        color=style.lift_color, linewidth=style.force_linewidth)

            # Drag vector
            drag = v[wname]['drag']
            if np.linalg.norm(drag) > FORCE_THRESHOLD:
                end = cp + config.force_scale * drag
                ax.plot([cp[0], end[0]], [cp[1], end[1]], [cp[2], end[2]],
                        color=style.drag_color, linewidth=style.force_linewidth)

    # Add velocity text
    ux, uz = states[frame_idx][3], states[frame_idx][5]
    ax.text2D(0.02, 0.98, f"$u_x$ = {ux:.2f}, $u_z$ = {uz:.2f}",
              transform=ax.transAxes, fontsize=10, verticalalignment='top')

    # Save to file
    output_file = output_path / f"mpl_{frame_idx:06d}.png"
    fig.savefig(output_file, dpi=config.camera.dpi, pad_inches=0)
    plt.close(fig)

    return str(output_file)


def render_tracking_frame(
    frame_idx: int,
    states: List[np.ndarray],
    wing_vectors: List[Dict],
    params: Dict,
    controller: Dict,
    config: HybridConfig,
    output_path: Path
) -> str:
    """
    Render matplotlib overlay for a tracking frame.

    Includes trajectory, target, and error visualization.

    Args:
        frame_idx: Current frame index
        states: List of state arrays
        wing_vectors: List of wing vector dicts
        params: Simulation parameters
        controller: Controller data dict
        config: Hybrid configuration
        output_path: Output directory

    Returns:
        Path to rendered PNG file
    """
    apply_style(config.style)
    fig, ax = create_overlay_figure(config.camera)
    setup_axes(ax, config.viewport, config.camera)

    style = config.style
    wing_lb0 = params.get('wing_lb0', {})

    # Current state
    xb = states[frame_idx][0:3]
    v = wing_vectors[frame_idx]
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
        trail_points = np.array([s[0:3] for s in states[0:frame_idx + 1]])
        ax.plot(trail_points[:, 0], trail_points[:, 1], trail_points[:, 2],
                color='black', linewidth=0.5, alpha=0.7)

    # Draw current target marker
    ax.scatter([target[0]], [target[1]], [target[2]],
               color=style.target_color, s=style.marker_size, marker='o', alpha=0.8)

    # Draw error line (current position to target)
    if error_mag > 0.01:
        ax.plot([xb[0], target[0]], [xb[1], target[1]], [xb[2], target[2]],
                color=style.error_color, linewidth=1.5, linestyle='--', alpha=0.7)

    # Draw force vectors
    if config.show_forces:
        wing_names = list(v.keys())

        for wname in wing_names:
            lb0 = wing_lb0.get(wname, DEFAULT_LB0)
            xoffset = FW_X0 if 'fore' in wname else HW_X0
            yoffset = RIGHT_WING_Y_OFFSET if 'right' in wname else LEFT_WING_Y_OFFSET

            origin = xb + np.array([xoffset, yoffset, 0])
            cp = origin + FORCE_CENTER_FRACTION * lb0 * v[wname]['e_r']

            # Lift vector
            lift = v[wname]['lift']
            if np.linalg.norm(lift) > FORCE_THRESHOLD:
                end = cp + config.force_scale * lift
                ax.plot([cp[0], end[0]], [cp[1], end[1]], [cp[2], end[2]],
                        color=style.lift_color, linewidth=style.force_linewidth)

            # Drag vector
            drag = v[wname]['drag']
            if np.linalg.norm(drag) > FORCE_THRESHOLD:
                end = cp + config.force_scale * drag
                ax.plot([cp[0], end[0]], [cp[1], end[1]], [cp[2], end[2]],
                        color=style.drag_color, linewidth=style.force_linewidth)

    # Add info text
    ux, uz = states[frame_idx][3], states[frame_idx][5]
    text = (f"Error: {error_mag:.3f}\n"
            f"Target: ({target[0]:.2f}, {target[1]:.2f}, {target[2]:.2f})\n"
            f"Actual: ({xb[0]:.2f}, {xb[1]:.2f}, {xb[2]:.2f})\n"
            f"$u_x$={ux:.2f}, $u_z$={uz:.2f}")
    ax.text2D(0.02, 0.98, text, transform=ax.transAxes, fontsize=9,
              verticalalignment='top', family='monospace')

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
              transform=ax.transAxes, fontsize=8, color='gray',
              verticalalignment='bottom', horizontalalignment='right')

    # Save to file
    output_file = output_path / f"mpl_{frame_idx:06d}.png"
    fig.savefig(output_file, dpi=config.camera.dpi, pad_inches=0)
    plt.close(fig)

    return str(output_file)


def render_all_simulation_frames(
    states: List[np.ndarray],
    wing_vectors: List[Dict],
    params: Dict,
    config: HybridConfig,
    output_path: Path,
    progress_callback=None
) -> List[str]:
    """
    Render all matplotlib overlay frames for simulation.

    Args:
        states: List of state arrays
        wing_vectors: List of wing vector dicts
        params: Simulation parameters
        config: Hybrid configuration
        output_path: Output directory
        progress_callback: Optional callback(frame_idx, total) for progress

    Returns:
        List of paths to rendered PNG files
    """
    output_path = Path(output_path)
    output_path.mkdir(parents=True, exist_ok=True)

    n_frames = len(states)
    output_files = []

    for i in range(n_frames):
        filepath = render_simulation_frame(
            i, states, wing_vectors, params, config, output_path
        )
        output_files.append(filepath)

        if progress_callback:
            progress_callback(i, n_frames)
        elif i % 100 == 0:
            print(f"  Matplotlib: frame {i}/{n_frames}")

    return output_files


def render_all_tracking_frames(
    states: List[np.ndarray],
    wing_vectors: List[Dict],
    params: Dict,
    controller: Dict,
    config: HybridConfig,
    output_path: Path,
    progress_callback=None
) -> List[str]:
    """
    Render all matplotlib overlay frames for tracking.

    Args:
        states: List of state arrays
        wing_vectors: List of wing vector dicts
        params: Simulation parameters
        controller: Controller data dict
        config: Hybrid configuration
        output_path: Output directory
        progress_callback: Optional callback(frame_idx, total) for progress

    Returns:
        List of paths to rendered PNG files
    """
    output_path = Path(output_path)
    output_path.mkdir(parents=True, exist_ok=True)

    n_frames = len(states)
    output_files = []

    for i in range(n_frames):
        filepath = render_tracking_frame(
            i, states, wing_vectors, params, controller, config, output_path
        )
        output_files.append(filepath)

        if progress_callback:
            progress_callback(i, n_frames)
        elif i % 100 == 0:
            print(f"  Matplotlib: frame {i}/{n_frames}")

    return output_files


def _simulation_frame_worker(args):
    """
    Worker function for parallel simulation frame rendering.

    Args:
        args: Tuple of (frame_idx, states, wing_vectors, params, config_dict, output_path_str)

    Returns:
        Path to rendered PNG file
    """
    frame_idx, states, wing_vectors, params, config_dict, output_path_str = args
    config = HybridConfig.from_dict(config_dict)
    output_path = Path(output_path_str)
    return render_simulation_frame(frame_idx, states, wing_vectors, params, config, output_path)


def _tracking_frame_worker(args):
    """
    Worker function for parallel tracking frame rendering.

    Args:
        args: Tuple of (frame_idx, states, wing_vectors, params, controller, config_dict, output_path_str)

    Returns:
        Path to rendered PNG file
    """
    frame_idx, states, wing_vectors, params, controller, config_dict, output_path_str = args
    config = HybridConfig.from_dict(config_dict)
    output_path = Path(output_path_str)
    return render_tracking_frame(frame_idx, states, wing_vectors, params, controller, config, output_path)


def render_all_simulation_frames_parallel(
    states: List[np.ndarray],
    wing_vectors: List[Dict],
    params: Dict,
    config: HybridConfig,
    output_path: Path,
    n_workers: Optional[int] = None
) -> List[str]:
    """
    Render all matplotlib overlay frames for simulation in parallel.

    Args:
        states: List of state arrays
        wing_vectors: List of wing vector dicts
        params: Simulation parameters
        config: Hybrid configuration
        output_path: Output directory
        n_workers: Number of parallel workers (None = auto)

    Returns:
        List of paths to rendered PNG files
    """
    if n_workers is None or n_workers <= 0:
        n_workers = os.cpu_count() or 4

    output_path = Path(output_path)
    output_path.mkdir(parents=True, exist_ok=True)

    n_frames = len(states)
    config_dict = config.to_dict()
    output_path_str = str(output_path)

    # Prepare work items
    work = [
        (i, states, wing_vectors, params, config_dict, output_path_str)
        for i in range(n_frames)
    ]

    if TQDM_AVAILABLE:
        results = process_map(
            _simulation_frame_worker, work,
            max_workers=n_workers, chunksize=10,
            desc="Matplotlib   ", ncols=60
        )
    else:
        print(f"  Matplotlib: rendering {n_frames} frames with {n_workers} workers...")
        with ProcessPoolExecutor(max_workers=n_workers) as executor:
            results = list(executor.map(_simulation_frame_worker, work, chunksize=10))
        print(f"  Matplotlib: done ({n_frames} frames)")

    return results


def render_all_tracking_frames_parallel(
    states: List[np.ndarray],
    wing_vectors: List[Dict],
    params: Dict,
    controller: Dict,
    config: HybridConfig,
    output_path: Path,
    n_workers: Optional[int] = None
) -> List[str]:
    """
    Render all matplotlib overlay frames for tracking in parallel.

    Args:
        states: List of state arrays
        wing_vectors: List of wing vector dicts
        params: Simulation parameters
        controller: Controller data dict
        config: Hybrid configuration
        output_path: Output directory
        n_workers: Number of parallel workers (None = auto)

    Returns:
        List of paths to rendered PNG files
    """
    if n_workers is None or n_workers <= 0:
        n_workers = os.cpu_count() or 4

    output_path = Path(output_path)
    output_path.mkdir(parents=True, exist_ok=True)

    n_frames = len(states)
    config_dict = config.to_dict()
    output_path_str = str(output_path)

    # Prepare work items
    work = [
        (i, states, wing_vectors, params, controller, config_dict, output_path_str)
        for i in range(n_frames)
    ]

    if TQDM_AVAILABLE:
        results = process_map(
            _tracking_frame_worker, work,
            max_workers=n_workers, chunksize=10,
            desc="Matplotlib   ", ncols=60
        )
    else:
        print(f"  Matplotlib: rendering {n_frames} frames with {n_workers} workers...")
        with ProcessPoolExecutor(max_workers=n_workers) as executor:
            results = list(executor.map(_tracking_frame_worker, work, chunksize=10))
        print(f"  Matplotlib: done ({n_frames} frames)")

    return results

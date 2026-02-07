"""
PNG compositing and video assembly for hybrid visualization.

Combines Blender-rendered mesh frames with matplotlib overlays,
then assembles into final video.
"""

import json
import math
import os
import platform
import shutil
import subprocess
import tempfile
import time
from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor, as_completed
from pathlib import Path
from typing import Dict, List, Optional, Tuple

# Optional tqdm support for progress bars
try:
    from tqdm import tqdm
    from tqdm.contrib.concurrent import thread_map
    TQDM_AVAILABLE = True
except ImportError:
    TQDM_AVAILABLE = False

import numpy as np
from PIL import Image

from .hybrid_config import HybridConfig, BlenderRenderConfig, compute_viewport
from .mpl_overlay import (
    compute_blender_ortho_scale,
    render_all_simulation_frames,
    render_all_tracking_frames,
    render_all_simulation_frames_parallel,
    render_all_tracking_frames_parallel,
)

# Common Blender installation paths by platform
BLENDER_PATHS = {
    'Darwin': [
        '/Applications/Blender.app/Contents/MacOS/Blender',
        '/Applications/Blender/Blender.app/Contents/MacOS/Blender',
    ],
    'Linux': [
        '/usr/bin/blender',
        '/snap/bin/blender',
    ],
    'Windows': [
        r'C:\Program Files\Blender Foundation\Blender\blender.exe',
        r'C:\Program Files\Blender Foundation\Blender 4.0\blender.exe',
        r'C:\Program Files\Blender Foundation\Blender 3.6\blender.exe',
    ],
}


def find_blender() -> Optional[str]:
    """
    Find Blender executable on the system.

    Checks PATH first, then common installation locations.

    Returns:
        Path to Blender executable, or None if not found
    """
    # First try PATH
    try:
        result = subprocess.run(
            ['blender', '--version'],
            capture_output=True,
            text=True
        )
        if result.returncode == 0:
            return 'blender'
    except FileNotFoundError:
        pass

    # Check platform-specific paths
    system = platform.system()
    for path in BLENDER_PATHS.get(system, []):
        if Path(path).exists():
            return path

    return None


def composite_frames(
    blender_dir: Path,
    mpl_dir: Path,
    output_dir: Path,
    n_frames: int
) -> List[str]:
    """
    Composite Blender and matplotlib frames.

    Blender frame is placed on top of matplotlib frame (axes/trail behind mesh).

    Args:
        blender_dir: Directory containing Blender frames (frame_NNNNNN.png)
        mpl_dir: Directory containing matplotlib frames (mpl_NNNNNN.png)
        output_dir: Output directory for composited frames
        n_frames: Number of frames to composite

    Returns:
        List of paths to composited PNG files
    """
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    output_files = []

    for i in range(n_frames):
        blender_path = blender_dir / f"frame_{i:06d}.png"
        mpl_path = mpl_dir / f"mpl_{i:06d}.png"
        output_path = output_dir / f"composite_{i:06d}.png"

        if not blender_path.exists():
            raise FileNotFoundError(f"Blender frame not found: {blender_path}")
        if not mpl_path.exists():
            raise FileNotFoundError(f"Matplotlib frame not found: {mpl_path}")

        # Load images
        mpl_img = Image.open(mpl_path).convert('RGBA')
        blender_img = Image.open(blender_path).convert('RGBA')

        # Resize if needed (Blender and matplotlib might have slight size differences)
        if mpl_img.size != blender_img.size:
            blender_img = blender_img.resize(mpl_img.size, Image.Resampling.LANCZOS)

        # Composite: matplotlib background, Blender on top
        composite = Image.alpha_composite(mpl_img, blender_img)

        # Save
        composite.save(output_path)
        output_files.append(str(output_path))

        if i % 100 == 0:
            print(f"  Compositing: frame {i}/{n_frames}")

    return output_files


def _composite_single_frame(args):
    """
    Worker for single frame compositing.

    Args:
        args: Tuple of (frame_idx, blender_dir, mpl_dir, output_dir)

    Returns:
        Path to composited PNG file
    """
    i, blender_dir, mpl_dir, output_dir = args
    blender_path = Path(blender_dir) / f"frame_{i:06d}.png"
    mpl_path = Path(mpl_dir) / f"mpl_{i:06d}.png"
    output_path = Path(output_dir) / f"composite_{i:06d}.png"

    if not blender_path.exists():
        raise FileNotFoundError(f"Blender frame not found: {blender_path}")
    if not mpl_path.exists():
        raise FileNotFoundError(f"Matplotlib frame not found: {mpl_path}")

    mpl_img = Image.open(mpl_path).convert('RGBA')
    blender_img = Image.open(blender_path).convert('RGBA')

    if mpl_img.size != blender_img.size:
        blender_img = blender_img.resize(mpl_img.size, Image.Resampling.LANCZOS)

    composite = Image.alpha_composite(mpl_img, blender_img)
    composite.save(output_path)
    return str(output_path)


def composite_frames_parallel(
    blender_dir: Path,
    mpl_dir: Path,
    output_dir: Path,
    n_frames: int,
    n_workers: Optional[int] = None
) -> List[str]:
    """
    Composite Blender and matplotlib frames in parallel.

    Uses ThreadPoolExecutor since this is I/O-bound work.

    Args:
        blender_dir: Directory containing Blender frames (frame_NNNNNN.png)
        mpl_dir: Directory containing matplotlib frames (mpl_NNNNNN.png)
        output_dir: Output directory for composited frames
        n_frames: Number of frames to composite
        n_workers: Number of parallel workers (None = auto, capped at 8)

    Returns:
        List of paths to composited PNG files
    """
    if n_workers is None or n_workers <= 0:
        n_workers = min(os.cpu_count() or 4, 8)  # Cap at 8 for I/O

    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    work = [
        (i, str(blender_dir), str(mpl_dir), str(output_dir))
        for i in range(n_frames)
    ]

    if TQDM_AVAILABLE:
        results = thread_map(
            _composite_single_frame, work,
            max_workers=n_workers,
            desc="Compositing  ", ncols=60
        )
    else:
        print(f"  Compositing: {n_frames} frames with {n_workers} workers...")
        with ThreadPoolExecutor(max_workers=n_workers) as executor:
            results = list(executor.map(_composite_single_frame, work))
        print(f"  Compositing: done ({n_frames} frames)")

    return results


def assemble_video(
    frame_dir: Path,
    output_file: str,
    framerate: int = 30,
    frame_pattern: str = "composite_%06d.png"
):
    """
    Assemble PNG frames into video using ffmpeg.

    Args:
        frame_dir: Directory containing frames
        output_file: Output video file path
        framerate: Video framerate
        frame_pattern: Printf-style frame filename pattern
    """
    frame_dir = Path(frame_dir)

    cmd = [
        'ffmpeg', '-y',
        '-framerate', str(framerate),
        '-i', str(frame_dir / frame_pattern),
        '-c:v', 'libx264',
        '-pix_fmt', 'yuv420p',
        '-crf', '18',
        output_file
    ]

    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        raise RuntimeError(f"ffmpeg failed: {result.stderr}")


def extract_frame_data(states, wing_vectors):
    """
    Extract frame data to a JSON-serializable format for Blender.

    Args:
        states: List of state arrays
        wing_vectors: List of wing vector dicts

    Returns:
        dict: Frame data suitable for JSON serialization
    """
    wing_names = list(wing_vectors[0].keys())
    n_frames = len(states)

    # Convert states to list of lists
    states_list = [s.tolist() if hasattr(s, 'tolist') else list(s) for s in states]

    # Extract wing vectors
    wing_data = {}
    for wname in wing_names:
        wing_data[wname] = {
            'e_r': [wing_vectors[i][wname]['e_r'].tolist()
                    if hasattr(wing_vectors[i][wname]['e_r'], 'tolist')
                    else list(wing_vectors[i][wname]['e_r'])
                    for i in range(n_frames)],
            'e_c': [wing_vectors[i][wname]['e_c'].tolist()
                    if hasattr(wing_vectors[i][wname]['e_c'], 'tolist')
                    else list(wing_vectors[i][wname]['e_c'])
                    for i in range(n_frames)],
        }

    return {
        'n_frames': n_frames,
        'states': states_list,
        'wing_names': wing_names,
        'wing_vectors': wing_data,
    }


def run_blender_render(
    states: List,
    wing_vectors: List,
    output_dir: Path,
    config_file: str,
    data_file: str,
    start_frame: int = 0,
    end_frame: int = -1
):
    """
    Run Blender to render dragonfly frames.

    Args:
        states: List of state arrays
        wing_vectors: List of wing vector dicts
        output_dir: Output directory for frames
        config_file: Configuration JSON file
        data_file: Path to write frame data JSON
        start_frame: Start frame index
        end_frame: End frame index (-1 for all)
    """
    blender_path = find_blender()
    if blender_path is None:
        raise RuntimeError("Blender not found")

    # Extract and save frame data
    frame_data = extract_frame_data(states, wing_vectors)
    with open(data_file, 'w') as f:
        json.dump(frame_data, f)

    script_path = Path(__file__).parent / "blender" / "render_dragonfly.py"

    cmd = [
        blender_path, '--background',
        '--python', str(script_path),
        '--',
        '--data', data_file,
        '--output-dir', str(output_dir),
        '--config', config_file,
        '--start', str(start_frame),
        '--end', str(end_frame),
    ]

    print(f"Running Blender: {' '.join(cmd)}")
    result = subprocess.run(cmd, capture_output=True, text=True)

    if result.returncode != 0:
        print(f"Blender stdout: {result.stdout}")
        print(f"Blender stderr: {result.stderr}")
        raise RuntimeError(f"Blender rendering failed with code {result.returncode}")


def _blender_worker(args):
    """
    Worker function for parallel Blender rendering.

    Each worker renders a chunk of frames.

    Args:
        args: Tuple of (blender_path, script_path, data_file, output_dir, config_file, start_frame, end_frame)

    Returns:
        Tuple of (start_frame, end_frame) on success
    """
    blender_path, script_path, data_file, output_dir, config_file, start_frame, end_frame = args

    cmd = [
        blender_path, '--background',
        '--python', str(script_path),
        '--',
        '--data', data_file,
        '--output-dir', str(output_dir),
        '--config', config_file,
        '--start', str(start_frame),
        '--end', str(end_frame),
    ]

    result = subprocess.run(cmd, capture_output=True, text=True)

    if result.returncode != 0:
        raise RuntimeError(
            f"Blender worker failed (frames {start_frame}-{end_frame}): {result.stderr}"
        )

    # Blender may exit 0 despite script errors — verify frames were produced
    expected = Path(output_dir) / f"frame_{start_frame:06d}.png"
    if not expected.exists():
        raise RuntimeError(
            f"Blender worker produced no frames (frames {start_frame}-{end_frame}).\n"
            f"stderr: {result.stderr[-500:]}"
        )

    return (start_frame, end_frame)


def run_blender_render_parallel(
    states: List,
    wing_vectors: List,
    output_dir: Path,
    config_file: str,
    data_file: str,
    n_frames: int,
    n_workers: Optional[int] = None
):
    """
    Run Blender to render dragonfly frames in parallel.

    Spawns multiple Blender processes, each rendering a chunk of frames.

    Args:
        states: List of state arrays
        wing_vectors: List of wing vector dicts
        output_dir: Output directory for frames
        config_file: Configuration JSON file
        data_file: Path to write frame data JSON
        n_frames: Total number of frames
        n_workers: Number of parallel workers (None = auto)
    """
    blender_path = find_blender()
    if blender_path is None:
        raise RuntimeError("Blender not found")

    if n_workers is None or n_workers <= 0:
        n_workers = os.cpu_count() or 4

    # Extract and save frame data (shared by all workers)
    frame_data = extract_frame_data(states, wing_vectors)
    with open(data_file, 'w') as f:
        json.dump(frame_data, f)

    script_path = Path(__file__).parent / "blender" / "render_dragonfly.py"

    # Split frame range into chunks
    chunk_size = math.ceil(n_frames / n_workers)
    ranges = [
        (i * chunk_size, min((i + 1) * chunk_size, n_frames))
        for i in range(n_workers)
    ]
    # Filter empty ranges
    ranges = [(s, e) for s, e in ranges if s < e]

    # Prepare work items
    work = [
        (blender_path, str(script_path), data_file, str(output_dir), config_file, start, end)
        for start, end in ranges
    ]

    output_dir = Path(output_dir)

    with ProcessPoolExecutor(max_workers=len(ranges)) as executor:
        futures = [executor.submit(_blender_worker, w) for w in work]

        # Poll output directory for rendered frames instead of waiting per-batch
        if TQDM_AVAILABLE:
            pbar = tqdm(total=n_frames, desc="Blender      ", ncols=60)
            while not all(f.done() for f in futures):
                rendered = len(list(output_dir.glob("frame_*.png")))
                pbar.n = rendered
                pbar.refresh()
                time.sleep(0.5)
            # Check for worker exceptions before finalizing progress bar
            for future in futures:
                future.result()
            pbar.n = n_frames
            pbar.refresh()
            pbar.close()
        else:
            last_printed = 0
            while not all(f.done() for f in futures):
                rendered = len(list(output_dir.glob("frame_*.png")))
                if rendered >= last_printed + 50:
                    print(f"  Blender: {rendered}/{n_frames} frames")
                    last_printed = rendered
                time.sleep(0.5)
            # Check for worker exceptions before reporting completion
            for future in futures:
                future.result()
            print(f"  Blender: done ({n_frames} frames)")


def check_blender_available() -> bool:
    """Check if Blender is available on the system."""
    return find_blender() is not None


def render_hybrid_simulation(
    states: List[np.ndarray],
    wing_vectors: List[Dict],
    params: Dict,
    input_file: str,
    output_file: str,
    config: Optional[HybridConfig] = None
):
    """
    Render simulation using hybrid Blender + matplotlib pipeline.

    Blender renders the 3D dragonfly mesh at full resolution with camera
    following the body position. Matplotlib renders axes, trails, and forces.
    Simple alpha compositing combines them.

    Args:
        states: List of state arrays
        wing_vectors: List of wing vector dicts
        params: Simulation parameters
        input_file: Path to input HDF5 file (for Blender)
        output_file: Output video file path
        config: Optional HybridConfig (uses defaults if None)
    """
    if not check_blender_available():
        raise RuntimeError(
            "Blender is not available. Install Blender or use --renderer pyvista"
        )

    # Setup configuration
    if config is None:
        config = HybridConfig()

    # Compute viewport if not set
    if config.viewport is None:
        config.viewport = compute_viewport(states)

    # Compute exact ortho_scale and center offset from matplotlib projection
    ortho_scale, (offset_x, offset_y) = compute_blender_ortho_scale(
        config.camera, config.viewport
    )
    config.blender.computed_ortho_scale = ortho_scale
    config.blender.center_offset_x = offset_x
    config.blender.center_offset_y = offset_y

    n_frames = len(states)
    n_workers = config.n_workers if config.n_workers > 0 else (os.cpu_count() or 4)

    # Create temp directory for all intermediate files
    with tempfile.TemporaryDirectory() as tmpdir:
        tmpdir = Path(tmpdir)
        blender_dir = tmpdir / "blender"
        mpl_dir = tmpdir / "mpl"
        composite_dir = tmpdir / "composite"
        config_file = tmpdir / "config.json"
        data_file = tmpdir / "frame_data.json"

        # Create output directories upfront
        blender_dir.mkdir(parents=True, exist_ok=True)
        mpl_dir.mkdir(parents=True, exist_ok=True)

        # Save config for Blender
        config.save(str(config_file))

        # Stage 1: Matplotlib overlays
        print(f"Rendering matplotlib frames ({n_workers} workers)...")
        render_all_simulation_frames_parallel(
            states, wing_vectors, params, config, mpl_dir, n_workers
        )

        # Stage 2: Blender mesh renders
        print(f"Rendering Blender frames ({n_workers} workers)...")
        run_blender_render_parallel(
            states, wing_vectors, blender_dir, str(config_file),
            str(data_file), n_frames, n_workers
        )

        # Stage 3: Composite frames
        print("Compositing frames...")
        composite_frames_parallel(
            blender_dir, mpl_dir, composite_dir, n_frames, n_workers
        )

        # Stage 4: Assemble video
        print("Assembling video...")
        assemble_video(composite_dir, output_file, config.framerate)

    print(f"Done: {output_file}")


def render_hybrid_tracking(
    states: List[np.ndarray],
    wing_vectors: List[Dict],
    params: Dict,
    controller: Dict,
    input_file: str,
    output_file: str,
    config: Optional[HybridConfig] = None
):
    """
    Render tracking simulation using hybrid Blender + matplotlib pipeline.

    Blender renders the 3D dragonfly mesh at full resolution with camera
    following the body position. Matplotlib renders axes, trails, targets,
    and forces. Simple alpha compositing combines them.

    Args:
        states: List of state arrays
        wing_vectors: List of wing vector dicts
        params: Simulation parameters
        controller: Controller data dict
        input_file: Path to input HDF5 file (for Blender)
        output_file: Output video file path
        config: Optional HybridConfig (uses defaults if None)
    """
    if not check_blender_available():
        raise RuntimeError(
            "Blender is not available. Install Blender or use --renderer pyvista"
        )

    # Setup configuration
    if config is None:
        config = HybridConfig()

    # Compute viewport if not set (include targets)
    if config.viewport is None:
        config.viewport = compute_viewport(
            states,
            targets=controller['target_position']
        )

    # Compute exact ortho_scale and center offset from matplotlib projection
    ortho_scale, (offset_x, offset_y) = compute_blender_ortho_scale(
        config.camera, config.viewport
    )
    config.blender.computed_ortho_scale = ortho_scale
    config.blender.center_offset_x = offset_x
    config.blender.center_offset_y = offset_y

    n_frames = len(states)
    n_workers = config.n_workers if config.n_workers > 0 else (os.cpu_count() or 4)

    # Create temp directory for all intermediate files
    with tempfile.TemporaryDirectory() as tmpdir:
        tmpdir = Path(tmpdir)
        blender_dir = tmpdir / "blender"
        mpl_dir = tmpdir / "mpl"
        composite_dir = tmpdir / "composite"
        config_file = tmpdir / "config.json"
        data_file = tmpdir / "frame_data.json"

        # Create output directories upfront
        blender_dir.mkdir(parents=True, exist_ok=True)
        mpl_dir.mkdir(parents=True, exist_ok=True)

        # Save config for Blender
        config.save(str(config_file))

        # Stage 1: Matplotlib overlays
        print(f"Rendering matplotlib frames ({n_workers} workers)...")
        render_all_tracking_frames_parallel(
            states, wing_vectors, params, controller, config, mpl_dir, n_workers
        )

        # Stage 2: Blender mesh renders
        print(f"Rendering Blender frames ({n_workers} workers)...")
        run_blender_render_parallel(
            states, wing_vectors, blender_dir, str(config_file),
            str(data_file), n_frames, n_workers
        )

        # Stage 3: Composite frames
        print("Compositing frames...")
        composite_frames_parallel(
            blender_dir, mpl_dir, composite_dir, n_frames, n_workers
        )

        # Stage 4: Assemble video
        print("Assembling video...")
        assemble_video(composite_dir, output_file, config.framerate)

    print(f"Done: {output_file}")


def render_mpl_only_simulation(
    states: List[np.ndarray],
    wing_vectors: List[Dict],
    params: Dict,
    output_file: str,
    config: Optional[HybridConfig] = None
):
    """
    Render simulation using matplotlib only (fallback when Blender unavailable).

    This renders the axes/trail/forces but without the 3D mesh.

    Args:
        states: List of state arrays
        wing_vectors: List of wing vector dicts
        params: Simulation parameters
        output_file: Output video file path
        config: Optional HybridConfig
    """
    if config is None:
        config = HybridConfig()

    if config.viewport is None:
        config.viewport = compute_viewport(states)

    n_frames = len(states)

    with tempfile.TemporaryDirectory() as tmpdir:
        tmpdir = Path(tmpdir)

        print("Rendering matplotlib frames...")
        render_all_simulation_frames(
            states, wing_vectors, params, config, tmpdir
        )

        # Rename files to match expected pattern for ffmpeg
        for i in range(n_frames):
            src = tmpdir / f"mpl_{i:06d}.png"
            dst = tmpdir / f"composite_{i:06d}.png"
            shutil.move(str(src), str(dst))

        print("Assembling video...")
        assemble_video(tmpdir, output_file, config.framerate)

    print(f"Done: {output_file}")


def render_mpl_only_tracking(
    states: List[np.ndarray],
    wing_vectors: List[Dict],
    params: Dict,
    controller: Dict,
    output_file: str,
    config: Optional[HybridConfig] = None
):
    """
    Render tracking using matplotlib only (fallback when Blender unavailable).

    Args:
        states: List of state arrays
        wing_vectors: List of wing vector dicts
        params: Simulation parameters
        controller: Controller data dict
        output_file: Output video file path
        config: Optional HybridConfig
    """
    if config is None:
        config = HybridConfig()

    if config.viewport is None:
        config.viewport = compute_viewport(
            states,
            targets=controller['target_position']
        )

    n_frames = len(states)

    with tempfile.TemporaryDirectory() as tmpdir:
        tmpdir = Path(tmpdir)

        print("Rendering matplotlib frames...")
        render_all_tracking_frames(
            states, wing_vectors, params, controller, config, tmpdir
        )

        # Rename files
        for i in range(n_frames):
            src = tmpdir / f"mpl_{i:06d}.png"
            dst = tmpdir / f"composite_{i:06d}.png"
            shutil.move(str(src), str(dst))

        print("Assembling video...")
        assemble_video(tmpdir, output_file, config.framerate)

    print(f"Done: {output_file}")

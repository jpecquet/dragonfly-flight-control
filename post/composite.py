"""
PNG compositing and video assembly for hybrid visualization.

Combines Blender-rendered mesh frames with matplotlib overlays,
then assembles into final video.
"""

import json
import hashlib
import math
import os
import platform
import shutil
import subprocess
import tempfile
import time
from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor, as_completed
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
from PIL import Image

from .hybrid_config import HybridConfig, BlenderRenderConfig, compute_viewport
from .mpl_overlay import (
    compute_blender_ortho_scale,
    render_all_frames_parallel,
)
from .annotation_overlay import render_annotation_frames, build_wingtip_paths_bottom_payload
from .progress import pip_progress

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

# Keep one recent Blender frame set alive long enough to reuse across
# back-to-back light/dark renders in the same process.
_LIGHT_BODY_HEX = "#111111"
_LIGHT_WING_HEX = "#d3d3d3"
_DARK_BODY_HEX = "#f2f5f7"
_DARK_WING_HEX = "#8f9aa6"
MAX_PARALLEL_WORKERS = 4


@dataclass
class _BlenderFrameCacheEntry:
    key: str
    n_frames: int
    frame_dir: Path
    tmpdir_handle: object


_BLENDER_FRAME_CACHE: Optional[_BlenderFrameCacheEntry] = None


def _clear_blender_frame_cache() -> None:
    global _BLENDER_FRAME_CACHE
    if _BLENDER_FRAME_CACHE is None:
        return
    try:
        cleanup = getattr(_BLENDER_FRAME_CACHE.tmpdir_handle, "cleanup", None)
        if cleanup is not None:
            cleanup()
    finally:
        _BLENDER_FRAME_CACHE = None


def _set_blender_frame_cache(key: str, n_frames: int, frame_dir: Path, tmpdir_handle: object) -> None:
    global _BLENDER_FRAME_CACHE
    old = _BLENDER_FRAME_CACHE
    _BLENDER_FRAME_CACHE = _BlenderFrameCacheEntry(
        key=key,
        n_frames=int(n_frames),
        frame_dir=Path(frame_dir),
        tmpdir_handle=tmpdir_handle,
    )
    if old is not None and old.tmpdir_handle is not tmpdir_handle:
        cleanup = getattr(old.tmpdir_handle, "cleanup", None)
        if cleanup is not None:
            cleanup()


def _get_cached_blender_frame_dir(key: str, n_frames: int) -> Optional[Path]:
    entry = _BLENDER_FRAME_CACHE
    if entry is None or entry.key != key or entry.n_frames != int(n_frames):
        return None
    if n_frames <= 0:
        return entry.frame_dir
    first = entry.frame_dir / "frame_000000.png"
    last = entry.frame_dir / f"frame_{int(n_frames) - 1:06d}.png"
    if not first.exists() or not last.exists():
        _clear_blender_frame_cache()
        return None
    return entry.frame_dir


def _normalized_blender_material_signature(config: HybridConfig) -> Dict[str, Optional[str]]:
    style_cfg = config.style.to_dict() if getattr(config, "style", None) is not None else {}
    theme = str(style_cfg.get("theme", "")).strip().lower()

    def _norm(key: str) -> Optional[str]:
        value = style_cfg.get(key)
        if value is None:
            return None
        normalized = str(value).strip().lower()
        if theme == "light":
            if key == "body_color" and normalized == _LIGHT_BODY_HEX:
                return _DARK_BODY_HEX
            if key == "wing_color" and normalized == _LIGHT_WING_HEX:
                return _DARK_WING_HEX
        return normalized

    return {
        "body_color": _norm("body_color"),
        "wing_color": _norm("wing_color"),
    }


def _path_fingerprint(path_like: str) -> Dict[str, object]:
    path = Path(path_like)
    try:
        st = path.stat()
        return {
            "path": str(path.resolve()),
            "size": int(st.st_size),
            "mtime_ns": int(st.st_mtime_ns),
        }
    except OSError:
        return {"path": str(path)}


def _blender_frame_reuse_key(
    *,
    input_file: str,
    n_frames: int,
    frame_step: int,
    time_values: np.ndarray,
    config: HybridConfig,
) -> str:
    time_arr = np.asarray(time_values, dtype=float).reshape(-1)
    if time_arr.size > 0:
        time_sig = {
            "n": int(time_arr.size),
            "start": float(time_arr[0]),
            "end": float(time_arr[-1]),
        }
    else:
        time_sig = {"n": 0}

    key_payload = {
        "source": _path_fingerprint(input_file),
        "n_frames": int(n_frames),
        "frame_step": int(frame_step),
        "time": time_sig,
        "camera": config.camera.to_dict() if config.camera is not None else None,
        "viewport": config.viewport.to_dict() if config.viewport is not None else None,
        "blender": config.blender.to_dict() if config.blender is not None else None,
        "materials": _normalized_blender_material_signature(config),
    }
    encoded = json.dumps(key_payload, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


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
        n_workers: Number of parallel workers (None = auto, capped at 4)

    Returns:
        List of paths to composited PNG files
    """
    if n_workers is None or n_workers <= 0:
        n_workers = os.cpu_count() or MAX_PARALLEL_WORKERS
    n_workers = max(1, min(int(n_workers), MAX_PARALLEL_WORKERS))

    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    work = [
        (i, str(blender_dir), str(mpl_dir), str(output_dir))
        for i in range(n_frames)
    ]

    results: List[Optional[str]] = [None] * len(work)
    with ThreadPoolExecutor(max_workers=n_workers) as executor:
        futures = {
            executor.submit(_composite_single_frame, args): idx
            for idx, args in enumerate(work)
        }
        with pip_progress(n_frames, "Compositing", unit="frame") as progress:
            for future in as_completed(futures):
                idx = futures[future]
                results[idx] = future.result()
                progress.update(1)

    return [r for r in results if r is not None]


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


def assemble_video_from_layers(
    mpl_dir: Path,
    blender_dir: Path,
    output_file: str,
    framerate: int = 30,
    mpl_pattern: str = "mpl_%06d.png",
    blender_pattern: str = "frame_%06d.png",
    annotation_dir: Optional[Path] = None,
    annotation_pattern: str = "ann_%06d.png",
):
    """
    Assemble hybrid video directly from matplotlib and Blender PNG sequences.

    Uses ffmpeg overlay filter so we avoid writing intermediate composited PNGs.

    Args:
        mpl_dir: Directory containing matplotlib frames
        blender_dir: Directory containing Blender frames
        output_file: Output video file path
        framerate: Video framerate
        mpl_pattern: Printf-style matplotlib frame pattern
        blender_pattern: Printf-style Blender frame pattern
        annotation_dir: Optional directory containing transparent annotation overlays
        annotation_pattern: Printf-style annotation frame pattern
    """
    mpl_dir = Path(mpl_dir)
    blender_dir = Path(blender_dir)

    cmd = [
        'ffmpeg', '-y',
        '-framerate', str(framerate),
        '-i', str(mpl_dir / mpl_pattern),
        '-framerate', str(framerate),
        '-i', str(blender_dir / blender_pattern),
    ]

    if annotation_dir is not None:
        annotation_dir = Path(annotation_dir)
        cmd.extend([
            '-framerate', str(framerate),
            '-i', str(annotation_dir / annotation_pattern),
        ])
        filter_complex = (
            '[1:v][0:v]scale2ref[fg][bg];'
            '[bg][fg]overlay=shortest=1:format=auto[tmp];'
            '[tmp][2:v]overlay=shortest=1:format=auto[v]'
        )
    else:
        filter_complex = '[1:v][0:v]scale2ref[fg][bg];[bg][fg]overlay=shortest=1:format=auto[v]'

    cmd.extend([
        '-filter_complex', filter_complex,
        '-map', '[v]',
        '-c:v', 'libx264',
        '-pix_fmt', 'yuv420p',
        '-crf', '18',
        output_file,
    ])

    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        raise RuntimeError(f"ffmpeg overlay assembly failed: {result.stderr}")


def _first_harmonic_coeff(series) -> float:
    """Return first harmonic coefficient from a per-wing harmonic series."""
    if series is None:
        return 0.0
    arr = np.asarray(series, dtype=float).reshape(-1)
    if arr.size < 1:
        return 0.0
    return float(arr[0])


def _extract_wing_twist_h1(time_values, wing_names, params) -> Dict[str, Dict]:
    """
    Build per-wing pitch-twist metadata for Blender deformation.

    Uses the same first-harmonic spanwise scaling model as the simulator.
    """
    if params is None:
        return {}

    has_twist = params.get("wing_has_psi_twist_h1", {})
    twist_root = params.get("wing_psi_twist_h1_root", {})
    twist_ref_eta = params.get("wing_psi_twist_ref_eta", {})
    wing_phase = params.get("wing_phase_offset", {})
    wing_omega = params.get("wing_omega", {})
    wing_period = params.get("wing_harmonic_period_wingbeats", {})
    psi_amp = params.get("wing_psi_amp", {})
    psi_phase = params.get("wing_psi_phase", {})

    twist_payload = {}
    for wname in wing_names:
        if not bool(has_twist.get(wname, 0)):
            continue

        ref_eta = float(twist_ref_eta.get(wname, 0.0))
        root_coeff = float(twist_root.get(wname, 0.0))
        omega = float(wing_omega.get(wname, 0.0))
        period = float(wing_period.get(wname, 0.0))
        phase_offset = float(wing_phase.get(wname, 0.0))
        if ref_eta <= 0.0 or abs(period) <= 1e-12:
            continue

        amp1 = _first_harmonic_coeff(psi_amp.get(wname))
        phase1 = _first_harmonic_coeff(psi_phase.get(wname))
        ref_coeff = abs(amp1)
        if ref_coeff <= 1e-12:
            continue

        basis_omega = omega / period
        psi_h1 = amp1 * np.cos(basis_omega * time_values + phase_offset + phase1)
        twist_payload[wname] = {
            "ref_eta": ref_eta,
            "root_coeff": root_coeff,
            "ref_coeff": ref_coeff,
            "psi_h1": psi_h1.tolist(),
        }

    return twist_payload


def extract_frame_data(states, wing_vectors, params=None, time_values=None):
    """
    Extract frame data to a JSON-serializable format for Blender.

    Args:
        states: State array (N, 6)
        wing_vectors: Dict-of-arrays keyed by wing name
        params: Optional simulation parameters dict (used for wing geometry metadata)
        time_values: Optional time array matching frame count

    Returns:
        dict: Frame data suitable for JSON serialization
    """
    wing_names = list(wing_vectors.keys())
    n_frames = len(states)

    states_list = states.tolist()
    if time_values is None:
        time_arr = np.arange(n_frames, dtype=float)
    else:
        time_arr = np.asarray(time_values, dtype=float).reshape(-1)
        if time_arr.size != n_frames:
            raise ValueError(
                f"time_values length ({time_arr.size}) must match frame count ({n_frames})"
            )

    wing_data = {}
    for wname in wing_names:
        wing_data[wname] = {
            'e_r': wing_vectors[wname]['e_r'].tolist(),
            'e_c': wing_vectors[wname]['e_c'].tolist(),
        }

    payload = {
        'n_frames': n_frames,
        'time': time_arr.tolist(),
        'states': states_list,
        'wing_names': wing_names,
        'wing_vectors': wing_data,
    }
    if params is not None:
        payload['wing_lb0'] = {
            str(name): float(value) for name, value in params.get('wing_lb0', {}).items()
        }
        payload['wing_gamma_mean'] = {
            str(name): float(value) for name, value in params.get('wing_gamma_mean', {}).items()
        }
        payload['wing_cone_angle'] = {
            str(name): float(value) for name, value in params.get('wing_cone_angle', {}).items()
        }
        payload['wing_twist_h1'] = _extract_wing_twist_h1(time_arr, wing_names, params)
    return payload


def run_blender_render(
    states: List,
    wing_vectors: List,
    output_dir: Path,
    config_file: str,
    data_file: str,
    start_frame: int = 0,
    end_frame: int = -1,
    params: Optional[Dict] = None,
    time_values: Optional[np.ndarray] = None,
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
    frame_data = extract_frame_data(states, wing_vectors, params=params, time_values=time_values)
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
    n_workers: Optional[int] = None,
    params: Optional[Dict] = None,
    time_values: Optional[np.ndarray] = None,
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
        n_workers = os.cpu_count() or MAX_PARALLEL_WORKERS
    n_workers = max(1, min(int(n_workers), MAX_PARALLEL_WORKERS))

    # Extract and save frame data (shared by all workers)
    frame_data = extract_frame_data(states, wing_vectors, params=params, time_values=time_values)
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
    if not ranges:
        with pip_progress(0, "Blender", unit="frame", min_interval=0.2):
            pass
        return

    # Prepare work items
    work = [
        (blender_path, str(script_path), data_file, str(output_dir), config_file, start, end)
        for start, end in ranges
    ]

    output_dir = Path(output_dir)

    with ProcessPoolExecutor(max_workers=len(ranges)) as executor:
        futures = [executor.submit(_blender_worker, w) for w in work]

        # Poll output directory for rendered frames instead of waiting per-batch
        with pip_progress(n_frames, "Blender", unit="frame", min_interval=0.2) as progress:
            while not all(f.done() for f in futures):
                rendered = len(list(output_dir.glob("frame_*.png")))
                progress.set(rendered)
                time.sleep(0.5)
            # Check for worker exceptions before finalizing progress bar
            for future in futures:
                future.result()
            progress.set(n_frames)


def check_blender_available() -> bool:
    """Check if Blender is available on the system."""
    return find_blender() is not None


def _subsample_animation_inputs(
    states: np.ndarray,
    wing_vectors: Dict,
    controller: Optional[Dict],
    frame_step: int,
) -> Tuple[np.ndarray, Dict, Optional[Dict]]:
    """Subsample animation inputs by stride."""
    if frame_step <= 1:
        return states, wing_vectors, controller

    states_sub = states[::frame_step]
    wings_sub = {
        wname: {k: v[::frame_step] for k, v in wdata.items()}
        for wname, wdata in wing_vectors.items()
    }

    controller_sub = None
    if controller is not None:
        controller_sub = {}
        for key, value in controller.items():
            try:
                controller_sub[key] = value[::frame_step]
            except Exception:
                controller_sub[key] = value

    return states_sub, wings_sub, controller_sub


def render_hybrid(
    states: List[np.ndarray],
    wing_vectors: List[Dict],
    params: Dict,
    input_file: str,
    output_file: str,
    time: Optional[np.ndarray] = None,
    controller: Optional[Dict] = None,
    config: Optional[HybridConfig] = None,
    frame_step: int = 1,
    annotation_overlay: Optional[Dict] = None,
):
    """
    Render using hybrid Blender + matplotlib pipeline.

    Blender renders the 3D dragonfly mesh at full resolution with camera
    following the body position. Matplotlib renders axes, trails, and forces
    (plus targets/errors in tracking mode).

    Args:
        states: List of state arrays
        wing_vectors: List of wing vector dicts
        params: Simulation parameters
        input_file: Path to input HDF5 file (for Blender)
        output_file: Output video file path
        time: Optional simulation time array matching states
        controller: Controller data dict (None for simulation mode)
        config: Optional HybridConfig (uses defaults if None)
        frame_step: Render every Nth frame (N>=1)
    """
    if frame_step < 1:
        raise ValueError(f"frame_step must be >= 1, got {frame_step}")

    if not check_blender_available():
        raise RuntimeError(
            "Blender is not available. Install Blender or use matplotlib-only fallback"
        )

    if config is None:
        config = HybridConfig()

    if time is None:
        time_values = np.arange(len(states), dtype=float)
    else:
        time_values = np.asarray(time, dtype=float).reshape(-1)
        if time_values.size != len(states):
            raise ValueError(
                f"time length ({time_values.size}) must match states length ({len(states)})"
            )

    original_n = len(states)
    states, wing_vectors, controller = _subsample_animation_inputs(
        states, wing_vectors, controller, frame_step
    )
    if frame_step > 1:
        time_values = time_values[::frame_step]
    if frame_step > 1:
        print(f"Frame skipping enabled: step={frame_step} ({original_n} -> {len(states)} frames)")

    if config.viewport is None:
        targets = controller['target_position'] if controller else None
        config.viewport = compute_viewport(states, targets=targets)

    # Compute exact ortho_scale and center offset from matplotlib projection
    ortho_scale, (offset_x, offset_y) = compute_blender_ortho_scale(
        config.camera, config.viewport, config.style, show_axes=bool(config.show_axes)
    )
    config.blender.computed_ortho_scale = ortho_scale
    config.blender.center_offset_x = offset_x
    config.blender.center_offset_y = offset_y

    n_frames = len(states)
    n_workers = config.n_workers if config.n_workers > 0 else (os.cpu_count() or MAX_PARALLEL_WORKERS)
    n_workers = max(1, min(int(n_workers), MAX_PARALLEL_WORKERS))
    blender_cache_key = _blender_frame_reuse_key(
        input_file=input_file,
        n_frames=n_frames,
        frame_step=frame_step,
        time_values=time_values,
        config=config,
    )
    cached_blender_dir = _get_cached_blender_frame_dir(blender_cache_key, n_frames)

    with tempfile.TemporaryDirectory() as tmpdir:
        tmpdir = Path(tmpdir)
        mpl_dir = tmpdir / "mpl"
        ann_dir = tmpdir / "ann"
        config_file = tmpdir / "config.json"
        data_file = tmpdir / "frame_data.json"

        mpl_dir.mkdir(parents=True, exist_ok=True)
        if annotation_overlay is not None:
            ann_dir.mkdir(parents=True, exist_ok=True)
        config.save(str(config_file))

        # Build bottom-layer wingtip path payload if applicable.
        bottom_overlay_payload = None
        if (
            annotation_overlay is not None
            and isinstance(annotation_overlay, dict)
            and str(annotation_overlay.get("kind", "")).strip() == "wingtip_paths"
            and annotation_overlay.get("bottom_layer_wings")
        ):
            bottom_overlay_payload = build_wingtip_paths_bottom_payload(
                wing_vectors, params, config, annotation_overlay,
                time_values=time_values,
            )

        print(f"Rendering matplotlib frames ({n_workers} workers)...")
        render_all_frames_parallel(
            states, wing_vectors, params, config, mpl_dir,
            controller=controller, n_workers=n_workers,
            bottom_overlay_payload=bottom_overlay_payload,
        )

        annotation_dir_for_assembly = None
        if annotation_overlay is not None:
            print(f"Rendering annotation overlay ({n_workers} workers)...")
            render_annotation_frames(
                states, wing_vectors, params, config, ann_dir, annotation_overlay,
                time_values=time_values,
                n_workers=n_workers,
            )
            annotation_dir_for_assembly = ann_dir

        blender_dir_for_assembly = cached_blender_dir
        if blender_dir_for_assembly is not None:
            print(f"Reusing cached Blender frames ({n_frames} frames)...")
        else:
            cache_tmpdir = tempfile.TemporaryDirectory(prefix="hybrid_blender_frames_")
            cache_blender_dir = Path(cache_tmpdir.name) / "blender"
            cache_blender_dir.mkdir(parents=True, exist_ok=True)
            try:
                print(f"Rendering Blender frames ({n_workers} workers)...")
                run_blender_render_parallel(
                    states, wing_vectors, cache_blender_dir, str(config_file),
                    str(data_file), n_frames, n_workers, params=params, time_values=time_values
                )
            except Exception:
                cache_tmpdir.cleanup()
                raise
            _set_blender_frame_cache(
                blender_cache_key, n_frames, cache_blender_dir, cache_tmpdir
            )
            blender_dir_for_assembly = cache_blender_dir

        print("Assembling video from Blender + matplotlib layers...")
        assemble_video_from_layers(
            mpl_dir,
            blender_dir_for_assembly,
            output_file,
            config.framerate,
            annotation_dir=annotation_dir_for_assembly,
        )

    print(f"Done: {output_file}")


def render_mpl_only(
    states: List[np.ndarray],
    wing_vectors: List[Dict],
    params: Dict,
    output_file: str,
    controller: Optional[Dict] = None,
    config: Optional[HybridConfig] = None,
    frame_step: int = 1,
):
    """
    Render using matplotlib only (fallback when Blender unavailable).

    Renders axes/trail/forces plus simplified body and wing models.

    Args:
        states: List of state arrays
        wing_vectors: List of wing vector dicts
        params: Simulation parameters
        output_file: Output video file path
        controller: Controller data dict (None for simulation mode)
        config: Optional HybridConfig
        frame_step: Render every Nth frame (N>=1)
    """
    if frame_step < 1:
        raise ValueError(f"frame_step must be >= 1, got {frame_step}")

    if config is None:
        config = HybridConfig()

    original_n = len(states)
    states, wing_vectors, controller = _subsample_animation_inputs(
        states, wing_vectors, controller, frame_step
    )
    if frame_step > 1:
        print(f"Frame skipping enabled: step={frame_step} ({original_n} -> {len(states)} frames)")

    if config.viewport is None:
        targets = controller['target_position'] if controller else None
        config.viewport = compute_viewport(states, targets=targets)

    n_frames = len(states)
    n_workers = config.n_workers if config.n_workers > 0 else (os.cpu_count() or MAX_PARALLEL_WORKERS)
    n_workers = max(1, min(int(n_workers), MAX_PARALLEL_WORKERS))

    with tempfile.TemporaryDirectory() as tmpdir:
        tmpdir = Path(tmpdir)

        print(f"Rendering matplotlib frames ({n_workers} workers)...")
        print("  Using simplified matplotlib body/wing models")
        render_all_frames_parallel(
            states, wing_vectors, params, config, tmpdir, controller=controller,
            draw_models=True, n_workers=n_workers,
        )

        # Rename files to match expected pattern for ffmpeg
        for i in range(n_frames):
            src = tmpdir / f"mpl_{i:06d}.png"
            dst = tmpdir / f"composite_{i:06d}.png"
            shutil.move(str(src), str(dst))

        print("Assembling video...")
        assemble_video(tmpdir, output_file, config.framerate)

    print(f"Done: {output_file}")

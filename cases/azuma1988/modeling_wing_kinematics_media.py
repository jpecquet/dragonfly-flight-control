#!/usr/bin/env python3
"""Regenerate modeling/wing_kinematics side-view animation media.

Produces one themed MP4 output per run (light or dark) using:
- Azuma 1988 experiment 1 tethered simulation (1 wingbeat)
- the shared `post.plot_simulation` pipeline
- a section-specific side-view config (+Y normal to view)

Usage:
  python cases/azuma1988/modeling_wing_kinematics_media.py --theme light
  python cases/azuma1988/modeling_wing_kinematics_media.py --theme dark

Preferred (centralized docs media workflow):
  python scripts/update_docs_media.py --only modeling_wing_kinematics
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from post.hybrid_config import CameraConfig, HybridConfig, StyleConfig, ViewportConfig


DEFAULT_RUN_DIR = REPO_ROOT / "runs" / "modeling" / "wing_kinematics" / "section1_anim1"
DEFAULT_OUTPUT_DIR = REPO_ROOT / "docs" / "_static" / "media" / "modeling" / "wing_kinematics"
SIDE_VIEW_X_LIM = (-1.0, 0.6)
SIDE_VIEW_Z_LIM = (-0.8, 0.8)


def run_cmd(cmd: list[str], *, cwd: Path | None = None, env: dict[str, str] | None = None) -> None:
    printable = " ".join(str(x) for x in cmd)
    print(f"[cmd] {printable}")
    cmd_env = os.environ.copy()
    if env:
        cmd_env.update(env)
    subprocess.run(cmd, cwd=str(cwd) if cwd else None, check=True, env=cmd_env)


def resolve_path(path_arg: str, *, base: Path) -> Path:
    path = Path(path_arg).expanduser()
    if not path.is_absolute():
        path = base / path
    return path.resolve()


def ensure_sim_output(run_dir: Path, dragonfly_binary: str | None, force_sim: bool) -> Path:
    output_h5 = run_dir / "sim" / "output.h5"
    if output_h5.exists() and not force_sim:
        print(f"[skip] simulation exists: {output_h5}")
        return output_h5

    cmd = [
        sys.executable,
        "cases/azuma1988/pipeline.py",
        "sim",
        "--experiment",
        "1",
        "--tether",
        "--n-wingbeats",
        "1",
        "--steps-per-wingbeat",
        "200",
        "--run-dir",
        str(run_dir),
    ]
    if dragonfly_binary:
        cmd.extend(["--binary", dragonfly_binary])
    run_cmd(cmd, cwd=REPO_ROOT)

    if not output_h5.exists():
        raise FileNotFoundError(f"Simulation output not found: {output_h5}")
    return output_h5


def write_render_config(run_dir: Path) -> Path:
    render_dir = run_dir / "render"
    render_dir.mkdir(parents=True, exist_ok=True)
    config_path = render_dir / "config.side_y.json"

    x_center = 0.5 * (SIDE_VIEW_X_LIM[0] + SIDE_VIEW_X_LIM[1])
    x_extent = SIDE_VIEW_X_LIM[1] - SIDE_VIEW_X_LIM[0]
    z_center = 0.5 * (SIDE_VIEW_Z_LIM[0] + SIDE_VIEW_Z_LIM[1])
    z_extent = SIDE_VIEW_Z_LIM[1] - SIDE_VIEW_Z_LIM[0]

    config = HybridConfig()
    config.camera = CameraConfig(
        elevation=0.0,
        azimuth=90.0,
        figsize_width=6.5,
        figsize_height=4.333333333333333,
        dpi=300,
    )
    config.style = StyleConfig.themed("light")
    config.style.body_color = "#111111"
    config.style.wing_color = "#d3d3d3"
    config.style.wing_edge_color = "#2e2e2e"
    config.viewport = ViewportConfig(
        center=np.array([x_center, 0.0, z_center], dtype=float),
        extent_xyz=np.array([x_extent, 2.2, z_extent], dtype=float),
    )
    config.blender.scale_factor = 0.82
    config.show_axes = False
    config.show_velocity_text = False
    config.save(str(config_path))
    return config_path

def output_path_for_theme(theme: str, output_arg: str | None) -> Path:
    if output_arg is not None:
        return resolve_path(output_arg, base=REPO_ROOT)
    return (
        DEFAULT_OUTPUT_DIR
        / f"wing_kinematics_section1_anim1_side_y.{theme}.mp4"
    )


def render_animation(
    *,
    h5_path: Path,
    output_mp4: Path,
    config_path: Path,
    theme: str,
    frame_step: int,
    no_blender: bool,
) -> None:
    output_mp4.parent.mkdir(parents=True, exist_ok=True)
    cmd = [
        sys.executable,
        "-m",
        "post.plot_simulation",
        str(h5_path),
        str(output_mp4),
        "--config",
        str(config_path),
        "--theme",
        theme,
    ]
    if frame_step != 1:
        cmd.extend(["--frame-step", str(frame_step)])
    if no_blender:
        cmd.append("--no-blender")
    run_cmd(cmd, cwd=REPO_ROOT)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--theme", choices=["light", "dark"], required=True)
    parser.add_argument(
        "--run-dir",
        default=str(DEFAULT_RUN_DIR.relative_to(REPO_ROOT)),
        help="Run directory for simulation and intermediate render artifacts.",
    )
    parser.add_argument(
        "--output",
        default=None,
        help="Output MP4 path (default: docs/_static/media/modeling/wing_kinematics/...<theme>.mp4)",
    )
    parser.add_argument(
        "--dragonfly-binary",
        default=None,
        help="Optional dragonfly simulator binary path passed to azuma1988 pipeline.",
    )
    parser.add_argument(
        "--frame-step",
        type=int,
        default=1,
        help="Render every Nth frame via post.plot_simulation (default: 1).",
    )
    parser.add_argument(
        "--no-blender",
        action="store_true",
        help="Pass through to post.plot_simulation to force matplotlib-only rendering.",
    )
    parser.add_argument("--force-sim", action="store_true")
    args = parser.parse_args()

    if args.frame_step < 1:
        raise ValueError("--frame-step must be >= 1")

    run_dir = resolve_path(args.run_dir, base=REPO_ROOT)
    output_mp4 = output_path_for_theme(args.theme, args.output)

    h5_path = ensure_sim_output(run_dir, args.dragonfly_binary, args.force_sim)
    config_path = write_render_config(run_dir)
    render_animation(
        h5_path=h5_path,
        output_mp4=output_mp4,
        config_path=config_path,
        theme=args.theme,
        frame_step=args.frame_step,
        no_blender=args.no_blender,
    )

    print(f"[done] output: {output_mp4}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

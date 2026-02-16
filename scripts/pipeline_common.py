#!/usr/bin/env python3
"""Shared helpers for pipeline scripts."""

from __future__ import annotations

import json
import os
import subprocess
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


def now_utc_iso() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


def ensure_dir(path: Path) -> Path:
    path.mkdir(parents=True, exist_ok=True)
    return path


def write_json(path: Path, data: dict[str, Any]) -> None:
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def read_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def run_cmd(cmd: list[str], cwd: Path | None = None, env: dict[str, str] | None = None) -> None:
    printable = " ".join(str(x) for x in cmd)
    print(f"[cmd] {printable}")
    cmd_env = os.environ.copy()
    if env:
        cmd_env.update(env)
    subprocess.run(cmd, cwd=str(cwd) if cwd else None, check=True, env=cmd_env)


def resolve_run_dir(run_dir_arg: str | None, *, repo_root: Path, runs_root: Path) -> Path:
    if run_dir_arg:
        run_dir = Path(run_dir_arg).expanduser()
        if not run_dir.is_absolute():
            run_dir = repo_root / run_dir
        return run_dir
    run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
    return runs_root / run_id


def build_plot_env(run_dir: Path) -> dict[str, str]:
    cache_root = ensure_dir(run_dir / ".cache")
    mpl_cache = ensure_dir(cache_root / "matplotlib")
    return {
        "MPLCONFIGDIR": str(mpl_cache.resolve()),
        "XDG_CACHE_HOME": str(cache_root.resolve()),
    }


def get_git_commit(repo_root: Path) -> str:
    try:
        out = subprocess.check_output(
            ["git", "rev-parse", "HEAD"],
            cwd=str(repo_root),
            text=True,
            stderr=subprocess.DEVNULL,
        )
        return out.strip()
    except Exception:
        return "unknown"


def update_manifest(
    run_dir: Path,
    stage: str,
    artifacts: list[Path],
    metadata: dict[str, Any],
    *,
    repo_root: Path,
) -> None:
    manifest_path = run_dir / "manifest.json"
    if manifest_path.exists():
        manifest = read_json(manifest_path)
    else:
        manifest = {
            "created_at_utc": now_utc_iso(),
            "repo_root": str(repo_root),
            "git_commit": get_git_commit(repo_root),
            "stages": {},
        }

    manifest["updated_at_utc"] = now_utc_iso()
    manifest["stages"][stage] = {
        "completed_at_utc": now_utc_iso(),
        "artifacts": [str(p.resolve()) for p in artifacts],
        "metadata": metadata,
    }
    write_json(manifest_path, manifest)

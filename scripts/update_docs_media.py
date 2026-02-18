#!/usr/bin/env python3
"""Regenerate and sync documentation media from a checked-in registry.

Usage examples:
  python scripts/update_docs_media.py --list
  python scripts/update_docs_media.py
  python scripts/update_docs_media.py --only wang2007_animation_light
  python scripts/update_docs_media.py --only wang2007_animation_dark
  python scripts/update_docs_media.py --dry-run
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_REGISTRY = REPO_ROOT / "docs" / "media_registry.json"
DEFAULT_LOG = REPO_ROOT / "docs" / "_build" / "media_build_manifest.json"


def now_utc_iso() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


def _validate_entry(entry: Any, index: int) -> None:
    if not isinstance(entry, dict):
        raise ValueError(f"Invalid registry entry at index {index}: expected object")

    entry_id = entry.get("id", f"<missing-id-{index}>")
    context = f"entry '{entry_id}'"

    if not isinstance(entry.get("id"), str) or not entry["id"]:
        raise ValueError(f"Invalid {context}: 'id' must be a non-empty string")
    if "description" in entry and not isinstance(entry["description"], str):
        raise ValueError(f"Invalid {context}: 'description' must be a string when present")

    commands = entry.get("commands")
    if not isinstance(commands, list):
        raise ValueError(f"Invalid {context}: 'commands' must be a list")
    for cmd_idx, cmd in enumerate(commands):
        if not isinstance(cmd, list) or not cmd:
            raise ValueError(f"Invalid {context}: commands[{cmd_idx}] must be a non-empty list")
        if not all(isinstance(arg, str) for arg in cmd):
            raise ValueError(f"Invalid {context}: commands[{cmd_idx}] must contain only strings")

    outputs = entry.get("outputs")
    if not isinstance(outputs, list):
        raise ValueError(f"Invalid {context}: 'outputs' must be a list")
    if not all(isinstance(path, str) and path for path in outputs):
        raise ValueError(f"Invalid {context}: 'outputs' must contain non-empty strings")

    env = entry.get("env")
    if env is not None:
        if not isinstance(env, dict):
            raise ValueError(f"Invalid {context}: 'env' must be an object when present")
        for key, value in env.items():
            if not isinstance(key, str) or not isinstance(value, str):
                raise ValueError(f"Invalid {context}: 'env' keys and values must be strings")

    sync_items = entry.get("sync")
    if sync_items is not None:
        if not isinstance(sync_items, list):
            raise ValueError(f"Invalid {context}: 'sync' must be a list when present")
        for sync_idx, item in enumerate(sync_items):
            if not isinstance(item, dict):
                raise ValueError(f"Invalid {context}: sync[{sync_idx}] must be an object")
            src = item.get("from")
            dst = item.get("to")
            if not isinstance(src, str) or not src:
                raise ValueError(f"Invalid {context}: sync[{sync_idx}].from must be a non-empty string")
            if not isinstance(dst, str) or not dst:
                raise ValueError(f"Invalid {context}: sync[{sync_idx}].to must be a non-empty string")


def validate_registry(data: Any, path: Path) -> None:
    if not isinstance(data, dict):
        raise ValueError(f"Invalid registry format at {path}: expected object")
    if "entries" not in data:
        raise ValueError(f"Invalid registry format at {path}: missing 'entries'")
    entries = data["entries"]
    if not isinstance(entries, list):
        raise ValueError(f"Invalid registry format at {path}: 'entries' must be a list")

    seen_ids: set[str] = set()
    for idx, entry in enumerate(entries):
        _validate_entry(entry, idx)
        entry_id = entry["id"]
        if entry_id in seen_ids:
            raise ValueError(f"Invalid registry format at {path}: duplicate entry id '{entry_id}'")
        seen_ids.add(entry_id)


def load_registry(path: Path) -> dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    validate_registry(data, path)
    return data


def run_command(cmd: list[str], env: dict[str, str], dry_run: bool) -> None:
    printable = " ".join(cmd)
    print(f"[cmd] {printable}")
    if dry_run:
        return
    result = subprocess.run(cmd, cwd=str(REPO_ROOT), env=env)
    if result.returncode != 0:
        raise RuntimeError(f"Command failed with exit code {result.returncode}: {printable}")


def ensure_outputs(paths: list[str], dry_run: bool) -> None:
    for rel in paths:
        p = REPO_ROOT / rel
        if dry_run:
            print(f"[check] {rel}")
            continue
        if not p.exists():
            raise FileNotFoundError(f"Expected output not found: {rel}")


def sync_artifacts(sync_items: list[dict[str, str]], dry_run: bool) -> None:
    for item in sync_items:
        src = REPO_ROOT / item["from"]
        dst = REPO_ROOT / item["to"]
        print(f"[sync] {item['from']} -> {item['to']}")
        if dry_run:
            continue
        if not src.exists():
            raise FileNotFoundError(f"Sync source missing: {item['from']}")
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dst)


def build_env(extra_env: dict[str, str] | None) -> dict[str, str]:
    env = dict(os.environ)
    if extra_env:
        for key, value in extra_env.items():
            env[key] = str((REPO_ROOT / value).resolve()) if not Path(value).is_absolute() else value
    return env


def run_entry(entry: dict[str, Any], dry_run: bool) -> dict[str, Any]:
    """Run one registry entry sequentially."""
    eid = entry["id"]
    print(f"\n== {eid} ==")
    env = build_env(entry.get("env"))
    commands = entry.get("commands", [])
    for cmd in commands:
        run_command([str(x) for x in cmd], env=env, dry_run=dry_run)

    outputs = entry.get("outputs", [])
    ensure_outputs(outputs, dry_run=dry_run)

    sync_items = entry.get("sync", [])
    sync_artifacts(sync_items, dry_run=dry_run)

    return {
        "id": eid,
        "completed_at_utc": now_utc_iso(),
        "commands": commands,
        "outputs": outputs,
        "sync": sync_items,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--registry", default=str(DEFAULT_REGISTRY), help="Path to docs media registry JSON.")
    parser.add_argument(
        "--only",
        nargs="*",
        default=[],
        help="Entry ids or prefixes to run (e.g. 'wang2007' matches all wang2007_* entries). If omitted, runs all.",
    )
    parser.add_argument("--list", action="store_true", help="List available entry ids and exit.")
    parser.add_argument("--dry-run", action="store_true", help="Print actions without executing.")
    parser.add_argument("--log", default=str(DEFAULT_LOG), help="Path to output build manifest JSON.")
    args = parser.parse_args()

    registry_path = Path(args.registry)
    if not registry_path.is_absolute():
        registry_path = (REPO_ROOT / registry_path).resolve()
    registry = load_registry(registry_path)
    entries = registry.get("entries", [])

    if args.list:
        for entry in entries:
            print(f"{entry['id']}: {entry.get('description', '')}")
        return 0

    filters = args.only
    if filters:
        selected = [e for e in entries if any(e["id"] == f or e["id"].startswith(f + "_") for f in filters)]
        unmatched = [f for f in filters if not any(e["id"] == f or e["id"].startswith(f + "_") for e in entries)]
        if unmatched:
            raise ValueError(f"No entries matched: {', '.join(unmatched)}")
    else:
        selected = entries

    run_log: dict[str, Any] = {
        "generated_at_utc": now_utc_iso(),
        "registry": str(registry_path.relative_to(REPO_ROOT)),
        "entries": [],
    }

    for entry in selected:
        run_log["entries"].append(run_entry(entry, dry_run=args.dry_run))

    if not args.dry_run:
        log_path = Path(args.log)
        if not log_path.is_absolute():
            log_path = REPO_ROOT / log_path
        log_path.parent.mkdir(parents=True, exist_ok=True)
        log_path.write_text(json.dumps(run_log, indent=2) + "\n", encoding="utf-8")
        print(f"\n[done] wrote build manifest: {log_path.relative_to(REPO_ROOT)}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

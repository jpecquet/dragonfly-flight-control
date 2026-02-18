#!/usr/bin/env python3
"""Validate all data/case_studies/*/case.json files against the JSON schema."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

try:
    from jsonschema import Draft202012Validator
except ModuleNotFoundError as exc:
    raise SystemExit(
        "Missing dependency 'jsonschema'. Install it with: python -m pip install jsonschema"
    ) from exc

try:
    from case_data import load_case_data
except ModuleNotFoundError:
    from scripts.case_data import load_case_data


REPO_ROOT = Path(__file__).resolve().parents[1]
CASE_ROOT = REPO_ROOT / "data" / "case_studies"
SCHEMA_PATH = REPO_ROOT / "data" / "schema" / "case_study.schema.json"


def _iter_case_files(case_ids: list[str]) -> list[Path]:
    if case_ids:
        return [CASE_ROOT / case_id / "case.json" for case_id in case_ids]
    return sorted(CASE_ROOT.glob("*/case.json"))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--case",
        action="append",
        default=[],
        help="Case id to validate (repeatable). Defaults to all cases.",
    )
    args = parser.parse_args()

    schema = json.loads(SCHEMA_PATH.read_text(encoding="utf-8"))
    validator = Draft202012Validator(schema)

    case_files = _iter_case_files(args.case)
    if not case_files:
        print("No case files found.")
        return 1

    failed = False
    for case_path in case_files:
        if not case_path.exists():
            print(f"[error] missing: {case_path.relative_to(REPO_ROOT)}")
            failed = True
            continue

        payload = json.loads(case_path.read_text(encoding="utf-8"))
        errors = sorted(validator.iter_errors(payload), key=lambda e: list(e.path))
        if errors:
            failed = True
            print(f"[error] schema validation failed: {case_path.relative_to(REPO_ROOT)}")
            for err in errors:
                path = "/".join(str(x) for x in err.path) or "<root>"
                print(f"  - {path}: {err.message}")
            continue

        case_id = case_path.parent.name
        try:
            load_case_data(case_id)
        except Exception as exc:  # noqa: BLE001 - report rich validation errors
            failed = True
            print(f"[error] semantic validation failed: {case_path.relative_to(REPO_ROOT)}")
            print(f"  - {exc}")
            continue

        print(f"[ok] {case_path.relative_to(REPO_ROOT)}")

    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())

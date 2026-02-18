#!/usr/bin/env python3
"""Helpers for loading and validating case-study data specs."""

from __future__ import annotations

import copy
import json
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[1]
CASE_STUDY_ROOT = REPO_ROOT / "data" / "case_studies"


class CaseDataError(ValueError):
    """Raised when a case data file is missing or invalid."""


def _require_dict(value: Any, *, context: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise CaseDataError(f"{context}: expected object")
    return value


def _require_list(value: Any, *, context: str) -> list[Any]:
    if not isinstance(value, list):
        raise CaseDataError(f"{context}: expected array")
    return value


def _require_number(value: Any, *, context: str) -> float:
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        raise CaseDataError(f"{context}: expected number")
    return float(value)


def _require_string(value: Any, *, context: str) -> str:
    if not isinstance(value, str) or not value:
        raise CaseDataError(f"{context}: expected non-empty string")
    return value


def case_file(case_id: str) -> Path:
    if not case_id:
        raise CaseDataError("case_id must be non-empty")
    return CASE_STUDY_ROOT / case_id / "case.json"


def validate_case_payload(payload: dict[str, Any], *, path: Path | None = None) -> None:
    context = str(path) if path else "case payload"

    schema_version = payload.get("schema_version")
    if not isinstance(schema_version, int) or schema_version < 1:
        raise CaseDataError(f"{context}: schema_version must be integer >= 1")

    _require_string(payload.get("case_id"), context=f"{context}.case_id")
    paper = _require_dict(payload.get("paper"), context=f"{context}.paper")
    _require_string(paper.get("citation_key"), context=f"{context}.paper.citation_key")

    _require_dict(payload.get("specimen"), context=f"{context}.specimen")
    _require_dict(payload.get("simulation_defaults"), context=f"{context}.simulation_defaults")

    convention = _require_dict(payload.get("convention"), context=f"{context}.convention")
    source_world_axes = _require_dict(
        convention.get("source_world_axes"),
        context=f"{context}.convention.source_world_axes",
    )
    for axis in ("X", "Y", "Z"):
        _require_string(source_world_axes.get(axis), context=f"{context}.convention.source_world_axes.{axis}")

    rotation = _require_list(
        convention.get("source_to_sim_rotation"),
        context=f"{context}.convention.source_to_sim_rotation",
    )
    if len(rotation) != 3:
        raise CaseDataError(f"{context}.convention.source_to_sim_rotation: expected 3 rows")
    for i, row in enumerate(rotation):
        row_list = _require_list(row, context=f"{context}.convention.source_to_sim_rotation[{i}]")
        if len(row_list) != 3:
            raise CaseDataError(f"{context}.convention.source_to_sim_rotation[{i}]: expected 3 columns")
        for j, val in enumerate(row_list):
            _require_number(val, context=f"{context}.convention.source_to_sim_rotation[{i}][{j}]")

    source_angle_names = _require_dict(
        convention.get("source_angle_names"),
        context=f"{context}.convention.source_angle_names",
    )
    for angle in ("gamma", "phi", "psi"):
        _require_string(source_angle_names.get(angle), context=f"{context}.convention.source_angle_names.{angle}")

    sim_transforms = _require_dict(
        convention.get("sim_transforms"),
        context=f"{context}.convention.sim_transforms",
    )
    for angle in ("gamma", "phi", "psi"):
        tr = _require_dict(sim_transforms.get(angle), context=f"{context}.convention.sim_transforms.{angle}")
        _require_number(tr.get("scale"), context=f"{context}.convention.sim_transforms.{angle}.scale")
        _require_number(tr.get("offset_deg"), context=f"{context}.convention.sim_transforms.{angle}.offset_deg")
        _require_number(
            tr.get("phase_shift_deg"),
            context=f"{context}.convention.sim_transforms.{angle}.phase_shift_deg",
        )

    _require_list(convention.get("notes"), context=f"{context}.convention.notes")

    kinematics = _require_dict(payload.get("kinematics"), context=f"{context}.kinematics")
    kind = _require_string(kinematics.get("kind"), context=f"{context}.kinematics.kind")
    if kind == "fourier_series_deg":
        _validate_fourier_kinematics(kinematics, context=f"{context}.kinematics")
    elif kind == "timeseries_csv":
        _validate_timeseries_kinematics(kinematics, context=f"{context}.kinematics")
    else:
        raise CaseDataError(f"{context}.kinematics.kind: unsupported kind '{kind}'")

    output_references = _require_list(payload.get("output_references"), context=f"{context}.output_references")
    for idx, output in enumerate(output_references):
        output_obj = _require_dict(output, context=f"{context}.output_references[{idx}]")
        output_kind = _require_string(
            output_obj.get("kind"),
            context=f"{context}.output_references[{idx}].kind",
        )
        _require_string(output_obj.get("name"), context=f"{context}.output_references[{idx}].name")
        if output_kind == "flight_condition":
            _require_number(output_obj.get("speed_m_s"), context=f"{context}.output_references[{idx}].speed_m_s")
            _require_number(output_obj.get("direction_deg"), context=f"{context}.output_references[{idx}].direction_deg")
        elif output_kind == "force_timeseries_csv":
            _require_string(output_obj.get("csv_path"), context=f"{context}.output_references[{idx}].csv_path")
            _require_string(
                output_obj.get("time_column"),
                context=f"{context}.output_references[{idx}].time_column",
            )
            _require_string(
                output_obj.get("value_column"),
                context=f"{context}.output_references[{idx}].value_column",
            )
        else:
            raise CaseDataError(
                f"{context}.output_references[{idx}].kind: unsupported kind '{output_kind}'"
            )


def _validate_fourier_kinematics(kinematics: dict[str, Any], *, context: str) -> None:
    angles = _require_dict(kinematics.get("angles"), context=f"{context}.angles")
    for wing in ("fore", "hind"):
        wing_payload = _require_dict(angles.get(wing), context=f"{context}.angles.{wing}")
        for angle in ("gamma", "phi", "psi"):
            series = _require_dict(wing_payload.get(angle), context=f"{context}.angles.{wing}.{angle}")
            _require_number(series.get("mean_deg"), context=f"{context}.angles.{wing}.{angle}.mean_deg")
            terms = _require_list(series.get("terms"), context=f"{context}.angles.{wing}.{angle}.terms")
            for idx, term in enumerate(terms):
                term_obj = _require_dict(term, context=f"{context}.angles.{wing}.{angle}.terms[{idx}]")
                harmonic = term_obj.get("harmonic")
                if not isinstance(harmonic, int) or harmonic < 1:
                    raise CaseDataError(
                        f"{context}.angles.{wing}.{angle}.terms[{idx}].harmonic: expected integer >= 1"
                    )
                _require_number(
                    term_obj.get("amplitude_deg"),
                    context=f"{context}.angles.{wing}.{angle}.terms[{idx}].amplitude_deg",
                )
                _require_number(
                    term_obj.get("phase_deg"),
                    context=f"{context}.angles.{wing}.{angle}.terms[{idx}].phase_deg",
                )


def _validate_timeseries_kinematics(kinematics: dict[str, Any], *, context: str) -> None:
    timeseries = _require_dict(kinematics.get("timeseries"), context=f"{context}.timeseries")
    _require_string(timeseries.get("csv_path"), context=f"{context}.timeseries.csv_path")

    skip_header = timeseries.get("skip_header", 0)
    if not isinstance(skip_header, int) or skip_header < 0:
        raise CaseDataError(f"{context}.timeseries.skip_header: expected integer >= 0")

    columns = _require_dict(timeseries.get("columns"), context=f"{context}.timeseries.columns")
    if not columns:
        raise CaseDataError(f"{context}.timeseries.columns: expected at least one entry")
    for name, mapping in columns.items():
        _require_string(name, context=f"{context}.timeseries.columns.<name>")
        mapping_obj = _require_dict(mapping, context=f"{context}.timeseries.columns.{name}")
        time_col = mapping_obj.get("time_col")
        value_col = mapping_obj.get("value_col")
        if not isinstance(time_col, int) or time_col < 0:
            raise CaseDataError(f"{context}.timeseries.columns.{name}.time_col: expected integer >= 0")
        if not isinstance(value_col, int) or value_col < 0:
            raise CaseDataError(f"{context}.timeseries.columns.{name}.value_col: expected integer >= 0")


def load_case_data(case_id: str) -> dict[str, Any]:
    path = case_file(case_id)
    if not path.exists():
        raise CaseDataError(f"Case data file not found: {path}")

    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise CaseDataError(f"Invalid JSON in case data file {path}: {exc}") from exc

    payload_obj = _require_dict(payload, context=str(path))
    validate_case_payload(payload_obj, path=path)

    if payload_obj["case_id"] != case_id:
        raise CaseDataError(
            f"{path}: case_id mismatch (expected '{case_id}', got '{payload_obj['case_id']}')"
        )

    return payload_obj


def _merge_dict_recursive(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    merged = copy.deepcopy(base)
    for key, value in override.items():
        if key in merged and isinstance(merged[key], dict) and isinstance(value, dict):
            merged[key] = _merge_dict_recursive(merged[key], value)
        else:
            merged[key] = copy.deepcopy(value)
    return merged


def select_experiment(case_data: dict[str, Any], experiment_id: str | int | None = None) -> dict[str, Any]:
    """Return case data resolved for a specific experiment, if present."""

    if experiment_id is None:
        default_experiment_id = case_data.get("default_experiment_id")
        if default_experiment_id is None:
            return copy.deepcopy(case_data)
        experiment_key = str(default_experiment_id)
    else:
        experiment_key = str(experiment_id)

    experiments = case_data.get("experiments")
    if experiments is None:
        if experiment_id is None:
            return copy.deepcopy(case_data)
        raise CaseDataError(
            f"Case '{case_data.get('case_id', '<case>')}' has no experiments map; "
            f"cannot select experiment '{experiment_key}'"
        )

    experiments_obj = _require_dict(
        experiments,
        context=f"{case_data.get('case_id', '<case>')}.experiments",
    )
    experiment_payload = _require_dict(
        experiments_obj.get(experiment_key),
        context=f"{case_data.get('case_id', '<case>')}.experiments.{experiment_key}",
    )

    resolved = copy.deepcopy(case_data)
    for field in ("paper", "specimen", "simulation_defaults", "convention"):
        if field in experiment_payload:
            base_field = _require_dict(
                resolved.get(field),
                context=f"{case_data.get('case_id', '<case>')}.{field}",
            )
            override_field = _require_dict(
                experiment_payload[field],
                context=f"{case_data.get('case_id', '<case>')}.experiments.{experiment_key}.{field}",
            )
            resolved[field] = _merge_dict_recursive(base_field, override_field)

    for field in ("kinematics", "output_references"):
        if field in experiment_payload:
            resolved[field] = copy.deepcopy(experiment_payload[field])

    resolved["selected_experiment"] = {
        "id": experiment_key,
        "label": str(experiment_payload.get("label", f"Experiment {experiment_key}")),
        "dragonfly": experiment_payload.get("dragonfly"),
    }
    return resolved


def find_output_reference(case_data: dict[str, Any], *, kind: str, name: str) -> dict[str, Any]:
    output_references = _require_list(
        case_data.get("output_references"),
        context=f"{case_data.get('case_id', '<case>')}.output_references",
    )
    for output in output_references:
        if not isinstance(output, dict):
            continue
        if output.get("kind") == kind and output.get("name") == name:
            return output
    raise CaseDataError(
        f"Output reference not found for kind='{kind}', name='{name}' "
        f"in case '{case_data.get('case_id', '<case>')}'"
    )

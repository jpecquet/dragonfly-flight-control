#!/usr/bin/env python3
"""Validation helpers for per-case docs media `post.yaml` configs."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml


SUPPORTED_SIMULATION_DRIVERS = {"yaml_case", "yaml_track_case", "reachable", "multi_yaml_case", "hover", "hover-control", "pursuit"}
SUPPORTED_THEMES = {"light", "dark"}
SUPPORTED_ARTIFACT_KINDS = {
    "case_fore_hind_kinematics",
    "exp_kinematics_scatter",
    "body_flight_metrics_vs_reference",
    "wing_aoa_timeseries",
    "wing_force_components_timeseries",
    "simulation_video",
    "stick_video",
    "force_comparison",
    "mass_regression",
    "reachable_set",
    "reachable_boundary",
    "forewing_stroke_diagram",
    "boundary_stroke_and_forces",
    "tracking_video",
    "pursuit_video",
    "hover_power",
    "hover_params",
    "hover_control",
}


def _ctx(path: Path | None) -> str:
    return f" in {path}" if path is not None else ""


def _expect_mapping(value: Any, name: str, *, path: Path | None = None) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"{name} must be a mapping{_ctx(path)}")
    return value


def _expect_nonempty_str(value: Any, name: str, *, path: Path | None = None) -> str:
    if not isinstance(value, str) or not value:
        raise ValueError(f"{name} must be a non-empty string{_ctx(path)}")
    return value


def _expect_int_ge(value: Any, name: str, minimum: int, *, path: Path | None = None) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{name} must be an integer >= {minimum}{_ctx(path)}")
    if value < minimum:
        raise ValueError(f"{name} must be >= {minimum}{_ctx(path)}")
    return value


def _expect_list(value: Any, name: str, *, path: Path | None = None) -> list[Any]:
    if not isinstance(value, list):
        raise ValueError(f"{name} must be a list{_ctx(path)}")
    return value


def _validate_themes(themes: Any, *, path: Path | None = None) -> list[str]:
    values = _expect_list(themes, "docs_media.themes", path=path)
    if not values:
        raise ValueError(f"docs_media.themes must be non-empty{_ctx(path)}")
    result: list[str] = []
    for idx, item in enumerate(values):
        name = _expect_nonempty_str(item, f"docs_media.themes[{idx}]", path=path)
        if name not in SUPPORTED_THEMES:
            raise ValueError(
                f"Unsupported theme {name!r} at docs_media.themes[{idx}]{_ctx(path)}; "
                f"expected one of {sorted(SUPPORTED_THEMES)}"
            )
        result.append(name)
    return result


def _validate_artifact_common(
    artifact: dict[str, Any],
    idx: int,
    *,
    docs_media_cfg: dict[str, Any],
    path: Path | None = None,
) -> None:
    kind = _expect_nonempty_str(artifact.get("kind"), f"artifacts[{idx}].kind", path=path)
    if kind not in SUPPORTED_ARTIFACT_KINDS:
        raise ValueError(
            f"Unsupported artifact kind {kind!r} at artifacts[{idx}].kind{_ctx(path)}; "
            f"expected one of {sorted(SUPPORTED_ARTIFACT_KINDS)}"
        )
    _expect_nonempty_str(artifact.get("output"), f"artifacts[{idx}].output", path=path)

    if kind == "case_fore_hind_kinematics":
        _expect_nonempty_str(artifact.get("source_case_file"), f"artifacts[{idx}].source_case_file", path=path)
        angle_keys = artifact.get("angle_keys")
        if angle_keys is not None:
            values = _expect_list(angle_keys, f"artifacts[{idx}].angle_keys", path=path)
            if len(values) != 2:
                raise ValueError(f"artifacts[{idx}].angle_keys must contain exactly two items{_ctx(path)}")
            for j, item in enumerate(values):
                _expect_nonempty_str(item, f"artifacts[{idx}].angle_keys[{j}]", path=path)
        layout = artifact.get("layout")
        if layout is not None:
            layout_value = _expect_nonempty_str(layout, f"artifacts[{idx}].layout", path=path)
            if layout_value not in {"vertical", "horizontal"}:
                raise ValueError(
                    f"artifacts[{idx}].layout must be 'vertical' or 'horizontal'{_ctx(path)}"
                )
        return

    if kind == "exp_kinematics_scatter":
        _expect_nonempty_str(artifact.get("source_case_file"), f"artifacts[{idx}].source_case_file", path=path)
        _expect_nonempty_str(artifact.get("csv_path"), f"artifacts[{idx}].csv_path", path=path)
        return

    if kind == "body_flight_metrics_vs_reference":
        _expect_nonempty_str(artifact.get("source_case_file"), f"artifacts[{idx}].source_case_file", path=path)
        _expect_nonempty_str(artifact.get("input_h5"), f"artifacts[{idx}].input_h5", path=path)
        ref_kind = artifact.get("reference_kind")
        if ref_kind is not None:
            _expect_nonempty_str(ref_kind, f"artifacts[{idx}].reference_kind", path=path)
        return

    if kind == "wing_aoa_timeseries":
        _expect_nonempty_str(artifact.get("input_h5"), f"artifacts[{idx}].input_h5", path=path)
        source_case_file = artifact.get("source_case_file")
        if source_case_file is not None:
            _expect_nonempty_str(source_case_file, f"artifacts[{idx}].source_case_file", path=path)
        csv_path = artifact.get("csv_path")
        if csv_path is not None:
            _expect_nonempty_str(csv_path, f"artifacts[{idx}].csv_path", path=path)
        wing_names = artifact.get("wing_names")
        if wing_names is not None:
            values = _expect_list(wing_names, f"artifacts[{idx}].wing_names", path=path)
            if len(values) != 2:
                raise ValueError(f"artifacts[{idx}].wing_names must contain exactly two items{_ctx(path)}")
            for j, item in enumerate(values):
                _expect_nonempty_str(item, f"artifacts[{idx}].wing_names[{j}]", path=path)
        eta = artifact.get("eta")
        if eta is not None:
            if isinstance(eta, bool) or not isinstance(eta, (int, float)):
                raise ValueError(f"artifacts[{idx}].eta must be numeric{_ctx(path)}")
            eta_val = float(eta)
            if eta_val < 0.0 or eta_val > 1.0:
                raise ValueError(f"artifacts[{idx}].eta must be in [0, 1]{_ctx(path)}")
        curve_variant = artifact.get("curve_variant")
        if curve_variant is not None:
            variant = _expect_nonempty_str(curve_variant, f"artifacts[{idx}].curve_variant", path=path)
            if variant not in {"model", "simplified"}:
                raise ValueError(
                    f"artifacts[{idx}].curve_variant must be one of 'model' or 'simplified'{_ctx(path)}"
                )
        simplified_speed_m_s = artifact.get("simplified_speed_m_s")
        if simplified_speed_m_s is not None and (
            isinstance(simplified_speed_m_s, bool) or not isinstance(simplified_speed_m_s, (int, float))
        ):
            raise ValueError(f"artifacts[{idx}].simplified_speed_m_s must be numeric{_ctx(path)}")
        return

    if kind == "wing_force_components_timeseries":
        _expect_nonempty_str(artifact.get("input_h5"), f"artifacts[{idx}].input_h5", path=path)
        return

    if kind == "simulation_video":
        _expect_nonempty_str(artifact.get("input_h5"), f"artifacts[{idx}].input_h5", path=path)
        render_config = artifact.get("render_config", docs_media_cfg.get("render_config"))
        _expect_nonempty_str(render_config, f"artifacts[{idx}].render_config", path=path)
        frame_step = artifact.get("frame_step")
        if frame_step is not None:
            _expect_int_ge(frame_step, f"artifacts[{idx}].frame_step", 1, path=path)
        return

    if kind == "tracking_video":
        _expect_nonempty_str(artifact.get("input_h5"), f"artifacts[{idx}].input_h5", path=path)
        render_config = artifact.get("render_config", docs_media_cfg.get("render_config"))
        _expect_nonempty_str(render_config, f"artifacts[{idx}].render_config", path=path)
        frame_step = artifact.get("frame_step")
        if frame_step is not None:
            _expect_int_ge(frame_step, f"artifacts[{idx}].frame_step", 1, path=path)
        return

    if kind == "stick_video":
        _expect_nonempty_str(artifact.get("input_h5"), f"artifacts[{idx}].input_h5", path=path)
        stations = artifact.get("stations")
        if stations is not None:
            values = _expect_list(stations, f"artifacts[{idx}].stations", path=path)
            for j, item in enumerate(values):
                if isinstance(item, bool) or not isinstance(item, (int, float)):
                    raise ValueError(f"artifacts[{idx}].stations[{j}] must be numeric{_ctx(path)}")
                s = float(item)
                if s < 0.0 or s > 1.0:
                    raise ValueError(f"artifacts[{idx}].stations[{j}] must be in [0, 1]{_ctx(path)}")
        return

    if kind == "reachable_set":
        _expect_nonempty_str(artifact.get("input_h5"), f"artifacts[{idx}].input_h5", path=path)
        return

    if kind == "forewing_stroke_diagram":
        _expect_nonempty_str(artifact.get("input_h5"), f"artifacts[{idx}].input_h5", path=path)
        return

    if kind == "boundary_stroke_and_forces":
        _expect_nonempty_str(artifact.get("input_h5"), f"artifacts[{idx}].input_h5", path=path)
        return

    if kind == "reachable_boundary":
        datasets = artifact.get("datasets")
        if isinstance(datasets, list):
            for j, d in enumerate(datasets):
                entry = _expect_mapping(d, f"artifacts[{idx}].datasets[{j}]", path=path)
                _expect_nonempty_str(entry.get("h5"), f"artifacts[{idx}].datasets[{j}].h5", path=path)
                _expect_nonempty_str(entry.get("label"), f"artifacts[{idx}].datasets[{j}].label", path=path)
        else:
            _expect_nonempty_str(artifact.get("input_h5"), f"artifacts[{idx}].input_h5", path=path)
        return

    if kind == "force_comparison":
        h5_series = _expect_list(artifact.get("h5_series"), f"artifacts[{idx}].h5_series", path=path)
        for j, item in enumerate(h5_series):
            entry = _expect_mapping(item, f"artifacts[{idx}].h5_series[{j}]", path=path)
            _expect_nonempty_str(entry.get("input_h5"), f"artifacts[{idx}].h5_series[{j}].input_h5", path=path)
            _expect_nonempty_str(entry.get("label"), f"artifacts[{idx}].h5_series[{j}].label", path=path)
        secondary = artifact.get("secondary_simulation")
        if secondary is not None:
            sec = _expect_mapping(secondary, f"artifacts[{idx}].secondary_simulation", path=path)
            _expect_nonempty_str(sec.get("case_file"), f"artifacts[{idx}].secondary_simulation.case_file", path=path)
            _expect_nonempty_str(sec.get("run_dir"), f"artifacts[{idx}].secondary_simulation.run_dir", path=path)
        csv_series = artifact.get("csv_series")
        if csv_series is not None:
            csv_list = _expect_list(csv_series, f"artifacts[{idx}].csv_series", path=path)
            for j, item in enumerate(csv_list):
                entry = _expect_mapping(item, f"artifacts[{idx}].csv_series[{j}]", path=path)
                _expect_nonempty_str(entry.get("path"), f"artifacts[{idx}].csv_series[{j}].path", path=path)
                _expect_nonempty_str(entry.get("label"), f"artifacts[{idx}].csv_series[{j}].label", path=path)
        return


def validate_post_config(config: Any, *, path: Path | None = None) -> dict[str, Any]:
    """Validate a loaded `post.yaml` config and return it unchanged if valid."""
    root = _expect_mapping(config, "post config", path=path)

    version = root.get("version")
    if version is None:
        raise ValueError(f"post config missing 'version'{_ctx(path)}")
    _expect_int_ge(version, "version", 1, path=path)

    simulation = _expect_mapping(root.get("simulation"), "simulation", path=path)
    driver = _expect_nonempty_str(simulation.get("driver"), "simulation.driver", path=path)
    if driver not in SUPPORTED_SIMULATION_DRIVERS:
        raise ValueError(
            f"Unsupported simulation.driver {driver!r}{_ctx(path)}; "
            f"expected one of {sorted(SUPPORTED_SIMULATION_DRIVERS)}"
        )
    _expect_nonempty_str(simulation.get("run_dir"), "simulation.run_dir", path=path)
    if driver in ("yaml_case", "yaml_track_case"):
        _expect_nonempty_str(simulation.get("case_file"), "simulation.case_file", path=path)
    if driver == "multi_yaml_case":
        cases = simulation.get("cases")
        if not isinstance(cases, list) or not cases:
            raise ValueError(f"simulation.cases must be a non-empty list for driver=multi_yaml_case{_ctx(path)}")
        for i, item in enumerate(cases):
            _expect_nonempty_str(item, f"simulation.cases[{i}]", path=path)
    if driver == "reachable":
        has_legacy = simulation.get("reachable_config") is not None
        has_sweep = simulation.get("base_config") is not None
        if not has_legacy and not has_sweep:
            raise ValueError(f"reachable driver requires reachable_config or base_config{_ctx(path)}")
        if has_legacy:
            rc = simulation.get("reachable_config")
            if isinstance(rc, list):
                for i, item in enumerate(rc):
                    _expect_nonempty_str(item, f"simulation.reachable_config[{i}]", path=path)
            else:
                _expect_nonempty_str(rc, "simulation.reachable_config", path=path)
        if has_sweep:
            _expect_nonempty_str(simulation.get("base_config"), "simulation.base_config", path=path)
            runs = simulation.get("runs")
            if not isinstance(runs, list) or not runs:
                raise ValueError(f"simulation.runs must be a non-empty list when base_config is used{_ctx(path)}")
            for i, run in enumerate(runs):
                entry = _expect_mapping(run, f"simulation.runs[{i}]", path=path)
                _expect_nonempty_str(entry.get("output"), f"simulation.runs[{i}].output", path=path)
                if entry.get("overrides") is not None:
                    ovr = entry.get("overrides")
                    if not isinstance(ovr, dict) or not ovr:
                        raise ValueError(f"simulation.runs[{i}].overrides must be a non-empty mapping{_ctx(path)}")
                    for k in ovr:
                        _expect_nonempty_str(k, f"simulation.runs[{i}].overrides key", path=path)
                elif entry.get("param") is not None:
                    _expect_nonempty_str(entry.get("param"), f"simulation.runs[{i}].param", path=path)
                    if "value" not in entry:
                        raise ValueError(f"simulation.runs[{i}].value is required when param is set{_ctx(path)}")
    if driver in ("hover", "hover-control"):
        _expect_nonempty_str(simulation.get("hover_config"), "simulation.hover_config", path=path)
    if driver == "pursuit":
        _expect_nonempty_str(simulation.get("pursuit_config"), "simulation.pursuit_config", path=path)
    if simulation.get("binary") is not None:
        _expect_nonempty_str(simulation.get("binary"), "simulation.binary", path=path)

    docs_media = _expect_mapping(root.get("docs_media"), "docs_media", path=path)
    _expect_nonempty_str(docs_media.get("media_dir"), "docs_media.media_dir", path=path)
    if docs_media.get("render_config") is not None:
        _expect_nonempty_str(docs_media.get("render_config"), "docs_media.render_config", path=path)
    if docs_media.get("themes") is not None:
        _validate_themes(docs_media.get("themes"), path=path)
    if docs_media.get("frame_step") is not None:
        _expect_int_ge(docs_media.get("frame_step"), "docs_media.frame_step", 1, path=path)

    artifacts = _expect_list(root.get("artifacts"), "artifacts", path=path)
    if not artifacts:
        raise ValueError(f"artifacts must be non-empty{_ctx(path)}")
    for idx, item in enumerate(artifacts):
        artifact = _expect_mapping(item, f"artifacts[{idx}]", path=path)
        _validate_artifact_common(artifact, idx, docs_media_cfg=docs_media, path=path)

    return root


def load_post_config(path: Path) -> dict[str, Any]:
    """Load and validate a per-case `post.yaml` config."""
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    return validate_post_config(data, path=path)

#!/usr/bin/env python3
"""Generic docs-media runner driven by per-case `post.yaml` configs.

Usage examples:
  python -m scripts.docs_media_runner cases/azuma1985/post.yaml
  python -m scripts.docs_media_runner --check-all
  python -m scripts.docs_media_runner --list
  python -m scripts.docs_media_runner --run-all --only azuma1988
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
from typing import Any

import yaml

from scripts.docs_media_config import load_post_config
from scripts.case_runner import run_case
from scripts.pipeline_common import build_plot_env, ensure_dir


REPO_ROOT = Path(__file__).resolve().parents[1]


def _display_path(path: Path) -> str:
    try:
        return str(path.relative_to(REPO_ROOT))
    except Exception:
        return str(path)


def _resolve_repo_path(value: str | Path) -> Path:
    path = Path(value).expanduser()
    if path.is_absolute():
        return path
    return REPO_ROOT / path


def _load_yaml(path: Path) -> dict[str, Any]:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"Expected YAML mapping in {path}")
    return data


def _format_obj(value: Any, context: dict[str, str]) -> Any:
    if isinstance(value, str):
        return value.format(**context)
    if isinstance(value, list):
        return [_format_obj(v, context) for v in value]
    if isinstance(value, dict):
        return {k: _format_obj(v, context) for k, v in value.items()}
    return value


def _artifact_uses_theme(value: Any) -> bool:
    if isinstance(value, str):
        return "{theme}" in value
    if isinstance(value, list):
        return any(_artifact_uses_theme(v) for v in value)
    if isinstance(value, dict):
        return any(_artifact_uses_theme(v) for v in value.values())
    return False


def _load_case_cache(cache: dict[Path, dict[str, Any]], case_path: Path) -> dict[str, Any]:
    if case_path not in cache:
        cache[case_path] = _load_yaml(case_path)
    return cache[case_path]


def find_post_configs(repo_root: Path = REPO_ROOT) -> list[Path]:
    """Find per-case docs media configs under `cases/*/post.yaml`."""
    return sorted((repo_root / "cases").glob("*/post.yaml"))


def post_config_case_id(path: Path) -> str:
    return path.parent.name


def _matches_case_filter(case_id: str, filt: str) -> bool:
    return case_id == filt or case_id.startswith(filt + "_")


def select_post_configs(paths: list[Path], filters: list[str]) -> list[Path]:
    """Filter post configs by case-id exact match or prefix (e.g. azuma1988)."""
    if not filters:
        return paths

    selected = [path for path in paths if any(_matches_case_filter(post_config_case_id(path), f) for f in filters)]
    unmatched = [f for f in filters if not any(_matches_case_filter(post_config_case_id(path), f) for path in paths)]
    if unmatched:
        raise ValueError(f"No post.yaml configs matched: {', '.join(unmatched)}")
    return selected


def list_post_configs(paths: list[Path]) -> None:
    for path in paths:
        print(f"{post_config_case_id(path)}: {_display_path(path)}")


def check_post_configs(paths: list[Path]) -> int:
    """Validate one or more post configs. Returns nonzero on any failure."""
    failed = 0
    for path in paths:
        try:
            load_post_config(path)
            print(f"[ok] {_display_path(path)}")
        except Exception as exc:
            failed += 1
            print(f"[error] {_display_path(path)}: {exc}")
    if failed:
        print(f"[done] {len(paths) - failed} passed, {failed} failed")
        return 1
    print(f"[done] validated {len(paths)} config(s)")
    return 0


def _run_simulation(
    config: dict[str, Any],
    *,
    skip_sim: bool,
    binary_override: str | None,
) -> tuple[Path, Path]:
    sim_cfg = config.get("simulation")
    if not isinstance(sim_cfg, dict):
        raise ValueError("post config missing 'simulation' mapping")

    driver = str(sim_cfg.get("driver", ""))
    run_dir_raw = sim_cfg.get("run_dir")
    if not isinstance(run_dir_raw, str) or not run_dir_raw:
        raise ValueError("simulation.run_dir must be a non-empty string")
    run_dir = ensure_dir(_resolve_repo_path(run_dir_raw))

    if driver != "yaml_case":
        raise ValueError(f"Unsupported simulation.driver: {driver!r}")

    case_file_raw = sim_cfg.get("case_file")
    if not isinstance(case_file_raw, str) or not case_file_raw:
        raise ValueError("simulation.case_file must be a non-empty string for driver=yaml_case")
    case_file = _resolve_repo_path(case_file_raw)

    if skip_sim:
        h5_path = run_dir / "output.h5"
        return run_dir, h5_path

    binary_value = binary_override if binary_override is not None else sim_cfg.get("binary")
    binary_path = Path(binary_value) if isinstance(binary_value, str) and binary_value else None
    h5_path = run_case(case_file, run_dir=run_dir, binary=binary_path)
    return run_dir, h5_path


def _run_artifact(
    artifact: dict[str, Any],
    *,
    theme: str | None,
    run_dir: Path,
    media_dir: Path,
    docs_media_cfg: dict[str, Any],
    case_cache: dict[Path, dict[str, Any]],
    no_blender: bool,
    frame_step_override: int | None,
) -> None:
    from post.docs_artifacts import (
        plot_body_flight_metrics_vs_reference,
        plot_case_fore_hind_kinematics,
        plot_exp_kinematics_scatter,
        plot_mass_regression,
        plot_wing_aoa_timeseries,
        plot_wing_force_components_timeseries,
        render_simulation_video_from_h5,
        render_stick_video_from_h5,
    )

    context = {
        "run_dir": str(run_dir),
        "media_dir": str(media_dir),
        "theme": "" if theme is None else theme,
    }
    resolved = _format_obj(artifact, context)
    kind = str(resolved.get("kind", ""))
    if not kind:
        raise ValueError("Artifact is missing 'kind'")

    output_raw = resolved.get("output")
    if not isinstance(output_raw, str) or not output_raw:
        raise ValueError(f"Artifact '{kind}' missing non-empty 'output'")
    output_path = Path(output_raw)
    if not output_path.is_absolute():
        output_path = media_dir / output_path
    output_path.parent.mkdir(parents=True, exist_ok=True)

    if kind == "case_fore_hind_kinematics":
        case_file_raw = resolved.get("source_case_file")
        if not isinstance(case_file_raw, str) or not case_file_raw:
            raise ValueError("case_fore_hind_kinematics requires source_case_file")
        case_path = _resolve_repo_path(case_file_raw)
        case = _load_case_cache(case_cache, case_path)
        angle_keys_raw = resolved.get("angle_keys", ["phi", "psi"])
        if not isinstance(angle_keys_raw, list) or len(angle_keys_raw) != 2:
            raise ValueError("angle_keys must be a list of two strings")
        plot_case_fore_hind_kinematics(
            case,
            output_path,
            angle_keys=(str(angle_keys_raw[0]), str(angle_keys_raw[1])),
            theme=theme,
            layout=str(resolved.get("layout", "vertical")),
        )
        return

    if kind == "exp_kinematics_scatter":
        case_file_raw = resolved.get("source_case_file")
        csv_path_raw = resolved.get("csv_path")
        if not isinstance(case_file_raw, str) or not case_file_raw:
            raise ValueError("exp_kinematics_scatter requires source_case_file")
        if not isinstance(csv_path_raw, str) or not csv_path_raw:
            raise ValueError("exp_kinematics_scatter requires csv_path")
        case_path = _resolve_repo_path(case_file_raw)
        case = _load_case_cache(case_cache, case_path)
        plot_exp_kinematics_scatter(
            case,
            _resolve_repo_path(csv_path_raw),
            output_path,
            theme=theme,
        )
        return

    if kind == "body_flight_metrics_vs_reference":
        case_file_raw = resolved.get("source_case_file")
        input_h5_raw = resolved.get("input_h5")
        if not isinstance(case_file_raw, str) or not case_file_raw:
            raise ValueError("body_flight_metrics_vs_reference requires source_case_file")
        if not isinstance(input_h5_raw, str) or not input_h5_raw:
            raise ValueError("body_flight_metrics_vs_reference requires input_h5")
        case_path = _resolve_repo_path(case_file_raw)
        case = _load_case_cache(case_cache, case_path)
        refs = list(case.get("references", []))
        ref_kind = resolved.get("reference_kind")
        if isinstance(ref_kind, str) and ref_kind:
            refs = [r for r in refs if isinstance(r, dict) and r.get("kind") == ref_kind]
        last_n_wb_raw = resolved.get("last_n_wingbeats")
        plot_body_flight_metrics_vs_reference(
            _resolve_repo_path(input_h5_raw),
            output_path,
            body_length_m=float(case["specimen"]["body_length"]),
            gravity_m_s2=float(case.get("environment", {}).get("gravity", 9.81)),
            references=refs,
            theme=theme,
            last_n_wingbeats=float(last_n_wb_raw) if last_n_wb_raw is not None else None,
        )
        return

    if kind == "wing_aoa_timeseries":
        input_h5_raw = resolved.get("input_h5")
        if not isinstance(input_h5_raw, str) or not input_h5_raw:
            raise ValueError("wing_aoa_timeseries requires input_h5")
        wing_names_raw = resolved.get("wing_names", ["fore_right", "hind_right"])
        if not isinstance(wing_names_raw, list) or len(wing_names_raw) != 2:
            raise ValueError("wing_aoa_timeseries wing_names must be a list of two strings")
        case_obj = None
        source_case_file_raw = resolved.get("source_case_file")
        if isinstance(source_case_file_raw, str) and source_case_file_raw:
            case_path = _resolve_repo_path(source_case_file_raw)
            case_obj = _load_case_cache(case_cache, case_path)
        last_n_wb_raw = resolved.get("last_n_wingbeats")
        plot_wing_aoa_timeseries(
            _resolve_repo_path(input_h5_raw),
            output_path,
            wing_names=(str(wing_names_raw[0]), str(wing_names_raw[1])),
            eta=float(resolved.get("eta", 2.0 / 3.0)),
            aoa_csv_path=(
                _resolve_repo_path(str(resolved["csv_path"]))
                if "csv_path" in resolved and resolved.get("csv_path") is not None
                else None
            ),
            source_case=case_obj,
            theme=theme,
            last_n_wingbeats=float(last_n_wb_raw) if last_n_wb_raw is not None else None,
        )
        return

    if kind == "wing_force_components_timeseries":
        input_h5_raw = resolved.get("input_h5")
        if not isinstance(input_h5_raw, str) or not input_h5_raw:
            raise ValueError("wing_force_components_timeseries requires input_h5")
        last_n_wb_raw = resolved.get("last_n_wingbeats")
        plot_wing_force_components_timeseries(
            _resolve_repo_path(input_h5_raw),
            output_path,
            theme=theme,
            last_n_wingbeats=float(last_n_wb_raw) if last_n_wb_raw is not None else None,
        )
        return

    if kind == "simulation_video":
        input_h5_raw = resolved.get("input_h5")
        if not isinstance(input_h5_raw, str) or not input_h5_raw:
            raise ValueError("simulation_video requires input_h5")
        render_config_raw = resolved.get("render_config", docs_media_cfg.get("render_config"))
        if not isinstance(render_config_raw, str) or not render_config_raw:
            raise ValueError("simulation_video requires render_config (artifact or docs_media default)")
        frame_step_val = (
            int(frame_step_override)
            if frame_step_override is not None
            else int(resolved.get("frame_step", docs_media_cfg.get("frame_step", 1)))
        )
        last_n_wb_raw = resolved.get("last_n_wingbeats")
        render_simulation_video_from_h5(
            _resolve_repo_path(input_h5_raw),
            output_path,
            render_config=_resolve_repo_path(render_config_raw),
            theme=theme,
            no_blender=no_blender,
            frame_step=frame_step_val,
            annotation_overlay=resolved.get("annotation_overlay"),
            last_n_wingbeats=float(last_n_wb_raw) if last_n_wb_raw is not None else None,
        )
        return

    if kind == "stick_video":
        input_h5_raw = resolved.get("input_h5")
        if not isinstance(input_h5_raw, str) or not input_h5_raw:
            raise ValueError("stick_video requires input_h5")
        stations_raw = resolved.get("stations")
        stations: list[float] | None = None
        if stations_raw is not None:
            if not isinstance(stations_raw, list):
                raise ValueError("stick_video stations must be a list of numbers")
            stations = [float(s) for s in stations_raw]
        render_stick_video_from_h5(
            _resolve_repo_path(input_h5_raw),
            output_path,
            theme=theme,
            stations=stations,
            show_axes=bool(resolved.get("show_axes", True)),
            show_grid=bool(resolved.get("show_grid", True)),
            show_timestamp=bool(resolved.get("show_timestamp", True)),
            show_pitch_angle=bool(resolved.get("show_pitch_angle", False)),
        )
        return

    if kind == "force_comparison":
        from post.plot_force_comparison import (
            ExternalForceSeries,
            compute_forewing_mass_regression_pi_factors,
            plot_force_comparison,
            read_force_series_csv,
        )

        h5_series_raw = resolved.get("h5_series", [])
        h5_inputs = [_resolve_repo_path(s["input_h5"]) for s in h5_series_raw]
        h5_labels = [str(s["label"]) for s in h5_series_raw]

        external_series: list[ExternalForceSeries] = []
        csv_series_raw = resolved.get("csv_series", [])
        for csv_entry in csv_series_raw:
            ext = read_force_series_csv(
                _resolve_repo_path(csv_entry["path"]),
                str(csv_entry.get("time_col", "t")),
                str(csv_entry.get("value_col", "Fz")),
                str(csv_entry["label"]),
            )
            # Pass through optional display fields.
            style = str(csv_entry.get("style", "line"))
            color = csv_entry.get("color")
            wrap_raw = csv_entry.get("wrap_period")
            wrap_period = float(wrap_raw) if wrap_raw is not None else None
            ext = ExternalForceSeries(
                x=ext.x, fz=ext.fz, label=ext.label,
                style=style,
                color=str(color) if color is not None else None,
                wrap_period=wrap_period,
            )
            external_series.append(ext)

        mass_envelope_factors: tuple[float, float] | None = None
        mass_uncertainty = resolved.get("mass_uncertainty")
        if isinstance(mass_uncertainty, dict):
            mode = str(mass_uncertainty.get("kind", ""))
            if mode == "forewing_regression_pi":
                case_file_raw = mass_uncertainty.get("source_case_file")
                body_csv_raw = mass_uncertainty.get("body_csv")
                forewing_csv_raw = mass_uncertainty.get("forewing_csv")
                if not isinstance(case_file_raw, str) or not case_file_raw:
                    raise ValueError("force_comparison.mass_uncertainty.forewing_regression_pi requires source_case_file")
                if not isinstance(body_csv_raw, str) or not body_csv_raw:
                    raise ValueError("force_comparison.mass_uncertainty.forewing_regression_pi requires body_csv")
                if not isinstance(forewing_csv_raw, str) or not forewing_csv_raw:
                    raise ValueError("force_comparison.mass_uncertainty.forewing_regression_pi requires forewing_csv")
                case_path = _resolve_repo_path(case_file_raw)
                case = _load_case_cache(case_cache, case_path)
                conf = float(mass_uncertainty.get("confidence", 0.68))
                extra = mass_uncertainty.get("extra_specimens")
                mass_envelope_factors = compute_forewing_mass_regression_pi_factors(
                    body_csv=_resolve_repo_path(body_csv_raw),
                    forewing_csv=_resolve_repo_path(forewing_csv_raw),
                    target_forewing_span_mm=float(case["wings"]["fore"]["span"]) * 1000.0,
                    confidence=conf,
                    extra_specimens=extra if isinstance(extra, list) else None,
                )

        plot_force_comparison(
            h5_inputs=h5_inputs,
            output_path=output_path,
            omega_nondim=resolved.get("omega_nondim"),
            labels=h5_labels,
            external_series=external_series if external_series else None,
            theme=theme,
            mass_envelope_factors=mass_envelope_factors,
        )
        return

    if kind == "mass_regression":
        case_file_raw = resolved.get("source_case_file")
        if not isinstance(case_file_raw, str) or not case_file_raw:
            raise ValueError("mass_regression requires source_case_file")
        case_path = _resolve_repo_path(case_file_raw)
        case = _load_case_cache(case_cache, case_path)
        body_csv_raw = resolved.get("body_csv")
        forewing_csv_raw = resolved.get("forewing_csv")
        if not isinstance(body_csv_raw, str) or not body_csv_raw:
            raise ValueError("mass_regression requires body_csv")
        if not isinstance(forewing_csv_raw, str) or not forewing_csv_raw:
            raise ValueError("mass_regression requires forewing_csv")
        extra = resolved.get("extra_specimens")
        plot_mass_regression(
            case,
            _resolve_repo_path(body_csv_raw),
            _resolve_repo_path(forewing_csv_raw),
            output_path,
            extra_specimens=extra if isinstance(extra, list) else None,
            theme=theme,
        )
        return

    raise ValueError(f"Unsupported artifact kind: {kind}")


def run_docs_media_config(
    post_config_path: Path,
    *,
    skip_sim: bool = False,
    no_blender: bool = False,
    themes_override: list[str] | None = None,
    frame_step_override: int | None = None,
    binary_override: str | None = None,
) -> None:
    config = load_post_config(post_config_path)

    docs_media_cfg = config.get("docs_media")
    if not isinstance(docs_media_cfg, dict):
        raise ValueError("post config missing 'docs_media' mapping")

    media_dir_raw = docs_media_cfg.get("media_dir")
    if not isinstance(media_dir_raw, str) or not media_dir_raw:
        raise ValueError("docs_media.media_dir must be a non-empty string")
    media_dir = ensure_dir(_resolve_repo_path(media_dir_raw))

    themes_cfg = docs_media_cfg.get("themes", ["light", "dark"])
    if themes_override:
        themes = [str(t) for t in themes_override]
    else:
        if not isinstance(themes_cfg, list) or not themes_cfg:
            raise ValueError("docs_media.themes must be a non-empty list")
        themes = [str(t) for t in themes_cfg]

    run_dir, h5_path = _run_simulation(config, skip_sim=skip_sim, binary_override=binary_override)
    if not h5_path.exists():
        raise FileNotFoundError(f"Simulation output not found: {h5_path}")

    plot_env = build_plot_env(run_dir)
    os.environ.update(plot_env)

    artifacts = config.get("artifacts")
    if not isinstance(artifacts, list) or not artifacts:
        raise ValueError("post config missing non-empty 'artifacts' list")

    case_cache: dict[Path, dict[str, Any]] = {}
    for artifact in artifacts:
        if not isinstance(artifact, dict):
            raise ValueError("Each artifact entry must be a mapping")
        use_themes = themes if _artifact_uses_theme(artifact) else [None]
        for theme in use_themes:
            if theme is not None:
                print(f"[theme] {theme} [{artifact.get('kind', '?')}]")
            _run_artifact(
                artifact,
                theme=theme,
                run_dir=run_dir,
                media_dir=media_dir,
                docs_media_cfg=docs_media_cfg,
                case_cache=case_cache,
                no_blender=no_blender,
                frame_step_override=frame_step_override,
            )

    print(f"[done] docs media generated from {post_config_path}")


def run_all_docs_media_configs(
    paths: list[Path],
    *,
    skip_sim: bool = False,
    no_blender: bool = False,
    themes_override: list[str] | None = None,
    frame_step_override: int | None = None,
    binary_override: str | None = None,
) -> None:
    for path in paths:
        print(f"\n== {post_config_case_id(path)} ==")
        run_docs_media_config(
            path,
            skip_sim=skip_sim,
            no_blender=no_blender,
            themes_override=themes_override,
            frame_step_override=frame_step_override,
            binary_override=binary_override,
        )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("post_config", nargs="?", help="Path to per-case post.yaml config.")
    parser.add_argument("--list", action="store_true", help="List discovered cases/*/post.yaml configs and exit.")
    parser.add_argument("--run-all", action="store_true", help="Run docs media generation for all cases/*/post.yaml.")
    parser.add_argument(
        "--only",
        nargs="*",
        default=[],
        help="Case ids or prefixes to select when using --list/--check-all/--run-all (e.g. 'azuma1988').",
    )
    parser.add_argument("--skip-sim", action="store_true", help="Skip simulation and reuse existing output.h5.")
    parser.add_argument("--no-blender", action="store_true", help="Force matplotlib-only simulation rendering.")
    parser.add_argument("--themes", nargs="+", choices=["light", "dark"], default=None)
    parser.add_argument("--frame-step", type=int, default=None, help="Override simulation_video frame step.")
    parser.add_argument("--binary", default=None, help="Override simulation binary path.")
    parser.add_argument(
        "--check-config",
        action="store_true",
        help="Validate the specified post.yaml and exit without running artifacts.",
    )
    parser.add_argument(
        "--check-all",
        action="store_true",
        help="Validate all cases/*/post.yaml configs and exit.",
    )
    args = parser.parse_args()

    if args.frame_step is not None and args.frame_step < 1:
        raise ValueError(f"--frame-step must be >= 1, got {args.frame_step}")

    discovery_mode_flags = [args.list, args.check_all, args.run_all]
    if sum(1 for flag in discovery_mode_flags if flag) > 1:
        parser.error("Choose only one of --list, --check-all, or --run-all")
    if args.check_config and any(discovery_mode_flags):
        parser.error("--check-config is only valid with a specific post_config")
    if any(discovery_mode_flags) and args.post_config is not None:
        parser.error("Do not provide post_config with --list, --check-all, or --run-all")
    if args.only and not any(discovery_mode_flags):
        parser.error("--only requires --list, --check-all, or --run-all")

    if any(discovery_mode_flags):
        paths = find_post_configs()
        if not paths:
            print("[done] no cases/*/post.yaml files found")
            return 0
        selected = select_post_configs(paths, args.only)
        if args.list:
            list_post_configs(selected)
            return 0
        if args.check_all:
            return check_post_configs(selected)
        if args.run_all:
            run_all_docs_media_configs(
                selected,
                skip_sim=args.skip_sim,
                no_blender=args.no_blender,
                themes_override=args.themes,
                frame_step_override=args.frame_step,
                binary_override=args.binary,
            )
            return 0

    if args.post_config is None:
        parser.error("post_config is required unless --list, --check-all, or --run-all is used")

    post_config_path = _resolve_repo_path(args.post_config)

    if args.check_config:
        return check_post_configs([post_config_path])

    run_docs_media_config(
        post_config_path,
        skip_sim=args.skip_sim,
        no_blender=args.no_blender,
        themes_override=args.themes,
        frame_step_override=args.frame_step,
        binary_override=args.binary,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Shared helpers for mapping experimental kinematics into simulator conventions."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

try:
    from case_data import load_case_data, select_experiment
except ModuleNotFoundError:
    from scripts.case_data import load_case_data, select_experiment


@dataclass(frozen=True)
class CosineTermDeg:
    """Single cosine term A*cos(k*omega*t + phase_deg), with A in degrees."""

    harmonic: int
    amplitude_deg: float
    phase_deg: float


@dataclass(frozen=True)
class HarmonicSeriesDeg:
    """Angle series in degrees: mean + sum A*cos(k*omega*t + phase)."""

    mean_deg: float
    terms: tuple[CosineTermDeg, ...]

    def transformed(self, transform: "SeriesTransform") -> "HarmonicSeriesDeg":
        shifted_terms: list[CosineTermDeg] = []
        for term in self.terms:
            shifted_terms.append(
                CosineTermDeg(
                    harmonic=term.harmonic,
                    amplitude_deg=transform.scale * term.amplitude_deg,
                    phase_deg=term.phase_deg + transform.phase_shift_deg * float(term.harmonic),
                )
            )
        return HarmonicSeriesDeg(
            mean_deg=transform.offset_deg + transform.scale * self.mean_deg,
            terms=tuple(shifted_terms),
        )

    def eval_deg(self, t_nondim: Any) -> Any:
        """Evaluate on scalar or numpy-like array t/T_wb."""
        w_t = 2.0 * math.pi * t_nondim
        out = 0.0 * t_nondim + float(self.mean_deg)
        for term in self.terms:
            out += float(term.amplitude_deg) * _cos(w_t * float(term.harmonic) + math.radians(term.phase_deg))
        return out


@dataclass(frozen=True)
class SeriesTransform:
    """Affine transform on a series: out(t) = offset + scale*in(t + phase_shift)."""

    scale: float = 1.0
    offset_deg: float = 0.0
    phase_shift_deg: float = 0.0

    def inverse_value_deg(self, value_deg: Any) -> Any:
        if self.scale == 0.0:
            raise ValueError("SeriesTransform scale must be non-zero")
        return (value_deg - self.offset_deg) / self.scale


@dataclass(frozen=True)
class WingSeriesPaper:
    gamma: HarmonicSeriesDeg
    phi: HarmonicSeriesDeg
    psi: HarmonicSeriesDeg


@dataclass(frozen=True)
class SimWingMotion:
    gamma_mean: float
    gamma_amp: tuple[float, ...]
    gamma_phase: tuple[float, ...]
    phi_mean: float
    phi_amp: tuple[float, ...]
    phi_phase: tuple[float, ...]
    psi_mean: float
    psi_amp: tuple[float, ...]
    psi_phase: tuple[float, ...]


@dataclass(frozen=True)
class ExperimentalConventionAdapter:
    dataset_id: str
    source_world_axes: dict[str, str]
    source_to_sim_rotation: tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]
    source_angle_names: dict[str, str]
    source_typos: tuple[str, ...]
    source_series: dict[str, WingSeriesPaper]
    sim_transforms: dict[str, SeriesTransform]
    notes: tuple[str, ...]

    def with_gamma_overrides(self, gamma_deg: dict[str, float] | None) -> "ExperimentalConventionAdapter":
        if not gamma_deg:
            return self

        updated: dict[str, WingSeriesPaper] = {}
        for wing, src in self.source_series.items():
            if wing in gamma_deg:
                updated[wing] = WingSeriesPaper(
                    gamma=HarmonicSeriesDeg(
                        mean_deg=float(gamma_deg[wing]),
                        terms=src.gamma.terms,
                    ),
                    phi=src.phi,
                    psi=src.psi,
                )
            else:
                updated[wing] = src
        return ExperimentalConventionAdapter(
            dataset_id=self.dataset_id,
            source_world_axes=self.source_world_axes,
            source_to_sim_rotation=self.source_to_sim_rotation,
            source_angle_names=self.source_angle_names,
            source_typos=self.source_typos,
            source_series=updated,
            sim_transforms=self.sim_transforms,
            notes=self.notes,
        )

    def sim_series(self) -> dict[str, WingSeriesPaper]:
        out: dict[str, WingSeriesPaper] = {}
        for wing, src in self.source_series.items():
            out[wing] = WingSeriesPaper(
                gamma=src.gamma.transformed(self.sim_transforms["gamma"]),
                phi=src.phi.transformed(self.sim_transforms["phi"]),
                psi=src.psi.transformed(self.sim_transforms["psi"]),
            )
        return out

    def summary(self) -> dict[str, Any]:
        return {
            "dataset_id": self.dataset_id,
            "source_world_axes": dict(self.source_world_axes),
            "source_to_sim_rotation": [list(row) for row in self.source_to_sim_rotation],
            "source_angle_names": dict(self.source_angle_names),
            "source_typos": list(self.source_typos),
            "sim_transforms": {
                name: {
                    "scale": tr.scale,
                    "offset_deg": tr.offset_deg,
                    "phase_shift_deg": tr.phase_shift_deg,
                }
                for name, tr in self.sim_transforms.items()
            },
            "notes": list(self.notes),
        }


def _build_series_from_case(case_data: dict[str, Any]) -> dict[str, WingSeriesPaper]:
    kinematics = case_data["kinematics"]
    if kinematics["kind"] != "fourier_series_deg":
        raise ValueError(
            f"Case '{case_data['case_id']}' uses unsupported kinematics kind "
            f"for adapter: {kinematics['kind']}"
        )

    def build_angle_series(payload: dict[str, Any]) -> HarmonicSeriesDeg:
        return HarmonicSeriesDeg(
            mean_deg=float(payload["mean_deg"]),
            terms=tuple(
                CosineTermDeg(
                    harmonic=int(term["harmonic"]),
                    amplitude_deg=float(term["amplitude_deg"]),
                    phase_deg=float(term["phase_deg"]),
                )
                for term in payload["terms"]
            ),
        )

    out: dict[str, WingSeriesPaper] = {}
    for wing in ("fore", "hind"):
        wing_payload = kinematics["angles"][wing]
        out[wing] = WingSeriesPaper(
            gamma=build_angle_series(wing_payload["gamma"]),
            phi=build_angle_series(wing_payload["phi"]),
            psi=build_angle_series(wing_payload["psi"]),
        )
    return out


def _adapter_from_case(case_data: dict[str, Any]) -> ExperimentalConventionAdapter:
    convention = case_data["convention"]
    dataset_id = str(case_data["case_id"])
    selected_experiment = case_data.get("selected_experiment")
    if isinstance(selected_experiment, dict) and "id" in selected_experiment:
        dataset_id = f"{dataset_id}:exp{selected_experiment['id']}"
    return ExperimentalConventionAdapter(
        dataset_id=dataset_id,
        source_world_axes={
            "X": str(convention["source_world_axes"]["X"]),
            "Y": str(convention["source_world_axes"]["Y"]),
            "Z": str(convention["source_world_axes"]["Z"]),
        },
        source_to_sim_rotation=tuple(
            tuple(float(v) for v in row) for row in convention["source_to_sim_rotation"]
        ),
        source_angle_names={
            "gamma": str(convention["source_angle_names"]["gamma"]),
            "phi": str(convention["source_angle_names"]["phi"]),
            "psi": str(convention["source_angle_names"]["psi"]),
        },
        source_typos=tuple(str(item) for item in convention.get("source_typos", [])),
        source_series=_build_series_from_case(case_data),
        sim_transforms={
            angle: SeriesTransform(
                scale=float(payload["scale"]),
                offset_deg=float(payload["offset_deg"]),
                phase_shift_deg=float(payload["phase_shift_deg"]),
            )
            for angle, payload in convention["sim_transforms"].items()
        },
        notes=tuple(str(item) for item in convention["notes"]),
    )


def _cos(x: Any) -> Any:
    try:
        import numpy as np  # type: ignore

        if hasattr(x, "__array__"):
            return np.cos(x)
    except Exception:
        pass
    return math.cos(float(x))


def harmonic_amp_phase_from_series_deg(
    series_deg: HarmonicSeriesDeg,
    n_harmonics: int,
) -> tuple[float, list[float], list[float]]:
    if n_harmonics <= 0:
        raise ValueError("n_harmonics must be >= 1")

    in_phase = [0.0] * n_harmonics
    quadrature = [0.0] * n_harmonics
    for term in series_deg.terms:
        if term.harmonic < 1 or term.harmonic > n_harmonics:
            raise ValueError(f"harmonic index out of range: {term.harmonic}")
        idx = term.harmonic - 1
        phase_rad = math.radians(term.phase_deg)
        amp_rad = math.radians(term.amplitude_deg)
        in_phase[idx] += amp_rad * math.cos(phase_rad)
        quadrature[idx] += -amp_rad * math.sin(phase_rad)

    amp_coeff = [0.0] * n_harmonics
    phase_coeff = [0.0] * n_harmonics
    for i, (x, y) in enumerate(zip(in_phase, quadrature)):
        amp = math.hypot(x, y)
        amp_coeff[i] = amp
        phase_coeff[i] = math.atan2(-y, x) if amp > 1e-15 else 0.0
    return math.radians(series_deg.mean_deg), amp_coeff, phase_coeff


def build_sim_wing_motion(
    adapter: ExperimentalConventionAdapter,
    n_harmonics: int,
) -> dict[str, SimWingMotion]:
    out: dict[str, SimWingMotion] = {}
    for wing, series in adapter.sim_series().items():
        gamma_mean, gamma_amp, gamma_phase = harmonic_amp_phase_from_series_deg(
            series.gamma, n_harmonics=n_harmonics
        )
        phi_mean, phi_amp, phi_phase = harmonic_amp_phase_from_series_deg(
            series.phi, n_harmonics=n_harmonics
        )
        psi_mean, psi_amp, psi_phase = harmonic_amp_phase_from_series_deg(
            series.psi, n_harmonics=n_harmonics
        )
        out[wing] = SimWingMotion(
            gamma_mean=gamma_mean,
            gamma_amp=tuple(gamma_amp),
            gamma_phase=tuple(gamma_phase),
            phi_mean=phi_mean,
            phi_amp=tuple(phi_amp),
            phi_phase=tuple(phi_phase),
            psi_mean=psi_mean,
            psi_amp=tuple(psi_amp),
            psi_phase=tuple(psi_phase),
        )
    return out


def azuma1985_adapter() -> ExperimentalConventionAdapter:
    """Azuma (1985) series and mappings loaded from data/case_studies."""
    return _adapter_from_case(load_case_data("azuma1985"))


def azuma1988_adapter(experiment: str | int | None = None) -> ExperimentalConventionAdapter:
    """Azuma (1988) series and mappings loaded from data/case_studies."""
    case = load_case_data("azuma1988")
    resolved = select_experiment(case, experiment_id=experiment)
    return _adapter_from_case(resolved)

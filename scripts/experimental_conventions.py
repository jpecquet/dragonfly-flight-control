#!/usr/bin/env python3
"""Shared helpers for mapping experimental kinematics into simulator conventions."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any


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
    gamma_cos: tuple[float, ...]
    gamma_sin: tuple[float, ...]
    phi_mean: float
    phi_cos: tuple[float, ...]
    phi_sin: tuple[float, ...]
    psi_mean: float
    psi_cos: tuple[float, ...]
    psi_sin: tuple[float, ...]


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


def _cos(x: Any) -> Any:
    try:
        import numpy as np  # type: ignore

        if hasattr(x, "__array__"):
            return np.cos(x)
    except Exception:
        pass
    return math.cos(float(x))


def harmonic_coeffs_from_series_deg(
    series_deg: HarmonicSeriesDeg,
    n_harmonics: int,
) -> tuple[float, list[float], list[float]]:
    if n_harmonics <= 0:
        raise ValueError("n_harmonics must be >= 1")

    cos_coeff = [0.0] * n_harmonics
    sin_coeff = [0.0] * n_harmonics
    for term in series_deg.terms:
        if term.harmonic < 1 or term.harmonic > n_harmonics:
            raise ValueError(f"harmonic index out of range: {term.harmonic}")
        idx = term.harmonic - 1
        phase_rad = math.radians(term.phase_deg)
        amp_rad = math.radians(term.amplitude_deg)
        cos_coeff[idx] += amp_rad * math.cos(phase_rad)
        sin_coeff[idx] += -amp_rad * math.sin(phase_rad)
    return math.radians(series_deg.mean_deg), cos_coeff, sin_coeff


def build_sim_wing_motion(
    adapter: ExperimentalConventionAdapter,
    n_harmonics: int,
) -> dict[str, SimWingMotion]:
    out: dict[str, SimWingMotion] = {}
    for wing, series in adapter.sim_series().items():
        gamma_mean, gamma_cos, gamma_sin = harmonic_coeffs_from_series_deg(series.gamma, n_harmonics=n_harmonics)
        phi_mean, phi_cos, phi_sin = harmonic_coeffs_from_series_deg(series.phi, n_harmonics=n_harmonics)
        psi_mean, psi_cos, psi_sin = harmonic_coeffs_from_series_deg(series.psi, n_harmonics=n_harmonics)
        out[wing] = SimWingMotion(
            gamma_mean=gamma_mean,
            gamma_cos=tuple(gamma_cos),
            gamma_sin=tuple(gamma_sin),
            phi_mean=phi_mean,
            phi_cos=tuple(phi_cos),
            phi_sin=tuple(phi_sin),
            psi_mean=psi_mean,
            psi_cos=tuple(psi_cos),
            psi_sin=tuple(psi_sin),
        )
    return out


def azuma1985_adapter() -> ExperimentalConventionAdapter:
    """Raw Azuma (1985) series in paper notation + explicit simulator mapping."""
    return ExperimentalConventionAdapter(
        dataset_id="azuma1985",
        source_world_axes={
            "X": "backward",
            "Y": "right",
            "Z": "up",
        },
        source_to_sim_rotation=(
            (-1.0, 0.0, 0.0),
            (0.0, -1.0, 0.0),
            (0.0, 0.0, 1.0),
        ),
        source_angle_names={
            "gamma": "stroke plane angle",
            "phi": "flapping angle theta (paper figure typo labels this phi)",
            "psi": "pitch angle",
        },
        source_typos=(
            "In azuma1985_coords.png, phi in the transform table should be theta.",
        ),
        source_series={
            "fore": WingSeriesPaper(
                gamma=HarmonicSeriesDeg(
                    mean_deg=37.0,
                    terms=(),
                ),
                phi=HarmonicSeriesDeg(
                    mean_deg=-3.0,
                    terms=(
                        CosineTermDeg(harmonic=1, amplitude_deg=-43.0, phase_deg=0.0),
                    ),
                ),
                psi=HarmonicSeriesDeg(
                    mean_deg=98.0,
                    terms=(
                        CosineTermDeg(harmonic=1, amplitude_deg=-77.0, phase_deg=-49.0),
                        CosineTermDeg(harmonic=2, amplitude_deg=-3.0, phase_deg=67.0),
                        CosineTermDeg(harmonic=3, amplitude_deg=-8.0, phase_deg=29.0),
                    ),
                ),
            ),
            "hind": WingSeriesPaper(
                gamma=HarmonicSeriesDeg(
                    mean_deg=40.0,
                    terms=(),
                ),
                phi=HarmonicSeriesDeg(
                    mean_deg=2.0,
                    terms=(
                        CosineTermDeg(harmonic=1, amplitude_deg=-47.0, phase_deg=77.0),
                    ),
                ),
                psi=HarmonicSeriesDeg(
                    mean_deg=93.0,
                    terms=(
                        CosineTermDeg(harmonic=1, amplitude_deg=-65.0, phase_deg=18.0),
                        CosineTermDeg(harmonic=2, amplitude_deg=8.0, phase_deg=74.0),
                        CosineTermDeg(harmonic=3, amplitude_deg=8.0, phase_deg=28.0),
                    ),
                ),
            ),
        },
        sim_transforms={
            "gamma": SeriesTransform(scale=1.0, offset_deg=0.0, phase_shift_deg=0.0),
            "phi": SeriesTransform(scale=-1.0, offset_deg=0.0, phase_shift_deg=0.0),
            "psi": SeriesTransform(scale=-1.0, offset_deg=90.0, phase_shift_deg=0.0),
        },
        notes=(
            "Paper world frame (X backward, Y right, Z up) is rotated 180 deg about +Z to simulator frame (X forward, Y left, Z up).",
            "Azuma flapping angle theta has opposite sign to simulator phi in current model mapping.",
            "Azuma pitch angle beta/theta maps to simulator psi by psi_sim = 90 deg - beta.",
        ),
    )

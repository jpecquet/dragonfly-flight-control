# Case Study Data

This directory is the source of truth for paper-derived case-study inputs.

Each case uses:

- `case.json`: normalized metadata, specimen/simulation defaults, kinematics source description, and experimental output references.

Schema:

- `data/schema/case_study.schema.json`

Notes:

- `kinematics.kind = fourier_series_deg` is used when the source paper reports harmonic coefficients directly.
- `kinematics.kind = timeseries_csv` is used when the source is digitized timeseries (CSV) later mapped/fitted by pipeline code.
- `output_references` supports both scalar flight conditions and force CSV references.

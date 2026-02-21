import json
import math
import tempfile
import unittest
from pathlib import Path

import h5py
import numpy as np

from cases.azuma1985.plot_flight_metrics import compute_flight_metrics


def _write_fixture(path: Path, *, time: np.ndarray, state: np.ndarray, omega: float) -> None:
    with h5py.File(str(path), "w") as f:
        f.create_dataset("/time", data=time)
        f.create_dataset("/state", data=state)
        f.create_dataset("/parameters/omega", data=float(omega))


class TestComputeFlightMetrics(unittest.TestCase):
    def test_speed_dimensionalization_and_direction(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            h5_path = Path(tmpdir) / "output.h5"
            time = np.array([0.0, 0.5, 1.0], dtype=float)
            state = np.array(
                [
                    [1.0, 0.0, 0.0, 3.0, 4.0, 0.0],   # speed_nd=5, dir=0 deg
                    [0.0, 0.0, 2.0, 0.0, 0.0, 2.0],   # speed_nd=2, dir=90 deg
                    [-2.0, 0.0, 0.0, 1.0, 2.0, 2.0],  # speed_nd=3, dir=180 deg
                ],
                dtype=float,
            )
            omega = 2.0 * math.pi
            _write_fixture(h5_path, time=time, state=state, omega=omega)

            out = compute_flight_metrics(h5_path, body_length_m=0.25, gravity_m_s2=16.0)
            scale = math.sqrt(0.25 * 16.0)

            np.testing.assert_allclose(out["wingbeats"], np.array([0.0, 0.5, 1.0]), atol=1e-12)
            np.testing.assert_allclose(out["speed_m_s"], np.array([5.0, 2.0, 3.0]) * scale, atol=1e-12)
            np.testing.assert_allclose(out["direction_deg"], np.array([0.0, 90.0, 180.0]), atol=1e-12)

    def test_loads_body_length_and_gravity_from_translate_summary(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            sim_dir = Path(tmpdir) / "sim"
            sim_dir.mkdir(parents=True, exist_ok=True)
            h5_path = sim_dir / "output.h5"
            _write_fixture(
                h5_path,
                time=np.array([0.0, 1.0], dtype=float),
                state=np.array(
                    [
                        [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                        [1.0, 0.0, 1.0, 2.0, 0.0, 0.0],
                    ],
                    dtype=float,
                ),
                omega=2.0 * math.pi,
            )
            summary = {
                "physical_inputs": {
                    "body_length_m": 0.09,
                    "gravity_m_s2": 25.0,
                }
            }
            (sim_dir / "translate_summary.json").write_text(
                json.dumps(summary, indent=2),
                encoding="utf-8",
            )

            out = compute_flight_metrics(h5_path)
            scale = math.sqrt(0.09 * 25.0)

            self.assertAlmostEqual(float(out["body_length_m"]), 0.09, places=12)
            self.assertAlmostEqual(float(out["gravity_m_s2"]), 25.0, places=12)
            np.testing.assert_allclose(out["speed_m_s"], np.array([1.0, 2.0]) * scale, atol=1e-12)


if __name__ == "__main__":
    unittest.main()

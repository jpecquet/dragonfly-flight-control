import importlib.util
import math
import sys
import tempfile
import unittest
from pathlib import Path

import h5py
import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[2]


def load_module(relpath: str, module_name: str):
    module_path = REPO_ROOT / relpath
    spec = importlib.util.spec_from_file_location(module_name, module_path)
    module = importlib.util.module_from_spec(spec)
    assert spec is not None and spec.loader is not None
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


pipeline = load_module("scripts/wang2007_pipeline.py", "wang2007_pipeline_test")


class TestMotionMapping(unittest.TestCase):
    """Verify compute_smoothed_motion_mapping returns correct structure."""

    def setUp(self):
        self.common = pipeline.load_common_module()
        self.omega_nd = 14.98  # approximate nondimensional omega

    def test_7_harmonic_structure(self):
        m = pipeline.compute_smoothed_motion_mapping(self.common, self.omega_nd, 7)
        self.assertEqual(m["method"]["n_harmonics"], 7)
        self.assertEqual(m["parameters"]["n_harmonics"], 7)
        for wing in ("fore", "hind"):
            ov = m["wing_motion_overrides"][wing]
            for key in ("phi_cos", "phi_sin", "psi_cos", "psi_sin", "gamma_cos", "gamma_sin"):
                self.assertEqual(len(ov[key]), 7, f"{wing}/{key} should have 7 harmonics")

    def test_1_harmonic_structure(self):
        m = pipeline.compute_smoothed_motion_mapping(self.common, self.omega_nd, 1)
        self.assertEqual(m["method"]["n_harmonics"], 1)
        for wing in ("fore", "hind"):
            ov = m["wing_motion_overrides"][wing]
            for key in ("phi_cos", "phi_sin", "psi_cos", "psi_sin"):
                self.assertEqual(len(ov[key]), 1, f"{wing}/{key} should have 1 harmonic")

    def test_gamma_values(self):
        m = pipeline.compute_smoothed_motion_mapping(self.common, self.omega_nd, 7)
        self.assertAlmostEqual(
            m["wing_motion_overrides"]["fore"]["gamma_mean"],
            float(self.common.gamma_fore),
            places=10,
        )
        self.assertAlmostEqual(
            m["wing_motion_overrides"]["hind"]["gamma_mean"],
            float(self.common.gamma_hind),
            places=10,
        )

    def test_high_fidelity_component_count(self):
        self.assertEqual(pipeline.HARMONICS_PER_WINGBEAT, 7)
        self.assertEqual(
            pipeline.N_FOURIER_COMPONENTS,
            pipeline.HARMONICS_PER_WINGBEAT * pipeline.N_WINGBEATS,
        )
        self.assertEqual(pipeline.N_FOURIER_COMPONENTS, 35)

    def test_high_fidelity_mapping_uses_multi_wingbeat_basis(self):
        m = pipeline.compute_smoothed_motion_mapping(
            self.common,
            self.omega_nd,
            pipeline.N_FOURIER_COMPONENTS,
            harmonic_period_wingbeats=float(pipeline.N_WINGBEATS),
        )
        self.assertEqual(m["method"]["n_harmonics"], 35)
        self.assertEqual(m["method"]["harmonics_per_wingbeat_for_smoothing"], 7)
        self.assertAlmostEqual(m["parameters"]["harmonic_period_wingbeats"], 5.0, places=12)
        for wing in ("fore", "hind"):
            ov = m["wing_motion_overrides"][wing]
            for key in ("phi_cos", "phi_sin", "psi_cos", "psi_sin", "gamma_cos", "gamma_sin"):
                self.assertEqual(len(ov[key]), 35, f"{wing}/{key} should have 35 coefficients")

    def test_default_wing_length_matches_explicit_default(self):
        implicit = pipeline.compute_smoothed_motion_mapping(self.common, self.omega_nd, 7)
        explicit = pipeline.compute_smoothed_motion_mapping(
            self.common,
            self.omega_nd,
            7,
            wing_length_mm=pipeline.DEFAULT_WING_LENGTH_MM,
        )
        self.assertAlmostEqual(
            implicit["wing_motion_overrides"]["fore"]["phi_mean"],
            explicit["wing_motion_overrides"]["fore"]["phi_mean"],
            places=12,
        )
        self.assertAlmostEqual(
            implicit["wing_motion_overrides"]["hind"]["phi_mean"],
            explicit["wing_motion_overrides"]["hind"]["phi_mean"],
            places=12,
        )

    def test_explicit_wing_length_override_changes_phi_mapping(self):
        default_mapping = pipeline.compute_smoothed_motion_mapping(self.common, self.omega_nd, 7)
        override_mapping = pipeline.compute_smoothed_motion_mapping(
            self.common,
            self.omega_nd,
            7,
            wing_length_mm=55.0,
        )
        self.assertNotAlmostEqual(
            default_mapping["wing_motion_overrides"]["fore"]["phi_mean"],
            override_mapping["wing_motion_overrides"]["fore"]["phi_mean"],
            places=12,
        )


class TestReadAeroForceZ(unittest.TestCase):
    """Verify read_aero_force_z sums lift and drag z-components across all wings."""

    def test_known_forces(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            h5_path = Path(tmpdir) / "test.h5"
            n_steps = 10
            time = np.linspace(0, 1, n_steps)

            with h5py.File(str(h5_path), "w") as f:
                f.create_dataset("/time", data=time)
                # Wing A: lift_z=1, drag_z=2 at each step
                lift_a = np.zeros((n_steps, 3))
                lift_a[:, 2] = 1.0
                drag_a = np.zeros((n_steps, 3))
                drag_a[:, 2] = 2.0
                f.create_dataset("/wings/wing_a/lift", data=lift_a)
                f.create_dataset("/wings/wing_a/drag", data=drag_a)
                # Wing B: lift_z=3, drag_z=4 at each step
                lift_b = np.zeros((n_steps, 3))
                lift_b[:, 2] = 3.0
                drag_b = np.zeros((n_steps, 3))
                drag_b[:, 2] = 4.0
                f.create_dataset("/wings/wing_b/lift", data=lift_b)
                f.create_dataset("/wings/wing_b/drag", data=drag_b)

            t, fz = pipeline.read_aero_force_z(h5_path)
            np.testing.assert_array_equal(t, time)
            np.testing.assert_allclose(fz, 10.0)  # 1+2+3+4 = 10


class TestBuildSimCfg(unittest.TestCase):
    """Verify build_sim_cfg produces valid config with expected fields."""

    def test_tether_and_wing_blocks(self):
        mapping = {
            "method": {"n_harmonics": 3},
            "parameters": {
                "omega": 15.0,
                "n_harmonics": 3,
                "gamma_mean": 0.8,
                "phi_mean": 0.1,
                "psi_mean": 0.5,
            },
            "wing_phase_offsets": {"fore": 0.0, "hind": 0.0},
            "wing_motion_overrides": {
                "fore": {
                    "gamma_mean": 0.9, "gamma_cos": [0.0, 0.0, 0.0], "gamma_sin": [0.0, 0.0, 0.0],
                    "phi_mean": 0.1, "phi_cos": [0.2, 0.0, 0.0], "phi_sin": [0.3, 0.0, 0.0],
                    "psi_mean": 0.5, "psi_cos": [0.4, 0.0, 0.0], "psi_sin": [0.5, 0.0, 0.0],
                },
                "hind": {
                    "gamma_mean": 0.7, "gamma_cos": [0.0, 0.0, 0.0], "gamma_sin": [0.0, 0.0, 0.0],
                    "phi_mean": 0.15, "phi_cos": [0.25, 0.0, 0.0], "phi_sin": [0.35, 0.0, 0.0],
                    "psi_mean": 0.55, "psi_cos": [0.45, 0.0, 0.0], "psi_sin": [0.55, 0.0, 0.0],
                },
            },
        }
        cfg = pipeline.build_sim_cfg(
            mapping,
            n_wingbeats=5,
            steps_per_wingbeat=200,
            output_name="output.h5",
            wing_mu0=0.065,
            wing_lb0=0.8,
            wing_cd0=0.4,
            wing_cl0=1.2,
        )
        self.assertIn("tether = true", cfg)
        self.assertIn("n_wingbeats = 5", cfg)
        self.assertIn("n_harmonics = 3", cfg)
        # 4 wing blocks (fore left, fore right, hind left, hind right)
        self.assertEqual(cfg.count("[[wing]]"), 4)
        self.assertIn("name = fore", cfg)
        self.assertIn("name = hind", cfg)
        self.assertIn("side = left", cfg)
        self.assertIn("side = right", cfg)


if __name__ == "__main__":
    unittest.main()

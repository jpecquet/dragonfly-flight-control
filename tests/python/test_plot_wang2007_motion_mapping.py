import unittest

import numpy as np

from post.plot_wang2007_kinematics import compute_smoothed_kinematics, load_common_module
from post.plot_wang2007_motion_mapping import DEFAULT_WING_LENGTH_MM, compute_motion_mapping_series


class TestComputeMotionMappingSeries(unittest.TestCase):
    def test_shapes_and_expected_gamma_constants(self):
        out = compute_motion_mapping_series(n_components=35, n_wingbeats=5, n_points=128, wing_length_mm=DEFAULT_WING_LENGTH_MM)
        self.assertEqual(out["t"].shape, (128,))
        for key in (
            "phi_fore_deg",
            "phi_hind_deg",
            "psi_fore_deg",
            "psi_hind_deg",
            "gamma_fore_deg",
            "gamma_hind_deg",
        ):
            self.assertEqual(out[key].shape, (128,))

        common = load_common_module()
        self.assertTrue(np.allclose(out["gamma_fore_deg"], np.degrees(float(common.gamma_fore))))
        self.assertTrue(np.allclose(out["gamma_hind_deg"], np.degrees(float(common.gamma_hind))))

    def test_phi_psi_match_formula_from_smoothed_inputs(self):
        smooth = compute_smoothed_kinematics(n_components=35, n_wingbeats=5, n_points=96)
        mapped = compute_motion_mapping_series(n_components=35, n_wingbeats=5, n_points=96, wing_length_mm=DEFAULT_WING_LENGTH_MM)

        r_mm = (2.0 / 3.0) * DEFAULT_WING_LENGTH_MM
        expected_phi_fore = np.degrees(
            np.arcsin(np.clip(smooth["s"]["y_fore"] / r_mm, -1.0, 1.0))
        )
        expected_phi_hind = np.degrees(
            np.arcsin(np.clip(smooth["s"]["y_hind"] / r_mm, -1.0, 1.0))
        )
        expected_psi_fore = np.degrees(np.pi / 2.0 - np.radians(smooth["beta"]["y_fore"]))
        expected_psi_hind = np.degrees(np.pi / 2.0 - np.radians(smooth["beta"]["y_hind"]))

        np.testing.assert_allclose(mapped["phi_fore_deg"], expected_phi_fore, atol=1e-12)
        np.testing.assert_allclose(mapped["phi_hind_deg"], expected_phi_hind, atol=1e-12)
        np.testing.assert_allclose(mapped["psi_fore_deg"], expected_psi_fore, atol=1e-12)
        np.testing.assert_allclose(mapped["psi_hind_deg"], expected_psi_hind, atol=1e-12)


if __name__ == "__main__":
    unittest.main()

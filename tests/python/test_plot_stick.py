import unittest

import numpy as np

from post.plot_stick import compute_stick_endpoints, resolve_right_wings


class TestResolveRightWings(unittest.TestCase):
    def test_prefers_right_side_names(self):
        fore, hind = resolve_right_wings(["fore_left", "fore_right", "hind_left", "hind_right"])
        self.assertEqual(fore, "fore_right")
        self.assertEqual(hind, "hind_right")

    def test_falls_back_to_fore_hind_if_right_not_present(self):
        fore, hind = resolve_right_wings(["fore", "hind"])
        self.assertEqual(fore, "fore")
        self.assertEqual(hind, "hind")

    def test_raises_when_fore_or_hind_missing(self):
        with self.assertRaises(ValueError):
            resolve_right_wings(["mid_right", "hind_right"])


class TestComputeStickEndpoints(unittest.TestCase):
    def test_endpoint_geometry_and_length(self):
        wing_state = {
            "e_r": np.array([0.4, 0.0, -0.3], dtype=float),
            "e_c": np.array([3.0, 0.0, 4.0], dtype=float),  # XZ direction = (0.6, 0.8)
        }
        center, leading, trailing = compute_stick_endpoints(
            wing_state, x_offset=0.1, stick_length=0.1, station=0.5, lambda0=0.8
        )

        np.testing.assert_allclose(center, np.array([0.26, -0.12]), atol=1e-12)
        np.testing.assert_allclose(leading - center, np.array([0.03, 0.04]), atol=1e-12)
        np.testing.assert_allclose(center - trailing, np.array([0.03, 0.04]), atol=1e-12)
        self.assertAlmostEqual(np.linalg.norm(leading - trailing), 0.1, places=12)

    def test_degenerate_chord_direction_uses_default(self):
        wing_state = {
            "e_r": np.array([0.0, 0.0, 0.0], dtype=float),
            "e_c": np.array([0.0, 0.0, 0.0], dtype=float),
        }
        center, leading, trailing = compute_stick_endpoints(
            wing_state, x_offset=0.0, stick_length=0.1, station=2.0 / 3.0, lambda0=0.8
        )

        np.testing.assert_allclose(center, np.array([0.0, 0.0]), atol=1e-12)
        np.testing.assert_allclose(leading, np.array([0.05, 0.0]), atol=1e-12)
        np.testing.assert_allclose(trailing, np.array([-0.05, 0.0]), atol=1e-12)

    def test_station_out_of_bounds_raises(self):
        wing_state = {
            "e_r": np.array([1.0, 0.0, 0.0], dtype=float),
            "e_c": np.array([1.0, 0.0, 0.0], dtype=float),
        }
        with self.assertRaises(ValueError):
            compute_stick_endpoints(wing_state, x_offset=0.0, stick_length=0.1, station=1.1, lambda0=1.0)


if __name__ == "__main__":
    unittest.main()

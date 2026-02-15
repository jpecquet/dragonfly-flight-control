import unittest

import numpy as np

from post.plot_wang2007_kinematics import compute_smoothed_kinematics, harmonics_per_wingbeat


class TestHarmonicsPerWingbeat(unittest.TestCase):
    def test_35_components_over_5_wingbeats(self):
        self.assertEqual(harmonics_per_wingbeat(35, 5), 7)

    def test_invalid_component_count_raises(self):
        with self.assertRaises(ValueError):
            harmonics_per_wingbeat(34, 5)


class TestComputeSmoothedKinematics(unittest.TestCase):
    def test_shapes_and_keys(self):
        out = compute_smoothed_kinematics(n_components=35, n_wingbeats=5, n_points=128)
        self.assertEqual(set(out.keys()), {"s", "d", "beta"})
        for key in ("s", "d", "beta"):
            series = out[key]
            self.assertEqual(set(series.keys()), {"t_fore", "y_fore", "t_hind", "y_hind"})
            self.assertEqual(series["t_fore"].shape, (128,))
            self.assertEqual(series["y_fore"].shape, (128,))
            self.assertEqual(series["t_hind"].shape, (128,))
            self.assertEqual(series["y_hind"].shape, (128,))
            self.assertTrue(np.all(np.diff(series["t_fore"]) >= 0.0))
            self.assertTrue(np.all(np.diff(series["t_hind"]) >= 0.0))


if __name__ == "__main__":
    unittest.main()

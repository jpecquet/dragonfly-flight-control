import unittest

import numpy as np

from post.hybrid_config import ViewportConfig, compute_viewport


class TestHybridViewportConfig(unittest.TestCase):
    def test_compute_viewport_uses_fixed_half_unit_padding(self):
        states = np.array(
            [
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [2.0, 1.0, -1.0, 0.0, 0.0, 0.0],
            ],
            dtype=float,
        )
        viewport = compute_viewport(states)

        np.testing.assert_allclose(viewport.center, np.array([1.0, 0.5, -0.5]), atol=1e-12)
        np.testing.assert_allclose(viewport.extent_xyz, np.array([3.0, 2.0, 2.0]), atol=1e-12)
        self.assertAlmostEqual(viewport.extent, 3.0, places=12)

    def test_from_dict_supports_legacy_scalar_extent(self):
        viewport = ViewportConfig.from_dict({"center": [1.0, 2.0, 3.0], "extent": 4.0})
        np.testing.assert_allclose(viewport.center, np.array([1.0, 2.0, 3.0]), atol=1e-12)
        np.testing.assert_allclose(viewport.extent_xyz, np.array([4.0, 4.0, 4.0]), atol=1e-12)
        payload = viewport.to_dict()
        self.assertEqual(payload["extent"], 4.0)
        self.assertEqual(payload["extent_xyz"], [4.0, 4.0, 4.0])


if __name__ == "__main__":
    unittest.main()

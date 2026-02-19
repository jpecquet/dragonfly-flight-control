import unittest

import numpy as np

from post.wing_geometry import composite_ellipse_polygon_local


class TestWingGeometry(unittest.TestCase):
    def test_composite_ellipse_is_full_span_envelope_and_edge_orientation(self):
        span = 1.0
        root_chord = 2.0
        n_span = 8
        poly = composite_ellipse_polygon_local(span, root_chord, n_span=n_span, span_sign=1.0)

        # Leading side samples are first n_span+1 vertices.
        leading = poly[: n_span + 1]
        trailing = poly[n_span + 1 :]

        # Full-ellipse envelope: zero chord at root and tip, max at mid-span.
        self.assertAlmostEqual(float(leading[0, 0]), 0.0, places=12)
        self.assertAlmostEqual(float(leading[-1, 0]), 0.0, places=12)
        self.assertGreater(float(np.max(leading[:, 0])), 0.0)

        # Leading edge should be on +X, trailing edge on -X.
        mid_idx = n_span // 2
        self.assertGreater(float(leading[mid_idx, 0]), 0.0)
        # Trailing list runs tip->root; corresponding mid sample exists at mirrored index.
        self.assertLess(float(trailing[-(mid_idx + 1), 0]), 0.0)


if __name__ == "__main__":
    unittest.main()

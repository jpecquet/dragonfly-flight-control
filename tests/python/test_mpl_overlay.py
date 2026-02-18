import unittest

import matplotlib.pyplot as plt
import numpy as np

from post.hybrid_config import CameraConfig, StyleConfig, ViewportConfig
from post.mpl_overlay import create_overlay_figure, setup_axes


def _axis_major_step(axis) -> float:
    lo, hi = axis.get_view_interval()
    locator = axis.get_major_locator()
    ticks = np.asarray(locator.tick_values(float(lo), float(hi)), dtype=float)
    diffs = np.diff(ticks)
    diffs = np.abs(diffs[np.abs(diffs) > 1e-12])
    if diffs.size == 0:
        raise AssertionError("No major tick spacing available")
    return float(np.median(diffs))


class TestMplOverlayTicks(unittest.TestCase):
    def test_tick_step_mixed_by_axis_extent(self):
        camera = CameraConfig()
        style = StyleConfig.themed("light")
        viewport = ViewportConfig(
            center=np.zeros(3, dtype=float),
            extent_xyz=np.array([1.4, 2.0, 1.6], dtype=float),
        )

        fig, ax = create_overlay_figure(camera, style)
        setup_axes(ax, viewport, camera, style)

        self.assertAlmostEqual(_axis_major_step(ax.xaxis), 0.5, places=12)
        self.assertAlmostEqual(_axis_major_step(ax.yaxis), 1.0, places=12)
        self.assertAlmostEqual(_axis_major_step(ax.zaxis), 0.5, places=12)
        plt.close(fig)

    def test_tick_step_threshold_at_two_uses_one(self):
        camera = CameraConfig()
        style = StyleConfig.themed("light")
        viewport = ViewportConfig(
            center=np.zeros(3, dtype=float),
            extent_xyz=np.array([2.0, 1.99, 2.01], dtype=float),
        )

        fig, ax = create_overlay_figure(camera, style)
        setup_axes(ax, viewport, camera, style)

        self.assertAlmostEqual(_axis_major_step(ax.xaxis), 1.0, places=12)
        self.assertAlmostEqual(_axis_major_step(ax.yaxis), 0.5, places=12)
        self.assertAlmostEqual(_axis_major_step(ax.zaxis), 1.0, places=12)
        plt.close(fig)


if __name__ == "__main__":
    unittest.main()

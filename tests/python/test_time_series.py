import unittest

import matplotlib.pyplot as plt
import numpy as np

from post.time_series import SeriesSpec, plot_time_series


class TestPlotTimeSeries(unittest.TestCase):
    def test_default_xlim_uses_series_beginning_and_end(self):
        series = [SeriesSpec(x=np.array([2.0, 1.0, 0.0]), y=np.array([0.0, 1.0, 2.0]))]
        fig, ax = plot_time_series(series=series, output_path=None)
        left, right = ax.get_xlim()
        self.assertAlmostEqual(left, 2.0, places=12)
        self.assertAlmostEqual(right, 0.0, places=12)
        plt.close(fig)

    def test_explicit_xlim_overrides_default(self):
        series = [SeriesSpec(x=np.array([0.0, 1.0, 2.0]), y=np.array([0.0, 1.0, 2.0]))]
        fig, ax = plot_time_series(series=series, output_path=None, xlim=(-1.0, 3.0))
        left, right = ax.get_xlim()
        self.assertAlmostEqual(left, -1.0, places=12)
        self.assertAlmostEqual(right, 3.0, places=12)
        plt.close(fig)


if __name__ == "__main__":
    unittest.main()

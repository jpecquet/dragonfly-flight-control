import unittest

import matplotlib.pyplot as plt

from post.style import apply_matplotlib_style, resolve_style


class TestMatplotlibColorCycle(unittest.TestCase):
    def test_cycle_starts_blue_red_green_light(self):
        apply_matplotlib_style()
        colors = plt.rcParams["axes.prop_cycle"].by_key()["color"]
        self.assertGreaterEqual(len(colors), 3)
        self.assertEqual(colors[0], "#1f77b4")
        self.assertEqual(colors[1], "#d62728")
        self.assertEqual(colors[2], "#2ca02c")

    def test_cycle_starts_blue_red_green_dark(self):
        apply_matplotlib_style(resolve_style(theme="dark"))
        colors = plt.rcParams["axes.prop_cycle"].by_key()["color"]
        self.assertGreaterEqual(len(colors), 3)
        self.assertEqual(colors[0], "#4aa3ff")
        self.assertEqual(colors[1], "#ff6b6b")
        self.assertEqual(colors[2], "#56d68b")


if __name__ == "__main__":
    unittest.main()

import unittest

from cases.azuma1988.plot_exp1_force_segments import (
    VelocitySpec,
    parse_segments,
    prepare_sim_config,
)


BASE_CFG = """# demo
tether = false
n_blade_elements = 20
n_wingbeats = 5
ux0 = 0.1
uy0 = 0.0
uz0 = -0.2
output = output.h5

[[wing]]
name = fore
psi_twist_h1_root_deg = 9.0
psi_twist_ref_eta = 0.75
"""


class TestParseSegments(unittest.TestCase):
    def test_parse_segments_deduplicates_and_preserves_order(self):
        self.assertEqual(parse_segments("1, 5,5, 20"), (1, 5, 20))

    def test_parse_segments_rejects_nonpositive(self):
        with self.assertRaises(ValueError):
            parse_segments("1,0,5")


class TestPrepareSimConfig(unittest.TestCase):
    def test_prepare_sim_config_with_twist(self):
        velocity = VelocitySpec(0.7, -12.0, 0.8, 0.0, -0.16)
        text = prepare_sim_config(
            BASE_CFG,
            n_blade_elements=10,
            enable_twist=True,
            output_h5_path="/tmp/a.h5",
            velocity=velocity,
        )
        self.assertIn("tether = true", text)
        self.assertIn("n_blade_elements = 10", text)
        self.assertIn("n_wingbeats = 1", text)
        self.assertIn("output = /tmp/a.h5", text)
        self.assertIn("ux0 = 0.800000000000", text)
        self.assertIn("uz0 = -0.160000000000", text)
        self.assertIn("psi_twist_h1_root_deg = 9.0", text)
        self.assertIn("psi_twist_ref_eta = 0.75", text)

    def test_prepare_sim_config_without_twist_removes_lines(self):
        velocity = VelocitySpec(0.7, -12.0, 0.8, 0.0, -0.16)
        text = prepare_sim_config(
            BASE_CFG,
            n_blade_elements=5,
            enable_twist=False,
            output_h5_path="/tmp/b.h5",
            velocity=velocity,
        )
        self.assertNotIn("psi_twist_h1_root_deg", text)
        self.assertNotIn("psi_twist_ref_eta", text)


if __name__ == "__main__":
    unittest.main()

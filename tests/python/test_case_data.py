import unittest

from scripts.case_data import find_output_reference, load_case_data, select_experiment


class TestCaseData(unittest.TestCase):
    def test_load_azuma1985_case(self):
        case = load_case_data("azuma1985")
        self.assertEqual(case["schema_version"], 1)
        self.assertEqual(case["kinematics"]["kind"], "fourier_series_deg")
        self.assertAlmostEqual(float(case["specimen"]["body_length_m"]), 4.0e-2, places=12)

    def test_load_wang2007_case(self):
        case = load_case_data("wang2007")
        self.assertEqual(case["kinematics"]["kind"], "timeseries_csv")
        self.assertEqual(int(case["simulation_defaults"]["n_wingbeats"]), 5)

    def test_find_flight_reference(self):
        case = load_case_data("azuma1988")
        ref = find_output_reference(
            case,
            kind="flight_condition",
            name="body_speed_and_direction",
        )
        self.assertAlmostEqual(float(ref["speed_m_s"]), 0.7, places=12)
        self.assertAlmostEqual(float(ref["direction_deg"]), -12.0, places=12)

    def test_select_azuma1988_experiment(self):
        case = load_case_data("azuma1988")
        exp4 = select_experiment(case, experiment_id="4")
        self.assertEqual(exp4["selected_experiment"]["id"], "4")
        self.assertAlmostEqual(float(exp4["simulation_defaults"]["frequency_hz"]), 27.0, places=12)
        self.assertAlmostEqual(float(exp4["kinematics"]["angles"]["fore"]["gamma"]["mean_deg"]), 63.0, places=12)

        ref = find_output_reference(
            exp4,
            kind="flight_condition",
            name="body_speed_and_direction",
        )
        self.assertAlmostEqual(float(ref["speed_m_s"]), 3.2, places=12)
        self.assertAlmostEqual(float(ref["direction_deg"]), 0.0, places=12)


if __name__ == "__main__":
    unittest.main()

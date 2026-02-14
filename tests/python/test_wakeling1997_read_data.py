import importlib.util
import sys
import unittest
from pathlib import Path

import numpy as np
import pandas as pd
from unittest.mock import patch


REPO_ROOT = Path(__file__).resolve().parents[2]


def load_module(relpath: str, module_name: str):
    module_path = REPO_ROOT / relpath
    spec = importlib.util.spec_from_file_location(module_name, module_path)
    module = importlib.util.module_from_spec(spec)
    assert spec is not None and spec.loader is not None
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


wakeling = load_module(
    "data/morphology/wakeling1997/read_data.py",
    "wakeling1997_read_data_test",
)


class FakeSpecimen:
    def __init__(self, m, L, R_f, S_f, R_h, S_h):
        self.m = m
        self.L = L
        self.R_f = R_f
        self.S_f = S_f
        self.R_h = R_h
        self.S_h = S_h


class TestWakelingFits(unittest.TestCase):
    def test_read_wakeling1997_requires_fore_hind_body_id_intersection(self):
        fore_df = pd.DataFrame({
            "ID": ["AB1", "AB2", "CS1"],
            "R": [40.0, 41.0, 39.0],
            "S": [100.0, 101.0, 99.0],
        })
        hind_df = pd.DataFrame({
            "ID": ["AB1", "CS1"],
            "R": [35.0, 34.0],
            "S": [90.0, 89.0],
        })
        body_df = pd.DataFrame({
            "ID": ["AB1", "AB2", "CS1"],
            "species": ["sp1", "sp2", "sp3"],
            "sex": ["F", "M", "F"],
            "m": [500.0, 510.0, 490.0],
            "L": [60.0, 61.0, 59.0],
        })

        with patch.object(wakeling.pd, "read_csv", side_effect=[fore_df, hind_df, body_df]):
            specimens = wakeling.read_wakeling1997("fore.csv", "hind.csv", "body.csv")

        self.assertEqual(len(specimens), 1)
        self.assertEqual(specimens[0].ID, "AB1")

    def test_collect_fit_samples_filters_fore_and_hind_independently(self):
        specimens = [
            FakeSpecimen(1e-3, 50e-3, 40e-3, 1.0e-3, 35e-3, 0.9e-3),
            FakeSpecimen(2e-3, 60e-3, 50e-3, 2.0e-3, np.nan, np.nan),
            FakeSpecimen(3e-3, 70e-3, np.nan, np.nan, 45e-3, 1.5e-3),
        ]
        samples = wakeling.collect_fit_samples(specimens)

        np.testing.assert_allclose(samples["fore_m_g"], [1.0, 2.0])
        np.testing.assert_allclose(samples["hind_m_g"], [1.0, 3.0])
        np.testing.assert_allclose(samples["fore_Lb_mm"], [50.0, 60.0])
        np.testing.assert_allclose(samples["hind_Lb_mm"], [50.0, 70.0])
        np.testing.assert_allclose(samples["fore_mair_g"], [0.0482, 0.1205], rtol=1e-12)
        np.testing.assert_allclose(samples["hind_mair_g"], [0.0379575, 0.0813375], rtol=1e-12)

    def test_fit_wakeling_trends_uses_expected_models_and_units(self):
        calls = []

        def fake_curve_fit(fn, x, y):
            calls.append((fn.__name__, np.array(x, copy=True), np.array(y, copy=True)))
            return np.array([0.5]), np.zeros((1, 1))

        samples = {
            "fore_m_g": np.array([0.4, 0.8]),
            "fore_mair_g": np.array([0.1, 0.2]),
            "fore_Lb_mm": np.array([40.0, 80.0]),
            "fore_R_mm": np.array([28.0, 56.0]),
            "hind_m_g": np.array([0.5, 1.0]),
            "hind_mair_g": np.array([0.12, 0.24]),
            "hind_Lb_mm": np.array([42.0, 84.0]),
            "hind_R_mm": np.array([30.0, 60.0]),
        }

        fits = wakeling.fit_wakeling_trends(samples, curve_fit_fn=fake_curve_fit)

        self.assertEqual([c[0] for c in calls], ["mair", "mair", "R", "R"])
        np.testing.assert_allclose(calls[2][1], samples["fore_Lb_mm"])
        np.testing.assert_allclose(calls[3][1], samples["hind_Lb_mm"])
        self.assertSetEqual(
            set(fits.keys()),
            {"mu0_fore", "mu0_hind", "lambda0_fore", "lambda0_hind"},
        )


if __name__ == "__main__":
    unittest.main()

import importlib.util
import json
import math
import sys
import tempfile
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPTS_DIR = REPO_ROOT / "scripts"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))


def load_module(relpath: str, module_name: str):
    module_path = REPO_ROOT / relpath
    spec = importlib.util.spec_from_file_location(module_name, module_path)
    module = importlib.util.module_from_spec(spec)
    assert spec is not None and spec.loader is not None
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


conventions = load_module("scripts/experimental_conventions.py", "experimental_conventions_azuma1988_test")
pipeline = load_module("scripts/azuma1988_pipeline.py", "azuma1988_pipeline_test")


class TestAzuma1988Experiments(unittest.TestCase):
    @staticmethod
    def _params_for_experiment(experiment_id: str) -> "pipeline.PipelineParams":
        exp_case = pipeline.resolve_experiment_case(experiment_id)
        specimen = exp_case["specimen"]
        defaults = exp_case["simulation_defaults"]
        return pipeline.PipelineParams(
            n_wingbeats=1,
            steps_per_wingbeat=10,
            tether=False,
            output_name="output.h5",
            wing_cd0=float(defaults["wing_cd0"]),
            wing_cl0=float(defaults["wing_cl0"]),
            body_length_m=float(specimen["body_length_m"]),
            body_mass_kg=float(specimen["body_mass_kg"]),
            fore_span_m=float(specimen["fore_span_m"]),
            fore_area_m2=float(specimen["fore_area_m2"]),
            hind_span_m=float(specimen["hind_span_m"]),
            hind_area_m2=float(specimen["hind_area_m2"]),
            frequency_hz=float(defaults["frequency_hz"]),
            stroke_plane_deg=None,
            fore_stroke_plane_deg=None,
            hind_stroke_plane_deg=None,
            rho_air=float(defaults["rho_air_kg_m3"]),
            gravity=float(defaults["gravity_m_s2"]),
        )

    def test_adapter_reads_experiment_specific_coefficients(self):
        adapter = conventions.azuma1988_adapter(experiment="2")
        fore = adapter.source_series["fore"]
        hind = adapter.source_series["hind"]
        self.assertEqual(fore.gamma.mean_deg, 55.0)
        self.assertEqual(hind.gamma.mean_deg, 48.0)
        self.assertEqual(fore.psi.mean_deg, 88.0)
        self.assertEqual(hind.psi.mean_deg, 81.0)

    def test_translate_summary_captures_selected_experiment(self):
        params = self._params_for_experiment("3")
        with tempfile.TemporaryDirectory() as td:
            run_dir = Path(td)
            pipeline.stage_translate(run_dir=run_dir, params=params, experiment_id="3")
            summary = json.loads((run_dir / "sim" / "translate_summary.json").read_text(encoding="utf-8"))
            cfg_text = (run_dir / "sim" / "sim_azuma1988.cfg").read_text(encoding="utf-8")

        self.assertEqual(summary["selected_experiment"]["id"], "3")
        mapping = summary["convention_mapping"]
        self.assertEqual(mapping["dataset_id"], "azuma1988:exp3")
        self.assertEqual(mapping["resolved_stroke_plane_deg"]["fore"], 58.0)
        self.assertEqual(mapping["resolved_stroke_plane_deg"]["hind"], 52.0)
        init_vel = summary["initial_velocity"]
        self.assertAlmostEqual(float(init_vel["experiment_reference"]["speed_m_s"]), 2.3, places=12)
        self.assertAlmostEqual(float(init_vel["experiment_reference"]["direction_deg"]), 4.8, places=12)
        self.assertGreater(float(init_vel["nondimensional"]["ux0"]), 0.0)
        self.assertGreater(float(init_vel["nondimensional"]["uz0"]), 0.0)
        self.assertIn("ux0 = ", cfg_text)
        self.assertIn("uz0 = ", cfg_text)

    def test_initial_velocity_uses_experimental_speed_and_direction(self):
        params = self._params_for_experiment("2")
        with tempfile.TemporaryDirectory() as td:
            run_dir = Path(td)
            pipeline.stage_translate(run_dir=run_dir, params=params, experiment_id="2")
            summary = json.loads((run_dir / "sim" / "translate_summary.json").read_text(encoding="utf-8"))

        init_vel = summary["initial_velocity"]["nondimensional"]
        speed_scale = math.sqrt(params.gravity * params.body_length_m)
        speed_nd = 1.5 / speed_scale
        angle = math.radians(-1.1)
        self.assertAlmostEqual(float(init_vel["ux0"]), speed_nd * math.cos(angle), places=12)
        self.assertAlmostEqual(float(init_vel["uy0"]), 0.0, places=12)
        self.assertAlmostEqual(float(init_vel["uz0"]), speed_nd * math.sin(angle), places=12)

    def test_experiment1_enables_linear_pitch_twist_model(self):
        params = self._params_for_experiment("1")
        with tempfile.TemporaryDirectory() as td:
            run_dir = Path(td)
            pipeline.stage_translate(run_dir=run_dir, params=params, experiment_id="1")
            summary = json.loads((run_dir / "sim" / "translate_summary.json").read_text(encoding="utf-8"))
            cfg_text = (run_dir / "sim" / "sim_azuma1988.cfg").read_text(encoding="utf-8")

        twist = summary["convention_mapping"]["pitch_twist_model"]
        self.assertTrue(bool(twist["enabled"]))
        self.assertAlmostEqual(float(twist["ref_eta"]), 0.75, places=12)
        self.assertAlmostEqual(float(twist["root_coeff_deg"]["fore"]), 9.0, places=12)
        self.assertAlmostEqual(float(twist["root_coeff_deg"]["hind"]), 9.0, places=12)
        self.assertIn("n_blade_elements = 5", cfg_text)
        self.assertEqual(cfg_text.count("psi_twist_h1_root_deg = 9.000000000000"), 4)
        self.assertEqual(cfg_text.count("psi_twist_ref_eta = 0.750000000000"), 4)

    def test_non_experiment1_disables_linear_pitch_twist_model(self):
        params = self._params_for_experiment("2")
        with tempfile.TemporaryDirectory() as td:
            run_dir = Path(td)
            pipeline.stage_translate(run_dir=run_dir, params=params, experiment_id="2")
            summary = json.loads((run_dir / "sim" / "translate_summary.json").read_text(encoding="utf-8"))
            cfg_text = (run_dir / "sim" / "sim_azuma1988.cfg").read_text(encoding="utf-8")

        twist = summary["convention_mapping"]["pitch_twist_model"]
        self.assertFalse(bool(twist["enabled"]))
        self.assertIn("n_blade_elements = 5", cfg_text)
        self.assertNotIn("psi_twist_h1_root_deg", cfg_text)
        self.assertNotIn("psi_twist_ref_eta", cfg_text)

    def test_experiment4_enables_case_driven_pitch_twist_model(self):
        params = self._params_for_experiment("4")
        with tempfile.TemporaryDirectory() as td:
            run_dir = Path(td)
            pipeline.stage_translate(run_dir=run_dir, params=params, experiment_id="4")
            summary = json.loads((run_dir / "sim" / "translate_summary.json").read_text(encoding="utf-8"))
            cfg_text = (run_dir / "sim" / "sim_azuma1988.cfg").read_text(encoding="utf-8")

        twist = summary["convention_mapping"]["pitch_twist_model"]
        self.assertTrue(bool(twist["enabled"]))
        self.assertAlmostEqual(float(twist["ref_eta"]), 0.75, places=12)
        self.assertAlmostEqual(float(twist["root_coeff_deg"]["fore"]), 0.0, places=12)
        self.assertAlmostEqual(float(twist["root_coeff_deg"]["hind"]), 12.0, places=12)
        self.assertIn("n_blade_elements = 5", cfg_text)
        self.assertEqual(cfg_text.count("psi_twist_h1_root_deg = 0.000000000000"), 2)
        self.assertEqual(cfg_text.count("psi_twist_h1_root_deg = 12.000000000000"), 2)
        self.assertEqual(cfg_text.count("psi_twist_ref_eta = 0.750000000000"), 4)


if __name__ == "__main__":
    unittest.main()

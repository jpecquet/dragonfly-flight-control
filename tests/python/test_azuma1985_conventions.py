import importlib.util
import json
import math
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch


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


conventions = load_module("scripts/experimental_conventions.py", "experimental_conventions_test")
pipeline = load_module("scripts/azuma1985_pipeline.py", "azuma1985_pipeline_test")


class TestAzumaConventionAdapter(unittest.TestCase):
    @staticmethod
    def _default_params() -> "pipeline.PipelineParams":
        return pipeline.PipelineParams(
            n_wingbeats=1,
            steps_per_wingbeat=10,
            tether=False,
            output_name="output.h5",
            wing_cd0=0.4,
            wing_cl0=1.2,
            body_length_m=4.0e-2,
            body_mass_kg=2.6e-4,
            fore_span_m=3.35e-2,
            fore_area_m2=2.21e-4,
            hind_span_m=3.25e-2,
            hind_area_m2=2.72e-4,
            frequency_hz=41.5,
            stroke_plane_deg=None,
            fore_stroke_plane_deg=None,
            hind_stroke_plane_deg=None,
            rho_air=1.225,
            gravity=9.81,
        )

    def test_adapter_defaults_match_case_study_doc(self):
        adapter = conventions.azuma1985_adapter()
        self.assertEqual(adapter.source_series["fore"].gamma.mean_deg, 37.0)
        self.assertEqual(adapter.source_series["hind"].gamma.mean_deg, 40.0)
        self.assertEqual(adapter.source_series["fore"].phi.mean_deg, -3.0)
        self.assertEqual(adapter.source_series["hind"].phi.mean_deg, 2.0)

    def test_mapping_preserves_gamma_and_flips_phi_sign(self):
        adapter = conventions.azuma1985_adapter()
        sim_motion = conventions.build_sim_wing_motion(adapter, n_harmonics=3)
        self.assertAlmostEqual(sim_motion["fore"].gamma_mean, math.radians(37.0), places=12)
        self.assertAlmostEqual(sim_motion["hind"].gamma_mean, math.radians(40.0), places=12)
        self.assertAlmostEqual(sim_motion["fore"].phi_mean, math.radians(3.0), places=12)
        self.assertAlmostEqual(sim_motion["hind"].phi_mean, math.radians(-2.0), places=12)
        self.assertAlmostEqual(sim_motion["fore"].psi_mean, math.radians(8.0), places=12)
        self.assertAlmostEqual(sim_motion["hind"].psi_mean, math.radians(3.0), places=12)

    def test_pitch_mapping_regression_sign_is_theta_minus_90(self):
        adapter = conventions.azuma1985_adapter()
        source_fore_psi = adapter.source_series["fore"].psi.eval_deg(0.25)
        sim_fore_psi = adapter.sim_series()["fore"].psi.eval_deg(0.25)
        self.assertAlmostEqual(sim_fore_psi, source_fore_psi - 90.0, places=12)

    def test_stage_translate_writes_convention_summary(self):
        params = self._default_params()
        with tempfile.TemporaryDirectory() as td:
            run_dir = Path(td)
            pipeline.stage_translate(run_dir=run_dir, params=params)
            summary = json.loads((run_dir / "sim" / "translate_summary.json").read_text(encoding="utf-8"))
        mapping = summary["convention_mapping"]
        self.assertEqual(mapping["resolved_stroke_plane_deg"]["fore"], 37.0)
        self.assertEqual(mapping["resolved_stroke_plane_deg"]["hind"], 40.0)
        self.assertEqual(mapping["source_world_axes"]["X"], "backward")
        self.assertEqual(mapping["source_world_axes"]["Y"], "right")

    def test_override_precedence(self):
        _, mapping = pipeline.azuma_wing_motion(
            n_harmonics=3,
            stroke_plane_deg=41.0,
            fore_stroke_plane_deg=36.0,
            hind_stroke_plane_deg=None,
        )
        self.assertEqual(mapping["resolved_stroke_plane_deg"]["fore"], 36.0)
        self.assertEqual(mapping["resolved_stroke_plane_deg"]["hind"], 41.0)

    def test_stage_sim_always_refreshes_translation(self):
        params = self._default_params()
        with tempfile.TemporaryDirectory() as td:
            run_dir = Path(td)
            sim_dir = run_dir / "sim"
            sim_dir.mkdir(parents=True, exist_ok=True)
            cfg_path = sim_dir / "sim_azuma1985.cfg"
            output_h5 = sim_dir / params.output_name
            cfg_path.write_text("# stale", encoding="utf-8")
            output_h5.write_bytes(b"")
            with patch.object(pipeline, "stage_translate", return_value=(cfg_path, output_h5)) as stage_translate:
                with patch.object(pipeline, "run_cmd") as run_cmd:
                    pipeline.stage_sim(run_dir=run_dir, binary="/bin/echo", params=params)
        stage_translate.assert_called_once_with(run_dir=run_dir, params=params)
        run_cmd.assert_called_once()

    def test_stage_post_returns_all_artifacts(self):
        params = self._default_params()
        with tempfile.TemporaryDirectory() as td:
            run_dir = Path(td)
            sim_dir = run_dir / "sim"
            sim_dir.mkdir(parents=True, exist_ok=True)
            h5_path = sim_dir / params.output_name
            h5_path.write_bytes(b"")

            def fake_run_cmd(cmd, cwd=None, env=None):
                output_path = Path(cmd[4])
                output_path.parent.mkdir(parents=True, exist_ok=True)
                output_path.touch()

            with patch.object(pipeline, "run_cmd", side_effect=fake_run_cmd):
                with patch.object(pipeline, "update_manifest") as update_manifest:
                    artifacts = pipeline.stage_post(
                        run_dir=run_dir,
                        params=params,
                        binary="/bin/echo",
                        input_h5=str(h5_path),
                        no_blender=True,
                        frame_step=1,
                    )

        self.assertEqual([p.name for p in artifacts], ["simulation.mp4", "flight_metrics.png"])
        update_manifest.assert_called_once()


if __name__ == "__main__":
    unittest.main()

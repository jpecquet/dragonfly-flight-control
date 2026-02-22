import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]


def load_module(relpath: str, module_name: str):
    module_path = REPO_ROOT / relpath
    spec = importlib.util.spec_from_file_location(module_name, module_path)
    module = importlib.util.module_from_spec(spec)
    assert spec is not None and spec.loader is not None
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


docs_media_config = load_module("scripts/docs_media_config.py", "docs_media_config_test")


class TestDocsMediaConfigValidation(unittest.TestCase):
    def test_current_azuma1985_post_yaml_validates(self):
        cfg = docs_media_config.load_post_config(REPO_ROOT / "cases" / "azuma1985" / "post.yaml")
        self.assertEqual(cfg["version"], 1)
        self.assertEqual(cfg["simulation"]["driver"], "yaml_case")
        self.assertGreaterEqual(len(cfg["artifacts"]), 1)

    def test_all_migrated_post_yamls_validate(self):
        case_dirs = [
            "azuma1985", "azuma1988_exp1", "azuma1988_exp2",
            "azuma1988_exp3", "azuma1988_exp4", "modeling_wing_kinematics", "wang2007",
        ]
        for case_dir in case_dirs:
            path = REPO_ROOT / "cases" / case_dir / "post.yaml"
            cfg = docs_media_config.load_post_config(path)
            self.assertEqual(cfg["version"], 1, f"{case_dir}: bad version")
            self.assertGreaterEqual(len(cfg["artifacts"]), 1, f"{case_dir}: no artifacts")

    def test_rejects_unknown_artifact_kind(self):
        bad = {
            "version": 1,
            "simulation": {
                "driver": "yaml_case",
                "case_file": "cases/azuma1985_test/case.yaml",
                "run_dir": "docs/validation/azuma1985_test/artifacts",
            },
            "docs_media": {
                "media_dir": "docs/_static/media/azuma1985_test",
                "themes": ["light", "dark"],
            },
            "artifacts": [
                {
                    "kind": "not_a_real_kind",
                    "output": "x.png",
                }
            ],
        }
        with self.assertRaises(ValueError) as ctx:
            docs_media_config.validate_post_config(bad)
        self.assertIn("Unsupported artifact kind", str(ctx.exception))

    def test_rejects_invalid_theme(self):
        bad = {
            "version": 1,
            "simulation": {
                "driver": "yaml_case",
                "case_file": "cases/azuma1985_test/case.yaml",
                "run_dir": "docs/validation/azuma1985_test/artifacts",
            },
            "docs_media": {
                "media_dir": "docs/_static/media/azuma1985_test",
                "themes": ["sepia"],
            },
            "artifacts": [
                {
                    "kind": "stick_video",
                    "input_h5": "{run_dir}/output.h5",
                    "output": "stick.{theme}.mp4",
                }
            ],
        }
        with self.assertRaises(ValueError) as ctx:
            docs_media_config.validate_post_config(bad)
        self.assertIn("Unsupported theme", str(ctx.exception))

    def test_reports_file_context_on_load(self):
        bad_yaml = """\
version: 1
simulation:
  driver: yaml_case
  case_file: cases/azuma1985_test/case.yaml
  run_dir: docs/validation/azuma1985_test/artifacts
docs_media:
  media_dir: docs/_static/media/azuma1985_test
artifacts:
  - kind: simulation_video
    input_h5: "{run_dir}/output.h5"
    output: simulation.{theme}.mp4
"""
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "post.yaml"
            path.write_text(bad_yaml, encoding="utf-8")
            with self.assertRaises(ValueError) as ctx:
                docs_media_config.load_post_config(path)

        self.assertIn("render_config", str(ctx.exception))
        self.assertIn("post.yaml", str(ctx.exception))


if __name__ == "__main__":
    unittest.main()

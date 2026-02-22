import contextlib
import io
import importlib.util
import sys
import tempfile
import unittest
from unittest import mock
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


docs_media_runner = load_module("scripts/docs_media_runner.py", "docs_media_runner_test")


class TestDocsMediaRunnerConfigChecks(unittest.TestCase):
    def test_find_post_configs_includes_migrated_cases(self):
        paths = docs_media_runner.find_post_configs(REPO_ROOT)
        rels = {str(p.relative_to(REPO_ROOT)) for p in paths}
        expected = [
            "cases/azuma1985/post.yaml",
            "cases/azuma1988_exp1/post.yaml",
            "cases/azuma1988_exp2/post.yaml",
            "cases/azuma1988_exp3/post.yaml",
            "cases/azuma1988_exp4/post.yaml",
            "cases/modeling_wing_kinematics/post.yaml",
            "cases/wang2007/post.yaml",
        ]
        for path in expected:
            self.assertIn(path, rels, f"Missing post.yaml: {path}")

    def test_check_post_configs_returns_nonzero_on_invalid(self):
        bad_yaml = """\
version: 1
simulation:
  driver: yaml_case
  case_file: cases/azuma1985_test/case.yaml
  run_dir: docs/validation/azuma1985_test/artifacts
docs_media:
  media_dir: docs/_static/media/azuma1985_test
artifacts:
  - kind: unknown_kind
    output: x
"""
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "post.yaml"
            path.write_text(bad_yaml, encoding="utf-8")
            rc = docs_media_runner.check_post_configs([path])
        self.assertEqual(rc, 1)

    def test_check_post_configs_accepts_valid_temp_config_outside_repo(self):
        good_yaml = """\
version: 1
simulation:
  driver: yaml_case
  case_file: cases/azuma1985/case.yaml
  run_dir: docs/validation/azuma1985/artifacts
docs_media:
  media_dir: docs/_static/media/azuma1985
  render_config: docs/validation/azuma1985/render_config.dark.json
artifacts:
  - kind: simulation_video
    input_h5: "{run_dir}/output.h5"
    output: simulation.{theme}.mp4
"""
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "post.yaml"
            path.write_text(good_yaml, encoding="utf-8")
            rc = docs_media_runner.check_post_configs([path])
        self.assertEqual(rc, 0)

    def test_select_post_configs_supports_prefix_filter(self):
        paths = docs_media_runner.find_post_configs(REPO_ROOT)
        selected = docs_media_runner.select_post_configs(paths, ["azuma1988"])
        ids = [docs_media_runner.post_config_case_id(p) for p in selected]
        self.assertEqual(ids, ["azuma1988_exp1", "azuma1988_exp2", "azuma1988_exp3", "azuma1988_exp4"])

    def test_main_list_supports_only_filter(self):
        argv = ["docs_media_runner.py", "--list", "--only", "azuma1988"]
        out = io.StringIO()
        with mock.patch.object(sys, "argv", argv), contextlib.redirect_stdout(out):
            rc = docs_media_runner.main()
        self.assertEqual(rc, 0)
        text = out.getvalue()
        self.assertIn("azuma1988_exp1:", text)
        self.assertIn("azuma1988_exp4:", text)
        self.assertNotIn("wang2007:", text)

    def test_main_run_all_dispatches_selected_configs(self):
        called: list[str] = []

        def _fake_run(path, **kwargs):
            called.append(str(Path(path).relative_to(REPO_ROOT)))

        argv = ["docs_media_runner.py", "--run-all", "--only", "azuma1988"]
        with mock.patch.object(sys, "argv", argv), mock.patch.object(
            docs_media_runner, "run_docs_media_config", side_effect=_fake_run
        ):
            rc = docs_media_runner.main()

        self.assertEqual(rc, 0)
        self.assertEqual(
            called,
            [
                "cases/azuma1988_exp1/post.yaml",
                "cases/azuma1988_exp2/post.yaml",
                "cases/azuma1988_exp3/post.yaml",
                "cases/azuma1988_exp4/post.yaml",
            ],
        )


if __name__ == "__main__":
    unittest.main()

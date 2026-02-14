import importlib.util
import sys
import unittest
from pathlib import Path
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


pipeline = load_module("scripts/wang2007_pipeline.py", "wang2007_pipeline_test")


class TestSkipStickForwarding(unittest.TestCase):
    def _run_main(self, argv: list[str]) -> int:
        run_dir = Path("/tmp/wang2007_pipeline_test")
        with patch.object(sys, "argv", argv):
            with patch.object(pipeline, "resolve_run_dir", return_value=run_dir):
                with patch.object(pipeline, "ensure_dir", return_value=run_dir):
                    return pipeline.main()

    def test_post_default_forwards_skip_stick_false(self):
        with patch.object(pipeline, "stage_post", return_value=[]) as stage_post:
            rc = self._run_main(["wang2007_pipeline.py", "post"])
        self.assertEqual(rc, 0)
        self.assertFalse(stage_post.call_args.kwargs["skip_stick"])

    def test_post_flag_forwards_skip_stick_true(self):
        with patch.object(pipeline, "stage_post", return_value=[]) as stage_post:
            rc = self._run_main(["wang2007_pipeline.py", "post", "--skip-stick"])
        self.assertEqual(rc, 0)
        self.assertTrue(stage_post.call_args.kwargs["skip_stick"])

    def test_all_default_forwards_skip_stick_false(self):
        with patch.object(pipeline, "stage_fit", return_value=Path("fit.json")):
            with patch.object(pipeline, "stage_translate", return_value=(Path("cfg"), Path("output.h5"))):
                with patch.object(pipeline, "stage_sim", return_value=Path("output.h5")):
                    with patch.object(pipeline, "stage_post", return_value=[]) as stage_post:
                        rc = self._run_main(["wang2007_pipeline.py", "all"])
        self.assertEqual(rc, 0)
        self.assertFalse(stage_post.call_args.kwargs["skip_stick"])

    def test_all_flag_forwards_skip_stick_true(self):
        with patch.object(pipeline, "stage_fit", return_value=Path("fit.json")):
            with patch.object(pipeline, "stage_translate", return_value=(Path("cfg"), Path("output.h5"))):
                with patch.object(pipeline, "stage_sim", return_value=Path("output.h5")):
                    with patch.object(pipeline, "stage_post", return_value=[]) as stage_post:
                        rc = self._run_main(["wang2007_pipeline.py", "all", "--skip-stick"])
        self.assertEqual(rc, 0)
        self.assertTrue(stage_post.call_args.kwargs["skip_stick"])


if __name__ == "__main__":
    unittest.main()

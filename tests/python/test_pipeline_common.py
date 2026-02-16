import importlib.util
import json
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


pipeline_common = load_module("scripts/pipeline_common.py", "pipeline_common_test")


class TestPipelineCommonManifest(unittest.TestCase):
    def test_update_manifest_deterministic_uses_repo_relative_paths(self):
        with tempfile.TemporaryDirectory(dir=REPO_ROOT) as tmpdir:
            run_dir = Path(tmpdir)
            artifact = run_dir / "sim" / "output.h5"
            artifact.parent.mkdir(parents=True, exist_ok=True)
            artifact.write_bytes(b"")

            pipeline_common.update_manifest(
                run_dir=run_dir,
                stage="sim",
                artifacts=[artifact],
                metadata={"output_h5": str(artifact.resolve())},
                repo_root=REPO_ROOT,
                deterministic=True,
            )

            manifest_path = run_dir / "manifest.json"
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))

        self.assertNotIn("created_at_utc", manifest)
        self.assertNotIn("updated_at_utc", manifest)
        stage = manifest["stages"]["sim"]
        self.assertNotIn("completed_at_utc", stage)
        self.assertFalse(stage["artifacts"][0].startswith("/"))
        self.assertFalse(stage["metadata"]["output_h5"].startswith("/"))

    def test_update_manifest_nondeterministic_keeps_absolute_paths_and_timestamps(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            run_dir = Path(tmpdir)
            artifact = run_dir / "sim" / "output.h5"
            artifact.parent.mkdir(parents=True, exist_ok=True)
            artifact.write_bytes(b"")

            pipeline_common.update_manifest(
                run_dir=run_dir,
                stage="sim",
                artifacts=[artifact],
                metadata={"output_h5": str(artifact.resolve())},
                repo_root=REPO_ROOT,
                deterministic=False,
            )

            manifest_path = run_dir / "manifest.json"
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))

        self.assertIn("created_at_utc", manifest)
        self.assertIn("updated_at_utc", manifest)
        stage = manifest["stages"]["sim"]
        self.assertIn("completed_at_utc", stage)
        self.assertTrue(stage["artifacts"][0].startswith("/"))
        self.assertTrue(stage["metadata"]["output_h5"].startswith("/"))


if __name__ == "__main__":
    unittest.main()

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


update_docs_media = load_module("scripts/update_docs_media.py", "update_docs_media_test")


class TestUpdateDocsMediaRegistryValidation(unittest.TestCase):
    def test_current_registry_validates(self):
        registry = update_docs_media.load_registry(REPO_ROOT / "docs" / "media_registry.json")
        self.assertIn("entries", registry)
        self.assertGreater(len(registry["entries"]), 0)

    def test_invalid_sync_item_reports_entry_context(self):
        bad_registry = {
            "entries": [
                {
                    "id": "broken_entry",
                    "commands": [["python", "--version"]],
                    "outputs": ["docs/_build/out.txt"],
                    "sync": [{"from": "a.txt", "to": 123}],
                }
            ]
        }
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "registry.json"
            path.write_text(json.dumps(bad_registry), encoding="utf-8")
            with self.assertRaises(ValueError) as ctx:
                update_docs_media.load_registry(path)

        self.assertIn("entry 'broken_entry'", str(ctx.exception))
        self.assertIn("sync[0].to", str(ctx.exception))


if __name__ == "__main__":
    unittest.main()

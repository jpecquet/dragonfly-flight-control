import subprocess
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import h5py
import numpy as np

from post.io import read_simulation, run_termvel_simulation


class TestReadSimulation(unittest.TestCase):
    def test_wing_order_is_deterministic(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            h5_path = Path(tmpdir) / "output.h5"
            with h5py.File(str(h5_path), "w") as f:
                f.create_dataset("/parameters/omega", data=1.0)
                f.create_dataset("/parameters/wings/names", data=np.array([b"wing_b", b"wing_a"]))
                f.create_dataset("/parameters/wings/lb0", data=np.array([0.8, 0.9]))
                f.create_dataset("/time", data=np.array([0.0, 1.0]))
                f.create_dataset("/state", data=np.zeros((2, 6)))
                f.create_dataset("/wings/num_wings", data=2)
                for wing_name in ("wing_b", "wing_a"):
                    for vec_name in ("e_s", "e_r", "e_c", "lift", "drag"):
                        f.create_dataset(f"/wings/{wing_name}/{vec_name}", data=np.zeros((2, 3)))

            _, _, _, wings = read_simulation(h5_path)
            self.assertEqual(list(wings[0].keys()), ["wing_a", "wing_b"])


class TestRunTermvelSimulation(unittest.TestCase):
    def test_tempfile_cleanup_on_subprocess_failure(self):
        captured_output: dict[str, Path] = {}
        real_exists = Path.exists

        def fake_run(cmd, check, capture_output):
            captured_output["path"] = Path(cmd[-1])
            raise subprocess.CalledProcessError(returncode=1, cmd=cmd)

        def fake_exists(path_obj: Path) -> bool:
            if str(path_obj).endswith("/build/bin/dragonfly"):
                return True
            return real_exists(path_obj)

        with patch("post.io.Path.exists", new=fake_exists):
            with patch("post.io.subprocess.run", side_effect=fake_run):
                with self.assertRaises(subprocess.CalledProcessError):
                    run_termvel_simulation(psi_deg=0.0, dt=0.01, tmax=0.05)

        self.assertIn("path", captured_output)
        self.assertFalse(captured_output["path"].exists())


if __name__ == "__main__":
    unittest.main()

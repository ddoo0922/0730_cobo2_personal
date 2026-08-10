"""The parts of the GraspGenX boundary that need no GPU.

``graspgen_client`` imports ``graspgenx`` lazily, inside the constructor, so the
config-reading helpers can be tested in the ordinary suite. The fingertip offset
is worth pinning here: it decides where the fingers close, and reading a wrong or
missing value silently moves every grasp along the approach axis.
"""

import json
import tempfile
import unittest
from pathlib import Path

from vla_system.grasp.graspgen_client import read_gripper_fingertip
from vla_system.grasp.poses import RG2_FINGERTIP_M


class FingertipConfigTest(unittest.TestCase):
    def setUp(self):
        self.root = Path(tempfile.mkdtemp())
        self.gripper_dir = (
            self.root
            / "ext/gripper_descriptions/gripper_descriptions/assets/x_grippers/onrobot_RG2"
        )
        self.gripper_dir.mkdir(parents=True)

    def write_config(self, payload):
        (self.gripper_dir / "config.json").write_text(json.dumps(payload), encoding="utf-8")

    def test_the_shipped_rg2_offset_is_read(self):
        """180 mm along the gripper's +Z, as the released config states."""
        self.write_config({"fingertip": [0.0, 0.0, 0.18], "type": "revolute_2f"})
        self.assertEqual(read_gripper_fingertip(self.root, "onrobot_RG2"), (0.0, 0.0, 0.18))

    def test_a_non_default_offset_is_honoured(self):
        self.write_config({"fingertip": [0.01, -0.02, 0.2]})
        self.assertEqual(read_gripper_fingertip(self.root, "onrobot_RG2"), (0.01, -0.02, 0.2))

    def test_a_config_without_a_fingertip_falls_back(self):
        self.write_config({"type": "revolute_2f"})
        self.assertEqual(read_gripper_fingertip(self.root, "onrobot_RG2"), RG2_FINGERTIP_M)

    def test_a_malformed_fingertip_falls_back(self):
        self.write_config({"fingertip": [0.0, 0.18]})
        self.assertEqual(read_gripper_fingertip(self.root, "onrobot_RG2"), RG2_FINGERTIP_M)

    def test_an_unknown_gripper_falls_back(self):
        self.write_config({"fingertip": [0.0, 0.0, 0.18]})
        self.assertEqual(read_gripper_fingertip(self.root, "no_such_gripper"), RG2_FINGERTIP_M)

    def test_no_repo_falls_back_rather_than_raising(self):
        self.assertEqual(read_gripper_fingertip(None, "onrobot_RG2"), RG2_FINGERTIP_M)

    def test_the_fallback_is_the_rg2_value(self):
        """The default is only safe because RG2 is the gripper on this robot."""
        self.assertEqual(RG2_FINGERTIP_M, (0.0, 0.0, 0.18))


if __name__ == "__main__":
    unittest.main()

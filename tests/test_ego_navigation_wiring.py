#!/usr/bin/env python3

import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CHECKER = ROOT / "scripts" / "check_ego_external.py"
SEARCH_MSGS = ROOT / "src/ego_fusion_search/search_msgs"


class EgoExternalContractTest(unittest.TestCase):
    def test_checker_exists_and_has_pinned_revision(self):
        self.assertTrue(CHECKER.is_file(), f"missing checker: {CHECKER}")
        checker = CHECKER.read_text(encoding="utf-8")
        self.assertIn("92fe9f7227b2da819133eb8e0e8c7fc000f6ae20", checker)
        self.assertIn(
            "src/uav_simulator/Utils/quadrotor_msgs/msg/PositionCommand.msg",
            checker,
        )
        self.assertIn("src/planner/traj_utils/msg/Bspline.msg", checker)


class ValidateTrajectoryContractTest(unittest.TestCase):
    def test_service_contract_is_generated(self):
        service = (SEARCH_MSGS / "srv/ValidateTrajectory.srv").read_text()
        self.assertIn("uint64 task_generation", service)
        self.assertIn("geometry_msgs/Point[] samples", service)
        self.assertIn("time map_stamp", service)
        cmake = (SEARCH_MSGS / "CMakeLists.txt").read_text()
        self.assertIn("add_service_files", cmake)
        self.assertIn("ValidateTrajectory.srv", cmake)
        self.assertIn("geometry_msgs", cmake)


if __name__ == "__main__":
    unittest.main()

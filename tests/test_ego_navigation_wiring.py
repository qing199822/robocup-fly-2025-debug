#!/usr/bin/env python3

import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CHECKER = ROOT / "scripts" / "check_ego_external.py"
SEARCH_MSGS = ROOT / "src/ego_fusion_search/search_msgs"
EGO_ADAPTER = ROOT / "src/ego_fusion_search/ego_adapter"


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


class NavigationModeWiringTest(unittest.TestCase):
    def test_modes_are_mutually_exclusive(self):
        text = (
            ROOT / "src/look_up/launch/navigation_single.launch"
        ).read_text(encoding="utf-8")
        self.assertIn("navigation_mode", text)
        self.assertIn("navigation_mode == 'ego'", text)
        self.assertIn("navigation_mode == 'static_patrol'", text)
        self.assertIn("navigation_mode is not implemented", text)
        self.assertNotIn("cmd_vel_flu", text)

    def test_ego_mode_does_not_start_simple_navigator(self):
        text = (
            ROOT / "src/look_up/launch/navigation_single.launch"
        ).read_text(encoding="utf-8")
        ego_group = text.split("navigation_mode == 'ego'", 1)[1].split(
            "</group>", 1
        )[0]
        self.assertNotIn("simple_navigator", ego_group)
        self.assertIn("ego_single.launch", ego_group)
        self.assertIn("search_single.launch", ego_group)

    def test_down_resume_replaces_unconditional_navigator_include(self):
        text = (
            ROOT / "src/look_up/launch/down_resume.launch"
        ).read_text(encoding="utf-8")
        self.assertIn("navigation_mode", text)
        self.assertIn("active_num_drones", text)
        self.assertIn("navigation_single.launch", text)
        self.assertNotIn(
            '<include file="$(find simple_navigator)/launch/nav.launch">',
            text.split("active_num_drones", 1)[0],
        )

    def test_manual_goal_source_does_not_start_task_manager(self):
        text = (
            ROOT / "src/look_up/launch/down_resume.launch"
        ).read_text(encoding="utf-8")
        self.assertIn("goal_source == 'mission'", text)
        task_include = text.split(
            '<include file="$(find task_manager)/launch/task.launch"', 1
        )[1].split("</include>", 1)[0]
        self.assertIn('if="$(eval goal_source == \'mission\')"', task_include)

    def test_final_perception_guard_is_enabled_only_for_ego_mode(self):
        text = (
            ROOT / "src/look_up/launch/down_resume.launch"
        ).read_text(encoding="utf-8")
        self.assertIn('name="perception_guard_enabled"', text)
        self.assertIn("navigation_mode == 'ego'", text)
        safety_launch = (
            ROOT
            / "src/ego_fusion_search/safety_filter/launch/safety_filter_swarm.launch"
        ).read_text(encoding="utf-8")
        self.assertIn('name="perception_guard_enabled" default="false"', safety_launch)


class EgoSingleLaunchContractTest(unittest.TestCase):
    def test_single_launch_guards_external_source_and_vehicle(self):
        text = (EGO_ADAPTER / "launch/ego_single.launch").read_text(
            encoding="utf-8"
        )
        self.assertIn("supports only typhoon_h480_0", text)
        self.assertIn("ego_external_guard.py", text)
        self.assertIn("ego_camera_bootstrap.py", text)
        self.assertIn('required="true"', text)
        self.assertIn("ego_adapter_node", text)

    def test_runtime_uses_official_planner_without_demo_simulator(self):
        text = (EGO_ADAPTER / "launch/ego_runtime.launch").read_text(
            encoding="utf-8"
        )
        self.assertIn("ego_planner", text)
        self.assertIn("advanced_param.xml", text)
        self.assertIn("traj_server", text)
        self.assertIn("$(arg fx)", text)
        self.assertIn("$(arg fy)", text)
        self.assertIn("$(arg cx)", text)
        self.assertIn("$(arg cy)", text)
        self.assertIn("/typhoon_h480_0/global_odom", text)
        self.assertIn("/typhoon_h480_0/local_mapping/planner_depth", text)
        self.assertIn("/typhoon_h480_0/local_mapping/planner_pose", text)
        self.assertIn("/typhoon_h480_0/ego/goal", text)
        self.assertIn("/typhoon_h480_0/ego/position_cmd", text)
        self.assertIn("/typhoon_h480_0/ego/broadcast_bspline", text)
        self.assertNotIn("random_forest", text)
        self.assertNotIn("simulator.xml", text)

    def test_bootstrap_validates_real_camera_contract(self):
        text = (EGO_ADAPTER / "scripts/ego_camera_bootstrap.py").read_text(
            encoding="utf-8"
        )
        for fragment in (
            "CameraInfo",
            "Image",
            "16UC1",
            "32FC1",
            "math.isfinite",
            "camera_info.width",
            "camera_info.height",
            "depth.width",
            "depth.height",
            "camera_info.K[0]",
            "camera_info.K[4]",
            "camera_info.K[2]",
            "camera_info.K[5]",
        ):
            self.assertIn(fragment, text)

    def test_external_checker_covers_depth_encoding_code_path(self):
        checker = CHECKER.read_text(encoding="utf-8")
        self.assertIn("plan_env/src/grid_map.cpp", checker)
        self.assertIn("TYPE_32FC1", checker)
        self.assertIn("CV_16UC1", checker)


if __name__ == "__main__":
    unittest.main()

#!/usr/bin/env python3

import pathlib
import unittest
import xml.etree.ElementTree as ET


ROOT = pathlib.Path(__file__).parents[1]
MUX_LAUNCH = ROOT / "src/look_up/launch/spawn_mux_swarm.launch"
MISSION_LAUNCH = ROOT / "src/look_up/launch/down_resume.launch"
FLY_SOURCE = ROOT / "src/mix_nav/fly/src/fly_takeoff.cpp"
SAFETY_LAUNCH = (
    ROOT
    / "src/ego_fusion_search/safety_filter/launch/safety_filter_swarm.launch"
)


class ControlSafetyWiringTest(unittest.TestCase):
    def test_mux_has_three_inputs_and_internal_output(self):
        root = ET.parse(str(MUX_LAUNCH)).getroot()
        node = root.find(".//node[@pkg='topic_tools'][@type='mux']")
        self.assertIsNotNone(node)
        args = node.get("args", "")
        self.assertIn("$(arg input_takeoff)", args)
        self.assertIn("$(arg input_navigator)", args)
        self.assertIn("$(arg input_external)", args)
        text = MUX_LAUNCH.read_text(encoding="utf-8")
        self.assertIn(
            "/$(arg vehicle_type)_$(arg drone_id)/control/raw_cmd_vel", text
        )
        self.assertNotIn('<arg name="mux_output" value="/xtdrone/', text)

    def test_takeoff_never_publishes_final_velocity(self):
        source = FLY_SOURCE.read_text(encoding="utf-8")
        self.assertIn("/mux_inputs/takeoff/cmd_vel", source)
        self.assertNotIn('"/cmd_vel_flu"', source)
        self.assertIn("topic_tools::MuxSelect", source)

    def test_mission_launch_starts_safety_before_mux_and_takeoff(self):
        root = ET.parse(str(MISSION_LAUNCH)).getroot()
        includes = [item.get("file", "") for item in root.findall("./include")]
        safety = "$(find safety_filter)/launch/safety_filter_swarm.launch"
        mux = "$(find look_up)/launch/spawn_mux_swarm.launch"
        fly = "$(find fly)/launch/fly.launch"
        self.assertLess(includes.index(safety), includes.index(mux))
        self.assertLess(includes.index(mux), includes.index(fly))

    def test_safety_launch_names_all_per_vehicle_interfaces(self):
        text = SAFETY_LAUNCH.read_text(encoding="utf-8")
        self.assertIn("/control/raw_cmd_vel", text)
        self.assertIn("/global_odom", text)
        self.assertIn(
            "/xtdrone/$(arg vehicle_type)_$(arg drone_id)/cmd_vel_flu", text
        )
        self.assertIn('name="safety_filter"', text)


if __name__ == "__main__":
    unittest.main()

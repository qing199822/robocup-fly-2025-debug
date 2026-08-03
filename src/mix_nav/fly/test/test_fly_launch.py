#!/usr/bin/env python3

import pathlib
import unittest
import xml.etree.ElementTree as ET


class FlyLaunchTest(unittest.TestCase):
    def test_completed_takeoff_does_not_shutdown_mission(self):
        launch_file = pathlib.Path(__file__).parents[1] / "launch" / "fly.launch"
        root = ET.parse(str(launch_file)).getroot()
        takeoff_node = root.find("./node[@name='confident_takeoff_node']")

        self.assertIsNotNone(takeoff_node)
        self.assertNotEqual(
            takeoff_node.get("required", "false").lower(),
            "true",
            "a normally completed takeoff node must not stop the entire mission launch",
        )

    def test_completed_aircraft_do_not_receive_more_climb_commands(self):
        source_file = pathlib.Path(__file__).parents[1] / "src" / "fly_takeoff.cpp"
        compacted_source = "".join(source_file.read_text().split())

        self.assertIn(
            "if(!mission_done_flags_[i]){vel_pubs_[i].publish(climb_twist);}",
            compacted_source,
            "each aircraft must stop climbing as soon as it reaches target altitude",
        )

    def test_takeoff_uses_mux_input_and_hands_off_to_navigator(self):
        source_file = pathlib.Path(__file__).parents[1] / "src" / "fly_takeoff.cpp"
        source = source_file.read_text()
        compacted = "".join(source.split())

        self.assertIn("/mux_inputs/takeoff/cmd_vel", source)
        self.assertNotIn('\"/cmd_vel_flu\"', source)
        self.assertIn("selectControl(i,takeoff_topic)", compacted)
        self.assertIn("selectControl(i,navigator_topic)", compacted)
        failure_guard = source.rindex("if (!allMissionDone())")
        navigator_handoff = source.rindex("selectControl(i, navigator_topic)")
        self.assertLess(failure_guard, navigator_handoff)

    def test_partial_navigator_handoff_rolls_every_drone_back_to_takeoff(self):
        source_file = pathlib.Path(__file__).parents[1] / "src" / "fly_takeoff.cpp"
        source = source_file.read_text()

        handoff_failure = source.index("if (!selectControl(i, navigator_topic))")
        rollback_loop = source.find("for (int rollback_id = 0;", handoff_failure)
        self.assertNotEqual(
            -1,
            rollback_loop,
            "a partial navigator handoff must roll every drone back to takeoff",
        )
        rollback_topic = source.index(
            "/mux_inputs/takeoff/cmd_vel", rollback_loop
        )
        rollback_select = source.index(
            "selectControl(rollback_id, takeoff_topic)", rollback_topic
        )
        self.assertLess(rollback_loop, rollback_topic)
        self.assertLess(rollback_topic, rollback_select)


if __name__ == "__main__":
    unittest.main()

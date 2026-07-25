#!/usr/bin/env python3

import pathlib
import unittest


class OneClickLaunchTest(unittest.TestCase):
    def test_communication_bridge_starts_before_mission(self):
        script = (pathlib.Path(__file__).parents[1] / "1.sh").read_text()

        bridge = 'start_communication "$XTDRONE_PYTHON"'
        mission = 'roslaunch look_up down_resume.launch'
        self.assertIn(bridge, script)
        self.assertIn(mission, script)
        self.assertLess(
            script.index(bridge),
            script.index(mission),
            "XTDrone communication bridges must start before the mission sends commands",
        )


if __name__ == "__main__":
    unittest.main()

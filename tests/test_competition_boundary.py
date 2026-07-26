#!/usr/bin/env python3

import pathlib
import unittest


PROJECT_ROOT = pathlib.Path(__file__).parents[1]
FORBIDDEN_PATHS = (
    pathlib.Path("src/gazebo_ros_pkgs"),
    pathlib.Path("typhoon_h480_zzufly"),
    pathlib.Path("src/gimbal"),
)


class CompetitionBoundaryTest(unittest.TestCase):
    def test_forbidden_bundled_simulator_paths_are_absent(self):
        existing_paths = [
            str(path) for path in FORBIDDEN_PATHS if (PROJECT_ROOT / path).exists()
        ]

        self.assertEqual([], existing_paths)


if __name__ == "__main__":
    unittest.main()

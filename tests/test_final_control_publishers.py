#!/usr/bin/env python3

import importlib.util
import pathlib
import unittest


ROOT = pathlib.Path(__file__).parents[1]
SCRIPT = ROOT / "scripts/check_final_control_publishers.py"


def load_module():
    spec = importlib.util.spec_from_file_location("publisher_guard", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class FinalControlPublisherTest(unittest.TestCase):
    def test_accepts_exactly_one_expected_publisher_per_vehicle(self):
        module = load_module()
        publishers = []
        for drone_id in range(6):
            publishers.append(
                (
                    f"/xtdrone/typhoon_h480_{drone_id}/cmd_vel_flu",
                    [f"/typhoon_h480_{drone_id}/safety_filter"],
                )
            )
        self.assertEqual(
            [], module.validate_publishers(publishers, 6, "typhoon_h480")
        )

    def test_rejects_missing_unexpected_and_duplicate_publishers(self):
        module = load_module()
        publishers = [
            ("/xtdrone/typhoon_h480_0/cmd_vel_flu", []),
            (
                "/xtdrone/typhoon_h480_1/cmd_vel_flu",
                [
                    "/typhoon_h480_1/safety_filter",
                    "/confident_takeoff_node",
                ],
            ),
        ]
        errors = module.validate_publishers(publishers, 2, "typhoon_h480")
        self.assertEqual(2, len(errors))
        self.assertIn("vehicle 0", errors[0])
        self.assertIn("vehicle 1", errors[1])


if __name__ == "__main__":
    unittest.main()

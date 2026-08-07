#!/usr/bin/env python3

import os
import math
import sys
from types import SimpleNamespace
import unittest


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPOSITORY_ROOT = os.path.dirname(SCRIPT_DIR)
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import check_local_mapping_single as checker


observed_rate_hz = checker.observed_rate_hz
validate_sample = checker.validate_sample


def healthy_sample():
    receive_times = [0.0, 0.2, 0.4, 0.6, 0.8, 1.0]
    return {
        "health_rate_hz": 5.0,
        "max_depth_age_s": 0.50,
        "planner_depth_publishers": 1,
        "control_publishers": [],
        "static_cloud_rate_hz": 5.0,
        "dynamic_cloud_rate_hz": 5.0,
        "sample_start_s": 0.0,
        "sample_end_s": 1.0,
        "output_receive_times": {
            "health": list(receive_times),
            "static_cloud": list(receive_times),
            "dynamic_cloud": list(receive_times),
        },
        "received_inputs": {
            "depth": True,
            "camera_info": True,
            "global_odom": True,
            "bounding_boxes": True,
        },
        "frame_ids": {
            "health": "map",
            "static_cloud": "map",
            "dynamic_cloud": "map",
            "clearance": "map",
        },
    }


class ValidateSampleTest(unittest.TestCase):
    def test_accepts_healthy_sample(self):
        self.assertEqual([], validate_sample(healthy_sample()))

    def test_rejects_four_hz_health(self):
        sample = healthy_sample()
        sample["health_rate_hz"] = 4.0
        self.assertEqual(
            ["health_rate_hz must be at least 5.0 (got 4.000)"],
            validate_sample(sample),
        )

    def test_rejects_depth_older_than_point_five_seconds(self):
        sample = healthy_sample()
        sample["max_depth_age_s"] = 0.60
        self.assertEqual(
            ["max_depth_age_s must be at most 0.50 (got 0.600)"],
            validate_sample(sample),
        )

    def test_rejects_two_planner_depth_publishers(self):
        sample = healthy_sample()
        sample["planner_depth_publishers"] = 2
        self.assertEqual(
            ["planner_depth_publishers must equal 1 (got 2)"],
            validate_sample(sample),
        )

    def test_rejects_unexpected_control_publisher(self):
        sample = healthy_sample()
        sample["control_publishers"] = [
            "/local_mapping_typhoon_h480_0 on /xtdrone/typhoon_h480_0/cmd_vel_flu"
        ]
        self.assertEqual(
            [
                "control_publishers must be empty (got "
                "['/local_mapping_typhoon_h480_0 on "
                "/xtdrone/typhoon_h480_0/cmd_vel_flu'])"
            ],
            validate_sample(sample),
        )

    def test_accepts_rate_and_depth_age_boundaries(self):
        sample = healthy_sample()
        sample["health_rate_hz"] = 5.0
        sample["max_depth_age_s"] = 0.50
        self.assertEqual([], validate_sample(sample))

    def test_multiple_errors_have_deterministic_order(self):
        sample = healthy_sample()
        sample.update(
            {
                "health_rate_hz": 4.0,
                "max_depth_age_s": 0.60,
                "planner_depth_publishers": 2,
                "control_publishers": ["/node_b on /cmd_vel", "/node_a on /cmd_vel"],
            }
        )
        self.assertEqual(
            [
                "health_rate_hz must be at least 5.0 (got 4.000)",
                "max_depth_age_s must be at most 0.50 (got 0.600)",
                "planner_depth_publishers must equal 1 (got 2)",
                "control_publishers must be empty (got "
                "['/node_a on /cmd_vel', '/node_b on /cmd_vel'])",
            ],
            validate_sample(sample),
        )

    def test_rejects_missing_inputs_wrong_frames_and_slow_map_clouds(self):
        sample = healthy_sample()
        sample["static_cloud_rate_hz"] = 4.9
        sample["dynamic_cloud_rate_hz"] = 0.0
        sample["received_inputs"]["camera_info"] = False
        sample["frame_ids"]["clearance"] = "base_link"
        self.assertEqual(
            [
                "static_cloud_rate_hz must be at least 5.0 (got 4.900)",
                "dynamic_cloud_rate_hz must be at least 5.0 (got 0.000)",
                "input camera_info was not received",
                "clearance frame_id must equal map (got 'base_link')",
            ],
            validate_sample(sample),
        )

    def test_rate_uses_first_and_last_receive_intervals(self):
        self.assertAlmostEqual(5.0, observed_rate_hz([10.0, 10.2, 10.4]))
        self.assertEqual(0.0, observed_rate_hz([10.0]))

    def test_rejects_brief_burst_followed_by_long_silence(self):
        sample = healthy_sample()
        stable_times = [index * 0.2 for index in range(151)]
        sample.update(
            {
                "health_rate_hz": 100.0,
                "static_cloud_rate_hz": 5.0,
                "dynamic_cloud_rate_hz": 5.0,
                "sample_start_s": 0.0,
                "sample_end_s": 30.0,
                "output_receive_times": {
                    "health": [0.0, 0.01],
                    "static_cloud": stable_times,
                    "dynamic_cloud": stable_times,
                },
            }
        )
        self.assertEqual(
            [
                "health last receive gap must be at most 0.250s "
                "(got 29.990s)"
            ],
            validate_sample(sample),
        )

    def test_accepts_stable_five_hz_and_gap_boundary(self):
        sample = healthy_sample()
        boundary_times = [0.0, 0.25, 0.4, 0.6, 0.8, 1.0]
        for name in ("health", "static_cloud", "dynamic_cloud"):
            sample["output_receive_times"][name] = list(boundary_times)
        self.assertEqual([], validate_sample(sample))

    def test_raw_cmd_vel_is_control_but_mapping_outputs_are_not(self):
        self.assertTrue(
            checker._is_control_topic(
                "/typhoon_h480_0/control/raw_cmd_vel"
            )
        )
        self.assertTrue(
            checker._is_control_topic(
                "/typhoon_h480_0/mux_inputs/navigator/cmd_vel"
            )
        )
        self.assertTrue(
            checker._is_control_topic(
                "/typhoon_h480_0/final_cmd_vel"
            )
        )
        self.assertFalse(
            checker._is_control_topic(
                "/typhoon_h480_0/local_mapping/health"
            )
        )
        self.assertFalse(
            checker._is_control_topic(
                "/typhoon_h480_0/local_mapping/frontier_goal"
            )
        )

    def test_future_depth_fails_closed_beyond_one_millisecond_tolerance(self):
        self.assertTrue(math.isinf(checker.depth_age_s(10.0, 10.01)))
        self.assertEqual(0.0, checker.depth_age_s(10.0, 10.0005))

    def test_missing_schema_fields_fail_closed_in_deterministic_order(self):
        self.assertEqual(
            [
                "missing sample field: control_publishers",
                "missing sample field: static_cloud_rate_hz",
                "missing sample field: dynamic_cloud_rate_hz",
                "missing sample field: received_inputs",
                "missing sample field: frame_ids",
                "missing sample field: sample_start_s",
                "missing sample field: sample_end_s",
                "missing sample field: output_receive_times",
            ],
            validate_sample(
                {
                    "health_rate_hz": 5.0,
                    "max_depth_age_s": 0.50,
                    "planner_depth_publishers": 1,
                }
            ),
        )

    def test_any_non_map_frame_seen_during_window_fails(self):
        sample = healthy_sample()
        sample["frame_ids"]["health"] = ["base_link", "map"]
        self.assertEqual(
            [
                "health frame_ids must equal ['map'] "
                "(got ['base_link', 'map'])"
            ],
            validate_sample(sample),
        )

    def test_input_flags_must_be_boolean_true(self):
        sample = healthy_sample()
        sample["received_inputs"] = {
            name: "false"
            for name in (
                "depth",
                "camera_info",
                "global_odom",
                "bounding_boxes",
            )
        }
        self.assertEqual(
            [
                "input depth must be boolean true (got 'false')",
                "input camera_info must be boolean true (got 'false')",
                "input global_odom must be boolean true (got 'false')",
                "input bounding_boxes must be boolean true (got 'false')",
            ],
            validate_sample(sample),
        )

    def test_collector_finish_captures_end_and_snapshot_under_one_api(self):
        ticks = iter([10.0, 10.2, 10.5])
        collector = checker._SampleCollector(None, monotonic=lambda: next(ticks))
        sample_start = collector.start_sample()
        collector.output_callback("health")(
            SimpleNamespace(header=SimpleNamespace(frame_id="map"))
        )

        sample = collector.finish_sample(sample_start)

        self.assertEqual(10.5, sample["sample_end_s"])
        self.assertEqual([10.2], sample["output_receive_times"]["health"])
        self.assertEqual(["map"], sample["frame_ids"]["health"])

    def test_connection_wait_continues_when_every_subscriber_is_connected(self):
        class Subscriber:
            def get_num_connections(self):
                return 1

        sleeps = []
        checker.wait_for_subscriber_connections(
            {"/a": Subscriber(), "/b": Subscriber()},
            timeout_s=0.1,
            monotonic=lambda: 0.0,
            sleep=sleeps.append,
        )
        self.assertEqual([], sleeps)

    def test_connection_wait_times_out_with_sorted_missing_topics(self):
        class Subscriber:
            def get_num_connections(self):
                return 0

        ticks = iter([0.0, 0.05, 0.11])
        with self.assertRaisesRegex(
            RuntimeError,
            "subscriber connection timeout; missing topics: /a, /b",
        ):
            checker.wait_for_subscriber_connections(
                {"/b": Subscriber(), "/a": Subscriber()},
                timeout_s=0.1,
                monotonic=lambda: next(ticks),
                sleep=lambda _duration: None,
            )


class SingleDroneLaunchGuardTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import roslaunch.config
        import roslaunch.xmlloader

        cls.ROSLaunchConfig = roslaunch.config.ROSLaunchConfig
        cls.XmlLoader = roslaunch.xmlloader.XmlLoader
        cls.launch_path = os.path.join(
            REPOSITORY_ROOT,
            "src",
            "ego_fusion_search",
            "local_mapping",
            "launch",
            "local_mapping_single.launch",
        )

    def _load(self, arguments):
        config = self.ROSLaunchConfig()
        self.XmlLoader().load(
            self.launch_path, config, argv=arguments, verbose=False
        )
        return config

    def test_legal_single_drone_arguments_keep_one_mapping_node(self):
        config = self._load(
            ["vehicle_type:=typhoon_h480", "drone_id:=0"]
        )
        self.assertEqual(1, len(config.nodes))
        self.assertEqual(
            "local_mapping_typhoon_h480_0", config.nodes[0].name
        )

    def test_illegal_vehicle_or_drone_id_fails_during_parsing(self):
        for arguments in (
            ["vehicle_type:=iris", "drone_id:=0"],
            ["vehicle_type:=typhoon_h480", "drone_id:=1"],
            [
                "vehicle_type:=iris",
                "drone_id:=1",
                "single_drone_guard:=valid",
            ],
        ):
            with self.subTest(arguments=arguments):
                with self.assertRaisesRegex(
                    Exception,
                    "supports only vehicle_type:=typhoon_h480 drone_id:=0",
                ):
                    self._load(arguments)


if __name__ == "__main__":
    unittest.main()

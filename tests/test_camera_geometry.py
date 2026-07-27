#!/usr/bin/env python3

import ast
import math
import pathlib
import sys
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
YOLO_DIR = ROOT / "src" / "yolo"
NODE_PATH = YOLO_DIR / "bbox2coord_node.py"
sys.path.insert(0, str(YOLO_DIR))

from camera_geometry import deproject_pixel, timestamps_within


class CameraGeometryTest(unittest.TestCase):
    def test_deprojection_uses_camera_info_matrix_at_center_and_off_center(self):
        camera_matrix = [
            554.256,
            0.0,
            320.0,
            0.0,
            554.256,
            240.0,
            0.0,
            0.0,
            1.0,
        ]

        self.assertEqual(
            (0.0, 0.0, 4.0),
            deproject_pixel(320, 240, 4.0, camera_matrix),
        )
        x, y, z = deproject_pixel(420, 290, 4.0, camera_matrix)
        self.assertAlmostEqual(100.0 * 4.0 / 554.256, x)
        self.assertAlmostEqual(50.0 * 4.0 / 554.256, y)
        self.assertEqual(4.0, z)

    def test_altered_camera_matrix_changes_deprojection(self):
        first_matrix = [500.0, 0.0, 300.0, 0.0, 400.0, 200.0, 0.0, 0.0, 1.0]
        second_matrix = [250.0, 0.0, 350.0, 0.0, 200.0, 250.0, 0.0, 0.0, 1.0]

        self.assertEqual((0.6, 0.75, 2.0), deproject_pixel(450, 350, 2, first_matrix))
        self.assertEqual((0.8, 1.0, 2.0), deproject_pixel(450, 350, 2, second_matrix))

    def test_malformed_matrix_and_invalid_focal_lengths_are_rejected(self):
        valid = [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0]
        invalid_matrices = (
            valid[:8],
            None,
            [0.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0],
            [-1.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0],
            [500.0, 0.0, 320.0, 0.0, 0.0, 240.0, 0.0, 0.0, 1.0],
            [500.0, 0.0, 320.0, 0.0, math.inf, 240.0, 0.0, 0.0, 1.0],
            [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, math.nan, 1.0],
            [500.0, 0.0, "bad", 0.0, 500.0, 240.0, 0.0, 0.0, 1.0],
        )

        for matrix in invalid_matrices:
            with self.subTest(matrix=matrix):
                with self.assertRaises(ValueError):
                    deproject_pixel(320, 240, 1.0, matrix)

    def test_nonfinite_pixel_and_depth_inputs_are_rejected(self):
        camera_matrix = [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0]

        for u, v, depth in (
            (math.nan, 240, 1.0),
            (320, math.inf, 1.0),
            (320, 240, -math.inf),
            ("bad", 240, 1.0),
        ):
            with self.subTest(u=u, v=v, depth=depth):
                with self.assertRaises(ValueError):
                    deproject_pixel(u, v, depth, camera_matrix)

    def test_timestamps_are_compared_at_and_beyond_the_boundary(self):
        self.assertTrue(timestamps_within(10.0, 10.08, 0.1))
        self.assertTrue(timestamps_within(10.0, 10.1, 0.1))
        self.assertFalse(timestamps_within(10.0, 10.2, 0.1))

    def test_invalid_timestamp_arguments_are_rejected(self):
        for first, second, maximum in (
            (10.0, 10.0, -0.1),
            (10.0, 10.0, math.inf),
            (10.0, 10.0, math.nan),
            (math.inf, 10.0, 0.1),
            (10.0, math.nan, 0.1),
            ("bad", 10.0, 0.1),
        ):
            with self.subTest(first=first, second=second, maximum=maximum):
                with self.assertRaises(ValueError):
                    timestamps_within(first, second, maximum)


class CameraInfoNodeSourceContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = NODE_PATH.read_text(encoding="utf-8")
        cls.tree = ast.parse(cls.source)

    @classmethod
    def function_source(cls, function_name):
        node = next(
            node
            for node in ast.walk(cls.tree)
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
            and node.name == function_name
        )
        lines = cls.source.splitlines()
        return "\n".join(lines[node.lineno - 1 : node.end_lineno])

    def test_node_imports_camera_info_image_and_geometry_helpers(self):
        sensor_imports = set()
        geometry_imports = set()
        for node in ast.walk(self.tree):
            if isinstance(node, ast.ImportFrom) and node.module == "sensor_msgs.msg":
                sensor_imports.update(alias.name for alias in node.names)
            if isinstance(node, ast.ImportFrom) and node.module == "camera_geometry":
                geometry_imports.update(alias.name for alias in node.names)

        self.assertTrue({"CameraInfo", "Image"}.issubset(sensor_imports))
        self.assertEqual({"deproject_pixel", "timestamps_within"}, geometry_imports)

    def test_node_initializes_camera_state_and_validates_sensor_delta(self):
        initializer = self.function_source("__init__")
        loader = self.function_source("_load_ros_params")

        self.assertIn("self.latest_camera_info = None", initializer)
        self.assertIn("self.latest_depth_stamp = None", initializer)
        self.assertIn('rospy.get_param("~camera_frame", "")', loader)
        self.assertIn('rospy.get_param("~maximum_sensor_delta", 0.15)', loader)
        self.assertIn("math.isfinite", loader)
        self.assertIn("< 0", loader)
        self.assertIn("rospy.logfatal", loader)

    def test_node_subscribes_to_and_rejects_malformed_camera_info(self):
        setup = self.function_source("_setup_subscriptions")
        callback = self.function_source("_camera_info_callback")

        self.assertIn("/realsense/depth_camera/color/camera_info", setup)
        self.assertIn("CameraInfo", setup)
        self.assertIn("queue_size=1", setup)
        self.assertIn("message.header.frame_id", callback)
        self.assertIn("len(message.K) == 9", callback)
        self.assertIn("message.width", callback)
        self.assertIn("message.height", callback)
        self.assertIn("math.isfinite", callback)
        self.assertIn("self.latest_camera_info = message", callback)

    def test_depth_and_detection_callbacks_enforce_sensor_freshness(self):
        depth_callback = self.function_source("_depth_image_callback")
        detection_callback = self.function_source("bounding_box_callback")

        self.assertIn("self.latest_depth_stamp = image_msg.header.stamp", depth_callback)
        self.assertGreaterEqual(depth_callback.count("self.latest_depth_stamp = None"), 1)
        self.assertIn("self.latest_depth_stamp is None", detection_callback)
        self.assertIn("self.latest_camera_info is None", detection_callback)
        self.assertIn("self.latest_camera_info.width", detection_callback)
        self.assertIn("self.latest_camera_info.height", detection_callback)
        self.assertIn("detections_msg.header.stamp", detection_callback)
        self.assertIn("timestamps_within", detection_callback)
        self.assertIn("self.maximum_sensor_delta", detection_callback)
        self.assertIn("rospy.logwarn_throttle", detection_callback)

    def test_point_uses_camera_info_frame_and_exact_depth_stamp(self):
        calculation = self.function_source("_calculate_3d_point")

        self.assertIn("deproject_pixel", calculation)
        self.assertIn("self.latest_camera_info.K", calculation)
        self.assertIn("self.latest_camera_info.header.frame_id", calculation)
        self.assertIn("self.camera_frame_override", calculation)
        self.assertIn("point.header.stamp = self.latest_depth_stamp", calculation)

    def test_old_intrinsics_and_default_frame_are_absent(self):
        for forbidden in (
            "cam_fx",
            "cam_fy",
            "cam_cx",
            "cam_cy",
            "205.47",
            "camera_optical_link",
        ):
            with self.subTest(forbidden=forbidden):
                self.assertNotIn(forbidden, self.source)


if __name__ == "__main__":
    unittest.main()

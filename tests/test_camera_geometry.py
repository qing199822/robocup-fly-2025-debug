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
    def function_node(cls, function_name):
        return next(
            node
            for node in ast.walk(cls.tree)
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
            and node.name == function_name
        )

    @classmethod
    def function_source(cls, function_name):
        node = cls.function_node(function_name)
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
        self.assertIn("self.latest_depth_sample = None", initializer)
        self.assertNotIn("self.latest_depth_frame", self.source)
        self.assertNotIn("self.latest_depth_stamp", self.source)
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

    def test_depth_callback_publishes_frame_and_stamp_as_one_sample(self):
        depth_callback = self.function_source("_depth_image_callback")

        self.assertIn("converted_frame = self.cv_bridge.imgmsg_to_cv2", depth_callback)
        self.assertIn(
            "self.latest_depth_sample = (converted_frame, image_msg.header.stamp)",
            depth_callback,
        )
        self.assertIn("self.latest_depth_sample = None", depth_callback)

    def test_detection_callback_snapshots_each_sensor_input_once_at_entry(self):
        callback_node = self.function_node("bounding_box_callback")
        detection_callback = self.function_source("bounding_box_callback")
        executable_body = callback_node.body
        if (
            executable_body
            and isinstance(executable_body[0], ast.Expr)
            and isinstance(executable_body[0].value, ast.Str)
        ):
            executable_body = executable_body[1:]

        first_assignment, second_assignment = executable_body[:2]
        self.assertIsInstance(first_assignment, ast.Assign)
        self.assertIsInstance(second_assignment, ast.Assign)
        self.assertEqual("depth_sample", first_assignment.targets[0].id)
        self.assertEqual("latest_depth_sample", first_assignment.value.attr)
        self.assertEqual("camera_info", second_assignment.targets[0].id)
        self.assertEqual("latest_camera_info", second_assignment.value.attr)
        self.assertEqual(1, detection_callback.count("self.latest_depth_sample"))
        self.assertEqual(1, detection_callback.count("self.latest_camera_info"))
        self.assertIn("depth_frame, depth_stamp = depth_sample", detection_callback)
        self.assertIn("camera_info.width", detection_callback)
        self.assertIn("camera_info.height", detection_callback)
        self.assertIn("detections_msg.header.stamp", detection_callback)
        self.assertIn("timestamps_within", detection_callback)
        self.assertIn("self.maximum_sensor_delta", detection_callback)
        self.assertIn("rospy.logwarn_throttle", detection_callback)

    def test_zero_stamps_are_rejected_before_timestamp_matching(self):
        callback_node = self.function_node("bounding_box_callback")
        detection_callback = self.function_source("bounding_box_callback")
        zero_stamp_if = next(
            node
            for node in ast.walk(callback_node)
            if isinstance(node, ast.If)
            and isinstance(node.test, ast.BoolOp)
            and isinstance(node.test.op, ast.Or)
            and "depth_stamp.is_zero()" in ast.get_source_segment(self.source, node.test)
            and "detections_msg.header.stamp.is_zero()"
            in ast.get_source_segment(self.source, node.test)
        )

        zero_stamp_source = ast.get_source_segment(self.source, zero_stamp_if)
        timestamp_call_position = detection_callback.index("timestamps_within")
        zero_check_position = detection_callback.index("depth_stamp.is_zero()")
        self.assertIn("rospy.logwarn_throttle", zero_stamp_source)
        self.assertTrue(any(isinstance(node, ast.Return) for node in zero_stamp_if.body))
        self.assertLess(zero_check_position, timestamp_call_position)

    def test_roi_and_point_helpers_only_use_snapshotted_sensor_inputs(self):
        roi_source = self.function_source("_get_roi_mean_depth")
        calculation = self.function_source("_calculate_3d_point")

        self.assertIn("def _get_roi_mean_depth(self, depth_frame, u, v)", roi_source)
        self.assertIn("depth_frame.shape", roi_source)
        self.assertNotIn("self.latest_", roi_source)
        self.assertIn(
            "def _calculate_3d_point(self, u, v, depth_in_meters, camera_info, depth_stamp)",
            calculation,
        )
        self.assertIn("deproject_pixel", calculation)
        self.assertIn("camera_info.K", calculation)
        self.assertIn("camera_info.header.frame_id", calculation)
        self.assertIn("self.camera_frame_override", calculation)
        self.assertIn("point.header.stamp = depth_stamp", calculation)
        self.assertNotIn("self.latest_", calculation)

        detection_callback = self.function_source("bounding_box_callback")
        self.assertIn(
            "self._get_roi_mean_depth(depth_frame, center_u, center_v)",
            detection_callback,
        )
        self.assertIn(
            "self._calculate_3d_point(\n"
            "                        center_u, center_v, mean_depth, camera_info, depth_stamp\n"
            "                    )",
            detection_callback,
        )

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

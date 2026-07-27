#!/usr/bin/env python3

import ast
import math
import pathlib
import sys
import unittest

import numpy as np


ROOT = pathlib.Path(__file__).resolve().parents[1]
YOLO_DIR = ROOT / "src" / "yolo"
NODE_PATH = YOLO_DIR / "bbox2coord_node.py"
sys.path.insert(0, str(YOLO_DIR))

from camera_geometry import (
    DepthSample,
    deproject_pixel,
    depth_image_to_meters,
    roi_mean_depth,
    select_closest_depth_sample,
    timestamps_within,
)


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
            (320, 240, 0.0),
            (320, 240, -0.01),
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


class DepthImageConversionTest(unittest.TestCase):
    def test_32fc1_is_copied_without_changing_meter_values(self):
        source = np.array([[0.05, 0.1], [1.25, math.nan]], dtype=np.float32)

        converted = depth_image_to_meters(source, "32FC1")
        source[0, 0] = 99.0

        self.assertAlmostEqual(0.05, float(converted[0, 0]))
        self.assertAlmostEqual(0.1, float(converted[0, 1]))
        self.assertAlmostEqual(1.25, float(converted[1, 0]))
        self.assertTrue(math.isnan(float(converted[1, 1])))
        self.assertFalse(np.shares_memory(source, converted))

    def test_16uc1_converts_every_value_from_millimeters(self):
        source = np.array([[0, 50, 100, 101, 1000]], dtype=np.uint16)

        converted = depth_image_to_meters(source, "16UC1")

        np.testing.assert_allclose(
            np.array([[0.0, 0.05, 0.1, 0.101, 1.0]], dtype=np.float32),
            converted,
        )

    def test_unsupported_encoding_and_non_2d_images_are_rejected(self):
        with self.assertRaises(ValueError):
            depth_image_to_meters(np.ones((2, 2), dtype=np.uint16), "mono16")
        with self.assertRaises(ValueError):
            depth_image_to_meters(np.ones((2, 2, 1), dtype=np.float32), "32FC1")
        with self.assertRaises(ValueError):
            depth_image_to_meters(np.array([["bad"]]), "32FC1")


class RoiMeanDepthTest(unittest.TestCase):
    def test_roi_clamps_at_edges_and_filters_nonpositive_nonfinite_depth(self):
        depth = np.array(
            [
                [0.05, 0.1, math.nan],
                [0.0, -1.0, math.inf],
                [2.0, 4.0, 6.0],
            ],
            dtype=np.float32,
        )

        self.assertAlmostEqual(0.075, roi_mean_depth(depth, 0, 0, 1))
        self.assertAlmostEqual(5.0, roi_mean_depth(depth, 2, 2, 1))
        self.assertIsNone(roi_mean_depth(np.zeros((2, 2)), 0, 0, 1))

    def test_roi_rejects_out_of_range_or_malformed_inputs(self):
        depth = np.ones((2, 3), dtype=np.float32)
        for u, v, half_size in (
            (-1, 0, 1),
            (3, 0, 1),
            (0, -1, 1),
            (0, 2, 1),
            (0.5, 0, 1),
            (0, 0, -1),
        ):
            with self.subTest(u=u, v=v, half_size=half_size):
                with self.assertRaises(ValueError):
                    roi_mean_depth(depth, u, v, half_size)
        with self.assertRaises(ValueError):
            roi_mean_depth(np.ones((2, 2, 1)), 0, 0, 1)


class DepthSampleSelectionTest(unittest.TestCase):
    @staticmethod
    def sample(stamp_seconds):
        return DepthSample(
            depth_meters=np.array([[stamp_seconds]], dtype=np.float32),
            stamp=object(),
            stamp_seconds=stamp_seconds,
            frame_id="camera_color_optical_frame",
            encoding="32FC1",
        )

    def test_older_matching_sample_remains_selectable_after_newer_samples(self):
        older = self.sample(10.0)
        samples = (older, self.sample(10.3), self.sample(10.4))

        selected = select_closest_depth_sample(samples, 10.04, 0.1)

        self.assertIs(older, selected)

    def test_no_sample_within_delta_returns_none(self):
        samples = (self.sample(10.0), self.sample(10.3))

        self.assertIsNone(select_closest_depth_sample(samples, 10.15, 0.1))

    def test_equal_delta_tie_selects_earlier_timestamp(self):
        earlier = self.sample(9.9)
        later = self.sample(10.1)

        self.assertIs(
            earlier,
            select_closest_depth_sample((later, earlier), 10.0, 0.11),
        )

    def test_zero_nonfinite_and_malformed_sample_timestamps_are_rejected(self):
        for target in (0.0, math.nan, math.inf):
            with self.subTest(target=target):
                with self.assertRaises(ValueError):
                    select_closest_depth_sample((self.sample(1.0),), target, 0.1)
        for sample_stamp in (0.0, math.nan, math.inf):
            with self.subTest(sample_stamp=sample_stamp):
                with self.assertRaises(ValueError):
                    select_closest_depth_sample((self.sample(sample_stamp),), 1.0, 0.1)


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
        self.assertEqual(
            {
                "DepthSample",
                "deproject_pixel",
                "depth_image_to_meters",
                "roi_mean_depth",
                "select_closest_depth_sample",
            },
            geometry_imports,
        )

    def test_node_initializes_locked_bounded_queue_and_validates_parameters(self):
        initializer = self.function_source("__init__")
        loader = self.function_source("_load_ros_params")

        self.assertIn("self.latest_camera_info = None", initializer)
        self.assertIn("self.sensor_lock = threading.Lock()", initializer)
        self.assertIn("self.depth_samples = deque(maxlen=self.depth_queue_size)", initializer)
        self.assertIn('rospy.get_param("~maximum_sensor_delta", 0.15)', loader)
        self.assertIn('rospy.get_param("~depth_queue_size", 60)', loader)
        self.assertIn("isinstance(configured_queue_size, bool)", loader)
        self.assertIn("isinstance(configured_queue_size, int)", loader)
        self.assertIn("configured_queue_size <= 0", loader)
        self.assertIn("math.isfinite", loader)
        self.assertIn("< 0", loader)
        self.assertIn("rospy.logfatal", loader)
        self.assertNotIn("~camera_frame", self.source)

    def test_transform_listener_owns_static_tf_and_robot_dynamic_tf_stays_manual(self):
        initializer = self.function_source("__init__")
        subscriptions = self.function_source("_setup_subscriptions")

        self.assertIn(
            "tf2_ros.TransformListener(self.transform_buffer)", initializer
        )
        self.assertNotIn('"/tf_static"', subscriptions)
        self.assertNotIn("_tf_static_callback", self.source)
        self.assertIn('f"/{self.robot_name}/tf"', subscriptions)
        self.assertIn("TFMessage", subscriptions)
        self.assertIn("self._tf_callback", subscriptions)

    def test_node_keeps_latest_valid_camera_info_when_invalid_message_arrives(self):
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
        self.assertIn("with self.sensor_lock:", callback)
        self.assertIn("self.latest_camera_info = message", callback)
        self.assertNotIn("self.latest_camera_info = None", callback)
        self.assertIn("CameraInfo 無效", callback)

    def test_depth_callback_validates_and_appends_complete_sample_under_lock(self):
        depth_callback = self.function_source("_depth_image_callback")

        self.assertIn("converted_frame = self.cv_bridge.imgmsg_to_cv2", depth_callback)
        self.assertIn("depth_image_to_meters(converted_frame, encoding)", depth_callback)
        self.assertIn("stamp.is_zero()", depth_callback)
        self.assertIn("math.isfinite(stamp_seconds)", depth_callback)
        self.assertIn("image_msg.header.frame_id", depth_callback)
        self.assertIn("DepthSample(", depth_callback)
        for field in ("depth_meters", "stamp", "stamp_seconds", "frame_id", "encoding"):
            self.assertIn(f"{field}=", depth_callback)
        self.assertIn("with self.sensor_lock:", depth_callback)
        self.assertIn("self.depth_samples.append(sample)", depth_callback)
        self.assertNotIn(".clear(", depth_callback)

    def test_detection_callback_snapshots_queue_and_camera_once_under_lock(self):
        callback_node = self.function_node("bounding_box_callback")
        detection_callback = self.function_source("bounding_box_callback")
        executable_body = callback_node.body
        if (
            executable_body
            and isinstance(executable_body[0], ast.Expr)
            and isinstance(executable_body[0].value, ast.Str)
        ):
            executable_body = executable_body[1:]

        snapshot_block = executable_body[0]
        self.assertIsInstance(snapshot_block, ast.With)
        snapshot_source = ast.get_source_segment(self.source, snapshot_block)
        self.assertIn("with self.sensor_lock:", snapshot_source)
        self.assertIn("depth_samples = tuple(self.depth_samples)", snapshot_source)
        self.assertIn("camera_info = self.latest_camera_info", snapshot_source)
        self.assertEqual(1, detection_callback.count("self.depth_samples"))
        self.assertEqual(1, detection_callback.count("self.latest_camera_info"))
        self.assertIn("camera_info.width", detection_callback)
        self.assertIn("camera_info.height", detection_callback)
        self.assertIn("detections_msg.header.stamp", detection_callback)
        self.assertIn("select_closest_depth_sample", detection_callback)
        self.assertIn("self.maximum_sensor_delta", detection_callback)
        self.assertIn("rospy.logwarn_throttle", detection_callback)

    def test_detection_requires_matching_official_frames_and_selected_dimensions(self):
        detection_callback = self.function_source("bounding_box_callback")

        self.assertIn("selected_sample.frame_id", detection_callback)
        self.assertIn("camera_info.header.frame_id", detection_callback)
        self.assertIn("detections_msg.header.frame_id", detection_callback)
        self.assertIn("selected_sample.depth_meters.shape", detection_callback)
        self.assertIn("camera_info.width", detection_callback)
        self.assertIn("camera_info.height", detection_callback)

    def test_roi_and_point_use_only_selected_sample_and_camera_snapshot(self):
        calculation = self.function_source("_calculate_3d_point")

        self.assertNotIn("def _get_roi_mean_depth", self.source)
        self.assertIn(
            "def _calculate_3d_point(self, u, v, depth_in_meters, camera_info, depth_stamp)",
            calculation,
        )
        self.assertIn("deproject_pixel", calculation)
        self.assertIn("camera_info.K", calculation)
        self.assertIn("camera_info.header.frame_id", calculation)
        self.assertIn("point.header.stamp = depth_stamp", calculation)
        self.assertNotIn("self.latest_", calculation)
        self.assertNotIn("camera_frame_override", self.source)

        detection_callback = self.function_source("bounding_box_callback")
        self.assertIn("roi_mean_depth(", detection_callback)
        self.assertIn("selected_sample.depth_meters", detection_callback)
        self.assertIn(
            "self._calculate_3d_point(\n"
            "                        center_u, center_v, mean_depth, camera_info, selected_sample.stamp\n"
            "                    )",
            detection_callback,
        )

    def test_obsolete_heuristic_and_unused_ros_imports_are_absent(self):
        self.assertNotIn("mean_depth_mm", self.source)
        self.assertNotIn("> 100", self.source)
        imported_names = set()
        imported_modules = set()
        for node in ast.walk(self.tree):
            if isinstance(node, ast.Import):
                imported_modules.update(alias.name for alias in node.names)
            elif isinstance(node, ast.ImportFrom):
                imported_names.update(alias.name for alias in node.names)
                imported_modules.add(node.module)
        for forbidden_import in (
            "TransformStamped",
            "Odometry",
            "BoundingBox",
        ):
            with self.subTest(forbidden_import=forbidden_import):
                self.assertNotIn(forbidden_import, imported_names)
        for forbidden_module in ("tf.transformations",):
            with self.subTest(forbidden_module=forbidden_module):
                self.assertNotIn(forbidden_module, imported_modules)
        self.assertIn("tf2_geometry_msgs", imported_modules)

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

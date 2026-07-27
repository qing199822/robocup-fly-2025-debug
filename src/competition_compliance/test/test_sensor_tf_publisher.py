#!/usr/bin/env python3

import importlib.util
import pathlib
import tempfile
import types
import unittest
from unittest import mock


WORKSPACE = pathlib.Path(__file__).resolve().parents[3]
PUBLISHER_PATH = (
    WORKSPACE / "src/competition_compliance/scripts/sensor_tf_publisher.py"
)


class FakeTransformStamped:
    def __init__(self):
        self.header = types.SimpleNamespace(stamp=None, frame_id=None)
        self.child_frame_id = None
        self.transform = types.SimpleNamespace(
            translation=types.SimpleNamespace(x=None, y=None, z=None),
            rotation=types.SimpleNamespace(x=None, y=None, z=None, w=None),
        )


def load_publisher_module():
    spec = importlib.util.spec_from_file_location(
        "competition_sensor_tf_test_module", str(PUBLISHER_PATH)
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class SensorTfPublisherTest(unittest.TestCase):
    def setUp(self):
        self.publisher = load_publisher_module()
        self.rospy = mock.Mock()
        self.stamp = object()
        self.rospy.Time.now.return_value = self.stamp
        self.broadcaster = mock.Mock()
        self.tf2_ros = mock.Mock()
        self.tf2_ros.StaticTransformBroadcaster.return_value = self.broadcaster
        self.publisher.rospy = self.rospy
        self.publisher.tf2_ros = self.tf2_ros
        self.publisher.TransformStamped = FakeTransformStamped

    def test_main_publishes_one_fixed_frame_transform_and_spins(self):
        with tempfile.TemporaryDirectory() as directory:
            config = pathlib.Path(directory) / "mount.yaml"
            config.write_text(
                "realsense_mount: [0.09, 0, -0.04, 0, 0, 0]\n",
                encoding="utf-8",
            )
            self.rospy.get_param.return_value = str(config)

            result = self.publisher.main()

        self.assertEqual(0, result)
        self.rospy.init_node.assert_called_once_with("competition_sensor_tf")
        self.rospy.get_param.assert_called_once_with("~mount_config")
        self.tf2_ros.StaticTransformBroadcaster.assert_called_once_with()
        self.broadcaster.sendTransform.assert_called_once()
        transform = self.broadcaster.sendTransform.call_args.args[0]
        self.assertIs(self.stamp, transform.header.stamp)
        self.assertEqual("base_link", transform.header.frame_id)
        self.assertEqual("depth_camera_base", transform.child_frame_id)
        self.assertEqual(
            (0.09, 0.0, -0.04),
            (
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ),
        )
        for expected, actual in zip(
            (-0.5, 0.5, -0.5, 0.5),
            (
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            ),
        ):
            self.assertAlmostEqual(expected, actual)
        self.rospy.spin.assert_called_once_with()

    def test_missing_mount_config_logs_fatal_and_returns_two(self):
        self.rospy.get_param.side_effect = KeyError("~mount_config")

        result = self.publisher.main()

        self.assertEqual(2, result)
        self.rospy.logfatal.assert_called_once()
        self.assertIn("合规自检失败", self.rospy.logfatal.call_args.args[0])
        self.tf2_ros.StaticTransformBroadcaster.assert_not_called()
        self.rospy.spin.assert_not_called()

    def test_malformed_mount_config_logs_fatal_and_returns_two(self):
        with tempfile.TemporaryDirectory() as directory:
            config = pathlib.Path(directory) / "mount.yaml"
            config.write_text("realsense_mount: [0, 0\n", encoding="utf-8")
            self.rospy.get_param.return_value = str(config)

            result = self.publisher.main()

        self.assertEqual(2, result)
        self.rospy.logfatal.assert_called_once()
        self.assertIn("合规自检失败", self.rospy.logfatal.call_args.args[0])
        self.tf2_ros.StaticTransformBroadcaster.assert_not_called()
        self.rospy.spin.assert_not_called()


if __name__ == "__main__":
    unittest.main()

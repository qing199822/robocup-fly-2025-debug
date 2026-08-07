#!/usr/bin/env python3

import struct
import threading
import unittest

import rospy
import rostest
import tf2_ros
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from search_msgs.msg import LocalClearance, PerceptionHealth
from sensor_msgs.msg import CameraInfo, Image, PointCloud2


class LocalMappingNodeTest(unittest.TestCase):
    WIDTH = 16
    HEIGHT = 16

    def setUp(self):
        self._lock = threading.Lock()
        self._planner_depth = None
        self._static_cloud = None
        self._dynamic_cloud = None
        self._health = None
        self._clearance = None
        self._frontier_goal = None

        self._depth_pub = rospy.Publisher(
            "/test_drone_0/realsense/depth_camera/depth/image_raw",
            Image,
            queue_size=1,
        )
        self._camera_info_pub = rospy.Publisher(
            "/test_drone_0/realsense/depth_camera/depth/camera_info",
            CameraInfo,
            queue_size=1,
        )
        self._odom_pub = rospy.Publisher(
            "/test_drone_0/global_odom", Odometry, queue_size=1
        )
        self._boxes_pub = rospy.Publisher(
            "/test_drone_0/yolo11n/bounding_boxes",
            BoundingBoxes,
            queue_size=1,
        )

        self._subscribers = [
            rospy.Subscriber(
                "/test_drone_0/local_mapping/planner_depth",
                Image,
                self._store("_planner_depth"),
                queue_size=1,
            ),
            rospy.Subscriber(
                "/test_drone_0/local_mapping/static_cloud",
                PointCloud2,
                self._store("_static_cloud"),
                queue_size=1,
            ),
            rospy.Subscriber(
                "/test_drone_0/local_mapping/dynamic_cloud",
                PointCloud2,
                self._store("_dynamic_cloud"),
                queue_size=1,
            ),
            rospy.Subscriber(
                "/test_drone_0/local_mapping/health",
                PerceptionHealth,
                self._store("_health"),
                queue_size=1,
            ),
            rospy.Subscriber(
                "/test_drone_0/local_mapping/clearance",
                LocalClearance,
                self._store("_clearance"),
                queue_size=1,
            ),
            rospy.Subscriber(
                "/test_drone_0/local_mapping/frontier_goal",
                PoseStamped,
                self._store("_frontier_goal"),
                queue_size=1,
            ),
        ]

        static_tf = TransformStamped()
        static_tf.header.stamp = rospy.Time.now()
        static_tf.header.frame_id = "base_link"
        static_tf.child_frame_id = "test_camera"
        # 相机光学坐标 (X, Y, Z) 对应机体坐标 (Y, Z, X)。
        static_tf.transform.rotation.x = 0.5
        static_tf.transform.rotation.y = 0.5
        static_tf.transform.rotation.z = 0.5
        static_tf.transform.rotation.w = 0.5
        self._static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self._static_tf_broadcaster.sendTransform(static_tf)

    def _store(self, attribute):
        def callback(message):
            with self._lock:
                setattr(self, attribute, message)

        return callback

    def _snapshot(self, attribute):
        with self._lock:
            return getattr(self, attribute)

    def _publish_odom(self, stamp):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.z = 3.0
        odom.pose.pose.orientation.w = 1.0
        self._odom_pub.publish(odom)

    def _publish_synchronized_inputs(self):
        stamp = rospy.Time.now()

        boxes = BoundingBoxes()
        boxes.header.stamp = stamp
        boxes.image_header.stamp = stamp
        person = BoundingBox()
        person.Class = "green0"
        person.probability = 0.99
        person.xmin = 4
        person.ymin = 4
        person.xmax = 7
        person.ymax = 7
        boxes.bounding_boxes = [person]
        self._boxes_pub.publish(boxes)

        depth = Image()
        depth.header.stamp = stamp
        depth.header.frame_id = "test_camera"
        depth.height = self.HEIGHT
        depth.width = self.WIDTH
        depth.encoding = "16UC1"
        depth.is_bigendian = False
        depth.step = self.WIDTH * 2
        depth.data = struct.pack(
            "<{}H".format(self.WIDTH * self.HEIGHT),
            *([2000] * (self.WIDTH * self.HEIGHT))
        )

        camera_info = CameraInfo()
        camera_info.header = depth.header
        camera_info.height = self.HEIGHT
        camera_info.width = self.WIDTH
        camera_info.K = [
            12.0,
            0.0,
            7.5,
            0.0,
            12.0,
            7.5,
            0.0,
            0.0,
            1.0,
        ]

        self._depth_pub.publish(depth)
        self._camera_info_pub.publish(camera_info)
        self._publish_odom(stamp)

    def _publish_inputs_for(self, seconds):
        deadline = rospy.Time.now() + rospy.Duration(seconds)
        rate = rospy.Rate(30)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            self._publish_synchronized_inputs()
            rate.sleep()

    def _keep_odom_for(self, seconds):
        deadline = rospy.Time.now() + rospy.Duration(seconds)
        rate = rospy.Rate(30)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            self._publish_odom(rospy.Time.now())
            rate.sleep()

    def _wait_for(self, predicate, message, timeout=4.0):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(100)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if predicate():
                return
            rate.sleep()
        self.fail(message)

    def test_semantic_map_health_timeout_and_dynamic_ttl(self):
        self._publish_inputs_for(1.0)
        self._wait_for(
            lambda: self._snapshot("_health") is not None
            and self._snapshot("_health").map_healthy,
            "mapping health did not recover",
        )
        self._wait_for(
            lambda: self._snapshot("_static_cloud") is not None
            and len(self._snapshot("_static_cloud").data) > 0,
            "static point cloud stayed empty",
        )
        self._wait_for(
            lambda: self._snapshot("_dynamic_cloud") is not None
            and len(self._snapshot("_dynamic_cloud").data) > 0,
            "dynamic point cloud stayed empty",
        )
        self._wait_for(
            lambda: self._snapshot("_clearance") is not None,
            "clearance was not published",
        )
        self._wait_for(
            lambda: self._snapshot("_frontier_goal") is not None,
            "frontier goal was not published",
        )

        health = self._snapshot("_health")
        planner_depth = self._snapshot("_planner_depth")
        static_cloud = self._snapshot("_static_cloud")
        dynamic_cloud = self._snapshot("_dynamic_cloud")
        clearance = self._snapshot("_clearance")
        frontier_goal = self._snapshot("_frontier_goal")
        self.assertTrue(health.synchronized)
        self.assertEqual("OK", health.fault_code)
        self.assertEqual("map", health.header.frame_id)
        self.assertIsNotNone(planner_depth)
        self.assertEqual("16UC1", planner_depth.encoding)
        self.assertEqual("test_camera", planner_depth.header.frame_id)
        person_offset = 4 * planner_depth.step + 4 * 2
        self.assertEqual(
            2000,
            struct.unpack_from("<H", planner_depth.data, person_offset)[0],
        )
        self.assertEqual("map", static_cloud.header.frame_id)
        self.assertEqual("map", dynamic_cloud.header.frame_id)
        self.assertEqual("map", clearance.header.frame_id)
        self.assertEqual("map", frontier_goal.header.frame_id)
        self.assertAlmostEqual(3.0, frontier_goal.pose.position.z)

        self._keep_odom_for(1.0)
        self._wait_for(
            lambda: self._snapshot("_health") is not None
            and self._snapshot("_health").fault_code == "DEPTH_TIMEOUT",
            "stale depth did not report DEPTH_TIMEOUT",
        )
        self._wait_for(
            lambda: self._snapshot("_dynamic_cloud") is not None
            and len(self._snapshot("_dynamic_cloud").data) == 0,
            "dynamic point cloud did not expire",
        )
        self.assertFalse(self._snapshot("_health").map_healthy)


if __name__ == "__main__":
    rospy.init_node("local_mapping_node_test")
    rostest.rosrun("local_mapping", "local_mapping_node_test", LocalMappingNodeTest)

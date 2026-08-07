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
from sensor_msgs import point_cloud2
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

    def _make_odom(self, stamp, parent_frame="map", child_frame="base_link"):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = parent_frame
        odom.child_frame_id = child_frame
        odom.pose.pose.position.z = 3.0
        odom.pose.pose.orientation.w = 1.0
        return odom

    def _publish_odom(
        self, stamp, parent_frame="map", child_frame="base_link"
    ):
        odom = self._make_odom(stamp, parent_frame, child_frame)
        self._odom_pub.publish(odom)

    def _publish_boxes(self, stamp, include_person=True):
        boxes = BoundingBoxes()
        boxes.header.stamp = stamp
        boxes.image_header.stamp = stamp
        if include_person:
            person = BoundingBox()
            person.Class = "green0"
            person.probability = 0.99
            person.xmin = 4
            person.ymin = 4
            person.xmax = 7
            person.ymax = 7
            boxes.bounding_boxes = [person]
        self._boxes_pub.publish(boxes)

    def _make_depth(
        self,
        stamp,
        depth_mm=2000,
        frame="test_camera",
        encoding="16UC1",
        sequence=0,
    ):
        depth = Image()
        depth.header.seq = sequence
        depth.header.stamp = stamp
        depth.header.frame_id = frame
        depth.height = self.HEIGHT
        depth.width = self.WIDTH
        depth.encoding = encoding
        depth.is_bigendian = False
        depth.step = self.WIDTH * 2
        depth.data = struct.pack(
            "<{}H".format(self.WIDTH * self.HEIGHT),
            *([depth_mm] * (self.WIDTH * self.HEIGHT))
        )
        return depth

    def _make_camera_info(self, stamp, frame="test_camera"):
        camera_info = CameraInfo()
        camera_info.header.stamp = stamp
        camera_info.header.frame_id = frame
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
        return camera_info

    def _publish_depth(
        self,
        stamp,
        depth_mm=2000,
        frame="test_camera",
        encoding="16UC1",
        sequence=0,
    ):
        self._depth_pub.publish(
            self._make_depth(
                stamp, depth_mm, frame, encoding, sequence
            )
        )

    def _publish_camera_info_and_odom(self, stamp):
        self._camera_info_pub.publish(self._make_camera_info(stamp))
        self._odom_pub.publish(self._make_odom(stamp))

    def _publish_synchronized_inputs(
        self,
        depth_mm=2000,
        depth_frame="test_camera",
        info_frame="test_camera",
        odom_parent="map",
        odom_child="base_link",
        boxes="person",
        boxes_stamp=None,
        stamp=None,
    ):
        if stamp is None:
            stamp = rospy.Time.now()
        if boxes is not None:
            self._publish_boxes(
                stamp if boxes_stamp is None else boxes_stamp,
                include_person=boxes == "person",
            )
        depth = self._make_depth(stamp, depth_mm, depth_frame)
        camera_info = self._make_camera_info(stamp, info_frame)
        odom = self._make_odom(stamp, odom_parent, odom_child)

        self._depth_pub.publish(depth)
        self._camera_info_pub.publish(camera_info)
        self._odom_pub.publish(odom)
        return stamp

    def _prime_box_cache(self, stamp, include_person=True):
        dropped_before = self._snapshot("_health").dropped_frames
        deadline = rospy.Time.now() + rospy.Duration(1.0)
        rate = rospy.Rate(100)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            self._publish_boxes(stamp, include_person=include_person)
            health = self._snapshot("_health")
            if health.dropped_frames > dropped_before:
                return
            rate.sleep()
        self.fail("BoundingBoxes cache did not acknowledge the test stamp")

    def _publish_for(self, seconds, publisher):
        deadline = rospy.Time.now() + rospy.Duration(seconds)
        rate = rospy.Rate(30)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            publisher()
            rate.sleep()

    def _publish_inputs_for(self, seconds, **kwargs):
        self._publish_for(
            seconds, lambda: self._publish_synchronized_inputs(**kwargs)
        )

    def _keep_depth_for(self, seconds):
        self._publish_for(
            seconds,
            lambda: self._publish_depth(rospy.Time.now()),
        )

    def _keep_odom_for(self, seconds):
        self._publish_for(
            seconds,
            lambda: self._publish_odom(rospy.Time.now()),
        )

    def _keep_depth_and_odom_for(self, seconds):
        def publish():
            stamp = rospy.Time.now()
            self._publish_depth(stamp)
            self._publish_odom(stamp)

        self._publish_for(seconds, publish)

    def _wait_for(self, predicate, message, timeout=4.0):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(100)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if predicate():
                return
            rate.sleep()
        self.fail(message)

    def _wait_for_mapping_publications_after_planner(
        self, planner_stamp, message
    ):
        self._wait_for(
            lambda: self._snapshot("_planner_depth") is not None
            and self._snapshot("_planner_depth").header.stamp > planner_stamp,
            message + " did not reach the synchronized callback",
        )
        health_stamp = self._snapshot("_health").header.stamp
        static_stamp = self._snapshot("_static_cloud").header.stamp
        dynamic_stamp = self._snapshot("_dynamic_cloud").header.stamp
        self._wait_for(
            lambda: self._snapshot("_health").header.stamp > health_stamp,
            message + " did not produce a new health publication",
        )
        self._wait_for(
            lambda: self._snapshot("_static_cloud").header.stamp
            > static_stamp,
            message + " did not produce a new static cloud publication",
        )
        self._wait_for(
            lambda: self._snapshot("_dynamic_cloud").header.stamp
            > dynamic_stamp,
            message + " did not produce a new dynamic cloud publication",
        )

    def _cloud_points(self, attribute):
        return set(
            point_cloud2.read_points(
                self._snapshot(attribute),
                field_names=("x", "y", "z"),
                skip_nans=True,
            )
        )

    def _assert_frame_error(self, message, **kwargs):
        planner_stamp = self._snapshot("_planner_depth").header.stamp
        dynamic_points = self._cloud_points("_dynamic_cloud")
        stamp = rospy.Time.now()
        self._prime_box_cache(stamp)
        self._publish_synchronized_inputs(stamp=stamp, boxes=None, **kwargs)
        self._wait_for_mapping_publications_after_planner(
            planner_stamp, message
        )
        health = self._snapshot("_health")
        self.assertFalse(health.map_healthy, message)
        self.assertEqual("FRAME_ERROR", health.fault_code, message)
        self.assertTrue(
            self._cloud_points("_dynamic_cloud").issubset(dynamic_points),
            message + ": the rejected frame added a dynamic voxel",
        )

    def test_semantic_map_health_timeout_and_dynamic_ttl(self):
        self._wait_for(
            lambda: self._snapshot("_health") is not None
            and self._snapshot("_static_cloud") is not None
            and self._snapshot("_dynamic_cloud") is not None,
            "initial mapping publications were not available",
        )

        planner_stamp = rospy.Time(0)
        static_before = self._cloud_points("_static_cloud")
        dynamic_before = self._cloud_points("_dynamic_cloud")
        self._publish_inputs_for(0.35, depth_mm=4200, boxes=None)
        self._wait_for_mapping_publications_after_planner(
            planner_stamp, "depth without BoundingBoxes"
        )
        self.assertTrue(
            self._cloud_points("_static_cloud").issubset(static_before),
            "depth without BoundingBoxes added a static voxel",
        )
        self.assertTrue(
            self._cloud_points("_dynamic_cloud").issubset(dynamic_before),
            "depth without BoundingBoxes added a dynamic voxel",
        )
        self._wait_for(
            lambda: not self._snapshot("_health").map_healthy
            and self._snapshot("_health").fault_code == "SYNC_ERROR",
            "depth without BoundingBoxes did not report SYNC_ERROR",
        )

        planner_stamp = self._snapshot("_planner_depth").header.stamp
        static_before = self._cloud_points("_static_cloud")
        dynamic_before = self._cloud_points("_dynamic_cloud")
        stale_boxes_stamp = rospy.Time.now() - rospy.Duration(1.0)
        self._publish_inputs_for(
            0.35,
            depth_mm=5200,
            boxes="person",
            boxes_stamp=stale_boxes_stamp,
        )
        self._wait_for_mapping_publications_after_planner(
            planner_stamp, "depth with stale BoundingBoxes"
        )
        self.assertTrue(
            self._cloud_points("_static_cloud").issubset(static_before),
            "depth with stale BoundingBoxes added a static voxel",
        )
        self.assertTrue(
            self._cloud_points("_dynamic_cloud").issubset(dynamic_before),
            "depth with stale BoundingBoxes added a dynamic voxel",
        )
        self._wait_for(
            lambda: not self._snapshot("_health").map_healthy
            and self._snapshot("_health").fault_code == "SYNC_ERROR",
            "depth with stale BoundingBoxes did not report SYNC_ERROR",
        )

        static_before = self._cloud_points("_static_cloud")
        dynamic_before = self._cloud_points("_dynamic_cloud")
        self._publish_inputs_for(0.5, depth_mm=3200, boxes="empty")
        self._wait_for(
            lambda: self._snapshot("_health") is not None
            and self._snapshot("_health").map_healthy,
            "fresh empty BoundingBoxes did not restore mapping",
        )
        self._wait_for(
            lambda: bool(
                self._cloud_points("_static_cloud") - static_before
            ),
            "fresh empty BoundingBoxes did not add static depth voxels",
        )
        self.assertTrue(
            self._cloud_points("_dynamic_cloud").issubset(dynamic_before),
            "fresh empty BoundingBoxes added a dynamic voxel",
        )

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

        health_stamp = health.header.stamp
        self._keep_depth_and_odom_for(0.55)
        self._wait_for(
            lambda: self._snapshot("_health").header.stamp > health_stamp,
            "CameraInfo stall did not produce a new health publication",
            timeout=0.2,
        )
        health = self._snapshot("_health")
        self.assertTrue(
            health.depth_healthy,
            "CameraInfo stall incorrectly marked depth unhealthy",
        )
        self.assertTrue(
            health.odom_healthy,
            "CameraInfo stall incorrectly marked odometry unhealthy",
        )
        self.assertTrue(
            health.synchronized,
            "CameraInfo stall incorrectly desynchronized depth and odometry",
        )
        self.assertFalse(
            health.map_healthy,
            "stale fusion stayed healthy after CameraInfo stopped",
        )
        self.assertEqual("SYNC_ERROR", health.fault_code)

        self._keep_depth_for(0.7)
        self._wait_for(
            lambda: self._snapshot("_health") is not None
            and self._snapshot("_health").depth_healthy
            and not self._snapshot("_health").odom_healthy
            and self._snapshot("_health").fault_code == "ODOM_TIMEOUT",
            "depth-only input did not isolate the odometry timeout",
        )

        self._publish_inputs_for(0.5)
        self._wait_for(
            lambda: self._snapshot("_health") is not None
            and self._snapshot("_health").map_healthy,
            "mapping did not recover after the odometry timeout",
        )

        self._keep_odom_for(0.7)
        self._wait_for(
            lambda: self._snapshot("_health") is not None
            and not self._snapshot("_health").depth_healthy
            and self._snapshot("_health").odom_healthy
            and self._snapshot("_health").fault_code == "DEPTH_TIMEOUT",
            "odometry-only input did not isolate the depth timeout",
        )
        self._wait_for(
            lambda: self._snapshot("_dynamic_cloud") is not None
            and len(self._snapshot("_dynamic_cloud").data) == 0,
            "dynamic point cloud did not expire",
        )
        self.assertFalse(self._snapshot("_health").map_healthy)

        planner_stamp_before_recovery = self._snapshot(
            "_planner_depth"
        ).header.stamp
        self._publish_synchronized_inputs(depth_mm=3000)
        self._wait_for_mapping_publications_after_planner(
            planner_stamp_before_recovery,
            "the isolated recovery frame",
        )
        self.assertFalse(
            self._snapshot("_health").map_healthy,
            "one synchronized frame bypassed the recovery window",
        )
        self.assertEqual(
            0,
            len(self._snapshot("_dynamic_cloud").data),
            "a frame inside the recovery window was fused",
        )

        self._keep_odom_for(0.4)
        self._assert_frame_error(
            "a frame error during health recovery was hidden",
            depth_mm=3600,
            odom_parent="/wrong_map",
        )

        self._publish_inputs_for(0.5, depth_mm=3000)
        self._wait_for(
            lambda: self._snapshot("_health") is not None
            and self._snapshot("_health").map_healthy,
            "continuous synchronized input did not restore mapping",
        )
        self._wait_for(
            lambda: self._snapshot("_dynamic_cloud") is not None
            and len(self._snapshot("_dynamic_cloud").data) > 0,
            "recovered synchronized input was not fused",
        )

        self._assert_frame_error(
            "wrong odometry parent frame did not fail closed",
            depth_mm=4000,
            odom_parent="/wrong_map",
        )

        self._assert_frame_error(
            "wrong odometry child frame did not fail closed",
            depth_mm=5000,
            odom_child="wrong_base",
        )

        self._assert_frame_error(
            "mismatched depth and CameraInfo frames did not fail closed",
            depth_mm=6000,
            depth_frame="wrong_camera",
            info_frame="test_camera",
        )

        self._assert_frame_error(
            "wrong effective camera frame did not fail closed",
            depth_mm=7000,
            depth_frame="/wrong_camera",
            info_frame="/wrong_camera",
        )

        self._publish_inputs_for(
            0.3,
            depth_frame="/test_camera",
            info_frame="/test_camera",
            odom_parent="/map",
            odom_child="/base_link",
        )
        self._wait_for(
            lambda: self._snapshot("_health") is not None
            and self._snapshot("_health").map_healthy,
            "correct normalized frames did not restore mapping",
        )

        dropped_before = self._snapshot("_health").dropped_frames
        bad_stamp_a = rospy.Time.now() + rospy.Duration(1.0)
        bad_stamp_b = bad_stamp_a + rospy.Duration(0.01)
        self._publish_depth(
            bad_stamp_a, encoding="8UC1", sequence=101
        )
        self._wait_for(
            lambda: self._snapshot("_health").dropped_frames
            >= dropped_before + 1,
            "first invalid depth frame was not counted",
        )
        self._publish_depth(
            bad_stamp_b, encoding="8UC1", sequence=102
        )
        self._wait_for(
            lambda: self._snapshot("_health").dropped_frames
            >= dropped_before + 2,
            "second invalid depth frame was not counted",
        )
        health_stamp = self._snapshot("_health").header.stamp
        self._publish_camera_info_and_odom(bad_stamp_a)
        self._wait_for(
            lambda: self._snapshot("_health").header.stamp > health_stamp
            and self._snapshot("_health").fault_code
            == "DEPTH_ENCODING_ERROR",
            "the delayed invalid depth synchronization was not processed",
        )
        self.assertEqual(
            dropped_before + 2,
            self._snapshot("_health").dropped_frames,
            "a delayed raw/sync duplicate depth frame was counted twice",
        )


if __name__ == "__main__":
    rospy.init_node("local_mapping_node_test")
    rostest.rosrun("local_mapping", "local_mapping_node_test", LocalMappingNodeTest)

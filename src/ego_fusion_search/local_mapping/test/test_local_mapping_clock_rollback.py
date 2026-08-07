#!/usr/bin/env python3

import struct
import threading
import time
import unittest

import rospy
import rostest
import tf2_ros
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from search_msgs.msg import PerceptionHealth
from sensor_msgs.msg import CameraInfo, Image, PointCloud2


class WallClockDriver:
    def __init__(self, publisher, initial_time):
        self._publisher = publisher
        self._lock = threading.Lock()
        self._seconds = initial_time
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run)
        self._thread.daemon = True

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=1.0)

    def jump(self, seconds):
        with self._lock:
            self._seconds = seconds
        self._publish(seconds)

    def now(self):
        with self._lock:
            return rospy.Time.from_sec(self._seconds)

    def _publish(self, seconds):
        message = Clock()
        message.clock = rospy.Time.from_sec(seconds)
        self._publisher.publish(message)

    def _run(self):
        last_wall = time.monotonic()
        while not self._stop.is_set() and not rospy.is_shutdown():
            current_wall = time.monotonic()
            with self._lock:
                self._seconds += current_wall - last_wall
                seconds = self._seconds
            last_wall = current_wall
            self._publish(seconds)
            self._stop.wait(0.01)


class LocalMappingClockRollbackTest(unittest.TestCase):
    WIDTH = 16
    HEIGHT = 16

    def setUp(self):
        self._lock = threading.Lock()
        self._health = None
        self._static_cloud = None
        self._dynamic_cloud = None
        self._frontier_count = 0
        self._input_stop = threading.Event()
        self._input_thread = None

        self._clock_pub = rospy.Publisher("/clock", Clock, queue_size=1)
        self._depth_pub = rospy.Publisher("/rollback/depth", Image, queue_size=1)
        self._camera_info_pub = rospy.Publisher(
            "/rollback/camera_info", CameraInfo, queue_size=1
        )
        self._odom_pub = rospy.Publisher("/rollback/odom", Odometry, queue_size=1)
        self._boxes_pub = rospy.Publisher(
            "/rollback/boxes", BoundingBoxes, queue_size=1
        )
        self._subscribers = [
            rospy.Subscriber(
                "/rollback/health", PerceptionHealth, self._store("_health")
            ),
            rospy.Subscriber(
                "/rollback/static_cloud", PointCloud2, self._store("_static_cloud")
            ),
            rospy.Subscriber(
                "/rollback/dynamic_cloud",
                PointCloud2,
                self._store("_dynamic_cloud"),
            ),
            rospy.Subscriber(
                "/rollback/frontier_goal", PoseStamped, self._frontier_callback
            ),
        ]

        static_tf = TransformStamped()
        static_tf.header.frame_id = "base_link"
        static_tf.child_frame_id = "rollback_camera"
        static_tf.transform.rotation.x = 0.5
        static_tf.transform.rotation.y = 0.5
        static_tf.transform.rotation.z = 0.5
        static_tf.transform.rotation.w = 0.5
        self._tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self._tf_broadcaster.sendTransform(static_tf)

        self._clock = WallClockDriver(self._clock_pub, 100.0)
        self._clock.start()

    def tearDown(self):
        self._stop_input_stream()
        self._clock.stop()

    def _store(self, attribute):
        def callback(message):
            with self._lock:
                setattr(self, attribute, message)

        return callback

    def _frontier_callback(self, _message):
        with self._lock:
            self._frontier_count += 1

    def _snapshot(self):
        with self._lock:
            return (
                self._health,
                self._static_cloud,
                self._dynamic_cloud,
                self._frontier_count,
            )

    def _wait_for(self, predicate, message, timeout=5.0):
        deadline = time.monotonic() + timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.01)
        self.fail(message)

    def _wait_for_connections(self):
        publishers = [
            self._depth_pub,
            self._camera_info_pub,
            self._odom_pub,
            self._boxes_pub,
        ]
        self._wait_for(
            lambda: all(publisher.get_num_connections() > 0 for publisher in publishers),
            "mapping input publishers did not connect",
        )

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

    def _publish_inputs(self, publish_boxes=True):
        stamp = self._clock.now()
        if publish_boxes:
            self._publish_boxes(stamp)

        depth = Image()
        depth.header.stamp = stamp
        depth.header.frame_id = "rollback_camera"
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
        camera_info.header.stamp = stamp
        camera_info.header.frame_id = "rollback_camera"
        camera_info.height = self.HEIGHT
        camera_info.width = self.WIDTH
        camera_info.K = [12.0, 0.0, 7.5, 0.0, 12.0, 7.5, 0.0, 0.0, 1.0]

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.z = 3.0
        odom.pose.pose.orientation.w = 1.0

        self._depth_pub.publish(depth)
        self._camera_info_pub.publish(camera_info)
        self._odom_pub.publish(odom)

    def _publish_inputs_for(self, wall_seconds, publish_boxes=True):
        deadline = time.monotonic() + wall_seconds
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            self._publish_inputs(publish_boxes=publish_boxes)
            time.sleep(0.02)

    def _start_input_stream(self):
        self._input_stop.clear()

        def publish():
            while not self._input_stop.is_set() and not rospy.is_shutdown():
                self._publish_inputs()
                self._input_stop.wait(0.02)

        self._input_thread = threading.Thread(target=publish)
        self._input_thread.daemon = True
        self._input_thread.start()

    def _stop_input_stream(self):
        self._input_stop.set()
        if self._input_thread is not None:
            self._input_thread.join(timeout=1.0)
            self._input_thread = None

    def test_clock_rollback_starts_empty_epoch_and_recovers(self):
        self._wait_for_connections()
        self._wait_for(
            lambda: all(message is not None for message in self._snapshot()[:3]),
            "initial mapping publications were not available",
        )

        self._publish_inputs_for(0.8)
        self._wait_for(
            lambda: self._snapshot()[0].map_healthy
            and self._snapshot()[1].width > 0
            and self._snapshot()[2].width > 0
            and self._snapshot()[3] > 0,
            "the first epoch did not build a healthy static and dynamic map",
        )

        old_epoch_stamp = self._clock.now()
        self._start_input_stream()
        self._clock.jump(1.0)
        self._wait_for(
            lambda: self._snapshot()[0].header.stamp.to_sec() < 5.0
            and not self._snapshot()[0].map_healthy
            and self._snapshot()[1].header.stamp.to_sec() < 5.0
            and self._snapshot()[1].width == 0
            and self._snapshot()[2].header.stamp.to_sec() < 5.0
            and self._snapshot()[2].width == 0,
            "clock rollback did not publish an unhealthy empty map epoch",
        )

        self._publish_boxes(old_epoch_stamp)

        frontier_count = self._snapshot()[3]
        time.sleep(0.10)
        self.assertEqual(
            frontier_count,
            self._snapshot()[3],
            "the old epoch frontier continued to publish after rollback",
        )

        self._wait_for(
            lambda: self._snapshot()[0].map_healthy
            and self._snapshot()[1].width > 0
            and self._snapshot()[2].width > 0,
            "continuous new-epoch input did not recover after an old future box",
        )
        self._stop_input_stream()

        self._wait_for(
            lambda: not self._snapshot()[0].map_healthy
            and self._snapshot()[2].width == 0,
            "new-epoch dynamic voxels did not expire by the configured TTL",
        )
        self.assertGreater(
            self._snapshot()[1].width,
            0,
            "new-epoch static voxels disappeared with the dynamic TTL",
        )

        previous_epoch_stamp = self._clock.now()
        self._clock.jump(previous_epoch_stamp.to_sec() - 0.14)
        self._wait_for(
            lambda: not self._snapshot()[0].map_healthy
            and self._snapshot()[1].width == 0
            and self._snapshot()[2].width == 0,
            "small clock rollback did not start an empty unhealthy epoch",
        )

        for _ in range(5):
            self._publish_boxes(previous_epoch_stamp, include_person=False)
            time.sleep(0.002)
        self._publish_inputs_for(0.30, publish_boxes=False)
        self.assertFalse(
            self._snapshot()[0].map_healthy,
            "an old-epoch empty box made the new epoch healthy",
        )
        self.assertEqual(
            0,
            self._snapshot()[1].width,
            "an old-epoch empty box polluted the new static map",
        )
        self.assertEqual(
            0,
            self._snapshot()[2].width,
            "an old-epoch box polluted the new dynamic map",
        )

        self._publish_inputs_for(0.8)
        self._wait_for(
            lambda: self._snapshot()[0].map_healthy
            and self._snapshot()[1].width > 0
            and self._snapshot()[2].width > 0,
            "fresh semantic input did not recover after the small rollback",
        )


if __name__ == "__main__":
    rospy.init_node("local_mapping_clock_rollback_test")
    rostest.rosrun(
        "local_mapping",
        "local_mapping_clock_rollback_test",
        LocalMappingClockRollbackTest,
    )

#!/usr/bin/env python3

import copy
import math
import struct
import threading
import time
import unittest

import rospy
import rostest
from darknet_ros_msgs.msg import BoundingBoxes
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from search_msgs.msg import LocalClearance, PerceptionHealth
from search_msgs.srv import (
    ValidateTrajectory,
    ValidateTrajectoryRequest,
)
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String, UInt64
from topic_tools.srv import MuxSelect
from traj_utils.msg import Bspline


class EgoSingleFaultsTest(unittest.TestCase):
    WIDTH = 16
    HEIGHT = 16

    def setUp(self):
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._depth_enabled = True
        self._odom_enabled = True
        self._position_x = -1.0
        self._altitude = 3.0
        self._depth_mm = 4000
        self._guard_mode = "clear"
        self._latest = {}
        self._history = {
            "generation": [],
            "adapter_status": [],
            "coordinator_status": [],
        }
        self._coordinator_validations = []
        self._adapter_validations = []

        self._depth_pub = rospy.Publisher("/fault/depth", Image, queue_size=1)
        self._camera_pub = rospy.Publisher(
            "/fault/camera_info", CameraInfo, queue_size=1
        )
        self._odom_pub = rospy.Publisher("/fault/odom", Odometry, queue_size=1)
        self._boxes_pub = rospy.Publisher(
            "/fault/boxes", BoundingBoxes, queue_size=1
        )
        self._guard_pub = rospy.Publisher(
            "/fault/guard_clearance", LocalClearance, queue_size=1
        )
        self._high_goal_pub = rospy.Publisher(
            "/fault/high_goal", PoseStamped, queue_size=1, latch=True
        )
        self._mission_pub = rospy.Publisher(
            "/fault/mission_active", Bool, queue_size=1, latch=True
        )
        self._takeoff_pub = rospy.Publisher(
            "/fault/takeoff_complete", Bool, queue_size=1, latch=True
        )
        self._tracking_pub = rospy.Publisher(
            "/fault/tracking_phase", String, queue_size=1, latch=True
        )
        self._external_pub = rospy.Publisher(
            "/fault/external", Twist, queue_size=1
        )
        self._selected_override_pub = rospy.Publisher(
            "/fault/mux/selected", String, queue_size=1
        )
        self._create_ego_publishers()

        subscriptions = (
            ("/fault/health", PerceptionHealth, "health"),
            ("/fault/map_clearance", LocalClearance, "map_clearance"),
            ("/fault/navigator", Twist, "navigator"),
            ("/fault/raw", Twist, "raw"),
            ("/fault/final", Twist, "final"),
            ("/fault/safety_status", String, "safety_status"),
            ("/fault/adapter_status", String, "adapter_status"),
            ("/fault/coordinator_status", String, "coordinator_status"),
            ("/fault/generation", UInt64, "generation"),
            ("/fault/ego_goal", PoseStamped, "ego_goal"),
            ("/fault/mux/selected", String, "selected"),
        )
        self._subscribers = [
            rospy.Subscriber(topic, message_type, self._store(name), queue_size=20)
            for topic, message_type, name in subscriptions
        ]

        rospy.wait_for_service("/fault/mux/select", timeout=5.0)
        rospy.wait_for_service("/fault/validate", timeout=5.0)
        self._select = rospy.ServiceProxy("/fault/mux/select", MuxSelect)
        self._validate = rospy.ServiceProxy(
            "/fault/validate", ValidateTrajectory
        )
        self._coordinator_validate_service = rospy.Service(
            "/fault/coordinator_validate",
            ValidateTrajectory,
            self._proxy_validation,
        )
        self._adapter_validate_service = rospy.Service(
            "/fault/adapter_validate",
            ValidateTrajectory,
            self._proxy_adapter_validation,
        )
        self._sensing_thread = threading.Thread(target=self._sensing_loop)
        self._sensing_thread.daemon = True
        self._sensing_thread.start()

    def tearDown(self):
        self._stop.set()
        if hasattr(self, "_sensing_thread"):
            self._sensing_thread.join(timeout=2.0)

    def _create_ego_publishers(self):
        self._spline_pub = rospy.Publisher(
            "/fault/bspline", Bspline, queue_size=1
        )
        self._position_pub = rospy.Publisher(
            "/fault/position_cmd", PositionCommand, queue_size=1
        )

    def _store(self, name):
        def callback(message):
            with self._lock:
                self._latest[name] = message
                if name in self._history:
                    value = message.data
                    self._history[name].append(value)
                    self._history[name] = self._history[name][-300:]

        return callback

    def _proxy_validation(self, request):
        response = self._validate(request)
        with self._lock:
            self._coordinator_validations.append(
                (
                    request.task_generation,
                    response.valid,
                    response.fault_code,
                    len(request.samples),
                )
            )
        return response

    def _proxy_adapter_validation(self, request):
        started = time.monotonic()
        response = self._validate(request)
        elapsed = time.monotonic() - started
        with self._lock:
            self._adapter_validations.append(
                (
                    request.task_generation,
                    response.valid,
                    response.fault_code,
                    len(request.samples),
                    round(elapsed, 4),
                )
            )
            self._adapter_validations = self._adapter_validations[-50:]
        return response

    def _snapshot(self, name):
        with self._lock:
            return self._latest.get(name)

    def _values(self, name):
        with self._lock:
            return list(self._history[name])

    @staticmethod
    def _zero(message):
        return message is not None and all(
            value == 0.0
            for value in (
                message.linear.x,
                message.linear.y,
                message.linear.z,
                message.angular.x,
                message.angular.y,
                message.angular.z,
            )
        )

    @staticmethod
    def _finite(message):
        return message is not None and all(
            math.isfinite(value)
            for value in (
                message.linear.x,
                message.linear.y,
                message.linear.z,
                message.angular.x,
                message.angular.y,
                message.angular.z,
            )
        )

    def _wait(self, predicate, message, timeout=5.0, action=None):
        deadline = time.monotonic() + timeout
        rate = rospy.Rate(80)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if action is not None:
                action()
            if predicate():
                return time.monotonic()
            rate.sleep()
        self.fail(
            "{}; adapter={!r}; safety={!r}; coordinator={!r}; generation={!r}".format(
                message,
                self._values("adapter_status"),
                getattr(self._snapshot("safety_status"), "data", None),
                self._values("coordinator_status"),
                (
                    self._values("generation"),
                    self._coordinator_validations,
                    self._adapter_validations,
                ),
            )
        )

    def _set_sensing(self, **changes):
        with self._lock:
            for name, value in changes.items():
                setattr(self, "_" + name, value)

    def _sensing_loop(self):
        rate = rospy.Rate(40)
        while not rospy.is_shutdown() and not self._stop.is_set():
            with self._lock:
                depth_enabled = self._depth_enabled
                odom_enabled = self._odom_enabled
                position_x = self._position_x
                altitude = self._altitude
                depth_mm = self._depth_mm
                guard_mode = self._guard_mode
            now = rospy.Time.now()
            if depth_enabled:
                boxes = BoundingBoxes()
                boxes.header.stamp = now
                boxes.image_header.stamp = now
                self._boxes_pub.publish(boxes)

                depth = Image()
                depth.header.stamp = now
                depth.header.frame_id = "test_camera"
                depth.height = self.HEIGHT
                depth.width = self.WIDTH
                depth.encoding = "16UC1"
                depth.step = self.WIDTH * 2
                depth.data = struct.pack(
                    "<{}H".format(self.WIDTH * self.HEIGHT),
                    *([depth_mm] * (self.WIDTH * self.HEIGHT))
                )
                camera = CameraInfo()
                camera.header = copy.deepcopy(depth.header)
                camera.height = self.HEIGHT
                camera.width = self.WIDTH
                camera.K = [
                    8.0, 0.0, 7.5,
                    0.0, 8.0, 7.5,
                    0.0, 0.0, 1.0,
                ]
                self._depth_pub.publish(depth)
                self._camera_pub.publish(camera)
            if odom_enabled:
                odom = Odometry()
                odom.header.stamp = now
                odom.header.frame_id = "map"
                odom.child_frame_id = "base_link"
                odom.pose.pose.position.x = position_x
                odom.pose.pose.position.z = altitude
                odom.pose.pose.orientation.w = 1.0
                self._odom_pub.publish(odom)

            clearance = LocalClearance()
            clearance.header.stamp = now
            clearance.header.frame_id = "map"
            for axis in (
                "forward", "backward", "left", "right", "upward", "downward"
            ):
                setattr(clearance, axis + "_known", True)
                setattr(clearance, axis + "_m", 10.0)
            if guard_mode == "blocked_forward":
                clearance.forward_m = 0.79
            elif guard_mode == "unknown_forward":
                clearance.forward_known = False
                clearance.forward_m = 0.0
            self._guard_pub.publish(clearance)
            rate.sleep()

    def _publish_goal_and_gates(self):
        self._takeoff_pub.publish(Bool(data=True))
        self._mission_pub.publish(Bool(data=True))
        self._tracking_pub.publish(String(data="IDLE::0.0"))
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"
        goal.pose.position.x = 0.9
        goal.pose.position.z = 3.0
        goal.pose.orientation.w = 1.0
        self._high_goal_pub.publish(goal)

    @staticmethod
    def _spline(traj_id, start, lateral=False):
        message = Bspline()
        message.order = 2
        message.traj_id = traj_id
        message.start_time = start
        message.knots = [0.0, 0.0, 0.0, 4.0, 4.0, 4.0]
        if lateral:
            message.pos_pts = [
                Point(x=0.5, y=0.5, z=3.0),
                Point(x=0.5, y=0.7, z=3.0),
                Point(x=0.5, y=0.9, z=3.0),
            ]
        else:
            message.pos_pts = [
                Point(x=0.0, z=3.0),
                Point(x=0.45, z=3.0),
                Point(x=0.9, z=3.0),
            ]
        return message

    @staticmethod
    def _position(start, non_finite=False):
        offset = max(0.0, (rospy.Time.now() - start).to_sec())
        message = PositionCommand()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "map"
        message.position.x = math.nan if non_finite else 0.225 * offset
        message.position.z = 3.0
        message.velocity.x = 0.225
        message.yaw = 0.0
        return message

    def _start_valid_trajectory(self, traj_id):
        self._select("/fault/navigator")
        self._set_sensing(guard_mode="clear", depth_enabled=True, odom_enabled=True)
        self._wait(
            lambda: self._snapshot("health") is not None
            and self._snapshot("health").map_healthy,
            "mapping did not recover before trajectory",
        )
        start = rospy.Time.now()
        self._spline_pub.publish(self._spline(traj_id, start))

        def publish():
            self._position_pub.publish(self._position(start))

        self._wait(
            lambda: self._finite(self._snapshot("navigator"))
            and self._snapshot("navigator").linear.x > 0.0
            and self._finite(self._snapshot("final"))
            and self._snapshot("final").linear.x > 0.0,
            "known-free trajectory did not reach navigator and final output",
            timeout=4.0,
            action=publish,
        )
        return start

    def _call_validation(self, points):
        generation = self._snapshot("generation")
        self.assertIsNotNone(generation)
        request = ValidateTrajectoryRequest()
        request.header.stamp = rospy.Time.now()
        request.header.frame_id = "map"
        request.task_generation = generation.data
        request.samples = points
        return self._validate(request)

    def test_single_drone_chain_fails_closed(self):
        self._select("/fault/navigator")
        self._wait(
            lambda: self._snapshot("selected") is not None
            and self._snapshot("selected").data == "/fault/navigator",
            "real mux did not select navigator",
        )
        self._wait(
            lambda: self._snapshot("health") is not None,
            "local_mapping did not publish health",
        )
        rospy.sleep(0.55)
        self._set_sensing(position_x=0.0, depth_mm=3000)
        rospy.sleep(0.55)
        self._wait(
            lambda: self._snapshot("health").map_healthy
            and self._snapshot("map_clearance") is not None,
            "local map did not become healthy after known-free scan",
        )

        self._publish_goal_and_gates()
        self._wait(
            lambda: self._snapshot("generation") is not None,
            "real coordinator did not publish a generation",
        )

        known = [Point(x=0.1 * index, z=3.0) for index in range(10)]
        known_response = self._call_validation(known)
        self.assertTrue(known_response.valid, known_response.fault_code)
        self._wait(
            lambda: self._snapshot("ego_goal") is not None,
            "real coordinator did not publish a known-free local goal",
        )

        trajectory_id = 1
        start = self._start_valid_trajectory(trajectory_id)

        stopped_at = time.monotonic()
        self._set_sensing(depth_enabled=False)
        self._wait(
            lambda: self._zero(self._snapshot("navigator"))
            and self._zero(self._snapshot("final"))
            and (
                "DEPTH_TIMEOUT" in self._values("adapter_status")
                or "HEALTH_STALE" in self._values("adapter_status")
            ),
            "depth loss did not stop adapter and final output",
            timeout=0.50,
            action=lambda: self._position_pub.publish(self._position(start)),
        )
        self.assertLessEqual(time.monotonic() - stopped_at, 0.50)

        trajectory_id += 1
        self._set_sensing(depth_enabled=True)
        start = self._start_valid_trajectory(trajectory_id)
        stopped_at = time.monotonic()
        self._set_sensing(odom_enabled=False)
        self._wait(
            lambda: self._zero(self._snapshot("navigator"))
            and self._zero(self._snapshot("final"))
            and (
                "ODOM_TIMEOUT" in self._values("adapter_status")
                or "ODOM_STALE" in self._values("adapter_status")
            ),
            "odometry loss did not stop adapter and final output",
            timeout=0.50,
            action=lambda: self._position_pub.publish(self._position(start)),
        )
        self.assertLessEqual(time.monotonic() - stopped_at, 0.50)

        trajectory_id += 1
        self._set_sensing(odom_enabled=True)
        start = self._start_valid_trajectory(trajectory_id)
        self._position_pub.publish(self._position(start, non_finite=True))
        self._wait(
            lambda: self._zero(self._snapshot("navigator"))
            and "NON_FINITE_INPUT" in self._values("adapter_status"),
            "non-finite EGO PositionCommand did not fail closed",
            timeout=0.20,
        )

        unknown = [Point(x=0.5, y=0.1 * index, z=3.0) for index in range(10)]
        unknown_response = self._call_validation(unknown)
        self.assertFalse(unknown_response.valid)
        self.assertEqual("UNKNOWN", unknown_response.fault_code)
        trajectory_id += 1
        unknown_start = rospy.Time.now()
        self._spline_pub.publish(
            self._spline(trajectory_id, unknown_start, lateral=True)
        )
        self._wait(
            lambda: self._zero(self._snapshot("navigator"))
            and "TRAJECTORY_REJECTED" in self._values("adapter_status"),
            "B-spline through unknown space was not rejected",
        )

        height_points = [
            Point(x=0.1 * index, z=3.0 + 1.01 * index / 10.0)
            for index in range(11)
        ]
        height_response = self._call_validation(height_points)
        self.assertFalse(height_response.valid)
        self.assertEqual("HEIGHT_LIMIT", height_response.fault_code)
        self.assertTrue(self._zero(self._snapshot("navigator")))

        trajectory_id += 1
        start = self._start_valid_trajectory(trajectory_id)
        self._set_sensing(guard_mode="blocked_forward")
        self._wait(
            lambda: self._zero(self._snapshot("navigator"))
            and self._zero(self._snapshot("final")),
            "0.79 metre forward clearance did not stop approach",
            action=lambda: self._position_pub.publish(self._position(start)),
        )

        self._set_sensing(guard_mode="clear")
        self._selected_override_pub.publish(String(data="/fault/rogue"))
        self._wait(
            lambda: self._zero(self._snapshot("navigator"))
            and self._zero(self._snapshot("final"))
            and getattr(self._snapshot("safety_status"), "data", None)
            == "MUX_UNKNOWN",
            "unknown mux selection did not fail closed",
        )

        self._select("/fault/external")
        self._wait(
            lambda: self._snapshot("selected").data == "/fault/external",
            "real mux did not leave the injected unknown selection",
        )
        self._select("/fault/navigator")
        self._wait(
            lambda: self._snapshot("selected").data == "/fault/navigator",
            "real mux did not restore navigator after unknown selection",
        )
        rospy.sleep(0.20)

        trajectory_id += 1
        start = self._start_valid_trajectory(trajectory_id)
        self._wait(
            lambda: self._snapshot("navigator").linear.x > 0.0,
            "EGO publisher exit precondition was not reached",
            action=lambda: self._position_pub.publish(self._position(start)),
        )
        stopped_at = time.monotonic()
        self._position_pub.unregister()
        self._spline_pub.unregister()
        self._wait(
            lambda: self._zero(self._snapshot("navigator"))
            and self._zero(self._snapshot("final")),
            "EGO publisher exit left stale motion",
            timeout=0.30,
        )
        self.assertLessEqual(time.monotonic() - stopped_at, 0.30)

        before_tracking = self._snapshot("generation").data
        self._tracking_pub.publish(String(data="TRACKING:green0:1.0"))
        self._select("/fault/external")
        self._wait(
            lambda: self._snapshot("generation").data > before_tracking,
            "tracking takeover did not invalidate the EGO generation",
        )

        external = Twist()
        external.linear.x = 0.3
        self._set_sensing(depth_enabled=False, odom_enabled=True, altitude=3.0)
        self._wait(
            lambda: self._zero(self._snapshot("final"))
            and getattr(self._snapshot("safety_status"), "data", None)
            == "PERCEPTION_TIMEOUT",
            "tracking bypassed final depth timeout",
            timeout=0.50,
            action=lambda: self._external_pub.publish(external),
        )

        self._set_sensing(depth_enabled=True, altitude=4.5)
        self._wait(
            lambda: self._snapshot("health").map_healthy
            and self._finite(self._snapshot("final"))
            and self._snapshot("final").linear.x > 0.0,
            "healthy tracking command was rejected between 4 and 6 metres",
            action=lambda: self._external_pub.publish(external),
        )

        climb = Twist()
        climb.linear.z = 0.3
        self._set_sensing(altitude=6.01)
        self._wait(
            lambda: self._snapshot("final") is not None
            and self._snapshot("final").linear.z == 0.0
            and getattr(self._snapshot("safety_status"), "data", None)
            == "ALTITUDE_LIMIT",
            "tracking climb above 6 metres was not stopped",
            action=lambda: self._external_pub.publish(climb),
        )

        takeover_generation = self._snapshot("generation").data
        self._set_sensing(altitude=3.0)
        self._wait(
            lambda: self._snapshot("health").map_healthy,
            "mapping did not recover at search altitude before resume",
        )
        self._tracking_pub.publish(String(data="IDLE::2.0"))
        self._select("/fault/navigator")
        self._wait(
            lambda: self._snapshot("generation").data > takeover_generation,
            "navigation resume did not create a fresh generation",
        )
        rospy.sleep(0.20)
        self._create_ego_publishers()
        old_start = rospy.Time.now()
        self._spline_pub.publish(self._spline(trajectory_id, old_start))
        executing_before = len(
            [
                status
                for status in self._values("adapter_status")
                if status.startswith("EXECUTING:")
            ]
        )
        deadline = time.monotonic() + 0.35
        rate = rospy.Rate(80)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            self._position_pub.publish(self._position(old_start))
            self.assertTrue(
                self._zero(self._snapshot("navigator")),
                "old EGO trajectory produced motion after tracking",
            )
            rate.sleep()
        executing_after = len(
            [
                status
                for status in self._values("adapter_status")
                if status.startswith("EXECUTING:")
            ]
        )
        self.assertEqual(
            executing_before,
            executing_after,
            "old EGO trajectory entered EXECUTING after tracking",
        )


if __name__ == "__main__":
    rospy.init_node("ego_single_faults_test")
    rostest.rosrun(
        "ego_adapter", "ego_single_faults", EgoSingleFaultsTest
    )

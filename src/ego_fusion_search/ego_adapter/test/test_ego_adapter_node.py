#!/usr/bin/env python3

import math
import threading
import unittest

import rospy
import rostest
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from search_msgs.msg import LocalClearance, PerceptionHealth
from search_msgs.srv import ValidateTrajectory, ValidateTrajectoryResponse
from std_msgs.msg import String, UInt64
from traj_utils.msg import Bspline


class EgoAdapterNodeTest(unittest.TestCase):
    def setUp(self):
        self._lock = threading.Lock()
        self._command = None
        self._command_count = 0
        self._status = None
        self._status_history = []
        self._service_requests = 0
        self._service_valid = True
        self._service_blocked = threading.Event()
        self._service_entered = threading.Event()
        self._command_sub = rospy.Subscriber(
            "/test/navigator/cmd_vel", Twist, self._store_command
        )
        self._status_sub = rospy.Subscriber(
            "/test/status", String, self._store_status
        )
        self._odom_pub = rospy.Publisher("/test/odom", Odometry, queue_size=1)
        self._health_pub = rospy.Publisher(
            "/test/health", PerceptionHealth, queue_size=1
        )
        self._clearance_pub = rospy.Publisher(
            "/test/clearance", LocalClearance, queue_size=1
        )
        self._generation_pub = rospy.Publisher(
            "/test/generation", UInt64, queue_size=1, latch=True
        )
        self._mux_pub = rospy.Publisher(
            "/test/mux/selected", String, queue_size=1, latch=True
        )
        self._spline_pub = rospy.Publisher(
            "/test/ego/bspline", Bspline, queue_size=1
        )
        self._position_pub = rospy.Publisher(
            "/test/ego/position_cmd", PositionCommand, queue_size=1
        )
        self._service = rospy.Service(
            "/test/validate", ValidateTrajectory, self._validate
        )

    def _store(self, attribute):
        def callback(message):
            with self._lock:
                setattr(self, attribute, message)
        return callback

    def _snapshot(self, attribute):
        with self._lock:
            return getattr(self, attribute)

    def _store_status(self, message):
        with self._lock:
            self._status = message
            self._status_history.append(message.data)
            self._status_history = self._status_history[-100:]

    def _store_command(self, message):
        with self._lock:
            self._command = message
            self._command_count += 1

    def _validate(self, request):
        with self._lock:
            self._service_requests += 1
        self._service_entered.set()
        if self._service_blocked.is_set():
            deadline = rospy.Time.now() + rospy.Duration(1.0)
            while self._service_blocked.is_set() and rospy.Time.now() < deadline:
                rospy.sleep(0.01)
        return ValidateTrajectoryResponse(
            valid=self._service_valid,
            task_generation=request.task_generation,
            map_stamp=rospy.Time.now(),
            min_clearance_m=0.1 if self._service_valid else 0.0,
            fault_code="OK" if self._service_valid else "OCCUPIED",
        )

    def _publish_odom(self):
        now = rospy.Time.now()
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "map"
        odom.pose.pose.position.z = 3.0
        odom.pose.pose.orientation.w = 1.0
        self._odom_pub.publish(odom)

    def _publish_invalid_odom(self):
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "map"
        odom.pose.pose.position.x = math.nan
        odom.pose.pose.position.z = 3.0
        odom.pose.pose.orientation.w = 1.0
        self._odom_pub.publish(odom)

    def _publish_health(self, healthy=True):
        now = rospy.Time.now()
        message = PerceptionHealth()
        message.header.stamp = now
        message.header.frame_id = "map"
        message.depth_healthy = healthy
        message.odom_healthy = healthy
        message.synchronized = healthy
        message.map_healthy = healthy
        message.fault_code = "OK" if healthy else "DEPTH_TIMEOUT"
        self._health_pub.publish(message)

    def _publish_clearance(self):
        now = rospy.Time.now()
        clearance = LocalClearance()
        clearance.header.stamp = now
        clearance.header.frame_id = "map"
        for name in ("forward", "backward", "left", "right", "upward", "downward"):
            setattr(clearance, name + "_known", True)
            setattr(clearance, name + "_m", 10.0)
        self._clearance_pub.publish(clearance)

    def _publish_safety(self, health=True, mux="/test/navigator/cmd_vel"):
        self._publish_odom()
        self._publish_health(health)
        self._publish_clearance()
        self._mux_pub.publish(String(data=mux))

    def _spline(self, traj_id, start_time):
        message = Bspline()
        message.order = 2
        message.traj_id = traj_id
        message.start_time = start_time
        message.knots = [0.0, 0.0, 0.0, 20.0, 20.0, 20.0]
        message.pos_pts = [Point(x=0.0, z=3.0), Point(x=1.0, z=3.0), Point(x=2.0, z=3.0)]
        return message

    def _position(self, spline_start, offset=0.15, position_error=0.0):
        message = PositionCommand()
        message.header.stamp = spline_start + rospy.Duration(offset)
        message.header.frame_id = "map"
        message.position.x = 0.1 * offset + position_error
        message.position.z = 3.0
        message.velocity.x = 0.1
        message.yaw = 0.0
        return message

    def _publish_command_cycle(self, start, safety_callback=None):
        if safety_callback is None:
            safety_callback = self._publish_safety
        safety_callback()
        offset = (rospy.Time.now() - start).to_sec()
        self._position_pub.publish(self._position(start, offset=offset))

    def _start_valid_trajectory(self, generation, traj_id):
        self._service_valid = True
        self._generation_pub.publish(UInt64(data=generation))
        self._publish_for(0.1, self._publish_safety)
        start = rospy.Time.now()
        self._spline_pub.publish(self._spline(traj_id, start))
        self._publish_for(0.4, lambda: self._publish_command_cycle(start))
        self._wait(
            lambda: self._snapshot("_command").linear.x > 0.0,
            "valid trajectory never produced forward command",
        )
        return start

    def _publish_for(self, seconds, callback):
        deadline = rospy.Time.now() + rospy.Duration(seconds)
        rate = rospy.Rate(40)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            callback()
            rate.sleep()

    def _wait(self, predicate, message, timeout=3.0):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(100)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if predicate():
                return
            rate.sleep()
        self.fail(message)

    def _is_zero(self):
        command = self._snapshot("_command")
        return command is not None and all(
            value == 0.0
            for value in (command.linear.x, command.linear.y, command.linear.z, command.angular.z)
        )

    def test_adapter_fails_closed_and_accepts_only_bound_trajectory(self):
        self._wait(lambda: self._snapshot("_command") is not None, "no fail-closed output")
        self.assertTrue(self._is_zero())
        self._wait(
            lambda: self._spline_pub.get_num_connections() > 0,
            "B-spline publisher never connected to adapter",
        )
        self._wait(
            lambda: self._position_pub.get_num_connections() > 0,
            "PositionCommand publisher never connected to adapter",
        )

        self._generation_pub.publish(UInt64(data=1))
        self._publish_for(0.2, self._publish_safety)
        start = rospy.Time.now()
        self._service_valid = False
        self._spline_pub.publish(self._spline(1, start))
        self._publish_for(0.25, lambda: self._publish_command_cycle(start))
        self._wait(
            lambda: "TRAJECTORY_REJECTED" in self._snapshot("_status_history"),
            "rejected trajectory status missing; requests={} statuses={}".format(
                self._snapshot("_service_requests"),
                self._snapshot("_status_history"),
            ),
        )
        self.assertTrue(self._is_zero())

        self._service_valid = True
        self._publish_for(0.35, lambda: self._publish_command_cycle(start))
        self._wait(lambda: self._snapshot("_command").linear.x > 0.0, "valid trajectory never produced forward command")
        self.assertLessEqual(self._snapshot("_command").linear.x, 1.5)

        self._spline_pub.publish(self._spline(-1, rospy.Time.now()))
        self._wait(self._is_zero, "signed older traj_id did not stop output")

        self._generation_pub.publish(UInt64(data=2))
        old_start = rospy.Time.now() - rospy.Duration(1.0)
        self._spline_pub.publish(self._spline(2, old_start))
        self._publish_for(0.15, self._publish_safety)
        self.assertTrue(self._is_zero())

        fresh_start = rospy.Time.now()
        self._spline_pub.publish(self._spline(3, fresh_start))
        self._publish_for(0.2, lambda: (self._publish_safety(), self._position_pub.publish(self._position(fresh_start, position_error=1.0))))
        self.assertTrue(self._is_zero())
        self._publish_for(0.3, lambda: self._publish_command_cycle(fresh_start))
        self._wait(lambda: self._snapshot("_command").linear.x > 0.0, "matching command did not recover output")

        self._generation_pub.publish(UInt64(data=1))
        self._wait(self._is_zero, "older task generation did not stop output")

        map_start = self._start_valid_trajectory(3, 4)
        with self._lock:
            self._status_history = []
        self._service_blocked.set()
        self._service_entered.clear()
        self._publish_for(0.2, lambda: self._publish_command_cycle(map_start))
        self.assertTrue(
            self._service_entered.is_set(),
            "periodic map revalidation did not start",
        )
        self.assertTrue(self._is_zero(), "command remained live while map validation was pending")
        self._service_valid = False
        self._service_blocked.clear()
        self._wait(
            lambda: "TRAJECTORY_REJECTED" in self._snapshot("_status_history"),
            "new obstacle did not reject the active trajectory; statuses={}".format(
                self._snapshot("_status_history")
            ),
        )
        self.assertTrue(self._is_zero())

        mux_start = self._start_valid_trajectory(4, 5)
        self._mux_pub.publish(String(data="/test/external"))
        self._wait(self._is_zero, "external MUX selection did not stop output")
        self._mux_pub.publish(String(data="/test/navigator/cmd_vel"))
        self._publish_for(0.25, lambda: self._publish_command_cycle(mux_start))
        self.assertTrue(self._is_zero())

        stale_start = self._start_valid_trajectory(5, 6)
        self._publish_for(
            0.3,
            lambda: self._publish_command_cycle(
                stale_start,
                lambda: (self._publish_odom(), self._publish_clearance(), self._mux_pub.publish(String(data="/test/navigator/cmd_vel"))),
            ),
        )
        self.assertTrue(self._is_zero(), "stale health did not stop output")

        self._publish_for(0.35, lambda: self._publish_command_cycle(stale_start))
        self._wait(lambda: self._snapshot("_command").linear.x > 0.0, "fresh health did not recover output")
        self._publish_for(
            0.3,
            lambda: self._publish_command_cycle(
                stale_start,
                lambda: (self._publish_health(), self._publish_clearance(), self._mux_pub.publish(String(data="/test/navigator/cmd_vel"))),
            ),
        )
        self.assertTrue(self._is_zero(), "stale odometry did not stop output")

        self._generation_pub.publish(UInt64(data=6))
        invalid_start = rospy.Time.now()
        self._spline_pub.publish(self._spline(7, invalid_start))
        requests_before = self._snapshot("_service_requests")
        commands_before = self._snapshot("_command_count")
        self._publish_for(
            0.25,
            lambda: self._publish_command_cycle(
                invalid_start,
                lambda: (
                    self._publish_invalid_odom(),
                    self._publish_health(),
                    self._publish_clearance(),
                    self._mux_pub.publish(String(data="/test/navigator/cmd_vel")),
                ),
            ),
        )
        self.assertEqual(
            requests_before,
            self._snapshot("_service_requests"),
            "non-finite odometry reached the trajectory service",
        )
        self.assertGreater(
            self._snapshot("_command_count"),
            commands_before,
            "non-finite odometry stopped the fail-closed output heartbeat",
        )
        self.assertTrue(self._is_zero())


if __name__ == "__main__":
    rospy.init_node("ego_adapter_node_test")
    rostest.rosrun("ego_adapter", "ego_adapter_node_test", EgoAdapterNodeTest)

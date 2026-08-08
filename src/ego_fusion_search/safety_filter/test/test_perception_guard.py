#!/usr/bin/env python3

import unittest

import rospy
import rostest
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from search_msgs.msg import LocalClearance, PerceptionHealth
from std_msgs.msg import String
from topic_tools.srv import MuxSelect


class PerceptionGuardTest(unittest.TestCase):
    def setUp(self):
        self.command = None
        self.status = None
        self.inputs = {
            name: rospy.Publisher("/test/" + name, Twist, queue_size=1)
            for name in ("takeoff", "navigator", "external")
        }
        self.odom_pub = rospy.Publisher("/test/odom", Odometry, queue_size=1)
        self.health_pub = rospy.Publisher(
            "/test/health", PerceptionHealth, queue_size=1
        )
        self.clearance_pub = rospy.Publisher(
            "/test/clearance", LocalClearance, queue_size=1
        )
        self.selected_override = rospy.Publisher(
            "/test_mux/selected", String, queue_size=1
        )
        rospy.Subscriber("/test/final", Twist, self._command)
        rospy.Subscriber("/test/status", String, self._status)
        rospy.wait_for_service("/test_mux/select", timeout=3.0)
        self.select = rospy.ServiceProxy("/test_mux/select", MuxSelect)

    def _command(self, message):
        self.command = message

    def _status(self, message):
        self.status = message.data

    @staticmethod
    def _zero(message):
        return message is not None and all(
            value == 0.0
            for value in (
                message.linear.x, message.linear.y, message.linear.z,
                message.angular.x, message.angular.y, message.angular.z,
            )
        )

    def _publish(self, source, altitude, command, health=True, clearance=None):
        now = rospy.Time.now()
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "map"
        odom.pose.pose.position.z = altitude
        self.odom_pub.publish(odom)
        self.inputs[source].publish(command)
        if health:
            status = PerceptionHealth()
            status.header.stamp = now
            status.header.frame_id = "map"
            status.depth_healthy = True
            status.odom_healthy = True
            status.synchronized = True
            status.map_healthy = True
            status.fault_code = "OK"
            self.health_pub.publish(status)
        if clearance is None:
            clearance = LocalClearance()
            for name in ("forward", "backward", "left", "right", "upward", "downward"):
                setattr(clearance, name + "_known", True)
                setattr(clearance, name + "_m", 10.0)
        clearance.header.stamp = now
        clearance.header.frame_id = "map"
        self.clearance_pub.publish(clearance)

    def _until(self, predicate, publish, timeout=3.0):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(50)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            publish()
            if predicate():
                return
            rate.sleep()
        self.fail("timed out: status={!r}, command={!r}".format(self.status, self.command))

    def test_mux_height_perception_and_directional_guards(self):
        climb = Twist()
        climb.linear.z = 0.5
        self._until(
            lambda: self.status == "PERCEPTION_TIMEOUT" and self._zero(self.command),
            lambda: self._publish("takeoff", 3.0, climb, health=False),
        )
        self._until(
            lambda: self.status == "OK" and self.command.linear.z > 0.0,
            lambda: self._publish("takeoff", 3.0, climb),
        )

        self.select("/test/navigator")
        self._until(
            lambda: self.status == "ALTITUDE_LIMIT" and self.command.linear.z == 0.0,
            lambda: self._publish("navigator", 4.01, climb),
        )
        self.select("/test/external")
        self._until(
            lambda: self.status == "OK" and self.command.linear.z > 0.0,
            lambda: self._publish("external", 4.01, climb),
        )
        self._until(
            lambda: self.status == "ALTITUDE_LIMIT" and self.command.linear.z == 0.0,
            lambda: self._publish("external", 6.01, climb),
        )

        forward = Twist()
        forward.linear.x = 1.0
        blocked = LocalClearance()
        for name in ("backward", "left", "right", "upward", "downward"):
            setattr(blocked, name + "_known", True)
            setattr(blocked, name + "_m", 10.0)
        blocked.forward_known = False
        self._until(
            lambda: self.status == "PERCEPTION_BLOCKED" and self.command.linear.x == 0.0,
            lambda: self._publish("external", 3.0, forward, clearance=blocked),
        )

        self._until(
            lambda: self.status == "PERCEPTION_TIMEOUT" and self._zero(self.command),
            lambda: self._publish("external", 3.0, forward, health=False),
        )
        self._until(
            lambda: self.status == "MUX_UNKNOWN" and self._zero(self.command),
            lambda: (
                self._publish("external", 3.0, forward),
                self.selected_override.publish(String(data="/test/rogue")),
            ),
        )


if __name__ == "__main__":
    rospy.init_node("perception_guard_test")
    rostest.rosrun("safety_filter", "perception_guard_test", PerceptionGuardTest)

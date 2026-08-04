#!/usr/bin/env python3

import unittest

import rospy
import rostest
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class SafetyFilterNodeTest(unittest.TestCase):
    def setUp(self):
        self.command = None
        self.status = None
        self.raw_pub = rospy.Publisher("/test/raw_cmd", Twist, queue_size=1)
        self.odom_pub = rospy.Publisher("/test/odom", Odometry, queue_size=1)
        rospy.Subscriber("/test/final_cmd", Twist, self._command_callback)
        rospy.Subscriber("/test/status", String, self._status_callback)

    def _command_callback(self, message):
        self.command = message

    def _status_callback(self, message):
        self.status = message.data

    def _wait_for(self, predicate, timeout=2.0):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(100)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if predicate():
                return
            rate.sleep()
        self.fail("timed out waiting for safety_filter state")

    def _publish_until(
        self,
        predicate,
        publish_raw,
        publish_odom,
        timeout=2.0,
        altitude=3.0,
        vertical_speed=0.0,
    ):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(50)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if publish_raw:
                command = Twist()
                command.linear.x = 1.0
                command.linear.z = vertical_speed
                self.raw_pub.publish(command)
            if publish_odom:
                odom = Odometry()
                odom.header.stamp = rospy.Time.now()
                odom.pose.pose.position.z = altitude
                self.odom_pub.publish(odom)
            if predicate():
                return
            rate.sleep()
        self.fail("timed out while publishing safety_filter inputs")

    def test_default_four_metre_ceiling_blocks_climb(self):
        self._publish_until(
            lambda: self.status == "ALTITUDE_LIMIT"
            and self.command is not None
            and self.command.linear.z == 0.0,
            publish_raw=True,
            publish_odom=True,
            altitude=4.0,
            vertical_speed=0.5,
        )

    def test_watchdogs_are_fail_closed_and_recover(self):
        self._wait_for(
            lambda: self.status == "ODOM_TIMEOUT"
            and self.command is not None
            and self.command.linear.x == 0.0
        )
        self.assertAlmostEqual(0.0, self.command.linear.x)

        self._publish_until(
            lambda: self.status == "COMMAND_TIMEOUT"
            and self.command is not None
            and self.command.linear.x == 0.0,
            publish_raw=False,
            publish_odom=True,
        )
        self.assertAlmostEqual(0.0, self.command.linear.x)

        self._publish_until(
            lambda: self.status == "OK" and self.command.linear.x > 0.0,
            publish_raw=True,
            publish_odom=True,
        )

        self._publish_until(
            lambda: self.status == "COMMAND_TIMEOUT"
            and self.command.linear.x == 0.0,
            publish_raw=False,
            publish_odom=True,
        )
        self.assertAlmostEqual(0.0, self.command.linear.x)

        self._publish_until(
            lambda: self.status == "ODOM_TIMEOUT"
            and self.command.linear.x == 0.0,
            publish_raw=True,
            publish_odom=False,
        )
        self.assertAlmostEqual(0.0, self.command.linear.x)


if __name__ == "__main__":
    rospy.init_node("safety_filter_node_test")
    rostest.rosrun(
        "safety_filter", "safety_filter_node_test", SafetyFilterNodeTest
    )

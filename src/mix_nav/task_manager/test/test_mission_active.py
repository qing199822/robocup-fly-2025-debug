#!/usr/bin/env python3

import threading
import unittest

import rospy
import rostest
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool


class MissionActiveTest(unittest.TestCase):
    def setUp(self):
        self.lock = threading.Lock()
        self.states = []
        self.state_sub = rospy.Subscriber(
            "/test_drone_0/mission/active",
            Bool,
            self._state_callback,
            queue_size=10,
        )
        self.pose_pub = rospy.Publisher(
            "/test_drone_0/global_pose",
            PoseStamped,
            queue_size=1,
        )

    def _state_callback(self, message):
        with self.lock:
            self.states.append(message.data)

    def _snapshot(self):
        with self.lock:
            return list(self.states)

    def _wait_for(self, predicate, message, timeout=5.0, publish_pose=False):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(50)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if publish_pose:
                pose = PoseStamped()
                pose.header.stamp = rospy.Time.now()
                pose.pose.orientation.w = 1.0
                self.pose_pub.publish(pose)
            if predicate():
                return
            rate.sleep()
        self.fail(message)

    def test_active_is_latched_false_until_mission_starts(self):
        self._wait_for(
            lambda: False in self._snapshot(),
            "mission manager did not publish the initial false state",
        )
        self.assertNotIn(True, self._snapshot())
        self._wait_for(
            lambda: True in self._snapshot(),
            "mission manager did not publish true after startup",
            publish_pose=True,
        )


if __name__ == "__main__":
    rospy.init_node("mission_active_test")
    rostest.rosrun("task_manager", "mission_active_test", MissionActiveTest)

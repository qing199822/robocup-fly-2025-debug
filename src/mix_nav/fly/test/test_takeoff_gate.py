#!/usr/bin/env python3

import threading
import unittest

import rospy
import rostest
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from topic_tools.srv import MuxSelect, MuxSelectResponse


class TakeoffGateTest(unittest.TestCase):
    def setUp(self):
        self.mode = rospy.get_param("~mode")
        self.lock = threading.Lock()
        self.status_history = []
        self.selected_topics = []
        self.fail_handoff_id = 1 if self.mode == "handoff_failure" else None

        self.pose_publishers = [
            rospy.Publisher(
                "/test_drone_{}/mavros/local_position/pose".format(drone_id),
                PoseStamped,
                queue_size=1,
            )
            for drone_id in range(2)
        ]
        self.mux_services = [
            rospy.Service(
                "/test_drone_{}/pose_cmd_mux/select".format(drone_id),
                MuxSelect,
                lambda request, drone_id=drone_id: self._mux_callback(
                    request, drone_id
                ),
            )
            for drone_id in range(2)
        ]
        self.status_subscriber = rospy.Subscriber(
            "/swarm/takeoff_complete", Bool, self._status_callback, queue_size=1
        )

    def _mux_callback(self, request, drone_id):
        with self.lock:
            self.selected_topics.append((drone_id, request.topic))
        if self.fail_handoff_id == drone_id and request.topic.endswith(
            "/mux_inputs/navigator/cmd_vel"
        ):
            raise rospy.ServiceException("planned navigator handoff failure")
        return MuxSelectResponse(prev_topic="")

    def _status_callback(self, message):
        with self.lock:
            self.status_history.append(message.data)

    def _snapshot(self, values):
        with self.lock:
            return list(values)

    def _wait_for(self, predicate, message, timeout=4.0):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(100)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if predicate():
                return
            rate.sleep()
        self.fail(message)

    def _publish_reached_poses_until(self, predicate, timeout=4.0):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(50)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            for publisher in self.pose_publishers:
                pose = PoseStamped()
                pose.header.stamp = rospy.Time.now()
                pose.pose.position.z = 3.1
                publisher.publish(pose)
            if predicate():
                return
            rate.sleep()
        self.fail("timed out waiting for reached-pose takeoff result")

    def test_takeoff_gate_contract(self):
        self._wait_for(
            lambda: False in self._snapshot(self.status_history),
            "takeoff node did not publish the initial closed gate",
        )

        if self.mode == "timeout":
            rospy.sleep(1.0)
            self.assertNotIn(True, self._snapshot(self.status_history))
            return

        if self.mode == "success":
            self._publish_reached_poses_until(
                lambda: True in self._snapshot(self.status_history)
            )
            late_status = []
            late_subscriber = rospy.Subscriber(
                "/swarm/takeoff_complete",
                Bool,
                lambda message: late_status.append(message.data),
                queue_size=1,
            )
            self.addCleanup(late_subscriber.unregister)
            self._wait_for(
                lambda: late_status == [True],
                "late subscriber did not receive latched open gate",
            )
            return

        self.assertEqual("handoff_failure", self.mode)
        takeoff_suffix = "/mux_inputs/takeoff/cmd_vel"
        navigator_suffix = "/mux_inputs/navigator/cmd_vel"

        def handoff_failed_and_rolled_back():
            selections = self._snapshot(self.selected_topics)
            attempted_failure = (1, "/test_drone_1" + navigator_suffix)
            rollback_ids = {
                drone_id
                for drone_id, topic in selections
                if topic.endswith(takeoff_suffix)
            }
            return attempted_failure in selections and rollback_ids == {0, 1}

        self._publish_reached_poses_until(handoff_failed_and_rolled_back)
        rospy.sleep(0.2)
        self.assertNotIn(True, self._snapshot(self.status_history))


if __name__ == "__main__":
    rospy.init_node("takeoff_gate_test")
    rostest.rosrun("fly", "takeoff_gate_test", TakeoffGateTest)

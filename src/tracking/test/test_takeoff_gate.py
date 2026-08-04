#!/usr/bin/env python3

import threading
import unittest

import rospy
import rostest
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes
from geometry_msgs.msg import PoseStamped, TwistStamped
from look_up.srv import (
    CompleteTarget,
    CompleteTargetResponse,
    ReleaseTarget,
    ReleaseTargetResponse,
    RequestTarget,
    RequestTargetResponse,
)
from std_msgs.msg import Bool, String
from topic_tools.srv import MuxSelect, MuxSelectResponse


TARGET_IDS = ("green0", "blue1", "brown2", "white3", "red4", "red5", "person")


class TrackingTakeoffGateTest(unittest.TestCase):
    def setUp(self):
        self.lock = threading.Lock()
        self.requested_targets = []
        self.released_targets = []
        self.completed_targets = []
        self.selected_topics = []
        self.mission_commands = []

        self.services = [
            rospy.Service(
                "/test_drone_0/pose_cmd_mux/select",
                MuxSelect,
                self._select_callback,
            )
        ]
        self.services.append(
            rospy.Service(
                "/lookup/complete_target",
                CompleteTarget,
                self._complete_callback,
            )
        )
        for target_id in TARGET_IDS:
            self.services.append(
                rospy.Service(
                    "/lookup/request_" + target_id,
                    RequestTarget,
                    self._request_callback,
                )
            )
            self.services.append(
                rospy.Service(
                    "/lookup/release_" + target_id,
                    ReleaseTarget,
                    self._release_callback,
                )
            )

        self.takeoff_gate_pub = rospy.Publisher(
            "/swarm/takeoff_complete", Bool, queue_size=1, latch=True
        )
        self.mission_gate_pub = rospy.Publisher(
            "/test_drone_0/mission/active", Bool, queue_size=1, latch=True
        )
        self.pose_pub = rospy.Publisher(
            "/test_drone_0/mavros/local_position/pose",
            PoseStamped,
            queue_size=1,
        )
        self.velocity_pub = rospy.Publisher(
            "/test_drone_0/mavros/local_position/velocity_body",
            TwistStamped,
            queue_size=1,
        )
        self.boxes_pub = rospy.Publisher(
            "/test_drone_0/yolo11n/bounding_boxes",
            BoundingBoxes,
            queue_size=1,
        )
        self.mission_sub = rospy.Subscriber(
            "/test_drone_0/mission/control",
            String,
            self._mission_callback,
            queue_size=10,
        )
        self.takeoff_gate_pub.publish(Bool(data=False))
        self.mission_gate_pub.publish(Bool(data=False))

    def _select_callback(self, request):
        with self.lock:
            self.selected_topics.append(request.topic)
        return MuxSelectResponse(prev_topic="")

    def _request_callback(self, request):
        with self.lock:
            self.requested_targets.append(request.target_id)
        return RequestTargetResponse(success=True)

    def _release_callback(self, request):
        with self.lock:
            self.released_targets.append(request.target_id)
        return ReleaseTargetResponse(success=True)

    def _complete_callback(self, request):
        with self.lock:
            self.completed_targets.append(request.target_id)
        return CompleteTargetResponse(success=True)

    def _mission_callback(self, message):
        with self.lock:
            self.mission_commands.append(message.data)

    def _snapshot(self, values):
        with self.lock:
            return list(values)

    def _publish_inputs_once(self):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.z = 3.1
        self.pose_pub.publish(pose)

        velocity = TwistStamped()
        velocity.header.stamp = rospy.Time.now()
        self.velocity_pub.publish(velocity)

        target = BoundingBox()
        target.Class = "green0"
        target.probability = 0.95
        target.xmin = 100
        target.ymin = 100
        target.xmax = 140
        target.ymax = 140
        boxes = BoundingBoxes()
        boxes.header.stamp = rospy.Time.now()
        boxes.bounding_boxes = [target]
        self.boxes_pub.publish(boxes)

    def _publish_inputs_for(self, seconds):
        deadline = rospy.Time.now() + rospy.Duration(seconds)
        rate = rospy.Rate(50)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            self._publish_inputs_once()
            rate.sleep()

    def _publish_until(self, predicate, message, timeout=4.0):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(50)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            self._publish_inputs_once()
            if predicate():
                return
            rate.sleep()
        self.fail(message)

    def _wait_for(self, predicate, message, timeout=2.0):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(100)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if predicate():
                return
            rate.sleep()
        self.fail(message)

    def test_both_gates_are_required_and_each_gate_resets_tracking(self):
        self._publish_inputs_for(0.5)
        self.assertEqual([], self._snapshot(self.requested_targets))
        self.assertEqual([], self._snapshot(self.selected_topics))
        self.assertEqual([], self._snapshot(self.mission_commands))

        self.takeoff_gate_pub.publish(Bool(data=True))
        self._publish_inputs_for(0.5)
        self.assertEqual([], self._snapshot(self.requested_targets))
        self.assertEqual([], self._snapshot(self.selected_topics))
        self.assertEqual([], self._snapshot(self.mission_commands))

        self.takeoff_gate_pub.publish(Bool(data=False))
        self.mission_gate_pub.publish(Bool(data=True))
        self._publish_inputs_for(0.5)
        self.assertEqual([], self._snapshot(self.requested_targets))
        self.assertEqual([], self._snapshot(self.selected_topics))
        self.assertEqual([], self._snapshot(self.mission_commands))

        self.takeoff_gate_pub.publish(Bool(data=True))
        self._publish_until(
            lambda: self._snapshot(self.requested_targets) == ["green0"],
            "both open gates did not allow target request",
        )
        self._publish_until(
            lambda: any(
                topic.endswith("/mux_inputs/external/pose_cmd")
                for topic in self._snapshot(self.selected_topics)
            ),
            "both open gates did not allow tracking takeover",
        )
        self._wait_for(
            lambda: "PAUSE" in self._snapshot(self.mission_commands),
            "tracking takeover did not pause the mission",
        )

        switch_count = len(self._snapshot(self.selected_topics))
        mission_count = len(self._snapshot(self.mission_commands))
        self.mission_gate_pub.publish(Bool(data=False))
        self._wait_for(
            lambda: self._snapshot(self.released_targets) == ["green0"],
            "closing mission gate did not release the tracked target",
        )
        self.mission_gate_pub.publish(Bool(data=False))
        self._publish_inputs_for(0.5)

        self.assertEqual(["green0"], self._snapshot(self.released_targets))
        self.assertEqual(["green0"], self._snapshot(self.requested_targets))
        self.assertEqual(switch_count, len(self._snapshot(self.selected_topics)))
        self.assertEqual(mission_count, len(self._snapshot(self.mission_commands)))
        self.assertEqual([], self._snapshot(self.completed_targets))

        self.mission_gate_pub.publish(Bool(data=True))
        self._publish_until(
            lambda: self._snapshot(self.requested_targets) ==
            ["green0", "green0"],
            "reopening both gates did not allow another target request",
        )
        self._publish_until(
            lambda: len(self._snapshot(self.selected_topics)) > switch_count,
            "reopening both gates did not allow another tracking takeover",
        )
        self._wait_for(
            lambda: len(self._snapshot(self.mission_commands)) > mission_count,
            "reopening both gates did not pause the mission",
        )

        switch_count = len(self._snapshot(self.selected_topics))
        mission_count = len(self._snapshot(self.mission_commands))
        self.takeoff_gate_pub.publish(Bool(data=False))
        self._wait_for(
            lambda: self._snapshot(self.released_targets) ==
            ["green0", "green0"],
            "closing takeoff gate did not release the tracked target",
        )
        self.takeoff_gate_pub.publish(Bool(data=False))
        self._publish_inputs_for(0.5)

        self.assertEqual(
            ["green0", "green0"], self._snapshot(self.released_targets)
        )
        self.assertEqual(
            ["green0", "green0"], self._snapshot(self.requested_targets)
        )
        self.assertEqual(switch_count, len(self._snapshot(self.selected_topics)))
        self.assertEqual(mission_count, len(self._snapshot(self.mission_commands)))


if __name__ == "__main__":
    rospy.init_node("tracking_takeoff_gate_test")
    rostest.rosrun(
        "tracking", "tracking_takeoff_gate_test", TrackingTakeoffGateTest
    )

#!/usr/bin/env python3

import threading
import unittest

import rospy
import rostest
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes
from geometry_msgs.msg import PoseStamped, TwistStamped
from look_up.msg import CoordinateBroadcastHeartbeat
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


class TrackingCompletionTest(unittest.TestCase):
    def setUp(self):
        self.lock = threading.Lock()
        self.requested_targets = []
        self.released_targets = []
        self.completed_targets = []
        self.selected_topics = []
        self.mission_commands = []
        self.events = []
        self.navigator_failures_remaining = 0

        self.services = [
            rospy.Service(
                "/test_drone_0/pose_cmd_mux/select",
                MuxSelect,
                self._select_callback,
            ),
            rospy.Service(
                "/lookup/complete_target",
                CompleteTarget,
                self._complete_callback,
            ),
        ]
        for target_id in TARGET_IDS:
            self.services.extend(
                (
                    rospy.Service(
                        "/lookup/request_" + target_id,
                        RequestTarget,
                        self._request_callback,
                    ),
                    rospy.Service(
                        "/lookup/release_" + target_id,
                        ReleaseTarget,
                        self._release_callback,
                    ),
                )
            )

        self.takeoff_gate_pub = rospy.Publisher(
            "/swarm/takeoff_complete", Bool, queue_size=1, latch=True
        )
        self.mission_gate_pub = rospy.Publisher(
            "/test_drone_0/mission/active", Bool, queue_size=1, latch=True
        )
        self.pose_pub = rospy.Publisher(
            "/test_drone_0/mavros/local_position/pose", PoseStamped, queue_size=1
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
        self.heartbeat_pub = rospy.Publisher(
            "/test_drone_0/coordinate_broadcast/heartbeat",
            CoordinateBroadcastHeartbeat,
            queue_size=10,
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
            if request.topic.endswith("/mux_inputs/navigator/cmd_vel"):
                if self.navigator_failures_remaining > 0:
                    self.navigator_failures_remaining -= 1
                    self.events.append("mux_fail")
                    raise rospy.ServiceException("planned navigator failure")
                self.events.append("mux_success")
        return MuxSelectResponse(prev_topic="")

    def _request_callback(self, request):
        with self.lock:
            self.requested_targets.append(request.target_id)
        return RequestTargetResponse(success=True)

    def _release_callback(self, request):
        with self.lock:
            self.released_targets.append(request.target_id)
            self.events.append("release:" + request.target_id)
        return ReleaseTargetResponse(success=True)

    def _complete_callback(self, request):
        with self.lock:
            self.completed_targets.append(request.target_id)
            self.events.append("complete:" + request.target_id)
        return CompleteTargetResponse(success=True)

    def _mission_callback(self, message):
        with self.lock:
            self.mission_commands.append(message.data)
            self.events.append("mission:" + message.data)

    def _snapshot(self, values):
        with self.lock:
            return list(values)

    def _publish_inputs_once(self, target_id=None):
        now = rospy.Time.now()
        pose = PoseStamped()
        pose.header.stamp = now
        pose.pose.position.z = 3.1
        self.pose_pub.publish(pose)

        velocity = TwistStamped()
        velocity.header.stamp = now
        self.velocity_pub.publish(velocity)

        boxes = BoundingBoxes()
        boxes.header.stamp = now
        if target_id is not None:
            target = BoundingBox()
            target.Class = target_id
            target.probability = 0.95
            target.xmin = 260
            target.ymin = 180
            target.xmax = 380
            target.ymax = 420
            boxes.bounding_boxes = [target]
        self.boxes_pub.publish(boxes)

    def _publish_heartbeat(self, target_id, vehicle_name="test_drone_0"):
        heartbeat = CoordinateBroadcastHeartbeat()
        heartbeat.header.stamp = rospy.Time.now()
        heartbeat.vehicle_name = vehicle_name
        heartbeat.target_id = target_id
        self.heartbeat_pub.publish(heartbeat)

    def _publish_for(self, seconds, target_id=None, heartbeat_id=None):
        deadline = rospy.Time.now() + rospy.Duration(seconds)
        rate = rospy.Rate(50)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            self._publish_inputs_once(target_id)
            if heartbeat_id is not None:
                self._publish_heartbeat(heartbeat_id)
            rate.sleep()

    def _publish_until(self, predicate, message, target_id=None, timeout=5.0):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(50)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            self._publish_inputs_once(target_id)
            if predicate():
                return
            rate.sleep()
        self.fail(message)

    def test_completed_and_failed_reporting_sessions_resume_once(self):
        self.takeoff_gate_pub.publish(Bool(data=True))
        self.mission_gate_pub.publish(Bool(data=True))

        self._publish_until(
            lambda: self._snapshot(self.requested_targets) == ["green0"],
            "green0 was not requested",
            target_id="green0",
        )
        self._publish_until(
            lambda: "PAUSE" in self._snapshot(self.mission_commands),
            "green0 takeover did not pause the mission",
            target_id="green0",
        )
        self._publish_for(0.45, target_id="green0", heartbeat_id="green0")

        with self.lock:
            self.navigator_failures_remaining = 1
        self._publish_until(
            lambda: self._snapshot(self.completed_targets) == ["green0"],
            "confirmed green0 was not completed after disappearing",
            target_id=None,
        )
        self._publish_until(
            lambda: self._snapshot(self.mission_commands).count("RESUME") == 1,
            "green0 completion did not resume the mission",
            target_id=None,
        )

        events = self._snapshot(self.events)
        self.assertLess(events.index("mux_fail"), events.index("mux_success"))
        self.assertLess(
            events.index("mux_success"), events.index("complete:green0")
        )
        self.assertLess(
            events.index("complete:green0"), events.index("mission:RESUME")
        )
        self.assertEqual(["green0"], self._snapshot(self.completed_targets))

        self._publish_until(
            lambda: self._snapshot(self.requested_targets).count("blue1") == 1,
            "blue1 was not requested",
            target_id="blue1",
        )
        self._publish_heartbeat("green0")
        self._publish_heartbeat("blue1", vehicle_name="other_drone_0")
        self._publish_until(
            lambda: self._snapshot(self.released_targets).count("blue1") == 1,
            "blue1 without valid heartbeat was not released at session timeout",
            target_id="blue1",
        )
        self._publish_until(
            lambda: self._snapshot(self.mission_commands).count("RESUME") == 2,
            "blue1 timeout did not resume the mission",
            target_id="blue1",
        )
        self.assertNotIn("blue1", self._snapshot(self.completed_targets))

        self._publish_for(0.2, target_id="blue1")
        self.assertEqual(1, self._snapshot(self.requested_targets).count("blue1"))
        self._publish_until(
            lambda: self._snapshot(self.requested_targets).count("blue1") == 2,
            "blue1 was not requestable after local cooldown",
            target_id="blue1",
            timeout=2.0,
        )


if __name__ == "__main__":
    rospy.init_node("tracking_completion_test")
    rostest.rosrun("tracking", "tracking_completion", TrackingCompletionTest)

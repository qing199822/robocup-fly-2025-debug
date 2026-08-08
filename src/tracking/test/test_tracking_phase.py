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


class TrackingPhaseTest(unittest.TestCase):
    def setUp(self):
        self.lock = threading.Lock()
        self.phases = []
        self.fail_navigator_once = True

        self.services = [
            rospy.Service(
                "/typhoon_h480_0/pose_cmd_mux/select",
                MuxSelect,
                self._select_callback,
            ),
            rospy.Service(
                "/lookup/complete_target",
                CompleteTarget,
                lambda request: CompleteTargetResponse(success=True),
            ),
        ]
        for target_id in TARGET_IDS:
            self.services.extend(
                (
                    rospy.Service(
                        "/lookup/request_" + target_id,
                        RequestTarget,
                        lambda request: RequestTargetResponse(success=True),
                    ),
                    rospy.Service(
                        "/lookup/release_" + target_id,
                        ReleaseTarget,
                        lambda request: ReleaseTargetResponse(success=True),
                    ),
                )
            )

        self.phase_sub = rospy.Subscriber(
            "/typhoon_h480_0/tracking/phase",
            String,
            self._phase_callback,
            queue_size=100,
        )
        self.takeoff_pub = rospy.Publisher(
            "/swarm/takeoff_complete", Bool, queue_size=1, latch=True
        )
        self.mission_pub = rospy.Publisher(
            "/typhoon_h480_0/mission/active", Bool, queue_size=1, latch=True
        )
        self.pose_pub = rospy.Publisher(
            "/typhoon_h480_0/mavros/local_position/pose",
            PoseStamped,
            queue_size=1,
        )
        self.velocity_pub = rospy.Publisher(
            "/typhoon_h480_0/mavros/local_position/velocity_body",
            TwistStamped,
            queue_size=1,
        )
        self.boxes_pub = rospy.Publisher(
            "/typhoon_h480_0/yolo11n/bounding_boxes",
            BoundingBoxes,
            queue_size=1,
        )

        self.takeoff_pub.publish(Bool(data=False))
        self.mission_pub.publish(Bool(data=False))

    def _select_callback(self, request):
        if request.topic.endswith("/mux_inputs/navigator/cmd_vel"):
            with self.lock:
                if self.fail_navigator_once:
                    self.fail_navigator_once = False
                    raise rospy.ServiceException("planned navigator failure")
        return MuxSelectResponse(prev_topic="")

    def _phase_callback(self, message):
        parts = message.data.split(":")
        if len(parts) != 3:
            return
        try:
            float(parts[2])
        except ValueError:
            return
        with self.lock:
            self.phases.append((parts[0], parts[1]))

    def _phase_names(self):
        with self.lock:
            return [phase for phase, _ in self.phases]

    def _publish_inputs(self, target_visible):
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
        if target_visible:
            target = BoundingBox()
            target.Class = "green0"
            target.probability = 0.95
            target.xmin = 260
            target.ymin = 180
            target.xmax = 380
            target.ymax = 420
            boxes.bounding_boxes = [target]
        self.boxes_pub.publish(boxes)

    def _publish_until(self, predicate, message, target_visible=False, timeout=4.0):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(50)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            self._publish_inputs(target_visible)
            if predicate():
                return
            rate.sleep()
        self.fail(message + "; observed phases=" + repr(self._phase_names()))

    def test_reports_existing_state_transitions_without_controlling_them(self):
        self._publish_until(
            lambda: "WAIT_READY" in self._phase_names(),
            "closed gates did not publish WAIT_READY",
        )

        self.takeoff_pub.publish(Bool(data=True))
        self.mission_pub.publish(Bool(data=True))
        self._publish_until(
            lambda: "IDLE" in self._phase_names(),
            "open gates did not publish IDLE",
        )
        self._publish_until(
            lambda: "DETECTING" in self._phase_names(),
            "first target lock did not publish DETECTING",
            target_visible=True,
        )
        self._publish_until(
            lambda: "DASH" in self._phase_names()
            or "TRACKING" in self._phase_names(),
            "confirmed target did not publish DASH or TRACKING",
            target_visible=True,
        )
        self._publish_until(
            lambda: "RETURNING" in self._phase_names(),
            "ending session did not publish RETURNING",
            target_visible=True,
        )

        phases = self._phase_names()
        ordered = ["WAIT_READY", "IDLE", "DETECTING", "RETURNING"]
        indexes = [phases.index(name) for name in ordered]
        active_index = min(
            index for index, name in enumerate(phases)
            if name in ("DASH", "TRACKING")
        )
        self.assertLess(indexes[0], indexes[1])
        self.assertLess(indexes[1], indexes[2])
        self.assertLess(indexes[2], active_index)
        self.assertLess(active_index, indexes[3])

        with self.lock:
            detecting_targets = [
                target for phase, target in self.phases if phase == "DETECTING"
            ]
        self.assertIn("green0", detecting_targets)


if __name__ == "__main__":
    rospy.init_node("tracking_phase_test")
    rostest.rosrun("tracking", "tracking_phase", TrackingPhaseTest)

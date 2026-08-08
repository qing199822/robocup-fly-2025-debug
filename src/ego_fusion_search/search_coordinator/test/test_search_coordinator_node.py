#!/usr/bin/env python3

import math
import threading
import unittest

import rospy
import rostest
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from search_msgs.msg import PerceptionHealth
from search_msgs.srv import ValidateTrajectory, ValidateTrajectoryResponse
from std_msgs.msg import Bool, String, UInt64


NAVIGATOR = "/typhoon_h480_0/mux_inputs/navigator/cmd_vel"
EXTERNAL = "/typhoon_h480_0/mux_inputs/external/pose_cmd"


class SearchCoordinatorNodeTest(unittest.TestCase):
    def setUp(self):
        self.lock = threading.Lock()
        self.generations = []
        self.goals = []
        self.statuses = []
        self.validation_requests = []
        self.validation_valid = True

        self.validation_service = rospy.Service(
            "/test/validate", ValidateTrajectory, self._validate
        )
        self.generation_sub = rospy.Subscriber(
            "/test/generation", UInt64, self._store_generation, queue_size=20
        )
        self.goal_sub = rospy.Subscriber(
            "/test/ego_goal", PoseStamped, self._store_goal, queue_size=20
        )
        self.status_sub = rospy.Subscriber(
            "/test/coordinator_status", String, self._store_status, queue_size=100
        )

        self.high_goal_pub = rospy.Publisher(
            "/test/high_level_goal", PoseStamped, queue_size=1, latch=True
        )
        self.odom_pub = rospy.Publisher("/test/odom", Odometry, queue_size=1)
        self.health_pub = rospy.Publisher(
            "/test/health", PerceptionHealth, queue_size=1
        )
        self.frontier_pub = rospy.Publisher(
            "/test/frontier", PoseStamped, queue_size=1
        )
        self.mission_pub = rospy.Publisher(
            "/test/mission_active", Bool, queue_size=1, latch=True
        )
        self.takeoff_pub = rospy.Publisher(
            "/test/takeoff_complete", Bool, queue_size=1, latch=True
        )
        self.tracking_pub = rospy.Publisher(
            "/test/tracking_phase", String, queue_size=1, latch=True
        )
        self.mux_pub = rospy.Publisher(
            "/test/mux_selected", String, queue_size=1, latch=True
        )
        self.adapter_pub = rospy.Publisher(
            "/test/adapter_status", String, queue_size=1, latch=True
        )

        self.mission_pub.publish(Bool(data=False))
        self.takeoff_pub.publish(Bool(data=False))
        self.tracking_pub.publish(String(data="WAIT_READY::0.0"))
        self.mux_pub.publish(String(data=NAVIGATOR))
        self.adapter_pub.publish(String(data="WAITING"))

    def _store_generation(self, message):
        with self.lock:
            self.generations.append(message.data)

    def _store_goal(self, message):
        with self.lock:
            self.goals.append(message)

    def _store_status(self, message):
        with self.lock:
            self.statuses.append(message.data)
            self.statuses = self.statuses[-200:]

    def _validate(self, request):
        with self.lock:
            self.validation_requests.append(request)
            valid = self.validation_valid
        return ValidateTrajectoryResponse(
            valid=valid,
            task_generation=request.task_generation,
            map_stamp=rospy.Time.now(),
            min_clearance_m=2.0 if valid else 0.0,
            fault_code="OK" if valid else "OCCUPIED",
        )

    def _snapshot(self, name):
        with self.lock:
            return list(getattr(self, name))

    def _publish_goal(self, x, y, z):
        message = PoseStamped()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "map"
        message.pose.position.x = x
        message.pose.position.y = y
        message.pose.position.z = z
        message.pose.orientation.w = 1.0
        self.high_goal_pub.publish(message)

    def _publish_inputs(self, x=0.0, y=0.0, z=3.0):
        now = rospy.Time.now()
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "map"
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = z
        odom.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(odom)

        health = PerceptionHealth()
        health.header.stamp = now
        health.header.frame_id = "map"
        health.depth_healthy = True
        health.odom_healthy = True
        health.synchronized = True
        health.map_healthy = True
        health.fault_code = "OK"
        self.health_pub.publish(health)

    def _publish_until(self, predicate, message, position=(0.0, 0.0, 3.0), timeout=4.0):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(50)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            self._publish_inputs(*position)
            if predicate():
                return
            rate.sleep()
        self.fail(
            message
            + "; generations="
            + repr(self._snapshot("generations"))
            + "; statuses="
            + repr(self._snapshot("statuses"))
        )

    def test_tracking_takeover_invalidates_and_rejoins_from_current_odom(self):
        self._publish_goal(20.0, 0.0, 3.0)
        self._publish_inputs()
        rospy.sleep(0.2)
        self.assertEqual([], self._snapshot("generations"))
        self.assertEqual([], self._snapshot("goals"))

        self.takeoff_pub.publish(Bool(data=True))
        self.mission_pub.publish(Bool(data=True))
        self.tracking_pub.publish(String(data="IDLE::1.0"))
        self._publish_until(
            lambda: 1 in self._snapshot("generations")
            and len(self._snapshot("goals")) >= 1,
            "ready coordinator did not publish generation 1 and EGO goal",
        )

        first_goal = self._snapshot("goals")[-1]
        self.assertLessEqual(
            math.hypot(first_goal.pose.position.x, first_goal.pose.position.y),
            8.0 + 1e-6,
        )
        first_request = self._snapshot("validation_requests")[-1]
        self.assertEqual(1, first_request.task_generation)
        self.assertAlmostEqual(0.0, first_request.samples[0].x)
        self.assertAlmostEqual(first_goal.pose.position.x, first_request.samples[-1].x)

        goal_count = len(self._snapshot("goals"))
        self.tracking_pub.publish(String(data="DETECTING:green0:2.0"))
        self._publish_until(
            lambda: 2 in self._snapshot("generations")
            and any(status.startswith("CANDIDATE_HOLD:") for status in self._snapshot("statuses")),
            "DETECTING did not invalidate generation 1",
        )
        rospy.sleep(0.15)
        self.assertEqual(goal_count, len(self._snapshot("goals")))

        self.tracking_pub.publish(String(data="TRACKING:green0:2.2"))
        self.mux_pub.publish(String(data=EXTERNAL))
        self._publish_until(
            lambda: any(status.startswith("TRACKING_EXTERNAL:") for status in self._snapshot("statuses")),
            "tracking takeover was not held under external control",
        )
        self.assertEqual(1, self._snapshot("generations").count(2))

        self.tracking_pub.publish(String(data="IDLE::2.4"))
        self._publish_inputs(5.0, 1.0, 3.0)
        rospy.sleep(0.15)
        self.assertEqual(2, self._snapshot("generations")[-1])

        self.mux_pub.publish(String(data=NAVIGATOR))
        self._publish_until(
            lambda: 3 in self._snapshot("generations")
            and len(self._snapshot("goals")) > goal_count,
            "navigator return did not create generation 3 from current odom",
            position=(5.0, 1.0, 3.0),
        )

        resumed_request = self._snapshot("validation_requests")[-1]
        resumed_goal = self._snapshot("goals")[-1]
        self.assertEqual(3, resumed_request.task_generation)
        self.assertAlmostEqual(5.0, resumed_request.samples[0].x)
        self.assertAlmostEqual(1.0, resumed_request.samples[0].y)
        self.assertAlmostEqual(resumed_goal.pose.position.x, resumed_request.samples[-1].x)
        self.assertAlmostEqual(resumed_goal.pose.position.y, resumed_request.samples[-1].y)

        with self.lock:
            self.validation_valid = False
        goal_count = len(self._snapshot("goals"))
        self._publish_goal(5.0, 20.0, 3.0)
        self._publish_until(
            lambda: any(
                status == "OBSERVING:NO_KNOWN_FREE_GOAL"
                for status in self._snapshot("statuses")
            ),
            "unsafe direct goal without a frontier did not enter observation hold",
            position=(5.0, 1.0, 3.0),
        )
        self.assertEqual(goal_count, len(self._snapshot("goals")))


if __name__ == "__main__":
    rospy.init_node("search_coordinator_node_test")
    rostest.rosrun(
        "search_coordinator", "search_coordinator_node", SearchCoordinatorNodeTest
    )

#!/usr/bin/env python3

import unittest

import rospy
import rostest
from look_up.srv import CompleteTarget, ReleaseTarget, RequestTarget


TARGET_IDS = ("green0", "blue1", "brown2", "white3", "red4", "red5")


class TargetLookupServiceTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        service_names = ["/lookup/complete_target"]
        for target_id in TARGET_IDS:
            service_names.extend(
                (
                    "/lookup/request_" + target_id,
                    "/lookup/release_" + target_id,
                )
            )
        for service_name in service_names:
            rospy.wait_for_service(service_name, timeout=5.0)

    def test_all_targets_release_and_completed_target_stays_unavailable(self):
        for target_id in TARGET_IDS:
            request = rospy.ServiceProxy(
                "/lookup/request_" + target_id, RequestTarget
            )
            release = rospy.ServiceProxy(
                "/lookup/release_" + target_id, ReleaseTarget
            )
            self.assertTrue(request(target_id).success)
            self.assertTrue(release(target_id).success)
            self.assertTrue(request(target_id).success)
            self.assertTrue(release(target_id).success)

        request_red5 = rospy.ServiceProxy("/lookup/request_red5", RequestTarget)
        release_red5 = rospy.ServiceProxy("/lookup/release_red5", ReleaseTarget)
        complete = rospy.ServiceProxy("/lookup/complete_target", CompleteTarget)

        self.assertTrue(request_red5("red5").success)
        self.assertTrue(complete("red5").success)
        self.assertTrue(complete("red5").success)
        self.assertTrue(release_red5("red5").success)
        self.assertFalse(request_red5("red5").success)
        self.assertFalse(complete("green0").success)
        self.assertFalse(complete("not-a-target").success)


if __name__ == "__main__":
    rospy.init_node("target_lookup_service_test")
    rostest.rosrun("look_up", "target_lookup_service", TargetLookupServiceTest)

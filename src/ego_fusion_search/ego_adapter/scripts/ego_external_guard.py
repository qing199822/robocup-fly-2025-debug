#!/usr/bin/env python3

import subprocess
import sys

import rospy


def main():
    rospy.init_node("ego_external_guard")
    ego_dir = rospy.get_param("~ego_dir")
    checker = rospy.get_param("~checker")
    result = subprocess.run(
        [sys.executable, checker, "--ego-dir", ego_dir],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    if result.returncode != 0:
        rospy.logfatal("EGO external check failed: %s", result.stderr.strip())
        return 1
    rospy.set_param("~ready", True)
    rospy.loginfo(result.stdout.strip())
    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        rate.sleep()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

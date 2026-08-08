#!/usr/bin/env python3

import math
import subprocess

import rospy
from sensor_msgs.msg import CameraInfo, Image


SUPPORTED_ENCODINGS = ("16UC1", "32FC1")


def camera_arguments(camera_info, depth):
    if depth.encoding not in SUPPORTED_ENCODINGS:
        raise ValueError("unsupported depth encoding: " + depth.encoding)
    if (
        camera_info.width <= 0
        or camera_info.height <= 0
        or camera_info.width != depth.width
        or camera_info.height != depth.height
    ):
        raise ValueError("CameraInfo and depth dimensions do not match")
    fx = camera_info.K[0]
    fy = camera_info.K[4]
    cx = camera_info.K[2]
    cy = camera_info.K[5]
    if not all(math.isfinite(value) for value in (fx, fy, cx, cy)):
        raise ValueError("camera intrinsics must be finite")
    if fx <= 0.0 or fy <= 0.0:
        raise ValueError("camera focal lengths must be positive")
    return fx, fy, cx, cy, depth.encoding


def runtime_command(runtime_launch, camera_values):
    fx, fy, cx, cy, encoding = camera_values
    return [
        "roslaunch",
        runtime_launch,
        "fx:=" + repr(fx),
        "fy:=" + repr(fy),
        "cx:=" + repr(cx),
        "cy:=" + repr(cy),
        "depth_encoding:=" + encoding,
    ]


def wait_for_guard(parameter_name, timeout):
    deadline = rospy.Time.now() + rospy.Duration(timeout)
    rate = rospy.Rate(20.0)
    while not rospy.is_shutdown() and rospy.Time.now() < deadline:
        if rospy.get_param(parameter_name, False):
            return
        rate.sleep()
    raise RuntimeError("EGO external guard did not become ready")


def wait_for_camera(camera_info_topic, depth_topic, timeout):
    deadline = rospy.Time.now() + rospy.Duration(timeout)
    last_error = "no camera data"
    while not rospy.is_shutdown() and rospy.Time.now() < deadline:
        remaining = max(0.05, (deadline - rospy.Time.now()).to_sec())
        try:
            camera_info = rospy.wait_for_message(
                camera_info_topic, CameraInfo, timeout=min(1.0, remaining)
            )
            depth = rospy.wait_for_message(
                depth_topic, Image, timeout=min(1.0, remaining)
            )
            return camera_arguments(camera_info, depth)
        except rospy.ROSException as error:
            last_error = str(error)
        except ValueError as error:
            last_error = str(error)
            if "unsupported depth encoding" in last_error:
                raise
    raise RuntimeError("valid EGO camera data not received: " + last_error)


def main():
    rospy.init_node("ego_camera_bootstrap")
    camera_info_topic = rospy.get_param("~camera_info_topic")
    depth_topic = rospy.get_param("~depth_topic")
    runtime_launch = rospy.get_param("~runtime_launch")
    guard_ready_param = rospy.get_param(
        "~guard_ready_param", "/ego_external_guard/ready"
    )
    timeout = float(rospy.get_param("~camera_timeout", 30.0))
    child = None
    try:
        wait_for_guard(guard_ready_param, timeout)
        camera_values = wait_for_camera(
            camera_info_topic, depth_topic, timeout
        )
        command = runtime_command(runtime_launch, camera_values)
        child = subprocess.Popen(command)
        while not rospy.is_shutdown():
            status = child.poll()
            if status is not None:
                rospy.logfatal("EGO child roslaunch exited with status %d", status)
                return status if status != 0 else 1
            rospy.sleep(0.1)
    except (RuntimeError, ValueError, OSError) as error:
        rospy.logfatal("EGO camera bootstrap failed: %s", error)
        return 1
    finally:
        if child is not None and child.poll() is None:
            child.terminate()
            try:
                child.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

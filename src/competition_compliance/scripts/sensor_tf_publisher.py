#!/usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

from competition_compliance.model import ComplianceError, load_mount_pose
from competition_compliance.tf_math import compose_mount_to_optical


PARENT_FRAME = "base_link"
CHILD_FRAME = "depth_camera_base"


def main():
    rospy.init_node("competition_sensor_tf")
    try:
        mount_config = rospy.get_param("~mount_config")
    except Exception as error:
        rospy.logfatal(
            "合规自检失败：无法读取必需参数 ~mount_config：{}".format(error)
        )
        return 2

    try:
        mount_pose = load_mount_pose(mount_config)
    except ComplianceError as error:
        rospy.logfatal("合规自检失败：{}".format(error))
        return 2
    except Exception as error:
        rospy.logfatal("合规自检失败：无法读取安装配置：{}".format(error))
        return 2

    translation, quaternion = compose_mount_to_optical(mount_pose)
    transform = TransformStamped()
    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = PARENT_FRAME
    transform.child_frame_id = CHILD_FRAME
    transform.transform.translation.x = translation[0]
    transform.transform.translation.y = translation[1]
    transform.transform.translation.z = translation[2]
    transform.transform.rotation.x = quaternion[0]
    transform.transform.rotation.y = quaternion[1]
    transform.transform.rotation.z = quaternion[2]
    transform.transform.rotation.w = quaternion[3]

    broadcaster = tf2_ros.StaticTransformBroadcaster()
    broadcaster.sendTransform(transform)
    rospy.loginfo(
        "已发布竞赛传感器静态变换：%s -> %s", PARENT_FRAME, CHILD_FRAME
    )
    rospy.spin()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

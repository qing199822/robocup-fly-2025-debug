import math

from tf.transformations import quaternion_from_euler, quaternion_multiply


OFFICIAL_OPTICAL_RPY = (-math.pi / 2.0, 0.0, -math.pi / 2.0)


def compose_mount_to_optical(mount_pose):
    mount_quaternion = quaternion_from_euler(
        mount_pose.roll, mount_pose.pitch, mount_pose.yaw
    )
    optical_quaternion = quaternion_from_euler(*OFFICIAL_OPTICAL_RPY)
    quaternion = quaternion_multiply(mount_quaternion, optical_quaternion)
    translation = (
        float(mount_pose.x),
        float(mount_pose.y),
        float(mount_pose.z),
    )
    return translation, tuple(float(component) for component in quaternion)

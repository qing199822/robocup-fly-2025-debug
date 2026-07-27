#!/usr/bin/env python3

import math
import unittest

import numpy
from tf.transformations import quaternion_from_euler, quaternion_matrix

from competition_compliance.model import MountPose
from competition_compliance.tf_math import (
    OFFICIAL_OPTICAL_RPY,
    compose_mount_to_optical,
)


class TransformCompositionTest(unittest.TestCase):
    def test_default_mount_maps_optical_forward_to_body_forward(self):
        pose = MountPose.from_values([0.09, 0, -0.04, 0, 0, 0])

        translation, quaternion = compose_mount_to_optical(pose)

        self.assertEqual((0.09, 0.0, -0.04), translation)
        body_direction = quaternion_matrix(quaternion).dot(
            numpy.array([0.0, 0.0, 1.0, 0.0])
        )
        numpy.testing.assert_allclose(
            numpy.array([1.0, 0.0, 0.0]), body_direction[:3], atol=1e-6
        )

    def test_nonzero_mount_rpy_composes_before_official_optical_rotation(self):
        pose = MountPose.from_values([0.12, -0.03, 0.07, 0.3, -0.2, 0.4])

        translation, quaternion = compose_mount_to_optical(pose)

        mount_matrix = quaternion_matrix(
            quaternion_from_euler(pose.roll, pose.pitch, pose.yaw)
        )
        optical_matrix = quaternion_matrix(
            quaternion_from_euler(*OFFICIAL_OPTICAL_RPY)
        )
        numpy.testing.assert_allclose(
            numpy.dot(mount_matrix, optical_matrix),
            quaternion_matrix(quaternion),
            atol=1e-12,
        )
        self.assertEqual((0.12, -0.03, 0.07), translation)
        self.assertTrue(all(math.isfinite(component) for component in quaternion))
        self.assertAlmostEqual(1.0, math.sqrt(sum(value * value for value in quaternion)))


if __name__ == "__main__":
    unittest.main()

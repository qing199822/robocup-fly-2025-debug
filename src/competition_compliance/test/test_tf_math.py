#!/usr/bin/env python3

import math
import unittest

import numpy
from tf.transformations import quaternion_matrix

from competition_compliance.model import MountPose
from competition_compliance.tf_math import compose_mount_to_optical


class TransformCompositionTest(unittest.TestCase):
    def test_default_mount_maps_optical_forward_to_body_forward(self):
        pose = MountPose.from_values([0.09, 0, -0.04, 0, 0, 0])

        translation, quaternion = compose_mount_to_optical(pose)

        self.assertEqual((0.09, 0.0, -0.04), translation)
        rotation = quaternion_matrix(quaternion)[:3, :3]
        numpy.testing.assert_allclose(
            numpy.array(
                [
                    [0.0, 0.0, 1.0],
                    [-1.0, 0.0, 0.0],
                    [0.0, -1.0, 0.0],
                ]
            ),
            rotation,
            atol=1e-6,
        )
        numpy.testing.assert_allclose([0.0, -1.0, 0.0], rotation[:, 0], atol=1e-6)
        numpy.testing.assert_allclose([0.0, 0.0, -1.0], rotation[:, 1], atol=1e-6)
        numpy.testing.assert_allclose([1.0, 0.0, 0.0], rotation[:, 2], atol=1e-6)

    def test_nonzero_mount_rpy_composes_before_official_optical_rotation(self):
        pose = MountPose.from_values([0.12, -0.03, 0.07, 0.3, -0.2, 0.4])

        translation, quaternion = compose_mount_to_optical(pose)

        numpy.testing.assert_allclose(
            [
                -0.46939595433268194,
                0.42465606396068334,
                -0.32652157154907696,
                0.7019389779109375,
            ],
            quaternion,
            atol=1e-12,
        )
        self.assertEqual((0.12, -0.03, 0.07), translation)
        self.assertTrue(all(math.isfinite(component) for component in quaternion))
        self.assertAlmostEqual(1.0, math.sqrt(sum(value * value for value in quaternion)))


if __name__ == "__main__":
    unittest.main()

#!/usr/bin/env python3

import importlib.util
import math
import pathlib
import unittest

from sensor_msgs.msg import CameraInfo, Image


ROOT = pathlib.Path(__file__).resolve().parents[1]
BOOTSTRAP = (
    ROOT
    / "src/ego_fusion_search/ego_adapter/scripts/ego_camera_bootstrap.py"
)


def load_bootstrap():
    spec = importlib.util.spec_from_file_location(
        "ego_camera_bootstrap_under_test", BOOTSTRAP
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class EgoCameraBootstrapTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.bootstrap = load_bootstrap()

    @staticmethod
    def camera(fx=400.0, fy=401.0, cx=320.0, cy=240.0, width=640, height=480):
        message = CameraInfo(width=width, height=height)
        message.K = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        return message

    @staticmethod
    def depth(encoding="16UC1", width=640, height=480):
        return Image(encoding=encoding, width=width, height=height)

    def test_runtime_command_uses_each_received_camera_matrix(self):
        first = self.bootstrap.runtime_command(
            "/tmp/ego_runtime.launch", (400.0, 401.0, 320.0, 240.0, "16UC1")
        )
        second = self.bootstrap.runtime_command(
            "/tmp/ego_runtime.launch", (525.5, 526.5, 319.5, 239.5, "32FC1")
        )

        self.assertEqual("fx:=400.0", first[2])
        self.assertEqual("fy:=401.0", first[3])
        self.assertEqual("depth_encoding:=16UC1", first[6])
        self.assertEqual("fx:=525.5", second[2])
        self.assertEqual("fy:=526.5", second[3])
        self.assertEqual("depth_encoding:=32FC1", second[6])

    def test_accepts_supported_depth_encodings(self):
        for encoding in ("16UC1", "32FC1"):
            with self.subTest(encoding=encoding):
                self.assertEqual(
                    (400.0, 401.0, 320.0, 240.0, encoding),
                    self.bootstrap.camera_arguments(
                        self.camera(), self.depth(encoding=encoding)
                    ),
                )

    def test_rejects_dimension_mismatch(self):
        with self.assertRaisesRegex(ValueError, "dimensions do not match"):
            self.bootstrap.camera_arguments(
                self.camera(), self.depth(width=320, height=240)
            )

    def test_rejects_nonfinite_intrinsics(self):
        for value in (math.nan, math.inf, -math.inf):
            with self.subTest(value=value), self.assertRaisesRegex(
                ValueError, "must be finite"
            ):
                self.bootstrap.camera_arguments(
                    self.camera(fx=value), self.depth()
                )

    def test_rejects_nonpositive_focal_lengths(self):
        for fx, fy in ((0.0, 401.0), (-1.0, 401.0), (400.0, 0.0)):
            with self.subTest(fx=fx, fy=fy), self.assertRaisesRegex(
                ValueError, "must be positive"
            ):
                self.bootstrap.camera_arguments(
                    self.camera(fx=fx, fy=fy), self.depth()
                )

    def test_rejects_unknown_encoding_immediately(self):
        with self.assertRaisesRegex(ValueError, "unsupported depth encoding"):
            self.bootstrap.camera_arguments(
                self.camera(), self.depth(encoding="8UC1")
            )


if __name__ == "__main__":
    unittest.main()

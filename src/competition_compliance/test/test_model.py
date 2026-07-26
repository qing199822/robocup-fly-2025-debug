#!/usr/bin/env python3

import copy
import math
import os
import pathlib
import tempfile
import unittest
import xml.etree.ElementTree as ET
from unittest import mock

from competition_compliance.model import (
    ComplianceError,
    MountPose,
    assert_only_mount_pose_changed,
    generate_model,
    load_mount_pose,
)


WORKSPACE = pathlib.Path(__file__).resolve().parents[3]


def _default_xtdrone_dir():
    for directory in (WORKSPACE, *WORKSPACE.parents):
        candidate = directory / "XTDrone"
        if candidate.is_dir():
            return candidate
    return WORKSPACE.parent / "XTDrone"


XTDRONE_DIR = pathlib.Path(
    os.environ.get("XTDRONE_DIR", str(_default_xtdrone_dir()))
)
OFFICIAL = (
    XTDRONE_DIR
    / "sitl_config/models/typhoon_h480_realsense/typhoon_h480_realsense.sdf"
)


def write_variant(directory, mutate):
    tree = ET.parse(str(OFFICIAL))
    mutate(tree.getroot())
    path = pathlib.Path(directory) / "official-variant.sdf"
    tree.write(str(path), encoding="utf-8", xml_declaration=True)
    return path


def realsense_includes(root):
    return [
        include
        for include in root.findall("./model/include")
        if (include.findtext("uri") or "").strip() == "model://realsense_camera"
    ]


class ModelGenerationTest(unittest.TestCase):
    def test_mount_pose_requires_six_finite_numbers(self):
        with self.assertRaises(ComplianceError):
            MountPose.from_values([0, 0, 0, 0, 0])
        with self.assertRaises(ComplianceError):
            MountPose.from_values([0, 0, 0, 0, 0, 0, 0])
        with self.assertRaises(ComplianceError):
            MountPose.from_values([0, 0, 0, 0, 0, math.nan])
        with self.assertRaises(ComplianceError):
            MountPose.from_values([0, 0, 0, 0, 0, "not-a-number"])

    def test_mount_pose_overflow_is_reported_as_compliance_error(self):
        with self.assertRaises(ComplianceError):
            MountPose.from_values([0, 0, 0, 0, 0, 10**10000])

    def test_generation_changes_only_realsense_include_pose(self):
        pose = MountPose.from_values([0.1, 0.0, -0.05, 0.0, 0.2, 0.0])
        with tempfile.TemporaryDirectory() as directory:
            output = pathlib.Path(directory) / "nested" / "model.sdf"
            generate_model(OFFICIAL, output, pose)
            assert_only_mount_pose_changed(OFFICIAL, output)
            self.assertIn(
                "0.1 0 -0.05 0 0.2 0", output.read_text(encoding="utf-8")
            )

    def test_non_pose_change_is_rejected(self):
        pose = MountPose.from_values([0.09, 0, -0.04, 0, 0, 0])
        with tempfile.TemporaryDirectory() as directory:
            output = pathlib.Path(directory) / "model.sdf"
            generate_model(OFFICIAL, output, pose)
            output.write_text(
                output.read_text(encoding="utf-8").replace(
                    "<mass>2.02</mass>", "<mass>1</mass>"
                ),
                encoding="utf-8",
            )
            with self.assertRaises(ComplianceError):
                assert_only_mount_pose_changed(OFFICIAL, output)

    def test_non_whitespace_tail_change_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            output = pathlib.Path(directory) / "model.sdf"
            generate_model(OFFICIAL, output, MountPose.from_values([0] * 6))
            tree = ET.parse(str(output))
            mass = tree.getroot().find("./model/link/inertial/mass")
            mass.tail = (mass.tail or "") + "tampered"
            tree.write(str(output), encoding="utf-8", xml_declaration=True)

            with self.assertRaises(ComplianceError):
                assert_only_mount_pose_changed(OFFICIAL, output)

    def test_missing_realsense_include_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:

            def remove_include(root):
                root.find("./model").remove(realsense_includes(root)[0])

            variant = write_variant(directory, remove_include)
            with self.assertRaises(ComplianceError):
                generate_model(
                    variant,
                    pathlib.Path(directory) / "out.sdf",
                    MountPose.from_values([0] * 6),
                )

    def test_duplicate_realsense_include_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:

            def duplicate_include(root):
                root.find("./model").append(copy.deepcopy(realsense_includes(root)[0]))

            variant = write_variant(directory, duplicate_include)
            with self.assertRaises(ComplianceError):
                generate_model(
                    variant,
                    pathlib.Path(directory) / "out.sdf",
                    MountPose.from_values([0] * 6),
                )

    def test_malformed_official_mount_pose_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:

            def truncate_pose(root):
                realsense_includes(root)[0].find("pose").text = "0 0 0"

            variant = write_variant(directory, truncate_pose)
            output = pathlib.Path(directory) / "out.sdf"
            with self.assertRaises(ComplianceError):
                generate_model(variant, output, MountPose.from_values([0] * 6))
            self.assertFalse(output.exists())

    def test_duplicate_official_mount_pose_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:

            def duplicate_pose(root):
                include = realsense_includes(root)[0]
                ET.SubElement(include, "pose").text = "0 0 0 0 0 0"

            variant = write_variant(directory, duplicate_pose)
            with self.assertRaises(ComplianceError):
                generate_model(
                    variant,
                    pathlib.Path(directory) / "out.sdf",
                    MountPose.from_values([0] * 6),
                )

    def test_fixed_joint_parent_change_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:

            def change_parent(root):
                for joint in root.findall("./model/joint"):
                    if (
                        joint.findtext("child") or ""
                    ).strip() == "realsense_camera::link":
                        joint.find("parent").text = "cgo3_camera_link"

            variant = write_variant(directory, change_parent)
            with self.assertRaises(ComplianceError):
                generate_model(
                    variant,
                    pathlib.Path(directory) / "out.sdf",
                    MountPose.from_values([0] * 6),
                )

    def test_mount_config_rejects_missing_extra_and_malformed_fields(self):
        invalid_documents = (
            "other_key: [0, 0, 0, 0, 0, 0]\n",
            "realsense_mount: [0, 0, 0, 0, 0, 0]\nextra: true\n",
            "realsense_mount: [0, 0, 0, 0, 0]\n",
            "realsense_mount: [0, 0\n",
        )
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "mount.yaml"
            for document in invalid_documents:
                with self.subTest(document=document):
                    path.write_text(document, encoding="utf-8")
                    with self.assertRaises(ComplianceError):
                        load_mount_pose(path)

    def test_mount_config_rejects_duplicate_keys(self):
        document = (
            "realsense_mount: [0, 0, 0, 0, 0, 0]\n"
            "realsense_mount: [1, 1, 1, 1, 1, 1]\n"
        )
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "mount.yaml"
            path.write_text(document, encoding="utf-8")
            with self.assertRaises(ComplianceError):
                load_mount_pose(path)

    def test_existing_output_file_is_refused(self):
        with tempfile.TemporaryDirectory() as directory:
            output = pathlib.Path(directory) / "model.sdf"
            output.write_text("keep-me", encoding="utf-8")
            with self.assertRaises(ComplianceError):
                generate_model(OFFICIAL, output, MountPose.from_values([0] * 6))
            self.assertEqual("keep-me", output.read_text(encoding="utf-8"))

    def test_post_write_validation_failure_removes_new_output(self):
        with tempfile.TemporaryDirectory() as directory:
            output = pathlib.Path(directory) / "model.sdf"
            with mock.patch(
                "competition_compliance.model.assert_only_mount_pose_changed",
                side_effect=ComplianceError("forced validation failure"),
            ):
                with self.assertRaises(ComplianceError):
                    generate_model(OFFICIAL, output, MountPose.from_values([0] * 6))
            self.assertFalse(output.exists())

    def test_malformed_xml_is_reported_as_compliance_error(self):
        with tempfile.TemporaryDirectory() as directory:
            broken = pathlib.Path(directory) / "broken.sdf"
            broken.write_text("<sdf><model>", encoding="utf-8")
            with self.assertRaisesRegex(ComplianceError, "XML"):
                generate_model(
                    broken,
                    pathlib.Path(directory) / "out.sdf",
                    MountPose.from_values([0] * 6),
                )


if __name__ == "__main__":
    unittest.main()

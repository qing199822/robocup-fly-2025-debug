#!/usr/bin/env python3

import hashlib
import json
import pathlib
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

from competition_compliance.manifest import (
    collect_versions,
    load_manifest,
    sha256_file,
    verify_manifest,
    verify_versions,
)
from competition_compliance.model import ComplianceError


WORKSPACE = pathlib.Path(__file__).resolve().parents[3]
OFFICIAL_MANIFEST = (
    WORKSPACE / "src/competition_compliance/config/official_manifest.json"
)
PREPARE_MODEL = WORKSPACE / "src/competition_compliance/scripts/prepare_model.py"

EXPECTED_OFFICIAL_MANIFEST = {
    "versions": {
        "gazebo11": "11.15.1-1~focal",
        "ros-noetic-gazebo-ros": "2.9.3-1focal.20250521.003802",
        "ros-noetic-gazebo-ros-pkgs": "2.9.3-1focal.20250521.011748",
        "xtdrone_commit": "8e88116dc15a19e5eba06300897fcfec4ab2da11",
    },
    "files": [
        {
            "root": "PX4_DIR",
            "path": "Tools/sitl_gazebo/models/typhoon_h480/typhoon_h480.sdf",
            "sha256": "4f3ae25801c704e1f9e640eaf1717e6a06a688256ad8f6ad5a0872a2843c4680",
        },
        {
            "root": "PX4_DIR",
            "path": "Tools/sitl_gazebo/worlds/robocup.world",
            "sha256": "b17daad2b9662760aba6defbd1637214e6d4832e3828ec13ca342f544c6e0b98",
        },
        {
            "root": "PX4_DIR",
            "path": "launch/single_vehicle_spawn_xtd.launch",
            "sha256": "05bb251d1bebf28890cc03191a7fbbe0e121a5e2929a18b8968eb3d9ac071e7e",
        },
        {
            "root": "XTDRONE_DIR",
            "path": "sitl_config/models/typhoon_h480/typhoon_h480.sdf",
            "sha256": "1346f71a33130e3f5634b1513cc5598d1dc2693fdf30d13c2cf9dda2ef2cd29e",
        },
        {
            "root": "XTDRONE_DIR",
            "path": "sitl_config/models/typhoon_h480_realsense/typhoon_h480_realsense.sdf",
            "sha256": "3b056f3676e8f47b90421c5357eca8154e6686304855eb14467aa82bf60ddd46",
        },
        {
            "root": "XTDRONE_DIR",
            "path": "sitl_config/models/realsense_camera/realsense_camera.sdf",
            "sha256": "0745c705ac3a90cf16529a9b49729d34f49ce7b457998a4d3cc3f2fb6aab921c",
        },
        {
            "root": "XTDRONE_DIR",
            "path": "communication/multirotor_communication.py",
            "sha256": "64c13f6ad6de9181208cf584ac1b796d49d4f153935369b41e64a4b893a74d27",
        },
    ],
}


class ManifestTest(unittest.TestCase):
    @staticmethod
    def write_manifest(root, data):
        manifest = root / "manifest.json"
        manifest.write_text(json.dumps(data), encoding="utf-8")
        return manifest

    def test_matching_file_passes_and_changed_file_fails(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            target = root / "official.txt"
            target.write_text("official", encoding="utf-8")
            digest = hashlib.sha256(b"official").hexdigest()
            manifest = root / "manifest.json"
            manifest.write_text(
                json.dumps(
                    {
                        "versions": {},
                        "files": [
                            {
                                "root": "XTDRONE_DIR",
                                "path": "official.txt",
                                "sha256": digest,
                            }
                        ],
                    }
                ),
                encoding="utf-8",
            )

            verified = verify_manifest(manifest, {"XTDRONE_DIR": root})
            self.assertEqual({}, verified["versions"])
            self.assertEqual(1, len(verified["files"]))

            target.write_text("changed", encoding="utf-8")
            with self.assertRaisesRegex(
                ComplianceError, r"(?s)official\.txt.*期望.*实际"
            ):
                verify_manifest(manifest, {"XTDRONE_DIR": root})

    def test_sha256_reads_one_mebibyte_blocks(self):
        class RecordingStream:
            def __init__(self):
                self.sizes = []
                self.blocks = iter((b"official", b""))

            def __enter__(self):
                return self

            def __exit__(self, *_args):
                return False

            def read(self, size):
                self.sizes.append(size)
                return next(self.blocks)

        stream = RecordingStream()
        with mock.patch.object(pathlib.Path, "open", return_value=stream):
            digest = sha256_file("official.txt")
        self.assertEqual(hashlib.sha256(b"official").hexdigest(), digest)
        self.assertEqual([1024 * 1024, 1024 * 1024], stream.sizes)

    def test_sha256_read_error_has_chinese_file_context(self):
        with self.assertRaisesRegex(ComplianceError, "无法读取校验文件.*missing"):
            sha256_file("missing")

    def test_manifest_read_and_json_errors_become_compliance_errors(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            with self.assertRaisesRegex(ComplianceError, "无法读取官方清单"):
                load_manifest(root / "missing.json")

            broken = root / "broken.json"
            broken.write_text("{", encoding="utf-8")
            with self.assertRaisesRegex(ComplianceError, "JSON"):
                load_manifest(broken)

    def test_manifest_requires_exact_top_level_shape(self):
        invalid_documents = (
            [],
            {},
            {"files": [], "versions": {}, "extra": True},
            {"files": {}, "versions": {}},
            {"files": [], "versions": []},
        )
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            for document in invalid_documents:
                with self.subTest(document=document):
                    manifest = self.write_manifest(root, document)
                    with self.assertRaises(ComplianceError):
                        load_manifest(manifest)

    def test_file_entries_reject_bad_shape_types_and_paths(self):
        digest = "0" * 64
        invalid_entries = (
            [],
            {"root": "ROOT", "path": "official.txt"},
            {
                "root": "ROOT",
                "path": "official.txt",
                "sha256": digest,
                "extra": True,
            },
            {"root": [], "path": "official.txt", "sha256": digest},
            {"root": "ROOT", "path": [], "sha256": digest},
            {"root": "ROOT", "path": "official.txt", "sha256": []},
            {"root": "ROOT", "path": "official.txt", "sha256": "short"},
            {"root": "ROOT", "path": "/official.txt", "sha256": digest},
            {"root": "ROOT", "path": "safe/../official.txt", "sha256": digest},
        )
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            for entry in invalid_entries:
                with self.subTest(entry=entry):
                    manifest = self.write_manifest(
                        root, {"versions": {}, "files": [entry]}
                    )
                    with self.assertRaises(ComplianceError):
                        verify_manifest(manifest, {"ROOT": root})

    def test_missing_root_and_file_are_compliance_errors(self):
        digest = "0" * 64
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            manifest = self.write_manifest(
                root,
                {
                    "versions": {},
                    "files": [
                        {"root": "ROOT", "path": "missing", "sha256": digest}
                    ],
                },
            )
            with self.assertRaisesRegex(ComplianceError, "缺少官方目录参数"):
                verify_manifest(manifest, {})
            with self.assertRaisesRegex(ComplianceError, "找不到官方文件"):
                verify_manifest(manifest, {"ROOT": root})

    @mock.patch(
        "competition_compliance.manifest.subprocess.check_output",
        side_effect=subprocess.CalledProcessError(
            1, ["dpkg-query"], output="not installed"
        ),
    )
    def test_failed_version_command_becomes_compliance_error(self, _check_output):
        with self.assertRaisesRegex(ComplianceError, "版本"):
            collect_versions(pathlib.Path("/tmp/not-used"))

    @mock.patch(
        "competition_compliance.manifest.subprocess.check_output",
        side_effect=FileNotFoundError("missing"),
    )
    def test_missing_version_command_becomes_compliance_error(self, _check_output):
        with self.assertRaisesRegex(ComplianceError, "版本.*找不到命令"):
            collect_versions(pathlib.Path("/tmp/not-used"))

    @mock.patch(
        "competition_compliance.manifest.subprocess.check_output",
        side_effect=PermissionError("permission denied"),
    )
    def test_version_command_os_error_becomes_compliance_error(self, _check_output):
        with self.assertRaisesRegex(ComplianceError, "版本.*dpkg-query"):
            collect_versions(pathlib.Path("/tmp/not-used"))

    @mock.patch(
        "competition_compliance.manifest.subprocess.check_output",
        side_effect=("1\n", "2\n", "3\n", "commit\n"),
    )
    def test_collect_versions_uses_exact_commands(self, check_output):
        xtdrone = pathlib.Path("/opt/XTDrone")
        self.assertEqual(
            {
                "gazebo11": "1",
                "ros-noetic-gazebo-ros": "2",
                "ros-noetic-gazebo-ros-pkgs": "3",
                "xtdrone_commit": "commit",
            },
            collect_versions(xtdrone),
        )
        self.assertEqual(
            [
                mock.call(
                    ["dpkg-query", "-W", "-f=${Version}", "gazebo11"],
                    stderr=subprocess.STDOUT,
                    text=True,
                ),
                mock.call(
                    [
                        "dpkg-query",
                        "-W",
                        "-f=${Version}",
                        "ros-noetic-gazebo-ros",
                    ],
                    stderr=subprocess.STDOUT,
                    text=True,
                ),
                mock.call(
                    [
                        "dpkg-query",
                        "-W",
                        "-f=${Version}",
                        "ros-noetic-gazebo-ros-pkgs",
                    ],
                    stderr=subprocess.STDOUT,
                    text=True,
                ),
                mock.call(
                    ["git", "-C", str(xtdrone), "rev-parse", "HEAD"],
                    stderr=subprocess.STDOUT,
                    text=True,
                ),
            ],
            check_output.call_args_list,
        )

    @mock.patch("competition_compliance.manifest.collect_versions")
    def test_verify_versions_requires_exact_match(self, collect):
        expected = {"gazebo11": "expected"}
        collect.return_value = expected.copy()
        self.assertEqual(expected, verify_versions({"versions": expected}, "/xtdrone"))

        collect.return_value = {"gazebo11": "actual"}
        with self.assertRaisesRegex(ComplianceError, "期望.*expected.*实际.*actual"):
            verify_versions({"versions": expected}, "/xtdrone")

    def test_repository_manifest_matches_official_inputs_exactly(self):
        self.assertEqual(EXPECTED_OFFICIAL_MANIFEST, load_manifest(OFFICIAL_MANIFEST))

    def test_prepare_cli_reports_compliance_failure_and_recovery(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            result = subprocess.run(
                [
                    sys.executable,
                    str(PREPARE_MODEL),
                    "--px4-dir",
                    str(root),
                    "--xtdrone-dir",
                    str(root),
                    "--manifest",
                    str(root / "missing.json"),
                    "--mount-config",
                    str(root / "mount.yaml"),
                    "--output",
                    str(root / "output.sdf"),
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
        self.assertEqual(2, result.returncode)
        self.assertEqual("", result.stdout)
        self.assertIn("合规自检失败：", result.stderr)
        self.assertIn(
            "恢复方法：不要修改官方目录；按 docs/TROUBLESHOOTING.md 恢复对应版本后重试。",
            result.stderr,
        )


if __name__ == "__main__":
    unittest.main()

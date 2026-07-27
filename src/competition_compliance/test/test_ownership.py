#!/usr/bin/env python3

import hashlib
import importlib.util
import json
import pathlib
import sys
import tempfile
import unittest
from unittest import mock


ROOT = pathlib.Path(__file__).resolve().parents[3]
OWNERSHIP = ROOT / "src/competition_compliance/config/ownership.json"
VERIFY_FULL = ROOT / "src/competition_compliance/scripts/verify_full.py"
XTDRONE = ROOT.parents[2] / "XTDrone"
ACTOR_SOURCE = (
    XTDRONE / "sitl_config/gazebo_plugin/gazebo_ros_actor_plugin"
)
ACTOR_COPY = ROOT / "src/gazebo_ros_actor_plugin"

TEAM_ENTRIES = {
    "src/competition_compliance": ("0.1.0", "LicenseRef-Team-Code"),
    "src/look_up": ("0.1.0", "LicenseRef-Team-Code"),
    "src/mix_nav/fly": ("1.0.0", "BSD"),
    "src/mix_nav/simple_navigator": ("0.1.0", "BSD"),
    "src/mix_nav/task_manager": ("0.0.0", "LicenseRef-Team-Code"),
    "src/pose_init": ("0.1.0", "Apache-2.0"),
    "src/tracking": ("1.0.0", "LicenseRef-Team-Code"),
    "src/transform_tree": ("0.1.0", "LicenseRef-Team-Code"),
    "src/yolo/actor_msgs": ("0.0.0", "LicenseRef-Team-Code"),
}

DARKNET_HASHES = {
    "CHANGELOG.rst": "c8f43f5497a4eafb3ca7a7b54bfae89688b419bf2d812a99466c16ac11313162",
    "CMakeLists.txt": "d8e46f9796c1e90ffc00a9c4ec73c8756504a779f099cbcad5549697b44fa100",
    "action/CheckForObjects.action": "a8bff28a58a021bcd4ed5e287de25b5cf6e4658fbb6bdc541f921e9dc30d94a4",
    "msg/BoundingBox.msg": "de8460e46657313444a294f1b03db298207e5f17ab92eca2c2b59cd25c51aeea",
    "msg/BoundingBoxes.msg": "efabf37197aba12ff013899a75c89b7a3aeb36ed3bda4c0d6e791b68d42ae4a0",
    "msg/ObjectCount.msg": "39f220042beeea9ee2138e3439a5d7d4cecd36d80a2a8aa97320964e29c44ea5",
    "package.xml": "e7c9ac183ed43f7964e735dd52915bae0d7807b86f180f58aee8ba2750e89071",
}


def load_verify_full():
    source_root = ROOT / "src/competition_compliance/src"
    sys.path.insert(0, str(source_root))
    try:
        spec = importlib.util.spec_from_file_location(
            "competition_compliance_verify_full", str(VERIFY_FULL)
        )
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    finally:
        sys.path.remove(str(source_root))


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


class OwnershipDocumentTest(unittest.TestCase):
    def load_entries(self):
        data = json.loads(OWNERSHIP.read_text(encoding="utf-8"))
        self.assertEqual({"entries"}, set(data))
        self.assertIsInstance(data["entries"], list)
        return data["entries"]

    def test_every_catkin_package_has_exactly_one_owner_entry(self):
        entries = self.load_entries()
        paths = [entry["path"] for entry in entries]
        packages = {
            path.parent.relative_to(ROOT).as_posix()
            for path in (ROOT / "src").rglob("package.xml")
        }
        self.assertEqual(packages, set(paths))
        self.assertEqual(len(paths), len(set(paths)))

    def test_exact_team_ownership_is_recorded(self):
        entries = {entry["path"]: entry for entry in self.load_entries()}
        for path, (version, license_name) in TEAM_ENTRIES.items():
            with self.subTest(path=path):
                self.assertEqual(
                    {
                        "path": path,
                        "kind": "team",
                        "source": "this repository",
                        "version": version,
                        "license": license_name,
                    },
                    entries[path],
                )

    def test_exact_third_party_provenance_and_darknet_hashes_are_recorded(self):
        entries = {entry["path"]: entry for entry in self.load_entries()}
        self.assertEqual(
            {
                "path": "src/darknet_ros_msgs",
                "kind": "third-party",
                "source": "https://github.com/leggedrobotics/darknet_ros",
                "version": "1.1.4",
                "license": "BSD",
                "files": DARKNET_HASHES,
            },
            entries["src/darknet_ros_msgs"],
        )
        expected_actor = {
            "kind": "third-party",
            "source": "XTDrone/sitl_config/gazebo_plugin/gazebo_ros_actor_plugin",
            "version": "XTDrone 8e88116dc15a19e5eba06300897fcfec4ab2da11",
            "license": "Apache-2.0",
        }
        for path in (
            "src/gazebo_ros_actor_plugin/gazebo_ros_actor_cmd_plugin",
            "src/gazebo_ros_actor_plugin/gazebo_ros_actor_cmd_plugin_msgs",
        ):
            with self.subTest(path=path):
                self.assertEqual({"path": path, **expected_actor}, entries[path])

    def test_no_package_metadata_contains_placeholders(self):
        tokens = (
            "<license>TODO",
            "todo.com",
            "todo.todo",
            "Your Name",
            "your-email",
            "your_email",
            "your.email",
        )
        offenders = []
        for package_xml in (ROOT / "src").rglob("package.xml"):
            text = package_xml.read_text(encoding="utf-8")
            if any(token in text for token in tokens):
                offenders.append(package_xml.relative_to(ROOT).as_posix())
            if "<!-- <maintainer " in text or "<!-- <author " in text:
                offenders.append(package_xml.relative_to(ROOT).as_posix())
        self.assertEqual([], sorted(set(offenders)))

    def test_darknet_files_match_recorded_hashes(self):
        package = ROOT / "src/darknet_ros_msgs"
        actual = {
            path.relative_to(package).as_posix(): sha256(path)
            for path in package.rglob("*")
            if path.is_file()
        }
        self.assertEqual(DARKNET_HASHES, actual)

    def test_actor_copy_is_byte_identical_to_external_xtdrone(self):
        self.assertTrue(ACTOR_SOURCE.is_dir(), "XTDrone Actor source is required")
        local = {
            path.relative_to(ACTOR_COPY).as_posix(): sha256(path)
            for path in ACTOR_COPY.rglob("*")
            if path.is_file()
        }
        external = {
            path.relative_to(ACTOR_SOURCE).as_posix(): sha256(path)
            for path in ACTOR_SOURCE.rglob("*")
            if path.is_file()
        }
        self.assertEqual(external, local)


class OwnershipVerifierTest(unittest.TestCase):
    def setUp(self):
        self.module = load_verify_full()

    @staticmethod
    def write_package(root, relative="src/example", version="1.2.3", license_name="BSD"):
        package = root / relative
        package.mkdir(parents=True)
        (package / "package.xml").write_text(
            "<package><name>example</name><version>{}</version>"
            "<license>{}</license></package>".format(version, license_name),
            encoding="utf-8",
        )
        return package

    @staticmethod
    def write_ownership(root, document):
        path = root / "ownership.json"
        path.write_text(
            document if isinstance(document, str) else json.dumps(document),
            encoding="utf-8",
        )
        return path

    @staticmethod
    def team_entry(path="src/example"):
        return {
            "path": path,
            "kind": "team",
            "source": "this repository",
            "version": "1.2.3",
            "license": "BSD",
        }

    def test_valid_ownership_matches_package_metadata(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            self.write_package(root)
            ownership = self.write_ownership(
                root, {"entries": [self.team_entry()]}
            )
            entries = self.module.verify_ownership(root, ownership)
            self.assertEqual(["src/example"], [entry["path"] for entry in entries])

    def test_duplicate_json_keys_and_duplicate_entry_paths_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            self.write_package(root)
            duplicate_key = self.write_ownership(
                root,
                '{"entries": [], "entries": [%s]}'
                % json.dumps(self.team_entry()),
            )
            with self.assertRaisesRegex(self.module.ComplianceError, "重复键"):
                self.module.verify_ownership(root, duplicate_key)

            duplicate_path = self.write_ownership(
                root,
                {"entries": [self.team_entry(), self.team_entry()]},
            )
            with self.assertRaisesRegex(self.module.ComplianceError, "重复.*路径"):
                self.module.verify_ownership(root, duplicate_path)

    def test_schema_kind_and_nonempty_metadata_are_strict(self):
        invalid_entries = (
            {**self.team_entry(), "extra": True},
            {key: value for key, value in self.team_entry().items() if key != "source"},
            {**self.team_entry(), "kind": "upstream"},
            {**self.team_entry(), "source": " "},
            {**self.team_entry(), "version": 1},
            {**self.team_entry(), "license": ""},
            {**self.team_entry(), "files": {}},
        )
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            self.write_package(root)
            for entry in invalid_entries:
                with self.subTest(entry=entry):
                    ownership = self.write_ownership(root, {"entries": [entry]})
                    with self.assertRaises(self.module.ComplianceError):
                        self.module.verify_ownership(root, ownership)

    def test_paths_must_be_normalized_contained_and_free_of_symlinks(self):
        invalid_paths = (
            "/src/example",
            "src/../src/example",
            "src//example",
            "src/example/.",
            "../example",
        )
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            self.write_package(root)
            for relative in invalid_paths:
                with self.subTest(relative=relative):
                    ownership = self.write_ownership(
                        root, {"entries": [self.team_entry(relative)]}
                    )
                    with self.assertRaises(self.module.ComplianceError):
                        self.module.verify_ownership(root, ownership)

            real = root / "real"
            self.write_package(real, "package")
            (root / "src/linked").symlink_to(
                real / "package", target_is_directory=True
            )
            ownership = self.write_ownership(
                root, {"entries": [self.team_entry("src/linked")]}
            )
            with self.assertRaisesRegex(self.module.ComplianceError, "符号链接"):
                self.module.verify_ownership(root, ownership)

    def test_third_party_file_paths_and_hashes_are_strict(self):
        digest = hashlib.sha256(b"official").hexdigest()
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            package = self.write_package(root)
            (package / "official.txt").write_bytes(b"official")
            base = {
                **self.team_entry(),
                "kind": "third-party",
                "source": "https://example.invalid/upstream",
                "files": {"official.txt": digest},
            }
            ownership = self.write_ownership(root, {"entries": [base]})
            self.module.verify_ownership(root, ownership)

            invalid_files = (
                {"../outside": digest},
                {"/absolute": digest},
                {"nested/../official.txt": digest},
                {"official.txt": "bad"},
                [],
            )
            for files in invalid_files:
                with self.subTest(files=files):
                    ownership = self.write_ownership(
                        root, {"entries": [{**base, "files": files}]}
                    )
                    with self.assertRaises(self.module.ComplianceError):
                        self.module.verify_ownership(root, ownership)

            (package / "alias.txt").symlink_to("official.txt")
            ownership = self.write_ownership(
                root,
                {"entries": [{**base, "files": {"alias.txt": digest}}]},
            )
            with self.assertRaisesRegex(self.module.ComplianceError, "符号链接"):
                self.module.verify_ownership(root, ownership)

    def test_package_set_and_declared_metadata_must_match(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            self.write_package(root)
            ownership = self.write_ownership(root, {"entries": []})
            with self.assertRaisesRegex(self.module.ComplianceError, "包集合"):
                self.module.verify_ownership(root, ownership)

            for field, value in (("version", "9.9.9"), ("license", "MIT")):
                with self.subTest(field=field):
                    entry = {**self.team_entry(), field: value}
                    ownership = self.write_ownership(root, {"entries": [entry]})
                    with self.assertRaisesRegex(self.module.ComplianceError, field):
                        self.module.verify_ownership(root, ownership)

    def test_compare_trees_rejects_file_set_content_and_symlinks(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            left = root / "left"
            right = root / "right"
            left.mkdir()
            right.mkdir()
            (left / "same").write_text("same", encoding="utf-8")
            (right / "same").write_text("same", encoding="utf-8")
            self.module.compare_trees(left, right)

            (right / "extra").write_text("extra", encoding="utf-8")
            with self.assertRaisesRegex(self.module.ComplianceError, "文件集合"):
                self.module.compare_trees(left, right)
            (right / "extra").unlink()

            (right / "same").write_text("changed", encoding="utf-8")
            with self.assertRaisesRegex(self.module.ComplianceError, "内容不同"):
                self.module.compare_trees(left, right)
            (right / "same").unlink()
            (right / "same").symlink_to(left / "same")
            with self.assertRaisesRegex(self.module.ComplianceError, "符号链接"):
                self.module.compare_trees(left, right)

    @staticmethod
    def write_entrypoints(root, script_text, launch_text="<launch/>"):
        (root / "robocup_zzufly.launch").write_text(
            launch_text, encoding="utf-8"
        )
        (root / "1.sh").write_text(script_text, encoding="utf-8")

    def test_repository_entrypoints_pass_the_static_official_input_guard(self):
        self.module.verify_entrypoints(ROOT)

    def test_shell_continuations_keep_the_starting_line_for_diagnostics(self):
        self.assertEqual(
            [(1, 'touch "$PX4_DIR/file"'), (3, "echo done")],
            list(
                self.module.logical_shell_commands(
                    'touch \\\n  "$PX4_DIR/file"\necho done\n'
                )
            ),
        )

    def test_official_reference_and_write_intent_helpers_are_fail_closed(self):
        probes = (
            'touch "$PX4_DIR/file"',
            'rm -f "$XTDRONE_DIR/file"',
            "install input PX4_Firmware/file",
            "ln --symbolic source /opt/XTDrone/file",
        )
        for command in probes:
            with self.subTest(command=command):
                self.assertTrue(self.module.has_official_reference(command))
                self.assertTrue(self.module.command_has_write_intent(command))

    def test_entrypoints_reject_all_known_official_directory_writes(self):
        unsafe_commands = (
            'touch "$PX4_DIR/official"\n',
            'rm -f "$XTDRONE_DIR/official"\n',
            'tee "$PX4_DIR/official" </dev/null\n',
            'install input "$XTDRONE_DIR/official"\n',
            'cp -t "$PX4_DIR" input\n',
            'sed --in-place s/old/new/ "$PX4_DIR/file"\n',
            'ln -sf source "$PX4_DIR/target"\n',
            'mv --target-directory="${XTDRONE_DIR}" input\n',
            'cp --target-directory="$PX4_DIR" input\n',
            'cp -at "$PX4_DIR" input\n',
            'sed -Ei.bak s/old/new/ "$PX4_DIR/file"\n',
            'sed --in-place=.bak s/old/new/ "$PX4_DIR/file"\n',
            'ln --symbolic --force source "$XTDRONE_DIR/target"\n',
            'rm --force --recursive "$PX4_DIR/generated"\n',
            'tee --append "$XTDRONE_DIR/official" </dev/null\n',
            'install --mode=644 input "$PX4_DIR/official"\n',
            'chmod --recursive 755 "$PX4_DIR/official"\n',
            'chown -R user "$XTDRONE_DIR/official"\n',
            'truncate --size 0 "$PX4_DIR/official"\n',
            'dd if=/dev/null of="$XTDRONE_DIR/official"\n',
            'rsync -a input "$PX4_DIR/official"\n',
            'python3 -c \'open(__import__("sys").argv[1], "w").close()\' "$PX4_DIR/file"\n',
            'perl -e \'open my $f, ">", $ARGV[0]\' "$XTDRONE_DIR/file"\n',
            'printf changed > "$XTDRONE_DIR/official"\n',
            'touch \\\n  "$PX4_DIR/continued"\n',
        )
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            for command in unsafe_commands:
                with self.subTest(command=command):
                    self.write_entrypoints(root, command)
                    with self.assertRaises(self.module.ComplianceError):
                        self.module.verify_entrypoints(root)

    def test_entrypoints_reject_literal_paths_aliases_and_unknown_reads(self):
        unsafe_commands = (
            "touch PX4_Firmware/official\n",
            "rm -f /opt/XTDrone/official\n",
            'OFFICIAL_ROOT="$PX4_DIR"\ntouch "$OFFICIAL_ROOT/official"\n',
            'UPSTREAM="${XTDRONE_DIR}/models"\ncat "$UPSTREAM/model.sdf"\n',
            'cp "$PX4_DIR/input" "$WORKSPACE_DIR/output"\n',
            'cat "$XTDRONE_DIR/arbitrary"\n',
        )
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            for command in unsafe_commands:
                with self.subTest(command=command):
                    self.write_entrypoints(root, command)
                    with self.assertRaises(self.module.ComplianceError) as context:
                        self.module.verify_entrypoints(root)
                    message = str(context.exception)
                    self.assertIn("1.sh:", message)
                    self.assertIn(command.splitlines()[0].split()[0], message)

    def test_entrypoints_reject_invalid_bash_syntax(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            self.write_entrypoints(root, "if then\n")
            with self.assertRaisesRegex(self.module.ComplianceError, "Bash 语法"):
                self.module.verify_entrypoints(root)

    def test_launch_rejects_official_directory_references(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            self.write_entrypoints(
                root,
                "#!/bin/bash\n",
                '<launch><node pkg="roslaunch" type="touch" '
                'args="$PX4_DIR/official"/></launch>',
            )
            with self.assertRaisesRegex(
                self.module.ComplianceError, "launch.*官方目录"
            ):
                self.module.verify_entrypoints(root)

    def test_entrypoints_reject_debug_model_and_symlink_commands(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            self.write_entrypoints(root, "#!/bin/bash\n", "typhoon_h480_zzufly")
            with self.assertRaisesRegex(self.module.ComplianceError, "调试模型"):
                self.module.verify_entrypoints(root)

            self.write_entrypoints(root, "ln -s source target\n")
            with self.assertRaisesRegex(self.module.ComplianceError, "符号链接"):
                self.module.verify_entrypoints(root)

    def test_evidence_path_must_stay_in_competition_artifacts(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            artifacts = root / "competition-artifacts"
            artifacts.mkdir()
            valid = artifacts / "result.json"
            self.assertEqual(
                valid.resolve(), self.module.validate_evidence_path(root, valid)
            )
            for invalid in (root / "result.json", artifacts / "../result.json"):
                with self.subTest(invalid=invalid):
                    with self.assertRaises(self.module.ComplianceError):
                        self.module.validate_evidence_path(root, invalid)

            outside = root / "outside"
            outside.mkdir()
            (artifacts / "linked").symlink_to(outside, target_is_directory=True)
            with self.assertRaisesRegex(self.module.ComplianceError, "符号链接"):
                self.module.validate_evidence_path(
                    root, artifacts / "linked/result.json"
                )

    def test_write_evidence_is_exclusive_and_wraps_os_errors(self):
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "evidence.json"
            payload = {"status": "pass"}
            self.module.write_evidence(path, payload)
            self.assertEqual(payload, json.loads(path.read_text(encoding="utf-8")))
            with self.assertRaisesRegex(self.module.ComplianceError, "已存在"):
                self.module.write_evidence(path, payload)
            with mock.patch.object(pathlib.Path, "open", side_effect=OSError("denied")):
                with self.assertRaisesRegex(self.module.ComplianceError, "无法写入"):
                    self.module.write_evidence(
                        pathlib.Path(directory) / "other.json", payload
                    )


if __name__ == "__main__":
    unittest.main()

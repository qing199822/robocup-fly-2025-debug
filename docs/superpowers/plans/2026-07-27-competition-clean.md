# Competition-Clean Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build and publish an isolated `competition-clean` branch that runs the six-drone mission with unmodified PX4, XTDrone, Gazebo, aircraft, and sensor parameters while allowing only an audited Realsense mount pose.

**Architecture:** A team-owned `competition_compliance` Catkin package verifies pinned external files, generates one temporary SDF by changing only the official Realsense include pose, validates the resulting XML tree, and publishes the matching static TF. The existing one-click launcher passes that temporary SDF to a team-owned spawn launch, waits for official PX4/MAVROS/XTDrone and camera readiness, and then starts only team-owned upper-layer algorithms.

**Tech Stack:** Ubuntu 20.04, ROS Noetic/Catkin, Python 3.8 `unittest`, `xml.etree.ElementTree`, PyYAML, Gazebo Classic 11, PX4 SITL 1.11, MAVROS, Bash, Git worktrees.

---

## Execution Preconditions

- Before Task 1, invoke `superpowers:using-git-worktrees` and create an isolated worktree for branch `competition-clean` from commit `0e5caf6` or its reviewed descendant.
- Never edit `/home/wangtao/robocup_fly/PX4_Firmware`, `/home/wangtao/robocup_fly/XTDrone`, system Gazebo files, or the user's `.vscode/settings.json`.
- Run every path below relative to the clean worktree root unless the command uses an absolute external dependency path.
- Use `/usr/bin/python3` for compliance tooling and the existing `.venv-yolo/bin/python` only for YOLO runtime nodes.
- Keep `main` as the community-debug branch; all removals in this plan occur only on `competition-clean`.
- Before running plan commands, export the external roots so worktree placement cannot redirect checks:

```bash
export PX4_DIR=/home/wangtao/robocup_fly/PX4_Firmware
export XTDRONE_DIR=/home/wangtao/robocup_fly/XTDrone
```

## File Structure

### New compliance package

- `src/competition_compliance/package.xml`: Catkin metadata and runtime dependencies.
- `src/competition_compliance/CMakeLists.txt`: install Python tools, config, launch files, and tests.
- `src/competition_compliance/setup.py`: expose the Python package to Catkin.
- `src/competition_compliance/src/competition_compliance/model.py`: mount parsing, SDF generation, and semantic XML comparison.
- `src/competition_compliance/src/competition_compliance/manifest.py`: SHA-256 verification of external official files.
- `src/competition_compliance/src/competition_compliance/tf_math.py`: compose mount orientation with the fixed XTDrone optical-frame convention.
- `src/competition_compliance/scripts/prepare_model.py`: mandatory fast preflight and temporary model generation CLI.
- `src/competition_compliance/scripts/sensor_tf_publisher.py`: publish `base_link` to official depth-camera frame from the same mount config.
- `src/competition_compliance/scripts/verify_full.py`: complete repository, dependency, ownership, build-input, and runtime-contract checks.
- `src/competition_compliance/config/sensor_mount.yaml`: the only user-editable sensor mount values.
- `src/competition_compliance/config/official_manifest.json`: pinned official input hashes and installed version expectations.
- `src/competition_compliance/config/ownership.json`: team/third-party ownership classification.
- `src/competition_compliance/launch/single_vehicle_spawn_clean.launch`: PX4 spawn behavior copied at the integration layer, reading an explicit temporary SDF path.
- `src/competition_compliance/launch/sensor_tf.launch`: launch the mount-derived static TF publisher.
- `src/competition_compliance/test/`: unit and launch-contract tests.

### Existing files changed

- `robocup_zzufly.launch`: pass one explicit generated SDF file to all six team-owned spawn includes.
- `1.sh`: remove model symlink and gimbal startup, run fast preflight, own the temporary directory, and wait for official camera topics.
- `src/yolo/bbox2coord_node.py`: consume `CameraInfo` instead of hard-coded intrinsics/frame.
- `src/yolo/camera_geometry.py`: pure tested pixel/depth deprojection.
- `src/mix_nav/simple_navigator/launch/static_tf.launch`: remove the old hard-coded custom-camera transforms.
- `src/look_up/launch/down_resume.launch`: include the compliant sensor TF launch.
- `tests/test_one_click_launch.py`: enforce the new startup and cleanup contract.
- `tests/test_competition_boundary.py`: forbid bundled core code and custom whole-aircraft models.
- `tests/test_camera_geometry.py`: verify official `CameraInfo` drives coordinate calculation.
- `README.md`, `docs/ENVIRONMENT.md`, `docs/COMPLIANCE.md`, `docs/THIRD_PARTY.md`, `docs/TROUBLESHOOTING.md`: user-facing clean-branch instructions and evidence.

### Files removed only from `competition-clean`

- `src/gazebo_ros_pkgs/`
- `typhoon_h480_zzufly/`
- `src/gimbal/`
- `tests/test_gimbal_launcher.py`

## Task 1: Establish the Clean Repository Boundary

**Files:**
- Create: `tests/test_competition_boundary.py`
- Modify: `.gitignore`
- Delete: `src/gazebo_ros_pkgs/`
- Delete: `typhoon_h480_zzufly/`
- Delete: `src/gimbal/`
- Delete: `tests/test_gimbal_launcher.py`

- [ ] **Step 1: Write the failing boundary test**

```python
#!/usr/bin/env python3

import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]


class CompetitionBoundaryTest(unittest.TestCase):
    def test_forbidden_bundled_components_are_absent(self):
        forbidden = (
            ROOT / "src" / "gazebo_ros_pkgs",
            ROOT / "typhoon_h480_zzufly",
            ROOT / "src" / "gimbal",
        )
        self.assertEqual([], [str(path.relative_to(ROOT)) for path in forbidden if path.exists()])


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run the boundary test and verify it fails on debug-only content**

Run: `python3 -m unittest tests.test_competition_boundary -v`

Expected: FAIL listing `src/gazebo_ros_pkgs`, `typhoon_h480_zzufly`, and `src/gimbal`.

- [ ] **Step 3: Remove only the forbidden clean-branch files**

```bash
git rm -r src/gazebo_ros_pkgs typhoon_h480_zzufly src/gimbal
git rm tests/test_gimbal_launcher.py
```

Do not remove the external `/home/wangtao/robocup_fly/PX4_Firmware` or `/home/wangtao/robocup_fly/XTDrone` directories.

- [ ] **Step 4: Ignore only generated compliance runtime files**

Add these exact entries to `.gitignore`:

```gitignore
# Competition-clean generated evidence and temporary launch artifacts
/logs/competition-clean/
/competition-artifacts/
```

- [ ] **Step 5: Verify the complete Task 1 boundary test passes and commit the clean removal**

Run: `python3 -m unittest tests.test_competition_boundary -v`

Expected: PASS.

```bash
git add .gitignore tests/test_competition_boundary.py
git commit -m "chore: remove forbidden bundled simulator code"
```

## Task 2: Add the Compliance Package and SDF Whitelist Generator

**Files:**
- Create: `src/competition_compliance/package.xml`
- Create: `src/competition_compliance/CMakeLists.txt`
- Create: `src/competition_compliance/setup.py`
- Create: `src/competition_compliance/src/competition_compliance/__init__.py`
- Create: `src/competition_compliance/src/competition_compliance/model.py`
- Create: `src/competition_compliance/config/sensor_mount.yaml`
- Create: `src/competition_compliance/test/test_model.py`

- [ ] **Step 1: Write failing mount and XML-difference tests**

```python
#!/usr/bin/env python3

import copy
import math
import os
import pathlib
import tempfile
import unittest
import xml.etree.ElementTree as ET

from competition_compliance.model import (
    ComplianceError,
    MountPose,
    assert_only_mount_pose_changed,
    generate_model,
    load_mount_pose,
)


WORKSPACE = pathlib.Path(__file__).resolve().parents[3]
XTDRONE_DIR = pathlib.Path(os.environ.get("XTDRONE_DIR", str(WORKSPACE.parent / "XTDrone")))
OFFICIAL = XTDRONE_DIR / "sitl_config/models/typhoon_h480_realsense/typhoon_h480_realsense.sdf"


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
            MountPose.from_values([0, 0, 0, 0, 0, math.nan])
        with self.assertRaises(ComplianceError):
            MountPose.from_values([0, 0, 0, 0, 0, 0, 0])
        with self.assertRaises(ComplianceError):
            MountPose.from_values([0, 0, 0, 0, 0, "not-a-number"])

    def test_generation_changes_only_realsense_include_pose(self):
        pose = MountPose.from_values([0.1, 0.0, -0.05, 0.0, 0.2, 0.0])
        with tempfile.TemporaryDirectory() as directory:
            output = pathlib.Path(directory) / "model.sdf"
            generate_model(OFFICIAL, output, pose)
            assert_only_mount_pose_changed(OFFICIAL, output)
            self.assertIn("0.1 0 -0.05 0 0.2 0", output.read_text(encoding="utf-8"))

    def test_non_pose_change_is_rejected(self):
        pose = MountPose.from_values([0.09, 0, -0.04, 0, 0, 0])
        with tempfile.TemporaryDirectory() as directory:
            output = pathlib.Path(directory) / "model.sdf"
            generate_model(OFFICIAL, output, pose)
            output.write_text(
                output.read_text(encoding="utf-8").replace("<mass>2.02</mass>", "<mass>1</mass>"),
                encoding="utf-8",
            )
            with self.assertRaises(ComplianceError):
                assert_only_mount_pose_changed(OFFICIAL, output)

    def test_missing_realsense_include_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            def remove_include(root):
                root.find("./model").remove(realsense_includes(root)[0])

            variant = write_variant(directory, remove_include)
            with self.assertRaises(ComplianceError):
                generate_model(variant, pathlib.Path(directory) / "out.sdf", MountPose.from_values([0] * 6))

    def test_duplicate_realsense_include_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            def duplicate_include(root):
                root.find("./model").append(copy.deepcopy(realsense_includes(root)[0]))

            variant = write_variant(directory, duplicate_include)
            with self.assertRaises(ComplianceError):
                generate_model(variant, pathlib.Path(directory) / "out.sdf", MountPose.from_values([0] * 6))

    def test_fixed_joint_parent_change_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            def change_parent(root):
                for joint in root.findall("./model/joint"):
                    if (joint.findtext("child") or "").strip() == "realsense_camera::link":
                        joint.find("parent").text = "cgo3_camera_link"

            variant = write_variant(directory, change_parent)
            with self.assertRaises(ComplianceError):
                generate_model(variant, pathlib.Path(directory) / "out.sdf", MountPose.from_values([0] * 6))

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

    def test_existing_output_file_is_refused(self):
        with tempfile.TemporaryDirectory() as directory:
            output = pathlib.Path(directory) / "model.sdf"
            output.write_text("keep-me", encoding="utf-8")
            with self.assertRaises(ComplianceError):
                generate_model(OFFICIAL, output, MountPose.from_values([0] * 6))
            self.assertEqual("keep-me", output.read_text(encoding="utf-8"))

    def test_malformed_xml_is_reported_as_compliance_error(self):
        with tempfile.TemporaryDirectory() as directory:
            broken = pathlib.Path(directory) / "broken.sdf"
            broken.write_text("<sdf><model>", encoding="utf-8")
            with self.assertRaisesRegex(ComplianceError, "XML"):
                generate_model(broken, pathlib.Path(directory) / "out.sdf", MountPose.from_values([0] * 6))


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run the model tests and verify the package does not exist yet**

Run:

```bash
source /opt/ros/noetic/setup.bash
PYTHONPATH=src/competition_compliance/src python3 src/competition_compliance/test/test_model.py -v
```

Expected: FAIL with `ModuleNotFoundError: No module named 'competition_compliance'`.

- [ ] **Step 3: Create Catkin package metadata**

`src/competition_compliance/package.xml`:

```xml
<?xml version="1.0"?>
<package format="2">
  <name>competition_compliance</name>
  <version>0.1.0</version>
  <description>Competition-safe model generation and environment verification.</description>
  <maintainer email="qing199822@users.noreply.github.com">ZZU FLY Team</maintainer>
  <license>LicenseRef-Team-Code</license>
  <buildtool_depend>catkin</buildtool_depend>
  <depend>geometry_msgs</depend>
  <depend>rospy</depend>
  <depend>tf</depend>
  <depend>tf2_ros</depend>
  <exec_depend>python3-yaml</exec_depend>
  <test_depend>python3-nose</test_depend>
  <test_depend>python3-numpy</test_depend>
</package>
```

`src/competition_compliance/setup.py`:

```python
from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup


setup(**generate_distutils_setup(
    packages=["competition_compliance"],
    package_dir={"": "src"},
))
```

`src/competition_compliance/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(competition_compliance)

find_package(catkin REQUIRED COMPONENTS geometry_msgs rospy tf tf2_ros)
catkin_python_setup()
catkin_package(CATKIN_DEPENDS geometry_msgs rospy tf tf2_ros)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_model.py)
endif()
```

Create an empty `src/competition_compliance/src/competition_compliance/__init__.py`.

- [ ] **Step 4: Implement the strict model generator**

`src/competition_compliance/src/competition_compliance/model.py`:

```python
import dataclasses
import math
import pathlib
import xml.etree.ElementTree as ET

import yaml


class ComplianceError(RuntimeError):
    pass


@dataclasses.dataclass(frozen=True)
class MountPose:
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float

    @classmethod
    def from_values(cls, values):
        if not isinstance(values, (list, tuple)) or len(values) != 6:
            raise ComplianceError("Realsense 安装位姿必须恰好包含 6 个数字")
        try:
            numbers = tuple(float(value) for value in values)
        except (TypeError, ValueError) as error:
            raise ComplianceError("Realsense 安装位姿包含非数字值") from error
        if not all(math.isfinite(value) for value in numbers):
            raise ComplianceError("Realsense 安装位姿不能包含 NaN 或无穷值")
        return cls(*numbers)

    def to_sdf(self):
        return " ".join(format(value, ".12g") for value in dataclasses.astuple(self))


def load_mount_pose(path):
    path = pathlib.Path(path)
    try:
        text = path.read_text(encoding="utf-8")
        data = yaml.safe_load(text)
    except OSError as error:
        raise ComplianceError("无法读取安装配置 {}：{}".format(path, error)) from error
    except yaml.YAMLError as error:
        raise ComplianceError("安装配置 YAML 格式错误 {}：{}".format(path, error)) from error
    if not isinstance(data, dict) or set(data) != {"realsense_mount"}:
        raise ComplianceError("安装配置只能包含 realsense_mount")
    return MountPose.from_values(data["realsense_mount"])


def _find_mount(root):
    matches = []
    for include in root.findall("./model/include"):
        uri = include.find("uri")
        if uri is not None and (uri.text or "").strip() == "model://realsense_camera":
            matches.append(include)
    if len(matches) != 1:
        raise ComplianceError("官方模型必须恰好包含一个 Realsense include")
    pose = matches[0].find("pose")
    if pose is None:
        raise ComplianceError("Realsense include 缺少 pose")

    joints = []
    for joint in root.findall("./model/joint"):
        child = joint.find("child")
        if child is not None and (child.text or "").strip() == "realsense_camera::link":
            joints.append(joint)
    if len(joints) != 1:
        raise ComplianceError("官方模型必须恰好包含一个 Realsense 固定关节")
    parent = joints[0].find("parent")
    if joints[0].get("type") != "fixed" or parent is None or (parent.text or "").strip() != "base_link":
        raise ComplianceError("Realsense 必须通过固定关节连接到 base_link")
    return pose


def _canonical(element, mount_pose):
    text = (element.text or "").strip()
    if element is mount_pose:
        text = "__ALLOWED_REALSENSE_MOUNT_POSE__"
    return (
        element.tag,
        tuple(sorted(element.attrib.items())),
        text,
        tuple(_canonical(child, mount_pose) for child in list(element)),
    )


def _parse_xml(path):
    path = pathlib.Path(path)
    try:
        return ET.parse(str(path))
    except OSError as error:
        raise ComplianceError("无法读取 SDF XML {}：{}".format(path, error)) from error
    except ET.ParseError as error:
        raise ComplianceError("SDF XML 格式错误 {}：{}".format(path, error)) from error


def generate_model(official_path, output_path, mount_pose):
    official_path = pathlib.Path(official_path)
    output_path = pathlib.Path(output_path)
    tree = _parse_xml(official_path)
    _find_mount(tree.getroot()).text = mount_pose.to_sdf()
    if output_path.exists():
        raise ComplianceError("拒绝覆盖已有生成模型：{}".format(output_path))
    try:
        output_path.parent.mkdir(parents=True, exist_ok=True)
        tree.write(str(output_path), encoding="utf-8", xml_declaration=True)
    except OSError as error:
        raise ComplianceError("无法写入临时模型 {}：{}".format(output_path, error)) from error
    assert_only_mount_pose_changed(official_path, output_path)


def assert_only_mount_pose_changed(official_path, generated_path):
    official_tree = _parse_xml(official_path)
    generated_tree = _parse_xml(generated_path)
    official_mount = _find_mount(official_tree.getroot())
    generated_mount = _find_mount(generated_tree.getroot())
    MountPose.from_values((generated_mount.text or "").split())
    if _canonical(official_tree.getroot(), official_mount) != _canonical(generated_tree.getroot(), generated_mount):
        raise ComplianceError("生成模型除 Realsense 安装 pose 外还存在其他差异")
```

- [ ] **Step 5: Add the sole mount configuration**

`src/competition_compliance/config/sensor_mount.yaml`:

```yaml
realsense_mount: [0.09, 0.0, -0.04, 0.0, 0.0, 0.0]
```

- [ ] **Step 6: Run the model tests and commit**

Run:

```bash
source /opt/ros/noetic/setup.bash
PYTHONPATH=src/competition_compliance/src python3 src/competition_compliance/test/test_model.py -v
```

Expected: 9 tests PASS.

```bash
git add src/competition_compliance
git commit -m "feat: generate model from official sensor baseline"
```

## Task 3: Pin and Verify Official Inputs During Fast Preflight

**Files:**
- Create: `src/competition_compliance/config/official_manifest.json`
- Create: `src/competition_compliance/src/competition_compliance/manifest.py`
- Create: `src/competition_compliance/scripts/prepare_model.py`
- Create: `src/competition_compliance/test/test_manifest.py`
- Modify: `src/competition_compliance/CMakeLists.txt`

- [ ] **Step 1: Write failing manifest tests**

```python
#!/usr/bin/env python3

import hashlib
import json
import pathlib
import subprocess
import tempfile
import unittest
from unittest import mock

from competition_compliance.manifest import collect_versions, verify_manifest
from competition_compliance.model import ComplianceError


class ManifestTest(unittest.TestCase):
    def test_matching_file_passes_and_changed_file_fails(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            target = root / "official.txt"
            target.write_text("official", encoding="utf-8")
            digest = hashlib.sha256(b"official").hexdigest()
            manifest = root / "manifest.json"
            manifest.write_text(json.dumps({
                "versions": {},
                "files": [{"root": "XTDRONE_DIR", "path": "official.txt", "sha256": digest}],
            }), encoding="utf-8")
            verify_manifest(manifest, {"XTDRONE_DIR": root})
            target.write_text("changed", encoding="utf-8")
            with self.assertRaises(ComplianceError):
                verify_manifest(manifest, {"XTDRONE_DIR": root})

    @mock.patch(
        "competition_compliance.manifest.subprocess.check_output",
        side_effect=subprocess.CalledProcessError(1, ["dpkg-query"], output="not installed"),
    )
    def test_failed_version_command_becomes_compliance_error(self, _check_output):
        with self.assertRaisesRegex(ComplianceError, "版本"):
            collect_versions(pathlib.Path("/tmp/not-used"))


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run the manifest test and verify it fails**

Run: `PYTHONPATH=src/competition_compliance/src python3 src/competition_compliance/test/test_manifest.py -v`

Expected: FAIL importing `competition_compliance.manifest`.

- [ ] **Step 3: Add the exact official manifest**

`src/competition_compliance/config/official_manifest.json`:

```json
{
  "versions": {
    "gazebo11": "11.15.1-1~focal",
    "ros-noetic-gazebo-ros": "2.9.3-1focal.20250521.003802",
    "ros-noetic-gazebo-ros-pkgs": "2.9.3-1focal.20250521.011748",
    "xtdrone_commit": "8e88116dc15a19e5eba06300897fcfec4ab2da11"
  },
  "files": [
    {"root": "PX4_DIR", "path": "Tools/sitl_gazebo/models/typhoon_h480/typhoon_h480.sdf", "sha256": "4f3ae25801c704e1f9e640eaf1717e6a06a688256ad8f6ad5a0872a2843c4680"},
    {"root": "PX4_DIR", "path": "Tools/sitl_gazebo/worlds/robocup.world", "sha256": "b17daad2b9662760aba6defbd1637214e6d4832e3828ec13ca342f544c6e0b98"},
    {"root": "PX4_DIR", "path": "launch/single_vehicle_spawn_xtd.launch", "sha256": "05bb251d1bebf28890cc03191a7fbbe0e121a5e2929a18b8968eb3d9ac071e7e"},
    {"root": "XTDRONE_DIR", "path": "sitl_config/models/typhoon_h480/typhoon_h480.sdf", "sha256": "1346f71a33130e3f5634b1513cc5598d1dc2693fdf30d13c2cf9dda2ef2cd29e"},
    {"root": "XTDRONE_DIR", "path": "sitl_config/models/typhoon_h480_realsense/typhoon_h480_realsense.sdf", "sha256": "3b056f3676e8f47b90421c5357eca8154e6686304855eb14467aa82bf60ddd46"},
    {"root": "XTDRONE_DIR", "path": "sitl_config/models/realsense_camera/realsense_camera.sdf", "sha256": "0745c705ac3a90cf16529a9b49729d34f49ce7b457998a4d3cc3f2fb6aab921c"},
    {"root": "XTDRONE_DIR", "path": "communication/multirotor_communication.py", "sha256": "64c13f6ad6de9181208cf584ac1b796d49d4f153935369b41e64a4b893a74d27"}
  ]
}
```

- [ ] **Step 4: Implement file verification**

`src/competition_compliance/src/competition_compliance/manifest.py`:

```python
import hashlib
import json
import pathlib
import subprocess

from competition_compliance.model import ComplianceError


def sha256_file(path):
    digest = hashlib.sha256()
    path = pathlib.Path(path)
    try:
        with path.open("rb") as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(block)
    except OSError as error:
        raise ComplianceError("无法读取校验文件 {}：{}".format(path, error)) from error
    return digest.hexdigest()


def load_manifest(path):
    path = pathlib.Path(path)
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except OSError as error:
        raise ComplianceError("无法读取官方清单 {}：{}".format(path, error)) from error
    except json.JSONDecodeError as error:
        raise ComplianceError("官方清单 JSON 格式错误 {}：{}".format(path, error)) from error
    if not isinstance(data, dict) or set(data) != {"files", "versions"}:
        raise ComplianceError("官方清单只能包含 files 和 versions")
    if not isinstance(data["files"], list) or not isinstance(data["versions"], dict):
        raise ComplianceError("官方清单格式无效")
    return data


def verify_manifest(path, roots):
    manifest = load_manifest(path)
    for entry in manifest["files"]:
        if not isinstance(entry, dict) or set(entry) != {"root", "path", "sha256"}:
            raise ComplianceError("官方文件条目必须只包含 root、path、sha256")
        root_name = entry["root"]
        if root_name not in roots:
            raise ComplianceError("缺少官方目录参数：{}".format(root_name))
        relative = pathlib.PurePosixPath(entry["path"])
        if relative.is_absolute() or ".." in relative.parts:
            raise ComplianceError("官方文件路径必须位于声明目录内：{}".format(entry["path"]))
        target = pathlib.Path(roots[root_name]) / pathlib.Path(*relative.parts)
        if not target.is_file():
            raise ComplianceError("找不到官方文件：{}".format(target))
        actual = sha256_file(target)
        if actual != entry["sha256"]:
            raise ComplianceError(
                "官方文件校验失败：{}\n期望：{}\n实际：{}".format(target, entry["sha256"], actual)
            )
    return manifest


def collect_versions(xtdrone_dir):
    def run_version_command(command, label):
        try:
            return subprocess.check_output(
                command,
                stderr=subprocess.STDOUT,
                text=True,
            ).strip()
        except FileNotFoundError as error:
            raise ComplianceError("无法执行版本检查 {}：找不到命令 {}".format(label, command[0])) from error
        except subprocess.CalledProcessError as error:
            detail = (error.output or "无输出").strip()
            raise ComplianceError("版本检查失败 {}：{}".format(label, detail)) from error

    def package_version(name):
        return run_version_command(["dpkg-query", "-W", "-f=${Version}", name], name)

    return {
        "gazebo11": package_version("gazebo11"),
        "ros-noetic-gazebo-ros": package_version("ros-noetic-gazebo-ros"),
        "ros-noetic-gazebo-ros-pkgs": package_version("ros-noetic-gazebo-ros-pkgs"),
        "xtdrone_commit": run_version_command(
            ["git", "-C", str(xtdrone_dir), "rev-parse", "HEAD"],
            "XTDrone Git 提交",
        ),
    }


def verify_versions(manifest, xtdrone_dir):
    actual = collect_versions(xtdrone_dir)
    if actual != manifest["versions"]:
        raise ComplianceError(
            "官方版本不匹配：期望 {}，实际 {}".format(manifest["versions"], actual)
        )
    return actual
```

- [ ] **Step 5: Implement the fast prepare CLI**

`src/competition_compliance/scripts/prepare_model.py`:

```python
#!/usr/bin/env python3

import argparse
import pathlib
import sys

from competition_compliance.manifest import verify_manifest, verify_versions
from competition_compliance.model import ComplianceError, generate_model, load_mount_pose


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--px4-dir", required=True, type=pathlib.Path)
    parser.add_argument("--xtdrone-dir", required=True, type=pathlib.Path)
    parser.add_argument("--manifest", required=True, type=pathlib.Path)
    parser.add_argument("--mount-config", required=True, type=pathlib.Path)
    parser.add_argument("--output", required=True, type=pathlib.Path)
    args = parser.parse_args()

    manifest = verify_manifest(
        args.manifest,
        {"PX4_DIR": args.px4_dir, "XTDRONE_DIR": args.xtdrone_dir},
    )
    verify_versions(manifest, args.xtdrone_dir)
    pose = load_mount_pose(args.mount_config)
    official = args.xtdrone_dir / "sitl_config/models/typhoon_h480_realsense/typhoon_h480_realsense.sdf"
    generate_model(official, args.output, pose)
    print(pose.to_sdf(), file=sys.stderr)
    print(args.output.resolve())


if __name__ == "__main__":
    try:
        main()
    except ComplianceError as error:
        print("合规自检失败：{}".format(error), file=sys.stderr)
        print("恢复方法：不要修改官方目录；按 docs/TROUBLESHOOTING.md 恢复对应版本后重试。", file=sys.stderr)
        raise SystemExit(2)
```

Make the script executable and add it to `catkin_install_python`.

Add the new script and test to `src/competition_compliance/CMakeLists.txt`:

```cmake
catkin_install_python(PROGRAMS
  scripts/prepare_model.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_manifest.py)
endif()
```

- [ ] **Step 6: Run unit and real-input fast-preflight tests**

Run:

```bash
PYTHONPATH=src/competition_compliance/src python3 src/competition_compliance/test/test_manifest.py -v
run_dir="$(mktemp -d /tmp/robocup-fly-plan-check.XXXXXX)"
PYTHONPATH=src/competition_compliance/src /usr/bin/time -f 'elapsed=%e' \
  python3 src/competition_compliance/scripts/prepare_model.py \
  --px4-dir "$PX4_DIR" \
  --xtdrone-dir "$XTDRONE_DIR" \
  --manifest src/competition_compliance/config/official_manifest.json \
  --mount-config src/competition_compliance/config/sensor_mount.yaml \
  --output "$run_dir/typhoon_h480_realsense.sdf"
```

Expected: unit test PASS; CLI exits 0, prints the generated absolute path, and reports less than `2.00` seconds on the current machine. Remove only the displayed `/tmp/robocup-fly-plan-check.*` directory after inspection.

- [ ] **Step 7: Commit the preflight verifier**

```bash
git add src/competition_compliance
git commit -m "feat: verify official simulator inputs before launch"
```

## Task 4: Spawn Six Vehicles From the Explicit Temporary Model

**Files:**
- Create: `src/competition_compliance/launch/single_vehicle_spawn_clean.launch`
- Create: `src/competition_compliance/test/test_launch_contract.py`
- Modify: `robocup_zzufly.launch`

- [ ] **Step 1: Write the failing launch-contract test**

```python
#!/usr/bin/env python3

import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[3]
MULTI_LAUNCH = ROOT / "robocup_zzufly.launch"
SINGLE_LAUNCH = ROOT / "src/competition_compliance/launch/single_vehicle_spawn_clean.launch"


class LaunchContractTest(unittest.TestCase):
    def test_all_six_groups_use_team_spawn_and_explicit_model_file(self):
        text = MULTI_LAUNCH.read_text(encoding="utf-8")
        include = "$(find competition_compliance)/launch/single_vehicle_spawn_clean.launch"
        self.assertEqual(6, text.count(include))
        self.assertEqual(6, text.count('<arg name="sdf_file" value="$(arg model_file)"/>'))
        self.assertIn('<arg name="model_file"/>', text)
        self.assertNotIn("typhoon_h480_zzufly", text)

    def test_single_spawn_reads_argument_and_never_writes_px4_models(self):
        text = SINGLE_LAUNCH.read_text(encoding="utf-8")
        self.assertIn('<arg name="sdf_file"/>', text)
        self.assertIn("$(arg sdf_file)", text)
        self.assertNotIn("Tools/sitl_gazebo/models/$(arg sdf)", text)
        self.assertNotIn("ln -s", text)
        self.assertIn("mavlink_tcp_port", text)
        self.assertIn("udp_gimbal_port_remote", text)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run the launch test and verify it fails**

Run: `PYTHONPATH=src/competition_compliance/src python3 src/competition_compliance/test/test_launch_contract.py -v`

Expected: FAIL because the team spawn launch is absent and the multi-launch still references the debug SDF.

- [ ] **Step 3: Create the team-owned single-vehicle spawn launch**

`src/competition_compliance/launch/single_vehicle_spawn_clean.launch`:

```xml
<?xml version="1.0"?>
<launch>
  <arg name="x" default="0"/>
  <arg name="y" default="0"/>
  <arg name="z" default="0"/>
  <arg name="R" default="0"/>
  <arg name="P" default="0"/>
  <arg name="Y" default="0"/>
  <arg name="sdf_file"/>
  <arg name="est" default="ekf2"/>
  <arg name="vehicle" default="typhoon_h480"/>
  <arg name="ID" default="0"/>
  <arg name="ID_in_group" default="0"/>
  <arg name="mavlink_udp_port" default="14560"/>
  <arg name="mavlink_tcp_port" default="4560"/>
  <arg name="udp_gimbal_port" default="13030"/>
  <arg name="interactive" default="true"/>

  <env name="PX4_SIM_MODEL" value="$(arg vehicle)"/>
  <env name="PX4_ESTIMATOR" value="$(arg est)"/>

  <!-- Same runtime port substitution as the XTDrone tutorial launch. -->
  <arg name="cmd" default="xmlstarlet ed -d '//plugin[@name=&quot;mavlink_interface&quot;]/mavlink_tcp_port' -s '//plugin[@name=&quot;mavlink_interface&quot;]' -t elem -n mavlink_tcp_port -v $(arg mavlink_tcp_port) -u '//plugin[@name=&quot;gimbal_controller&quot;]/udp_gimbal_port_remote' -v $(arg udp_gimbal_port) $(arg sdf_file)"/>
  <param command="$(arg cmd)" name="model_description"/>

  <arg unless="$(arg interactive)" name="px4_command_arg1" value=""/>
  <arg if="$(arg interactive)" name="px4_command_arg1" value="-d"/>
  <node name="sitl_$(arg ID)" pkg="px4" type="px4" output="screen"
        args="$(find px4)/ROMFS/px4fmu_common -s etc/init.d-posix/rcS -i $(arg ID) -w sitl_$(arg vehicle)_$(arg ID) $(arg px4_command_arg1)"/>
  <node name="$(arg vehicle)_$(arg ID)_spawn" pkg="gazebo_ros" type="spawn_model" output="screen"
        args="-sdf -param model_description -model $(arg vehicle)_$(arg ID_in_group) -x $(arg x) -y $(arg y) -z $(arg z) -R $(arg R) -P $(arg P) -Y $(arg Y)"/>
</launch>
```

- [ ] **Step 4: Convert all six groups in the existing team multi-launch**

Add this required argument near the top of `robocup_zzufly.launch`:

```xml
<arg name="model_file"/>
```

For each vehicle ID `0` through `5`, replace:

```xml
<include file="$(find px4)/launch/single_vehicle_spawn_xtd.launch">
...
<arg name="sdf" value="typhoon_h480_zzufly"/>
```

with:

```xml
<include file="$(find competition_compliance)/launch/single_vehicle_spawn_clean.launch">
...
<arg name="sdf_file" value="$(arg model_file)"/>
```

Keep every existing vehicle position, MAVLink UDP/TCP port, system ID, MAVROS URL, and namespace unchanged.

- [ ] **Step 5: Run launch-contract and XML syntax tests**

Run:

```bash
PYTHONPATH=src/competition_compliance/src python3 src/competition_compliance/test/test_launch_contract.py -v
xmllint --noout robocup_zzufly.launch src/competition_compliance/launch/single_vehicle_spawn_clean.launch
```

Expected: launch-contract tests PASS; `xmllint` exits 0.

Add the launch test to `src/competition_compliance/CMakeLists.txt` inside a testing block:

```cmake
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_launch_contract.py)
endif()
```

- [ ] **Step 6: Commit the team spawn integration**

```bash
git add robocup_zzufly.launch src/competition_compliance
git commit -m "feat: spawn clean vehicles from temporary model"
```

## Task 5: Replace Symlink Startup With Fast Preflight and Camera Readiness

**Files:**
- Modify: `1.sh`
- Modify: `tests/test_one_click_launch.py`

- [ ] **Step 1: Replace old one-click assertions with failing clean-start assertions**

Add these tests to `tests/test_one_click_launch.py` and remove assertions that require the deleted gimbal launcher or model link:

```python
    def test_fast_preflight_runs_before_roslaunch(self):
        script = (pathlib.Path(__file__).parents[1] / "1.sh").read_text(encoding="utf-8")
        prepare = '"$COMPLIANCE_PYTHON" "$PREPARE_MODEL"'
        launch = 'roslaunch "$SIMULATION_LAUNCH" model_file:="$GENERATED_MODEL"'
        self.assertIn(prepare, script)
        self.assertIn(launch, script)
        self.assertLess(script.index(prepare), script.index(launch))

    def test_startup_never_links_into_px4(self):
        script = (pathlib.Path(__file__).parents[1] / "1.sh").read_text(encoding="utf-8")
        self.assertNotIn("MODEL_LINK", script)
        self.assertNotIn("ln -s", script)
        self.assertNotIn("typhoon_h480_zzufly", script)

    def test_realsense_topics_are_ready_before_yolo(self):
        script = (pathlib.Path(__file__).parents[1] / "1.sh").read_text(encoding="utf-8")
        wait_call = "wait_for_cameras"
        yolo_call = 'start_helper "$WORKSPACE_DIR/src/yolo" "multi_yolo_detecting.sh"'
        self.assertIn(wait_call, script)
        self.assertIn(yolo_call, script)
        self.assertLess(script.index("if ! wait_for_cameras"), script.index(yolo_call))

    def test_clean_start_does_not_launch_realsense_gimbal_worker(self):
        script = (pathlib.Path(__file__).parents[1] / "1.sh").read_text(encoding="utf-8")
        self.assertNotIn("multi_gimbal_control.sh", script)
        self.assertNotIn('start_helper "$WORKSPACE_DIR/src/gimbal"', script)

    def test_cleanup_only_removes_validated_tmp_directory(self):
        script = (pathlib.Path(__file__).parents[1] / "1.sh").read_text(encoding="utf-8")
        self.assertIn('/tmp/robocup-fly-competition-clean.', script)
        self.assertIn('case "$RUN_TMP_DIR" in', script)
        self.assertIn('/tmp/robocup-fly-competition-clean.*)', script)

    def test_launch_writes_a_workspace_diagnostic_log(self):
        script = (pathlib.Path(__file__).parents[1] / "1.sh").read_text(encoding="utf-8")
        self.assertIn('LOG_DIR="$WORKSPACE_DIR/logs/competition-clean"', script)
        self.assertIn('exec > >(tee -a "$RUN_LOG") 2>&1', script)
```

- [ ] **Step 2: Run one-click tests and verify they fail**

Run: `python3 -m unittest tests.test_one_click_launch -v`

Expected: new clean-start tests FAIL on the old symlink and gimbal behavior.

- [ ] **Step 3: Add compliance variables and safe temporary cleanup**

At the top of `1.sh`, define:

```bash
COMPLIANCE_PACKAGE_DIR="$WORKSPACE_DIR/src/competition_compliance"
COMPLIANCE_PYTHON="${COMPLIANCE_PYTHON:-/usr/bin/python3}"
PREPARE_MODEL="$COMPLIANCE_PACKAGE_DIR/scripts/prepare_model.py"
OFFICIAL_MANIFEST="$COMPLIANCE_PACKAGE_DIR/config/official_manifest.json"
SENSOR_MOUNT_CONFIG="$COMPLIANCE_PACKAGE_DIR/config/sensor_mount.yaml"
RUN_TMP_DIR=""
GENERATED_MODEL=""
CAMERA_TIMEOUT_SECONDS="${CAMERA_TIMEOUT_SECONDS:-60}"
LOG_DIR="$WORKSPACE_DIR/logs/competition-clean"
RUN_LOG="$LOG_DIR/launch-$(date +%Y%m%d-%H%M%S).log"
```

Immediately after these definitions, enable one complete, persistent launch log:

```bash
mkdir -p "$LOG_DIR"
exec > >(tee -a "$RUN_LOG") 2>&1
echo "本次完整启动日志：$RUN_LOG"
```

Replace model-link cleanup with this exact guarded cleanup block:

```bash
if [ -n "$RUN_TMP_DIR" ]; then
    case "$RUN_TMP_DIR" in
        /tmp/robocup-fly-competition-clean.*)
            rm -rf -- "$RUN_TMP_DIR"
            ;;
        *)
            echo "拒绝清理非 competition-clean 临时目录：$RUN_TMP_DIR" >&2
            ;;
    esac
fi
```

- [ ] **Step 4: Replace custom-model requirements and symlink creation with fast preflight**

Require these files before launch:

```bash
require_file "$PREPARE_MODEL" "合规模型生成器"
require_file "$OFFICIAL_MANIFEST" "官方依赖校验清单"
require_file "$SENSOR_MOUNT_CONFIG" "Realsense 安装配置"
require_file "$XTDRONE_DIR/sitl_config/models/typhoon_h480_realsense/typhoon_h480_realsense.sdf" "XTDrone 官方 Realsense 机型"
require_file "$XTDRONE_DIR/sitl_config/models/realsense_camera/realsense_camera.sdf" "XTDrone 官方 Realsense 传感器"
```

Delete every `MODEL_LINK` branch and the old requirements for `typhoon_h480_zzufly` and `src/gimbal`. After sourcing the Catkin environment, prepare the model:

```bash
RUN_TMP_DIR="$(mktemp -d /tmp/robocup-fly-competition-clean.XXXXXX)"
GENERATED_MODEL="$RUN_TMP_DIR/typhoon_h480_realsense.sdf"
echo "执行快速合规自检并生成临时模型..."
"$COMPLIANCE_PYTHON" "$PREPARE_MODEL" \
    --px4-dir "$PX4_DIR" \
    --xtdrone-dir "$XTDRONE_DIR" \
    --manifest "$OFFICIAL_MANIFEST" \
    --mount-config "$SENSOR_MOUNT_CONFIG" \
    --output "$GENERATED_MODEL" >/dev/null
```

Start simulation with:

```bash
roslaunch "$SIMULATION_LAUNCH" model_file:="$GENERATED_MODEL" &
SIMULATION_PID=$!
```

- [ ] **Step 5: Add mandatory first-message camera readiness**

Add these functions before startup:

```bash
all_cameras_ready() {
    local id topic
    for id in $(seq 0 5); do
        for topic in \
            "/typhoon_h480_${id}/realsense/depth_camera/color/image_raw" \
            "/typhoon_h480_${id}/realsense/depth_camera/depth/image_raw" \
            "/typhoon_h480_${id}/realsense/depth_camera/color/camera_info"; do
            timeout 3s rostopic echo -n 1 "$topic" >/dev/null 2>&1 || return 1
        done
    done
}

wait_for_cameras() {
    local deadline
    deadline=$(( $(date +%s) + CAMERA_TIMEOUT_SECONDS ))
    echo "等待六组 Realsense 彩色图、深度图和 CameraInfo..."
    while [ "$(date +%s)" -lt "$deadline" ]; do
        if all_cameras_ready; then
            echo "六组 Realsense 话题均已就绪。"
            return 0
        fi
        sleep 1
    done
    echo "错误：Realsense 话题未在 ${CAMERA_TIMEOUT_SECONDS} 秒内全部就绪。" >&2
    return 1
}
```

Call `wait_for_cameras` after XTDrone communication is ready and before starting YOLO. Do not start `multi_gimbal_control.sh`.

- [ ] **Step 6: Run startup contract tests**

Run:

```bash
bash -n 1.sh
python3 -m unittest tests.test_one_click_launch tests.test_competition_boundary -v
```

Expected: Bash syntax check and all startup/boundary tests PASS.

- [ ] **Step 7: Commit the clean one-click startup**

```bash
git add 1.sh tests/test_one_click_launch.py
git commit -m "feat: add mandatory clean launch preflight"
```

## Task 6: Replace Hard-Coded Camera Intrinsics With CameraInfo

**Files:**
- Create: `src/yolo/camera_geometry.py`
- Create: `tests/test_camera_geometry.py`
- Modify: `src/yolo/bbox2coord_node.py`

- [ ] **Step 1: Write failing pure geometry and source-contract tests**

```python
#!/usr/bin/env python3

import pathlib
import sys
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/yolo"))

from camera_geometry import deproject_pixel, timestamps_within


class CameraGeometryTest(unittest.TestCase):
    def test_deprojection_uses_camera_info_matrix(self):
        k = [554.256, 0.0, 320.0, 0.0, 554.256, 240.0, 0.0, 0.0, 1.0]
        self.assertEqual((0.0, 0.0, 4.0), deproject_pixel(320, 240, 4.0, k))
        x, y, z = deproject_pixel(420, 290, 4.0, k)
        self.assertAlmostEqual(100.0 * 4.0 / 554.256, x)
        self.assertAlmostEqual(50.0 * 4.0 / 554.256, y)
        self.assertEqual(4.0, z)

    def test_invalid_focal_length_is_rejected(self):
        with self.assertRaises(ValueError):
            deproject_pixel(0, 0, 1.0, [0.0] * 9)

    def test_detection_and_depth_timestamps_must_be_close(self):
        self.assertTrue(timestamps_within(10.0, 10.08, 0.1))
        self.assertFalse(timestamps_within(10.0, 10.2, 0.1))

    def test_node_uses_official_frame_and_depth_time_without_old_intrinsics(self):
        source = (ROOT / "src/yolo/bbox2coord_node.py").read_text(encoding="utf-8")
        self.assertIn("CameraInfo", source)
        self.assertIn("color/camera_info", source)
        self.assertIn("self.latest_depth_stamp = image_msg.header.stamp", source)
        self.assertIn("detections_msg.header.stamp", source)
        self.assertIn("self.latest_camera_info.header.frame_id", source)
        self.assertNotIn('rospy.get_param("~fx", 205.47)', source)
        self.assertNotIn('rospy.get_param("~camera_frame", "camera_optical_link")', source)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run the geometry test and verify it fails**

Run: `python3 -m unittest tests.test_camera_geometry -v`

Expected: FAIL importing `camera_geometry`.

- [ ] **Step 3: Implement the pure deprojection helper**

`src/yolo/camera_geometry.py`:

```python
def deproject_pixel(u, v, depth_meters, camera_matrix):
    if len(camera_matrix) != 9:
        raise ValueError("CameraInfo.K must contain 9 values")
    fx = float(camera_matrix[0])
    fy = float(camera_matrix[4])
    cx = float(camera_matrix[2])
    cy = float(camera_matrix[5])
    if fx <= 0.0 or fy <= 0.0:
        raise ValueError("CameraInfo focal lengths must be positive")
    depth = float(depth_meters)
    return ((float(u) - cx) * depth / fx, (float(v) - cy) * depth / fy, depth)


def timestamps_within(first_seconds, second_seconds, maximum_delta_seconds):
    maximum = float(maximum_delta_seconds)
    if maximum < 0.0:
        raise ValueError("maximum timestamp delta must be non-negative")
    return abs(float(first_seconds) - float(second_seconds)) <= maximum
```

- [ ] **Step 4: Wire CameraInfo into the coordinate estimator**

In `src/yolo/bbox2coord_node.py`:

```python
from sensor_msgs.msg import CameraInfo, Image
from camera_geometry import deproject_pixel, timestamps_within
```

Initialize:

```python
self.latest_camera_info = None
self.latest_depth_stamp = None
self.camera_frame_override = rospy.get_param("~camera_frame", "")
self.maximum_sensor_delta = rospy.get_param("~maximum_sensor_delta", 0.15)
```

Subscribe beside the depth image:

```python
rospy.Subscriber(
    f"/{self.robot_name}/realsense/depth_camera/color/camera_info",
    CameraInfo,
    self._camera_info_callback,
    queue_size=1,
)
```

Store valid calibration:

```python
def _camera_info_callback(self, message):
    if message.header.frame_id and len(message.K) == 9 and message.K[0] > 0 and message.K[4] > 0:
        self.latest_camera_info = message
```

In `_depth_image_callback`, after a successful conversion, preserve the exact
official message timestamp:

```python
self.latest_depth_stamp = image_msg.header.stamp
```

Before processing a detection, require both depth and calibration:

```python
if self.latest_depth_frame is None or self.latest_camera_info is None:
    rospy.logwarn_throttle(2, "深度图或 CameraInfo 尚未接收，跳过处理。")
    return
```

Reject a stale color-detection/depth pairing and a calibration whose image size
does not match the depth array:

```python
depth_height, depth_width = self.latest_depth_frame.shape[:2]
if self.latest_camera_info.width != depth_width or self.latest_camera_info.height != depth_height:
    rospy.logwarn_throttle(2, "CameraInfo 尺寸与深度图不一致，跳过处理。")
    return
if (
    self.latest_depth_stamp is not None
    and not self.latest_depth_stamp.is_zero()
    and not detections_msg.header.stamp.is_zero()
    and not timestamps_within(
        self.latest_depth_stamp.to_sec(),
        detections_msg.header.stamp.to_sec(),
        self.maximum_sensor_delta,
    )
):
    rospy.logwarn_throttle(2, "彩色检测结果与深度图时间差过大，跳过处理。")
    return
```

Replace `_calculate_3d_point` with:

```python
def _calculate_3d_point(self, u, v, depth_in_meters):
    x, y, z = deproject_pixel(u, v, depth_in_meters, self.latest_camera_info.K)
    point = PointStamped()
    point.header.frame_id = self.camera_frame_override or self.latest_camera_info.header.frame_id
    point.header.stamp = self.latest_depth_stamp
    point.point.x = x
    point.point.y = y
    point.point.z = z
    return point
```

Delete `cam_fx`, `cam_fy`, `cam_cx`, `cam_cy`, and the old default `camera_optical_link` parameter.

- [ ] **Step 5: Run geometry and existing tracking tests**

Run:

```bash
python3 -m unittest tests.test_camera_geometry tests.test_tracking_mux_wiring -v
python3 -m py_compile src/yolo/camera_geometry.py src/yolo/bbox2coord_node.py
```

Expected: all tests PASS; both Python files compile.

- [ ] **Step 6: Commit the CameraInfo adaptation**

```bash
git add src/yolo/camera_geometry.py src/yolo/bbox2coord_node.py tests/test_camera_geometry.py
git commit -m "fix: derive target coordinates from camera info"
```

## Task 7: Publish TF From the Same Audited Mount Configuration

**Files:**
- Create: `src/competition_compliance/src/competition_compliance/tf_math.py`
- Create: `src/competition_compliance/scripts/sensor_tf_publisher.py`
- Create: `src/competition_compliance/launch/sensor_tf.launch`
- Create: `src/competition_compliance/test/test_tf_math.py`
- Modify: `src/mix_nav/simple_navigator/launch/static_tf.launch`
- Modify: `src/look_up/launch/down_resume.launch`
- Modify: `src/competition_compliance/test/test_launch_contract.py`

- [ ] **Step 1: Write the failing optical-axis composition test**

```python
#!/usr/bin/env python3

import unittest

import numpy
from tf.transformations import quaternion_matrix

from competition_compliance.model import MountPose
from competition_compliance.tf_math import compose_mount_to_optical


class TfMathTest(unittest.TestCase):
    def test_default_optical_z_points_along_body_x(self):
        translation, quaternion = compose_mount_to_optical(
            MountPose.from_values([0.09, 0, -0.04, 0, 0, 0])
        )
        self.assertEqual((0.09, 0.0, -0.04), translation)
        rotation = quaternion_matrix(quaternion)[:3, :3]
        body_direction = rotation.dot(numpy.array([0.0, 0.0, 1.0]))
        numpy.testing.assert_allclose([1.0, 0.0, 0.0], body_direction, atol=1e-6)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run the TF test and verify it fails**

Run:

```bash
source /opt/ros/noetic/setup.bash
PYTHONPATH=src/competition_compliance/src python3 src/competition_compliance/test/test_tf_math.py -v
```

Expected: FAIL importing `competition_compliance.tf_math`.

- [ ] **Step 3: Implement the fixed XTDrone optical convention composition**

`src/competition_compliance/src/competition_compliance/tf_math.py`:

```python
import math

from tf.transformations import quaternion_from_euler, quaternion_multiply


OFFICIAL_OPTICAL_RPY = (-math.pi / 2.0, 0.0, -math.pi / 2.0)


def compose_mount_to_optical(pose):
    mount = quaternion_from_euler(pose.roll, pose.pitch, pose.yaw)
    optical = quaternion_from_euler(*OFFICIAL_OPTICAL_RPY)
    quaternion = quaternion_multiply(mount, optical)
    return (pose.x, pose.y, pose.z), tuple(float(value) for value in quaternion)
```

- [ ] **Step 4: Implement the static TF publisher**

`src/competition_compliance/scripts/sensor_tf_publisher.py`:

```python
#!/usr/bin/env python3

import pathlib

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

from competition_compliance.model import ComplianceError, load_mount_pose
from competition_compliance.tf_math import compose_mount_to_optical


def main():
    rospy.init_node("competition_sensor_tf")
    config_path = pathlib.Path(rospy.get_param("~mount_config"))
    parent_frame = rospy.get_param("~parent_frame", "base_link")
    child_frame = rospy.get_param("~child_frame", "depth_camera_base")
    try:
        pose = load_mount_pose(config_path)
    except ComplianceError as error:
        rospy.logfatal("传感器安装配置无效：%s", error)
        raise SystemExit(2)

    translation, quaternion = compose_mount_to_optical(pose)
    message = TransformStamped()
    message.header.stamp = rospy.Time.now()
    message.header.frame_id = parent_frame
    message.child_frame_id = child_frame
    message.transform.translation.x = translation[0]
    message.transform.translation.y = translation[1]
    message.transform.translation.z = translation[2]
    message.transform.rotation.x = quaternion[0]
    message.transform.rotation.y = quaternion[1]
    message.transform.rotation.z = quaternion[2]
    message.transform.rotation.w = quaternion[3]
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    broadcaster.sendTransform(message)
    rospy.loginfo("已发布合规 Realsense TF: %s -> %s", parent_frame, child_frame)
    rospy.spin()


if __name__ == "__main__":
    main()
```

- [ ] **Step 5: Add TF launch and remove the old custom-camera transforms**

`src/competition_compliance/launch/sensor_tf.launch`:

```xml
<?xml version="1.0"?>
<launch>
  <arg name="mount_config" default="$(find competition_compliance)/config/sensor_mount.yaml"/>
  <node pkg="competition_compliance" type="sensor_tf_publisher.py" name="competition_sensor_tf" output="screen">
    <param name="mount_config" value="$(arg mount_config)"/>
    <param name="parent_frame" value="base_link"/>
    <param name="child_frame" value="depth_camera_base"/>
  </node>
</launch>
```

Reduce `src/mix_nav/simple_navigator/launch/static_tf.launch` to only the existing `map -> world` and `world -> ground_plane` nodes. Remove `base_to_camera_broadcaster` and `base_to_camera`.

Add this once near the TF section of `src/look_up/launch/down_resume.launch`:

```xml
<include file="$(find competition_compliance)/launch/sensor_tf.launch"/>
```

Extend `test_launch_contract.py` to assert the old `-0.106 0.03 -0.586` transform is absent and `down_resume.launch` includes `sensor_tf.launch` exactly once.

Add the TF publisher and TF test to `src/competition_compliance/CMakeLists.txt`:

```cmake
catkin_install_python(PROGRAMS
  scripts/sensor_tf_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_tf_math.py)
endif()
```

- [ ] **Step 6: Run TF and launch tests**

Run:

```bash
source /opt/ros/noetic/setup.bash
PYTHONPATH=src/competition_compliance/src python3 src/competition_compliance/test/test_tf_math.py -v
PYTHONPATH=src/competition_compliance/src python3 src/competition_compliance/test/test_launch_contract.py -v
xmllint --noout src/competition_compliance/launch/sensor_tf.launch src/mix_nav/simple_navigator/launch/static_tf.launch src/look_up/launch/down_resume.launch
```

Expected: all tests PASS and all launch files parse.

- [ ] **Step 7: Commit the mount-derived TF**

```bash
git add src/competition_compliance src/mix_nav/simple_navigator/launch/static_tf.launch src/look_up/launch/down_resume.launch
git commit -m "fix: derive camera transform from audited mount"
```

## Task 8: Record Ownership and Verify Third-Party Inputs

**Files:**
- Create: `src/competition_compliance/config/ownership.json`
- Create: `src/competition_compliance/scripts/verify_full.py`
- Create: `src/competition_compliance/test/test_ownership.py`
- Modify: `src/competition_compliance/config/official_manifest.json`
- Modify: `src/competition_compliance/CMakeLists.txt`
- Modify: team-owned `package.xml` files containing placeholder maintainers or licenses

- [ ] **Step 1: Write failing ownership tests**

```python
#!/usr/bin/env python3

import json
import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[3]
OWNERSHIP = ROOT / "src/competition_compliance/config/ownership.json"


class OwnershipTest(unittest.TestCase):
    def test_every_catkin_package_has_one_owner_entry(self):
        data = json.loads(OWNERSHIP.read_text(encoding="utf-8"))
        classified = {entry["path"] for entry in data["entries"]}
        packages = {
            str(path.parent.relative_to(ROOT))
            for path in (ROOT / "src").rglob("package.xml")
        }
        self.assertEqual(packages, classified)

    def test_no_package_metadata_contains_placeholders(self):
        offenders = []
        tokens = ("<license>TODO", "todo.com", "todo.todo", "Your Name", "your_email", "your.email")
        for package_xml in (ROOT / "src").rglob("package.xml"):
            text = package_xml.read_text(encoding="utf-8")
            if any(token in text for token in tokens):
                offenders.append(str(package_xml.relative_to(ROOT)))
        self.assertEqual([], offenders)

    def test_third_party_entries_have_source_version_and_license(self):
        data = json.loads(OWNERSHIP.read_text(encoding="utf-8"))
        third_party = [entry for entry in data["entries"] if entry["kind"] == "third-party"]
        self.assertTrue(third_party)
        for entry in third_party:
            self.assertTrue(entry["source"])
            self.assertTrue(entry["version"])
            self.assertTrue(entry["license"])


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run ownership tests and verify they fail**

Run: `python3 src/competition_compliance/test/test_ownership.py -v`

Expected: FAIL because `ownership.json` is absent and several team packages contain placeholder metadata.

- [ ] **Step 3: Create the exact package ownership map**

`src/competition_compliance/config/ownership.json`:

```json
{
  "entries": [
    {"path": "src/competition_compliance", "kind": "team", "source": "this repository", "version": "0.1.0", "license": "LicenseRef-Team-Code"},
    {"path": "src/look_up", "kind": "team", "source": "this repository", "version": "0.1.0", "license": "LicenseRef-Team-Code"},
    {"path": "src/mix_nav/fly", "kind": "team", "source": "this repository", "version": "1.0.0", "license": "BSD"},
    {"path": "src/mix_nav/simple_navigator", "kind": "team", "source": "this repository", "version": "0.1.0", "license": "BSD"},
    {"path": "src/mix_nav/task_manager", "kind": "team", "source": "this repository", "version": "0.0.0", "license": "LicenseRef-Team-Code"},
    {"path": "src/pose_init", "kind": "team", "source": "this repository", "version": "0.1.0", "license": "Apache-2.0"},
    {"path": "src/tracking", "kind": "team", "source": "this repository", "version": "1.0.0", "license": "LicenseRef-Team-Code"},
    {"path": "src/transform_tree", "kind": "team", "source": "this repository", "version": "0.1.0", "license": "LicenseRef-Team-Code"},
    {"path": "src/yolo/actor_msgs", "kind": "team", "source": "this repository", "version": "0.0.0", "license": "LicenseRef-Team-Code"},
    {"path": "src/darknet_ros_msgs", "kind": "third-party", "source": "https://github.com/leggedrobotics/darknet_ros", "version": "1.1.4", "license": "BSD", "files": {"CHANGELOG.rst": "c8f43f5497a4eafb3ca7a7b54bfae89688b419bf2d812a99466c16ac11313162", "CMakeLists.txt": "d8e46f9796c1e90ffc00a9c4ec73c8756504a779f099cbcad5549697b44fa100", "action/CheckForObjects.action": "a8bff28a58a021bcd4ed5e287de25b5cf6e4658fbb6bdc541f921e9dc30d94a4", "msg/BoundingBox.msg": "de8460e46657313444a294f1b03db298207e5f17ab92eca2c2b59cd25c51aeea", "msg/BoundingBoxes.msg": "efabf37197aba12ff013899a75c89b7a3aeb36ed3bda4c0d6e791b68d42ae4a0", "msg/ObjectCount.msg": "39f220042beeea9ee2138e3439a5d7d4cecd36d80a2a8aa97320964e29c44ea5", "package.xml": "e7c9ac183ed43f7964e735dd52915bae0d7807b86f180f58aee8ba2750e89071"}},
    {"path": "src/gazebo_ros_actor_plugin/gazebo_ros_actor_cmd_plugin", "kind": "third-party", "source": "XTDrone/sitl_config/gazebo_plugin/gazebo_ros_actor_plugin", "version": "XTDrone 8e88116dc15a19e5eba06300897fcfec4ab2da11", "license": "Apache-2.0"},
    {"path": "src/gazebo_ros_actor_plugin/gazebo_ros_actor_cmd_plugin_msgs", "kind": "third-party", "source": "XTDrone/sitl_config/gazebo_plugin/gazebo_ros_actor_plugin", "version": "XTDrone 8e88116dc15a19e5eba06300897fcfec4ab2da11", "license": "Apache-2.0"}
  ]
}
```

- [ ] **Step 4: Replace team package metadata placeholders without asserting a new open-source license**

In every team-owned `package.xml` reported by the failing test, replace the placeholder maintainer with:

```xml
<maintainer email="qing199822@users.noreply.github.com">ZZU FLY Team</maintainer>
```

Replace only `<license>TODO</license>` with `<license>LicenseRef-Team-Code</license>`, and replace placeholder `<author>` values with `ZZU FLY Team` where present. Preserve existing BSD and Apache declarations in both team and third-party packages.

- [ ] **Step 5: Add external Actor collision source hashes to the official manifest**

Append these entries to `official_manifest.json`'s `files` array. First add a comma
after the existing `communication/multirotor_communication.py` entry, then insert
the following entries before the array's closing `]`; keep the final entry below
without a trailing comma:

```json
{"root": "XTDRONE_DIR", "path": "sitl_config/gazebo_plugin/actor_collisions/ActorCollisionsPlugin.cc", "sha256": "e15f07b4a9cc19db1a05dd1aafd1b81557b2badf728cc28d666500034b34e499"},
{"root": "XTDRONE_DIR", "path": "sitl_config/gazebo_plugin/actor_collisions/ActorCollisionsPlugin.hh", "sha256": "78db47b17157eeb97676fc0ceecc95662dd1a8018c3730c492962ca431b61c29"},
{"root": "XTDRONE_DIR", "path": "sitl_config/gazebo_plugin/gazebo_ros_actor_plugin/gazebo_ros_actor_cmd_plugin/CMakeLists.txt", "sha256": "605eb23f6283b21fb67aa2efc3ddf0ca46dd79e292d05e8869f0638513efd786"},
{"root": "XTDRONE_DIR", "path": "sitl_config/gazebo_plugin/gazebo_ros_actor_plugin/gazebo_ros_actor_cmd_plugin/LICENSE", "sha256": "c71d239df91726fc519c6eb72d318ec65820627232b2f796219e87dcf35d0ab4"},
{"root": "XTDRONE_DIR", "path": "sitl_config/gazebo_plugin/gazebo_ros_actor_plugin/gazebo_ros_actor_cmd_plugin/README.md", "sha256": "71365aa2b8c92ae0dcfbfb970132ead41e224a8489d1ffe4fb2706394278ddb7"},
{"root": "XTDRONE_DIR", "path": "sitl_config/gazebo_plugin/gazebo_ros_actor_plugin/gazebo_ros_actor_cmd_plugin/include/actor_plugin_ros/ActorPluginRos.hpp", "sha256": "c10b714a548e3e1544df9b224e91a8d8d58acae4e7d7f45f54e1446ca042c411"},
{"root": "XTDRONE_DIR", "path": "sitl_config/gazebo_plugin/gazebo_ros_actor_plugin/gazebo_ros_actor_cmd_plugin/package.xml", "sha256": "6e662ad661893ded902e6035196328e902d800c5301c431b7c3a321ab3eac595"},
{"root": "XTDRONE_DIR", "path": "sitl_config/gazebo_plugin/gazebo_ros_actor_plugin/gazebo_ros_actor_cmd_plugin/res/waving.dae", "sha256": "7330302c492898d37fac0cff1cbd26b4381a7254fa14ae9780d7d0b9603a4db7"},
{"root": "XTDRONE_DIR", "path": "sitl_config/gazebo_plugin/gazebo_ros_actor_plugin/gazebo_ros_actor_cmd_plugin/src/ActorPluginRos.cpp", "sha256": "ac4bbe7b18aa7a89a50a1daba1648bd3563649bff67ac9b5868018d18664712c"},
{"root": "XTDRONE_DIR", "path": "sitl_config/gazebo_plugin/gazebo_ros_actor_plugin/gazebo_ros_actor_cmd_plugin_msgs/CMakeLists.txt", "sha256": "17e2d8b2c045a92d31b022bb4cf747d90911b288d73d7ab3715ae3a97b1e1b51"},
{"root": "XTDRONE_DIR", "path": "sitl_config/gazebo_plugin/gazebo_ros_actor_plugin/gazebo_ros_actor_cmd_plugin_msgs/msg/ActorInfo.msg", "sha256": "6a96273e5b133de9b94efd82940fb2fdb357234837d7a6a3dc05fa4878ff4ba1"},
{"root": "XTDRONE_DIR", "path": "sitl_config/gazebo_plugin/gazebo_ros_actor_plugin/gazebo_ros_actor_cmd_plugin_msgs/msg/ActorMotion.msg", "sha256": "f6f0f451411ba92053711251142483bda50cb1f06c41be4dfd45ad9b49c150ac"},
{"root": "XTDRONE_DIR", "path": "sitl_config/gazebo_plugin/gazebo_ros_actor_plugin/gazebo_ros_actor_cmd_plugin_msgs/package.xml", "sha256": "66f19fc8fb4fa7ae5d5e6d49475798a79976b865f5798d18e3cd0d9bc1c6601a"},
{"root": "XTDRONE_DIR", "path": "sitl_config/gazebo_plugin/gazebo_ros_actor_plugin/gazebo_ros_actor_cmd_plugin_msgs/srv/ToggleActorWaving.srv", "sha256": "d73e9d1a650517a232fcc9f41500815544035edcfc815fef640dba5d75967abd"}
```

- [ ] **Step 6: Implement the complete verifier**

`src/competition_compliance/scripts/verify_full.py`:

```python
#!/usr/bin/env python3

import argparse
import json
import pathlib
import sys

from competition_compliance.manifest import sha256_file, verify_manifest, verify_versions
from competition_compliance.model import ComplianceError


FORBIDDEN = ("src/gazebo_ros_pkgs", "typhoon_h480_zzufly", "src/gimbal")


def compare_trees(left, right):
    left = pathlib.Path(left)
    right = pathlib.Path(right)
    left_files = {str(path.relative_to(left)) for path in left.rglob("*") if path.is_file()}
    right_files = {str(path.relative_to(right)) for path in right.rglob("*") if path.is_file()}
    if left_files != right_files:
        raise ComplianceError("XTDrone Actor 插件文件集合与仓库副本不同")
    for relative in sorted(left_files):
        if sha256_file(left / relative) != sha256_file(right / relative):
            raise ComplianceError("XTDrone Actor 插件内容不同：{}".format(relative))


def verify_ownership(root, ownership_path):
    ownership_path = pathlib.Path(ownership_path)
    try:
        data = json.loads(ownership_path.read_text(encoding="utf-8"))
    except OSError as error:
        raise ComplianceError("无法读取所有权清单 {}：{}".format(ownership_path, error)) from error
    except json.JSONDecodeError as error:
        raise ComplianceError("所有权清单 JSON 格式错误 {}：{}".format(ownership_path, error)) from error
    if not isinstance(data, dict) or set(data) != {"entries"} or not isinstance(data["entries"], list):
        raise ComplianceError("所有权清单必须只包含 entries 数组")
    entries = {entry["path"]: entry for entry in data["entries"]}
    packages = {
        str(path.parent.relative_to(root))
        for path in (root / "src").rglob("package.xml")
    }
    if packages != set(entries):
        raise ComplianceError("所有权清单与实际 Catkin 包集合不一致")
    for entry in entries.values():
        for key in ("kind", "source", "version", "license"):
            if not entry.get(key):
                raise ComplianceError("所有权条目缺少 {}：{}".format(key, entry["path"]))
        for relative, expected in entry.get("files", {}).items():
            target = root / entry["path"] / relative
            if not target.is_file() or sha256_file(target) != expected:
                raise ComplianceError("第三方文件校验失败：{}".format(target))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", required=True, type=pathlib.Path)
    parser.add_argument("--px4-dir", required=True, type=pathlib.Path)
    parser.add_argument("--xtdrone-dir", required=True, type=pathlib.Path)
    parser.add_argument("--manifest", required=True, type=pathlib.Path)
    parser.add_argument("--ownership", required=True, type=pathlib.Path)
    parser.add_argument("--evidence", required=True, type=pathlib.Path)
    args = parser.parse_args()
    root = args.root.resolve()

    present = [path for path in FORBIDDEN if (root / path).exists()]
    if present:
        raise ComplianceError("clean 分支包含禁止目录：{}".format(", ".join(present)))

    manifest = verify_manifest(
        args.manifest,
        {"PX4_DIR": args.px4_dir, "XTDRONE_DIR": args.xtdrone_dir},
    )
    actual_versions = verify_versions(manifest, args.xtdrone_dir)

    verify_ownership(root, args.ownership)
    compare_trees(
        root / "src/gazebo_ros_actor_plugin",
        args.xtdrone_dir / "sitl_config/gazebo_plugin/gazebo_ros_actor_plugin",
    )

    launch_text = (root / "robocup_zzufly.launch").read_text(encoding="utf-8")
    script_text = (root / "1.sh").read_text(encoding="utf-8")
    if "typhoon_h480_zzufly" in launch_text + script_text or "ln -s" in script_text:
        raise ComplianceError("启动入口仍引用调试模型或写入官方模型目录")

    try:
        args.evidence.parent.mkdir(parents=True, exist_ok=True)
        args.evidence.write_text(json.dumps({
            "status": "pass",
            "versions": actual_versions,
            "checked_files": len(manifest["files"]),
            "critical_official_files_match": True,
        }, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    except OSError as error:
        raise ComplianceError("无法写入合规证据 {}：{}".format(args.evidence, error)) from error
    print("完整静态合规检查通过")


if __name__ == "__main__":
    try:
        main()
    except ComplianceError as error:
        print("完整合规检查失败：{}".format(error), file=sys.stderr)
        print("恢复方法：按 docs/TROUBLESHOOTING.md 恢复对应官方文件或依赖后重试。", file=sys.stderr)
        raise SystemExit(2)
```

At this point, replace the cumulative CMake fragments from Tasks 2, 3, 4, and 7
with this complete final `src/competition_compliance/CMakeLists.txt`, adding the
ownership verifier without dropping an earlier script, launch directory, or test:

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(competition_compliance)

find_package(catkin REQUIRED COMPONENTS geometry_msgs rospy tf tf2_ros)
catkin_python_setup()
catkin_package(CATKIN_DEPENDS geometry_msgs rospy tf tf2_ros)

catkin_install_python(PROGRAMS
  scripts/prepare_model.py
  scripts/sensor_tf_publisher.py
  scripts/verify_full.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_model.py)
  catkin_add_nosetests(test/test_manifest.py)
  catkin_add_nosetests(test/test_launch_contract.py)
  catkin_add_nosetests(test/test_tf_math.py)
  catkin_add_nosetests(test/test_ownership.py)
endif()
```

- [ ] **Step 7: Run ownership and complete static verification**

Run:

```bash
python3 src/competition_compliance/test/test_ownership.py -v
PYTHONPATH=src/competition_compliance/src python3 src/competition_compliance/scripts/verify_full.py \
  --root . \
  --px4-dir "$PX4_DIR" \
  --xtdrone-dir "$XTDRONE_DIR" \
  --manifest src/competition_compliance/config/official_manifest.json \
  --ownership src/competition_compliance/config/ownership.json \
  --evidence competition-artifacts/static-compliance.json
```

Expected: ownership tests PASS; verifier prints `完整静态合规检查通过`; evidence JSON has `"status": "pass"`.

- [ ] **Step 8: Commit ownership and static evidence tooling**

```bash
git add src/competition_compliance src/look_up/package.xml src/mix_nav src/pose_init/package.xml src/tracking/package.xml src/transform_tree/package.xml src/yolo/actor_msgs/package.xml
git commit -m "docs: classify competition package ownership"
```

## Task 9: Build External XTDrone Actor Collisions Without Editing XTDrone

**Files:**
- Create: `scripts/build_xtdrone_actor_collisions.sh`
- Create: `tests/test_external_plugin_build.py`
- Modify: `docs/ENVIRONMENT.md`

- [ ] **Step 1: Write the failing out-of-tree build contract test**

```python
#!/usr/bin/env python3

import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]


class ExternalPluginBuildTest(unittest.TestCase):
    def test_actor_collision_build_is_out_of_tree(self):
        script = (ROOT / "scripts/build_xtdrone_actor_collisions.sh").read_text(encoding="utf-8")
        self.assertIn('SOURCE_DIR="$XTDRONE_DIR/sitl_config/gazebo_plugin/actor_collisions"', script)
        self.assertIn('BUILD_DIR="$WORKSPACE_DIR/build/actor_collisions"', script)
        self.assertIn('OUTPUT_DIR="$WORKSPACE_DIR/devel/lib"', script)
        self.assertNotIn('cd "$SOURCE_DIR"', script)
        self.assertIn('cmake -S "$SOURCE_DIR" -B "$BUILD_DIR"', script)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run the test and verify the helper is absent**

Run: `python3 -m unittest tests.test_external_plugin_build -v`

Expected: FAIL with `FileNotFoundError`.

- [ ] **Step 3: Implement the verified out-of-tree build helper**

`scripts/build_xtdrone_actor_collisions.sh`:

```bash
#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
PROJECT_ROOT="$(cd "$WORKSPACE_DIR/.." && pwd)"
XTDRONE_DIR="${XTDRONE_DIR:-$PROJECT_ROOT/XTDrone}"
SOURCE_DIR="$XTDRONE_DIR/sitl_config/gazebo_plugin/actor_collisions"
BUILD_DIR="$WORKSPACE_DIR/build/actor_collisions"
OUTPUT_DIR="$WORKSPACE_DIR/devel/lib"

check_hash() {
    local expected="$1"
    local file="$2"
    local actual
    actual="$(sha256sum "$file" | cut -d' ' -f1)"
    if [ "$actual" != "$expected" ]; then
        echo "错误：XTDrone Actor 源码校验失败：$file" >&2
        exit 2
    fi
}

check_hash e15f07b4a9cc19db1a05dd1aafd1b81557b2badf728cc28d666500034b34e499 "$SOURCE_DIR/ActorCollisionsPlugin.cc"
check_hash 78db47b17157eeb97676fc0ceecc95662dd1a8018c3730c492962ca431b61c29 "$SOURCE_DIR/ActorCollisionsPlugin.hh"
cmake -S "$SOURCE_DIR" -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release
cmake --build "$BUILD_DIR" --parallel
mkdir -p "$OUTPUT_DIR"
cp "$BUILD_DIR/libActorCollisionsPlugin.so" "$OUTPUT_DIR/libActorCollisionsPlugin.so"
```

- [ ] **Step 4: Run the test and build the plugin**

Run:

```bash
python3 -m unittest tests.test_external_plugin_build -v
source /opt/ros/noetic/setup.bash
bash scripts/build_xtdrone_actor_collisions.sh
test -s devel/lib/libActorCollisionsPlugin.so
```

Expected: test PASS; build succeeds; final `test` exits 0. Confirm `git -C "$XTDRONE_DIR" status --short` is empty afterward.

- [ ] **Step 5: Document the out-of-tree dependency build and commit**

Add to `docs/ENVIRONMENT.md`:

```bash
source /opt/ros/noetic/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release
bash scripts/build_xtdrone_actor_collisions.sh
```

State explicitly that source is read from XTDrone commit `8e88116`, build artifacts stay under the team workspace, and the helper refuses changed source hashes.

```bash
git add scripts/build_xtdrone_actor_collisions.sh tests/test_external_plugin_build.py docs/ENVIRONMENT.md
git commit -m "build: compile official actor collision plugin out of tree"
```

## Task 10: Add the Full Verification and Six-Vehicle Smoke Commands

**Files:**
- Create: `scripts/verify_competition_clean.sh`
- Create: `scripts/smoke_competition_clean.sh`
- Create: `tests/test_verification_scripts.py`
- Modify: `docs/TROUBLESHOOTING.md`

- [ ] **Step 1: Write failing verification-script contract tests**

```python
#!/usr/bin/env python3

import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]


class VerificationScriptsTest(unittest.TestCase):
    def test_full_verifier_runs_static_tests_build_and_catkin_results(self):
        text = (ROOT / "scripts/verify_competition_clean.sh").read_text(encoding="utf-8")
        self.assertIn("verify_full.py", text)
        self.assertIn("python3 -m unittest discover -s tests", text)
        self.assertIn("catkin_make -DCMAKE_BUILD_TYPE=Release", text)
        self.assertIn("catkin_make run_tests", text)
        self.assertIn("catkin_test_results", text)
        self.assertIn("build_xtdrone_actor_collisions.sh", text)

    def test_smoke_checks_every_vehicle_and_all_realsense_topics(self):
        text = (ROOT / "scripts/smoke_competition_clean.sh").read_text(encoding="utf-8")
        self.assertIn("for id in $(seq 0 5)", text)
        self.assertIn("mavros/state", text)
        self.assertIn("color/image_raw", text)
        self.assertIn("depth/image_raw", text)
        self.assertIn("color/camera_info", text)
        self.assertIn("depth_camera_base", text)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run tests and verify both scripts are absent**

Run: `python3 -m unittest tests.test_verification_scripts -v`

Expected: FAIL with `FileNotFoundError`.

- [ ] **Step 3: Implement the full verifier orchestration**

`scripts/verify_competition_clean.sh`:

```bash
#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
PROJECT_ROOT="$(cd "$WORKSPACE_DIR/.." && pwd)"
PX4_DIR="${PX4_DIR:-$PROJECT_ROOT/PX4_Firmware}"
XTDRONE_DIR="${XTDRONE_DIR:-$PROJECT_ROOT/XTDrone}"
PACKAGE_DIR="$WORKSPACE_DIR/src/competition_compliance"
export PX4_DIR XTDRONE_DIR

source /opt/ros/noetic/setup.bash
export PYTHONPATH="$PACKAGE_DIR/src${PYTHONPATH:+:$PYTHONPATH}"

python3 "$PACKAGE_DIR/scripts/verify_full.py" \
  --root "$WORKSPACE_DIR" \
  --px4-dir "$PX4_DIR" \
  --xtdrone-dir "$XTDRONE_DIR" \
  --manifest "$PACKAGE_DIR/config/official_manifest.json" \
  --ownership "$PACKAGE_DIR/config/ownership.json" \
  --evidence "$WORKSPACE_DIR/competition-artifacts/static-compliance.json"
python3 -m unittest discover -s "$WORKSPACE_DIR/tests" -p 'test_*.py'
catkin_make -DCMAKE_BUILD_TYPE=Release
bash "$SCRIPT_DIR/build_xtdrone_actor_collisions.sh"
catkin_make run_tests
catkin_test_results
python3 "$PACKAGE_DIR/scripts/verify_full.py" \
  --root "$WORKSPACE_DIR" \
  --px4-dir "$PX4_DIR" \
  --xtdrone-dir "$XTDRONE_DIR" \
  --manifest "$PACKAGE_DIR/config/official_manifest.json" \
  --ownership "$PACKAGE_DIR/config/ownership.json" \
  --evidence "$WORKSPACE_DIR/competition-artifacts/post-build-compliance.json"
```

- [ ] **Step 4: Implement the running-system smoke checker**

`scripts/smoke_competition_clean.sh`:

```bash
#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
LOG_DIR="$WORKSPACE_DIR/logs/competition-clean"
REPORT="$LOG_DIR/smoke-$(date +%Y%m%d-%H%M%S).log"
mkdir -p "$LOG_DIR"

source /opt/ros/noetic/setup.bash
source "$WORKSPACE_DIR/devel/setup.bash"

check_message() {
    local topic="$1"
    if ! timeout 5s rostopic echo -n 1 "$topic" >/dev/null 2>&1; then
        echo "FAIL topic $topic" | tee -a "$REPORT" >&2
        return 1
    fi
    echo "PASS topic $topic" | tee -a "$REPORT"
}

for id in $(seq 0 5); do
    check_message "/typhoon_h480_${id}/mavros/state"
    check_message "/typhoon_h480_${id}/mavros/local_position/pose"
    check_message "/typhoon_h480_${id}/realsense/depth_camera/color/image_raw"
    check_message "/typhoon_h480_${id}/realsense/depth_camera/depth/image_raw"
    check_message "/typhoon_h480_${id}/realsense/depth_camera/color/camera_info"
    rosnode list | grep -qx "/typhoon_h480_${id}_communication"
done

timeout 5s rosrun tf tf_echo base_link depth_camera_base 2>&1 \
    | grep -m1 "Translation" | tee -a "$REPORT"
echo "PASS competition-clean six-vehicle smoke" | tee -a "$REPORT"
```

- [ ] **Step 5: Run static script tests and full non-GUI verification**

Run:

```bash
bash -n scripts/verify_competition_clean.sh scripts/smoke_competition_clean.sh
python3 -m unittest tests.test_verification_scripts -v
bash scripts/verify_competition_clean.sh
```

Expected: syntax/tests PASS; full verifier exits 0; Catkin reports no test failures; both compliance evidence files report `pass`.

- [ ] **Step 6: Run the six-vehicle GUI smoke check**

Terminal A:

```bash
bash 1.sh 6 mission_down.json
```

After Terminal A reports that the six cameras and mission are ready, Terminal B:

```bash
bash scripts/smoke_competition_clean.sh
```

Expected: six PX4/MAVROS and XTDrone communication instances are present; 18 camera messages arrive; TF lookup succeeds; smoke report ends with `PASS competition-clean six-vehicle smoke`.

- [ ] **Step 7: Stop with Ctrl-C and verify bounded cleanup**

In Terminal A press `Ctrl-C`, then run:

```bash
pgrep -af 'px4|gzserver|gzclient|multirotor_communication.py|yolo11n.py|bbox2coord_node.py'
find /tmp -maxdepth 1 -type d -name 'robocup-fly-competition-clean.*' -print
git -C "$XTDRONE_DIR" status --short
```

Expected: no process started by this run remains; no competition-clean temporary directory remains; XTDrone status is empty.

- [ ] **Step 8: Document smoke diagnostics and commit**

Add the exact Terminal A/B checks, report location, and failure interpretation to `docs/TROUBLESHOOTING.md`.

```bash
git add scripts/verify_competition_clean.sh scripts/smoke_competition_clean.sh tests/test_verification_scripts.py docs/TROUBLESHOOTING.md
git commit -m "test: verify clean six-drone runtime"
```

## Task 11: Publish Clean-Branch Documentation and Release Evidence

**Files:**
- Create: `docs/COMPLIANCE.md`
- Create: `docs/THIRD_PARTY.md`
- Modify: `README.md`
- Modify: `docs/ENVIRONMENT.md`
- Modify: `docs/TROUBLESHOOTING.md`

- [ ] **Step 1: Write clean-branch README content**

Replace debug-branch claims in `README.md` with these explicit boundaries:

```markdown
## Branch purpose

`competition-clean` is a competition-candidate branch. PX4 1.11, XTDrone,
Gazebo 11, and their official models remain external read-only dependencies.
The repository contains team upper-layer ROS algorithms, allowed launch/world
configuration, and verbatim third-party message/Actor packages recorded in
`docs/THIRD_PARTY.md`.

The Realsense sensor uses XTDrone's original optical and range parameters.
Only `src/competition_compliance/config/sensor_mount.yaml` may change its
installation position and angle. Final legality remains subject to the current
competition rules and referee inspection.
```

Document the exact build, fast start, full verify, smoke verify, stop, and log commands from Tasks 9 and 10. Remove references to `typhoon_h480_zzufly`, bundled `gazebo_ros_pkgs`, and Realsense gimbal control from the clean README.

- [ ] **Step 2: Write the compliance evidence guide**

`docs/COMPLIANCE.md` must include:

```markdown
# Competition-Clean Compliance Boundary

- Aircraft baseline: XTDrone `typhoon_h480_realsense` at commit `8e88116`.
- Sensor parameters: byte-verified XTDrone `realsense_camera.sdf`.
- Allowed generated difference: the direct `<pose>` child of the single
  `model://realsense_camera` include.
- Fixed joint parent: `base_link`; changing it is rejected.
- Fast preflight: mandatory on every launch, target <= 2 seconds.
- Full verification: after environment changes, before competition, and before release.
- Official source directories are read-only inputs and are checked before and after build.
- Physical plausibility of a changed mount must still be reviewed by the team and referee.
```

Add every official hash from `official_manifest.json`, the generated evidence paths, and instructions for presenting the printed six-value mount pose to a referee.

- [ ] **Step 3: Write third-party provenance**

`docs/THIRD_PARTY.md` must list:

- `darknet_ros_msgs` 1.1.4, `leggedrobotics/darknet_ros`, BSD.
- `gazebo_ros_actor_plugin`, XTDrone commit `8e88116`, Apache-2.0, verified byte-for-byte against the external XTDrone tree.
- `ActorCollisionsPlugin`, XTDrone commit `8e88116`, Apache-2.0 source, built out of tree.
- YOLO weight origin and redistribution status from the existing repository documentation; if no license grant is present, mark it `NOASSERTION` rather than inventing one.
- Team packages preserve existing BSD/Apache declarations; packages that previously had only `TODO` use `LicenseRef-Team-Code`, which does not grant a new open-source license.

- [ ] **Step 4: Run documentation and forbidden-reference checks**

Run:

```bash
rg -n "typhoon_h480_zzufly|src/gazebo_ros_pkgs|multi_gimbal_control" README.md docs/ENVIRONMENT.md docs/COMPLIANCE.md docs/THIRD_PARTY.md
rg -n "TBD|TODO|FIXME|待定" README.md docs/ENVIRONMENT.md docs/COMPLIANCE.md docs/THIRD_PARTY.md
```

Expected: both commands return no matches. References explaining historical debug-only incompatibility may remain only in the committed design document.

- [ ] **Step 5: Commit final clean documentation**

```bash
git add README.md docs/COMPLIANCE.md docs/THIRD_PARTY.md docs/ENVIRONMENT.md docs/TROUBLESHOOTING.md
git commit -m "docs: publish competition clean instructions"
```

- [ ] **Step 6: Run final verification from a clean build state**

Remove only generated workspace outputs after confirming the current directory is the isolated clean worktree, then run:

```bash
rm -rf -- build devel
source /opt/ros/noetic/setup.bash
catkin_init_workspace src
bash scripts/verify_competition_clean.sh
git status --short
git -C "$PX4_DIR" status --short 2>/dev/null || true
git -C "$XTDRONE_DIR" status --short
```

Expected: full verification PASS; clean branch has no uncommitted tracked changes; XTDrone is clean. PX4 integrity is established by the manifest because its archive-restored Git metadata is not authoritative.

- [ ] **Step 7: Request code review and address only verified findings**

Invoke `superpowers:requesting-code-review` against the complete `competition-clean` diff. If review identifies a defect, use `superpowers:receiving-code-review`, add a failing regression test, implement the minimal correction, rerun the relevant task checks, and commit the fix.

- [ ] **Step 8: Perform final completion verification**

Invoke `superpowers:verification-before-completion` and rerun:

```bash
bash scripts/verify_competition_clean.sh
bash scripts/smoke_competition_clean.sh
git status --short --branch
git log --oneline --decorate -12
```

Expected: full and running-system checks PASS; only intended clean commits appear; worktree is clean.

- [ ] **Step 9: Push the public clean branch**

```bash
git push -u public competition-clean
git ls-remote --heads public competition-clean
```

Expected: `git ls-remote` prints one `refs/heads/competition-clean` line at the verified local HEAD. Do not force-push and do not move `public/main`.

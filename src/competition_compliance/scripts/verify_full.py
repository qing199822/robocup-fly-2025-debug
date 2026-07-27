#!/usr/bin/env python3

import argparse
import errno
import hashlib
import json
import os
import pathlib
import re
import secrets
import shlex
import subprocess
import sys
import xml.etree.ElementTree as ET
from collections.abc import Mapping

from competition_compliance.manifest import (
    load_manifest_with_digest,
    sha256_file,
    verify_manifest_document,
    verify_versions,
)
from competition_compliance.model import ComplianceError


FORBIDDEN = ("src/gazebo_ros_pkgs", "typhoon_h480_zzufly", "src/gimbal")
_BASE_ENTRY_KEYS = {"path", "kind", "source", "version", "license"}
_THIRD_PARTY_ENTRY_KEYS = _BASE_ENTRY_KEYS | {"package_version", "verification"}
_HASH_PATTERN = re.compile(r"^[0-9a-f]{64}$")
_PLACEHOLDER_TOKENS = (
    "<license>TODO",
    "todo.com",
    "todo.todo",
    "Your Name",
    "your-email",
    "your_email",
    "your.email",
    "your-repo",
)
_COMMENTED_IDENTITY = re.compile(
    r"<!--(?:(?!-->).)*<(?:author|maintainer)\b", re.DOTALL
)
_ZERO_DESCRIPTION = re.compile(r"<description(?:\s[^>]*)?>\s*0\s*</description>")
_PROTECTED_VARIABLE = re.compile(
    r"\$(?:PX4_DIR|XTDRONE_DIR|PX4_BUILD_DIR)(?![A-Za-z0-9_])"
    r"|\$\{(?:PX4_DIR|XTDRONE_DIR|PX4_BUILD_DIR)(?:[^}]*)\}"
)
_OFFICIAL_LITERAL_NAMES = {"PX4_Firmware", "XTDrone"}
_OFFICIAL_LITERAL_PATH = re.compile(
    r"(?:\b(?:PX4_Firmware|XTDrone)/|/(?:PX4_Firmware|XTDrone)(?:/|[\"'}\s]|$))"
)
_DIRECT_WRITE_COMMANDS = {
    "chmod",
    "chown",
    "cp",
    "dd",
    "install",
    "mv",
    "rm",
    "rsync",
    "tee",
    "touch",
    "truncate",
}
_ALLOWED_OFFICIAL_SHELL_COMMANDS = frozenset(
    {
        'PX4_DIR="${PX4_DIR:-$PROJECT_ROOT/PX4_Firmware}"',
        'XTDRONE_DIR="${XTDRONE_DIR:-$PROJECT_ROOT/XTDrone}"',
        'PX4_BUILD_DIR="$PX4_DIR/build/px4_sitl_default"',
        'if ! official_root_is_readonly_mount "$PX4_DIR"; then',
        'echo "错误：sandbox 标记无效，PX4_DIR 并非独立只读挂载：$PX4_DIR" >&2',
        'if ! official_root_is_readonly_mount "$XTDRONE_DIR"; then',
        'echo "错误：sandbox 标记无效，XTDRONE_DIR 并非独立只读挂载：$XTDRONE_DIR" >&2',
        'echo "错误：缺少 bubblewrap，无法保护 PX4/XTDrone 官方目录。请运行 sudo apt install bubblewrap 后重试。" >&2',
        'if [ ! -d "$PX4_DIR" ] || [ -L "$PX4_DIR" ]; then',
        'echo "错误：PX4_DIR 必须是存在的普通目录且最终组件不能是符号链接：$PX4_DIR" >&2',
        'if [ ! -d "$XTDRONE_DIR" ] || [ -L "$XTDRONE_DIR" ]; then',
        'echo "错误：XTDRONE_DIR 必须是存在的普通目录且最终组件不能是符号链接：$XTDRONE_DIR" >&2',
        'if ! resolved_px4="$(cd "$PX4_DIR" && pwd -P)"; then',
        'echo "错误：无法解析 PX4_DIR：$PX4_DIR" >&2',
        'if ! resolved_xtdrone="$(cd "$XTDRONE_DIR" && pwd -P)"; then',
        'echo "错误：无法解析 XTDRONE_DIR：$XTDRONE_DIR" >&2',
        'PX4_BUILD_DIR="$PX4_DIR/build/px4_sitl_default"',
        '/usr/bin/setsid "$bwrap_path" --die-with-parent --json-status-fd "$status_fd" --dev-bind / / --ro-bind "$PX4_DIR" "$PX4_DIR" --ro-bind "$XTDRONE_DIR" "$XTDRONE_DIR" "$SCRIPT_DIR/1.sh" "$@" &',
        'require_file "$PX4_DIR/Tools/setup_gazebo.bash" "PX4 Gazebo 环境脚本"',
        'require_file "$PX4_BUILD_DIR/bin/px4" "PX4 SITL 编译产物"',
        'require_file "$PX4_DIR/Tools/sitl_gazebo/worlds/robocup.world" "RoboCup Gazebo 世界"',
        'require_file "$XTDRONE_DIR/sitl_config/models/walker/walk_0.dae" "XTDrone 行人模型"',
        'require_file "$XTDRONE_DIR/communication/multirotor_communication.py" "XTDrone 多旋翼通信脚本"',
        'require_file "$XTDRONE_DIR/sitl_config/models/typhoon_h480_realsense/typhoon_h480_realsense.sdf" "XTDrone 官方 Realsense 机型"',
        'require_file "$XTDRONE_DIR/sitl_config/models/realsense_camera/realsense_camera.sdf" "XTDrone 官方 Realsense 传感器"',
        'source "$PX4_DIR/Tools/setup_gazebo.bash" "$PX4_DIR" "$PX4_BUILD_DIR"',
        'export ROS_PACKAGE_PATH="${ROS_PACKAGE_PATH:+${ROS_PACKAGE_PATH}:}$PX4_DIR:$PX4_DIR/Tools/sitl_gazebo"',
        'export GAZEBO_MODEL_PATH="$PX4_DIR/Tools/sitl_gazebo/models:$XTDRONE_DIR/sitl_config/models:$GAZEBO_MODELS_DIR${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"',
        'if ! "$COMPLIANCE_PYTHON" "$PREPARE_MODEL" --px4-dir "$PX4_DIR" --xtdrone-dir "$XTDRONE_DIR" --gazebo-models-dir "$GAZEBO_MODELS_DIR" --xtdrone-pythonpath "$XTDRONE_PYTHONPATH" --manifest "$OFFICIAL_MANIFEST" --mount-config "$SENSOR_MOUNT_CONFIG" --output "$GENERATED_MODEL" >/dev/null; then',
        'start_communication "$XTDRONE_PYTHON" "$XTDRONE_DIR/communication/multirotor_communication.py" || return 1',
    }
)
_CANONICAL_MANIFEST_RELATIVE = pathlib.PurePosixPath(
    "src/competition_compliance/config/official_manifest.json"
)
_CANONICAL_OWNERSHIP_RELATIVE = pathlib.PurePosixPath(
    "src/competition_compliance/config/ownership.json"
)
_RUNTIME_ROOT_NAMES = (
    "PX4_DIR",
    "XTDRONE_DIR",
    "GAZEBO_MODELS_DIR",
    "XTDRONE_PYTHONPATH",
)
_REQUIRED_THIRD_PARTY_PROVENANCE = {
    "src/darknet_ros_msgs": {
        "source": "https://github.com/leggedrobotics/darknet_ros",
        "version": "1.1.4",
        "package_version": "1.1.4",
        "license": "BSD",
        "strategy": "complete-hash-map",
    },
    "src/gazebo_ros_actor_plugin/gazebo_ros_actor_cmd_plugin": {
        "source": "XTDrone/sitl_config/gazebo_plugin/gazebo_ros_actor_plugin",
        "version": "XTDrone 8e88116dc15a19e5eba06300897fcfec4ab2da11",
        "package_version": "1.0.0",
        "license": "Apache-2.0",
        "strategy": "external-tree",
        "external_path": (
            "sitl_config/gazebo_plugin/gazebo_ros_actor_plugin/"
            "gazebo_ros_actor_cmd_plugin"
        ),
    },
    "src/gazebo_ros_actor_plugin/gazebo_ros_actor_cmd_plugin_msgs": {
        "source": "XTDrone/sitl_config/gazebo_plugin/gazebo_ros_actor_plugin",
        "version": "XTDrone 8e88116dc15a19e5eba06300897fcfec4ab2da11",
        "package_version": "1.0.0",
        "license": "Apache-2.0",
        "strategy": "external-tree",
        "external_path": (
            "sitl_config/gazebo_plugin/gazebo_ros_actor_plugin/"
            "gazebo_ros_actor_cmd_plugin_msgs"
        ),
    },
}
_REQUIRED_OFFICIAL_IDENTITIES = frozenset(
    {
        ("PX4_DIR", "Tools/sitl_gazebo/models/typhoon_h480/typhoon_h480.sdf"),
        ("PX4_DIR", "Tools/sitl_gazebo/worlds/robocup.world"),
        ("PX4_DIR", "launch/single_vehicle_spawn_xtd.launch"),
        ("PX4_DIR", "Tools/setup_gazebo.bash"),
        ("PX4_DIR", "build/px4_sitl_default/bin/px4"),
        ("XTDRONE_DIR", "sitl_config/models/typhoon_h480/typhoon_h480.sdf"),
        ("XTDRONE_DIR", "sitl_config/models/typhoon_h480_realsense/typhoon_h480_realsense.sdf"),
        ("XTDRONE_DIR", "sitl_config/models/realsense_camera/realsense_camera.sdf"),
        ("XTDRONE_DIR", "sitl_config/models/walker/walk_0.dae"),
        ("XTDRONE_DIR", "communication/multirotor_communication.py"),
        ("XTDRONE_DIR", "sitl_config/gazebo_plugin/actor_collisions/ActorCollisionsPlugin.cc"),
        ("XTDRONE_DIR", "sitl_config/gazebo_plugin/actor_collisions/ActorCollisionsPlugin.hh"),
        ("XTDRONE_DIR", "sitl_config/gazebo_plugin/gazebo_ros_actor_plugin/gazebo_ros_actor_cmd_plugin/CMakeLists.txt"),
        ("XTDRONE_DIR", "sitl_config/gazebo_plugin/gazebo_ros_actor_plugin/gazebo_ros_actor_cmd_plugin/LICENSE"),
        ("XTDRONE_DIR", "sitl_config/gazebo_plugin/gazebo_ros_actor_plugin/gazebo_ros_actor_cmd_plugin/README.md"),
        ("XTDRONE_DIR", "sitl_config/gazebo_plugin/gazebo_ros_actor_plugin/gazebo_ros_actor_cmd_plugin/include/actor_plugin_ros/ActorPluginRos.hpp"),
        ("XTDRONE_DIR", "sitl_config/gazebo_plugin/gazebo_ros_actor_plugin/gazebo_ros_actor_cmd_plugin/package.xml"),
        ("XTDRONE_DIR", "sitl_config/gazebo_plugin/gazebo_ros_actor_plugin/gazebo_ros_actor_cmd_plugin/res/waving.dae"),
        ("XTDRONE_DIR", "sitl_config/gazebo_plugin/gazebo_ros_actor_plugin/gazebo_ros_actor_cmd_plugin/src/ActorPluginRos.cpp"),
        ("XTDRONE_DIR", "sitl_config/gazebo_plugin/gazebo_ros_actor_plugin/gazebo_ros_actor_cmd_plugin_msgs/CMakeLists.txt"),
        ("XTDRONE_DIR", "sitl_config/gazebo_plugin/gazebo_ros_actor_plugin/gazebo_ros_actor_cmd_plugin_msgs/msg/ActorInfo.msg"),
        ("XTDRONE_DIR", "sitl_config/gazebo_plugin/gazebo_ros_actor_plugin/gazebo_ros_actor_cmd_plugin_msgs/msg/ActorMotion.msg"),
        ("XTDRONE_DIR", "sitl_config/gazebo_plugin/gazebo_ros_actor_plugin/gazebo_ros_actor_cmd_plugin_msgs/package.xml"),
        ("XTDRONE_DIR", "sitl_config/gazebo_plugin/gazebo_ros_actor_plugin/gazebo_ros_actor_cmd_plugin_msgs/srv/ToggleActorWaving.srv"),
        ("GAZEBO_MODELS_DIR", "cessna/model.sdf"),
        ("XTDRONE_PYTHONPATH", "pyquaternion/__init__.py"),
    }
)


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ComplianceError("所有权清单 JSON 包含重复键：{}".format(key))
        result[key] = value
    return result


def _resolve_root(root):
    declared = pathlib.Path(root)
    if declared.is_symlink():
        raise ComplianceError("仓库根目录不得为符号链接：{}".format(declared))
    try:
        resolved = declared.resolve(strict=True)
    except (OSError, TypeError, ValueError, RuntimeError) as error:
        raise ComplianceError("无法解析仓库根目录 {}：{}".format(declared, error)) from error
    if not resolved.is_dir():
        raise ComplianceError("仓库根目录不是目录：{}".format(declared))
    return resolved


def _relative_path(value, label):
    if not isinstance(value, str) or not value:
        raise ComplianceError("{}必须为非空相对路径".format(label))
    if "\\" in value:
        raise ComplianceError("{}必须使用规范 POSIX 路径：{}".format(label, value))
    relative = pathlib.PurePosixPath(value)
    if (
        relative.is_absolute()
        or not relative.parts
        or ".." in relative.parts
        or relative.as_posix() != value
        or value == "."
    ):
        raise ComplianceError("{}必须是规范且受控的相对路径：{}".format(label, value))
    return relative


def _reject_symlink_components(root, relative, label):
    component = root
    for part in relative.parts:
        component = component / part
        if component.is_symlink():
            raise ComplianceError("{}路径包含符号链接：{}".format(label, component))


def _repository_file(root, path, label):
    declared = pathlib.Path(path)
    if declared.is_absolute():
        try:
            relative_path = declared.relative_to(root)
        except ValueError as error:
            raise ComplianceError("{}必须位于仓库内：{}".format(label, declared)) from error
        relative = _relative_path(relative_path.as_posix(), label)
    else:
        relative = _relative_path(pathlib.PurePosixPath(str(declared)).as_posix(), label)
    _reject_symlink_components(root, relative, label)
    target = root.joinpath(*relative.parts)
    try:
        resolved = target.resolve(strict=True)
    except FileNotFoundError as error:
        raise ComplianceError("找不到{}：{}".format(label, target)) from error
    except (OSError, ValueError, RuntimeError) as error:
        raise ComplianceError("无法解析{} {}：{}".format(label, target, error)) from error
    try:
        resolved.relative_to(root)
    except ValueError as error:
        raise ComplianceError("{}超出仓库目录：{}".format(label, target)) from error
    if not resolved.is_file():
        raise ComplianceError("{}不是普通文件：{}".format(label, target))
    return resolved


def canonical_manifest_path(root, manifest_path):
    root = _resolve_root(root)
    expected = root.joinpath(*_CANONICAL_MANIFEST_RELATIVE.parts)
    declared = pathlib.Path(manifest_path)
    if declared.is_absolute():
        if declared != expected:
            raise ComplianceError("必须使用仓库内唯一的规范官方清单：{}".format(expected))
    else:
        raw = pathlib.PurePosixPath(str(declared))
        if raw.as_posix() != str(declared) or raw != _CANONICAL_MANIFEST_RELATIVE:
            raise ComplianceError("必须使用仓库内唯一的规范官方清单：{}".format(expected))
    _reject_symlink_components(root, _CANONICAL_MANIFEST_RELATIVE, "官方清单")
    try:
        resolved = expected.resolve(strict=True)
    except (OSError, ValueError, RuntimeError) as error:
        raise ComplianceError("无法解析规范官方清单 {}：{}".format(expected, error)) from error
    if resolved != expected or not resolved.is_file():
        raise ComplianceError("规范官方清单不得为符号链接或别名：{}".format(expected))
    return resolved


def canonical_ownership_path(root, ownership_path):
    root = _resolve_root(root)
    expected = root.joinpath(*_CANONICAL_OWNERSHIP_RELATIVE.parts)
    declared = pathlib.Path(ownership_path)
    if declared.is_absolute():
        if declared != expected:
            raise ComplianceError("必须使用仓库内唯一的规范所有权清单：{}".format(expected))
    else:
        raw = pathlib.PurePosixPath(str(declared))
        if raw.as_posix() != str(declared) or raw != _CANONICAL_OWNERSHIP_RELATIVE:
            raise ComplianceError("必须使用仓库内唯一的规范所有权清单：{}".format(expected))
    _reject_symlink_components(root, _CANONICAL_OWNERSHIP_RELATIVE, "所有权清单")
    try:
        resolved = expected.resolve(strict=True)
    except (OSError, ValueError, RuntimeError) as error:
        raise ComplianceError("无法解析规范所有权清单 {}：{}".format(expected, error)) from error
    if resolved != expected or not resolved.is_file():
        raise ComplianceError("规范所有权清单不得为符号链接或别名：{}".format(expected))
    return resolved


def canonical_runtime_roots(roots):
    if not isinstance(roots, Mapping) or set(roots) != set(_RUNTIME_ROOT_NAMES):
        raise ComplianceError("运行时根目录必须恰好包含四个规定名称")
    resolved_roots = {}
    for name in _RUNTIME_ROOT_NAMES:
        try:
            declared = pathlib.Path(roots[name])
        except (TypeError, ValueError) as error:
            raise ComplianceError("运行时根目录参数无效 {}：{}".format(name, error)) from error
        if declared.is_symlink():
            raise ComplianceError("运行时根目录不得为符号链接 {}：{}".format(name, declared))
        try:
            resolved = declared.resolve(strict=True)
        except (OSError, ValueError, RuntimeError) as error:
            raise ComplianceError("无法解析运行时根目录 {}：{}".format(name, error)) from error
        if declared != resolved:
            raise ComplianceError("运行时根目录必须使用规范绝对路径 {}：{}".format(name, declared))
        if not resolved.is_dir():
            raise ComplianceError("运行时根目录不是目录 {}：{}".format(name, declared))
        resolved_roots[name] = resolved
    return resolved_roots


def verify_required_manifest_identities(manifest):
    if not isinstance(manifest, dict) or not isinstance(manifest.get("files"), list):
        raise ComplianceError("官方清单 files 必须为数组")
    identities = [(entry.get("root"), entry.get("path")) for entry in manifest["files"]]
    identity_set = set(identities)
    if len(identities) != len(identity_set):
        raise ComplianceError("官方清单包含重复文件身份")
    missing = sorted(_REQUIRED_OFFICIAL_IDENTITIES - identity_set)
    extra = sorted(identity_set - _REQUIRED_OFFICIAL_IDENTITIES)
    if missing or extra:
        raise ComplianceError(
            "规范官方清单身份集合不完整：缺少 {}，多余 {}".format(missing, extra)
        )
    return sorted(
        (
            {"root": entry["root"], "path": entry["path"], "sha256": entry["sha256"]}
            for entry in manifest["files"]
        ),
        key=lambda item: (item["root"], item["path"]),
    )


def build_evidence(
    manifest_digest,
    manifest,
    actual_versions,
    ownership_digest,
    ownership_entries,
    runtime_roots,
):
    if not isinstance(manifest_digest, str) or not _HASH_PATTERN.fullmatch(
        manifest_digest
    ):
        raise ComplianceError("规范官方清单 sha256 无效")
    if not isinstance(ownership_digest, str) or not _HASH_PATTERN.fullmatch(
        ownership_digest
    ):
        raise ComplianceError("规范所有权清单 sha256 无效")
    checked = verify_required_manifest_identities(manifest)
    classifications = sorted(
        (
            {"path": entry["path"], "kind": entry["kind"]}
            for entry in ownership_entries
        ),
        key=lambda item: item["path"],
    )
    canonical_roots = canonical_runtime_roots(runtime_roots)
    return {
        "status": "pass",
        "versions": actual_versions,
        "manifest_sha256": manifest_digest,
        "ownership_sha256": ownership_digest,
        "package_classifications": classifications,
        "runtime_roots": {
            name: str(canonical_roots[name]) for name in _RUNTIME_ROOT_NAMES
        },
        "checked_identities": checked,
        "checked_files": len(checked),
        "critical_official_files_match": True,
        "integrity_basis": (
            "critical runtime hashes; PX4 archive Git metadata is not authoritative"
        ),
        "entrypoint_guard": (
            "bubblewrap read-only runtime mounts; static entrypoint guard is supplemental"
        ),
    }


def load_ownership_with_digest(path):
    path = pathlib.Path(path)
    try:
        encoded = path.read_bytes()
    except OSError as error:
        raise ComplianceError("无法读取所有权清单 {}：{}".format(path, error)) from error
    try:
        text = encoded.decode("utf-8")
    except UnicodeError as error:
        raise ComplianceError("所有权清单不是有效 UTF-8 {}：{}".format(path, error)) from error
    try:
        data = json.loads(text, object_pairs_hook=_unique_object)
    except json.JSONDecodeError as error:
        raise ComplianceError("所有权清单 JSON 格式错误 {}：{}".format(path, error)) from error
    if not isinstance(data, dict) or set(data) != {"entries"}:
        raise ComplianceError("所有权清单必须只包含 entries")
    if not isinstance(data["entries"], list):
        raise ComplianceError("所有权清单 entries 必须为数组")
    return data["entries"], hashlib.sha256(encoded).hexdigest()


def load_ownership(path):
    entries, _digest = load_ownership_with_digest(path)
    return entries


def _validate_entry(entry):
    if not isinstance(entry, dict):
        raise ComplianceError("所有权条目必须为对象")
    kind = entry.get("kind")
    expected_keys = _THIRD_PARTY_ENTRY_KEYS if kind == "third-party" else _BASE_ENTRY_KEYS
    if set(entry) != expected_keys:
        raise ComplianceError("所有权条目字段不符合 {} 类型要求".format(kind))
    if kind not in ("team", "third-party"):
        raise ComplianceError("所有权 kind 只能是 team 或 third-party")
    relative = _relative_path(entry["path"], "所有权包路径")
    for key in ("source", "version", "license"):
        value = entry[key]
        if not isinstance(value, str) or not value.strip():
            raise ComplianceError("所有权条目 {} 必须为非空字符串：{}".format(key, entry["path"]))
    if kind == "third-party":
        package_version = entry["package_version"]
        if not isinstance(package_version, str) or not package_version.strip():
            raise ComplianceError("第三方 package_version 必须为非空字符串")
        verification = entry["verification"]
        if not isinstance(verification, dict):
            raise ComplianceError("第三方 verification 必须为对象")
        strategy = verification.get("strategy")
        if strategy == "complete-hash-map":
            if set(verification) != {"strategy", "files"}:
                raise ComplianceError("complete-hash-map 只能包含 strategy 和 files")
            files = verification["files"]
            _validate_hash_map(entry["path"], files)
        elif strategy == "external-tree":
            if set(verification) != {"strategy", "external_path"}:
                raise ComplianceError("external-tree 只能包含 strategy 和 external_path")
            _relative_path(verification["external_path"], "第三方外部目录路径")
        else:
            raise ComplianceError("第三方 verification strategy 无效：{}".format(strategy))
    return relative


def _validate_hash_map(package_path, files):
    if not isinstance(files, dict) or not files:
        raise ComplianceError("第三方 files 必须为非空哈希对象：{}".format(package_path))
    for path, digest in files.items():
        _relative_path(path, "第三方文件路径")
        if not isinstance(digest, str) or not _HASH_PATTERN.fullmatch(digest):
            raise ComplianceError("第三方文件 sha256 无效：{}/{}".format(package_path, path))


def _walk_package_files(root):
    source = root / "src"
    if not source.is_dir() or source.is_symlink():
        raise ComplianceError("仓库缺少普通 src 目录")
    packages = set()

    def walk_error(error):
        raise ComplianceError("无法遍历 Catkin 包：{}".format(error))

    for directory, names, files in os.walk(str(source), followlinks=False, onerror=walk_error):
        current = pathlib.Path(directory)
        for name in names:
            child = current / name
            if child.is_symlink():
                raise ComplianceError("Catkin 源码目录包含符号链接：{}".format(child))
        for name in files:
            child = current / name
            if child.is_symlink():
                raise ComplianceError("Catkin 源码文件包含符号链接：{}".format(child))
            if name == "package.xml":
                packages.add(child.parent.relative_to(root).as_posix())
    return packages


def _package_metadata(package_xml):
    try:
        xml_root = ET.parse(str(package_xml)).getroot()
    except (OSError, UnicodeError, ET.ParseError) as error:
        raise ComplianceError("无法解析 Catkin 包元数据 {}：{}".format(package_xml, error)) from error
    versions = [node.text.strip() for node in xml_root.findall("version") if node.text and node.text.strip()]
    licenses = [node.text.strip() for node in xml_root.findall("license") if node.text and node.text.strip()]
    if len(versions) != 1 or not licenses:
        raise ComplianceError("Catkin 包必须声明一个版本和至少一个许可证：{}".format(package_xml))
    return versions[0], licenses


def _normalized_license(value):
    if value == "Apache License, Version 2.0":
        return "Apache-2.0"
    return value


def verify_ownership_entries(root, entries, xtdrone_dir=None):
    root = _resolve_root(root)
    validated = []
    paths = set()
    for entry in entries:
        relative = _validate_entry(entry)
        if entry["path"] in paths:
            raise ComplianceError("所有权清单包含重复包路径：{}".format(entry["path"]))
        paths.add(entry["path"])
        _reject_symlink_components(root, relative, "所有权包")
        package_dir = root.joinpath(*relative.parts)
        if not package_dir.is_dir():
            raise ComplianceError("所有权包目录不存在：{}".format(package_dir))
        validated.append((entry, relative, package_dir))

    packages = _walk_package_files(root)
    if packages != paths:
        missing = sorted(packages - paths)
        extra = sorted(paths - packages)
        raise ComplianceError("所有权清单与实际 Catkin 包集合不一致：缺少 {}，多余 {}".format(missing, extra))

    for entry, _relative, package_dir in validated:
        package_xml = package_dir / "package.xml"
        version, licenses = _package_metadata(package_xml)
        expected_version = (
            entry["version"] if entry["kind"] == "team" else entry["package_version"]
        )
        if version != expected_version:
            raise ComplianceError("所有权 package version 与 package.xml 不一致：{}".format(entry["path"]))
        normalized_licenses = {_normalized_license(value) for value in licenses}
        if _normalized_license(entry["license"]) not in normalized_licenses:
            raise ComplianceError("所有权 license 与 package.xml 不一致：{}".format(entry["path"]))
        try:
            metadata_text = package_xml.read_text(encoding="utf-8")
        except (OSError, UnicodeError) as error:
            raise ComplianceError("无法读取 Catkin 包元数据 {}：{}".format(package_xml, error)) from error
        if (
            any(token in metadata_text for token in _PLACEHOLDER_TOKENS)
            or _COMMENTED_IDENTITY.search(metadata_text)
            or _ZERO_DESCRIPTION.search(metadata_text)
        ):
            raise ComplianceError("Catkin 包元数据仍含占位内容：{}".format(package_xml))
        if entry["kind"] == "third-party":
            verification = entry["verification"]
            if verification["strategy"] == "complete-hash-map":
                package_root, actual_files = _tree_files(package_dir, "第三方包")
                expected_files = set(verification["files"])
                if actual_files != expected_files:
                    raise ComplianceError(
                        "第三方包文件集合不一致：{}".format(entry["path"])
                    )
                for file_path, expected in verification["files"].items():
                    if sha256_file(package_root / file_path) != expected:
                        raise ComplianceError(
                            "第三方文件校验失败：{}".format(package_root / file_path)
                        )
            else:
                if xtdrone_dir is None:
                    raise ComplianceError("external-tree 策略缺少 XTDrone 目录")
                external_root = _resolve_root(xtdrone_dir)
                external_relative = _relative_path(
                    verification["external_path"], "第三方外部目录路径"
                )
                _reject_symlink_components(
                    external_root, external_relative, "第三方外部目录"
                )
                compare_trees(
                    package_dir,
                    external_root.joinpath(*external_relative.parts),
                )
    return entries


def verify_competition_ownership_boundary(entries):
    if not isinstance(entries, list):
        raise ComplianceError("比赛所有权清单 entries 必须为数组")
    by_path = {
        entry.get("path"): entry
        for entry in entries
        if isinstance(entry, dict) and isinstance(entry.get("path"), str)
    }
    actual_third_party = {
        path for path, entry in by_path.items() if entry.get("kind") == "third-party"
    }
    expected_third_party = set(_REQUIRED_THIRD_PARTY_PROVENANCE)
    if actual_third_party != expected_third_party:
        raise ComplianceError(
            "比赛第三方边界不一致：必须且只能声明 {}".format(
                sorted(expected_third_party)
            )
        )
    for path, expected in _REQUIRED_THIRD_PARTY_PROVENANCE.items():
        entry = by_path[path]
        if any(
            entry.get(field) != expected[field]
            for field in ("source", "version", "package_version", "license")
        ):
            raise ComplianceError("比赛第三方来源声明不一致：{}".format(path))
        verification = entry.get("verification")
        if (
            not isinstance(verification, dict)
            or verification.get("strategy") != expected["strategy"]
            or (
                "external_path" in expected
                and verification.get("external_path") != expected["external_path"]
            )
        ):
            raise ComplianceError("比赛第三方来源声明不一致：{}".format(path))
    return entries


def verify_competition_ownership_entries(root, entries, xtdrone_dir=None):
    verify_competition_ownership_boundary(entries)
    return verify_ownership_entries(root, entries, xtdrone_dir)


def verify_ownership(root, ownership_path, xtdrone_dir=None):
    root = _resolve_root(root)
    ownership_file = _repository_file(root, ownership_path, "所有权清单")
    return verify_ownership_entries(
        root,
        load_ownership(ownership_file),
        xtdrone_dir,
    )


def _tree_files(root, label):
    root = pathlib.Path(root)
    if root.is_symlink():
        raise ComplianceError("{}根目录为符号链接：{}".format(label, root))
    try:
        resolved = root.resolve(strict=True)
    except (OSError, ValueError, RuntimeError) as error:
        raise ComplianceError("无法解析{}目录 {}：{}".format(label, root, error)) from error
    if not resolved.is_dir():
        raise ComplianceError("{}不是目录：{}".format(label, root))
    files = set()

    def walk_error(error):
        raise ComplianceError("无法遍历{}：{}".format(label, error))

    for directory, names, filenames in os.walk(str(resolved), followlinks=False, onerror=walk_error):
        current = pathlib.Path(directory)
        for name in names:
            child = current / name
            if child.is_symlink():
                raise ComplianceError("{}包含符号链接：{}".format(label, child))
        for name in filenames:
            child = current / name
            if child.is_symlink():
                raise ComplianceError("{}包含符号链接：{}".format(label, child))
            if not child.is_file():
                raise ComplianceError("{}包含非普通文件：{}".format(label, child))
            files.add(child.relative_to(resolved).as_posix())
    return resolved, files


def compare_trees(left, right):
    left_root, left_files = _tree_files(left, "仓库 Actor 插件")
    right_root, right_files = _tree_files(right, "XTDrone Actor 插件")
    if left_files != right_files:
        raise ComplianceError("XTDrone Actor 插件文件集合与仓库副本不同")
    for relative in sorted(left_files):
        if sha256_file(left_root / relative) != sha256_file(right_root / relative):
            raise ComplianceError("XTDrone Actor 插件内容不同：{}".format(relative))


def logical_shell_commands(script_text):
    """Yield normalized backslash-continued commands with their first line."""
    pending = []
    start_line = None
    for line_number, raw_line in enumerate(script_text.splitlines(), 1):
        line = raw_line.rstrip()
        if start_line is None:
            start_line = line_number
        continued = line.endswith("\\")
        if continued:
            line = line[:-1]
        pending.append(line.strip())
        if continued:
            continue
        command = re.sub(r"\s+", " ", " ".join(pending)).strip()
        if command:
            yield start_line, command
        pending = []
        start_line = None
    if pending:
        command = re.sub(r"\s+", " ", " ".join(pending)).strip()
        if command:
            yield start_line, command


def _shell_tokens(command):
    try:
        return shlex.split(command, comments=True, posix=True)
    except ValueError:
        return command.split()


def has_official_reference(command):
    if _PROTECTED_VARIABLE.search(command):
        return True
    dequoted = command.replace('"', "").replace("'", "")
    if _OFFICIAL_LITERAL_PATH.search(dequoted):
        return True
    try:
        tokens = shlex.split(command, comments=True, posix=True)
    except ValueError:
        return bool(_OFFICIAL_LITERAL_PATH.search(command))
    for token in tokens:
        for component in token.split("/"):
            normalized = component.strip("\"'{}()[],:;=+!? ")
            if normalized in _OFFICIAL_LITERAL_NAMES:
                return True
    return False


def _command_names(tokens):
    names = set()
    for token in tokens:
        if not token or token.startswith("-") or "=" in token:
            continue
        names.add(pathlib.PurePosixPath(token).name)
    return names


def _short_option_contains(option, letter):
    return option.startswith("-") and not option.startswith("--") and letter in option[1:]


def _is_in_place_sed(tokens):
    if "sed" not in _command_names(tokens):
        return False
    return any(
        token == "--in-place"
        or token.startswith("--in-place=")
        or _short_option_contains(token, "i")
        for token in tokens
    )


def _is_symbolic_link_command(tokens):
    if "ln" not in _command_names(tokens):
        return False
    return any(
        token == "--symbolic"
        or token.startswith("--symbolic=")
        or _short_option_contains(token, "s")
        for token in tokens
    )


def _redirects_to_official(command):
    redirection = re.compile(
        r"(?:^|[\s;|&])(?:[0-9]+|&)?(?:>>?|&>)\s*(?P<target>[^\s;|&]+)"
    )
    return any(
        has_official_reference(match.group("target"))
        for match in redirection.finditer(command)
    )


def command_has_write_intent(command):
    tokens = _shell_tokens(command)
    names = _command_names(tokens)
    if names & _DIRECT_WRITE_COMMANDS:
        return True
    if _is_in_place_sed(tokens) or _is_symbolic_link_command(tokens):
        return True
    if _redirects_to_official(command):
        return True
    if "perl" in names and "-e" in tokens:
        return True
    if any(re.fullmatch(r"python(?:[0-9]+(?:\.[0-9]+)?)?", name) for name in names):
        if "-c" in tokens:
            return True
    return False


def _validate_bash_syntax(script_path):
    try:
        completed = subprocess.run(
            ["bash", "-n", str(script_path)],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
    except (OSError, UnicodeError) as error:
        raise ComplianceError(
            "无法执行 Bash 语法检查 {}：{}".format(script_path, error)
        ) from error
    if completed.returncode != 0:
        detail = completed.stderr.strip() or "未知语法错误"
        raise ComplianceError("比赛启动脚本 Bash 语法无效：{}".format(detail))


def _validate_official_shell_references(script_text, filename):
    """Enforce the checked entrypoint contract, not arbitrary shell semantics."""
    for line_number, command in logical_shell_commands(script_text):
        if not has_official_reference(command):
            continue
        location = "{}:{}".format(filename, line_number)
        if command_has_write_intent(command):
            raise ComplianceError(
                "入口静态守卫检测到官方目录写入：{}：{}".format(
                    location, command
                )
            )
        if command not in _ALLOWED_OFFICIAL_SHELL_COMMANDS:
            raise ComplianceError(
                "入口静态守卫拒绝未知官方目录引用：{}：{}；"
                "该引用不在当前入口只读许可中".format(location, command)
            )


def _validate_disallowed_shell_constructs(script_text, filename):
    for line_number, command in logical_shell_commands(script_text):
        reason = None
        if re.search(r"\$\{![^}]+\}", command):
            reason = "间接变量展开"
        elif "eval" in _shell_tokens(command):
            reason = "eval"
        elif re.search(r"\$\(\s*env\s+(?:PX4_DIR|XTDRONE_DIR)\s*\)", command):
            reason = "官方目录环境替换"
        elif "`" in command:
            reason = "反引号命令替换"
        if reason is not None:
            raise ComplianceError(
                "入口静态守卫拒绝{}：{}:{}：{}".format(
                    reason, filename, line_number, command
                )
            )


_ALLOWED_ROS_SUBSTITUTION = re.compile(
    r"\$\((?:find|arg)\s+[A-Za-z_][A-Za-z0-9_]*\)"
    r"|\$\(eval 1 \+ arg\('ID'\)\)"
)


def _validate_ros_substitutions(launch_text, launch_path):
    remainder = _ALLOWED_ROS_SUBSTITUTION.sub("", launch_text)
    if "$(" in remainder:
        raise ComplianceError(
            "比赛 launch 包含未经许可的 ROS 替换：{}".format(launch_path)
        )


def _validate_launch_official_references(launch_text, launch_path):
    try:
        launch_root = ET.fromstring(launch_text)
    except ET.ParseError as error:
        raise ComplianceError("比赛 launch XML 格式错误 {}：{}".format(launch_path, error)) from error
    for element in launch_root.iter():
        for name, value in element.attrib.items():
            if has_official_reference(value):
                raise ComplianceError(
                    "比赛 launch 包含未经许可的官方目录引用：{} <{} {}={}>".format(
                        launch_path, element.tag, name, value
                    )
                )
        for value in (element.text, element.tail):
            if value and has_official_reference(value):
                raise ComplianceError(
                    "比赛 launch 包含未经许可的官方目录文本引用：{} <{}>".format(
                        launch_path, element.tag
                    )
                )


def verify_entrypoints(root):
    root = _resolve_root(root)
    launch_path = _repository_file(root, "robocup_zzufly.launch", "比赛启动文件")
    script_path = _repository_file(root, "1.sh", "比赛启动脚本")
    try:
        launch_text = launch_path.read_text(encoding="utf-8")
        script_text = script_path.read_text(encoding="utf-8")
    except (OSError, UnicodeError) as error:
        raise ComplianceError("无法读取比赛启动入口：{}".format(error)) from error
    if "typhoon_h480_zzufly" in launch_text + script_text:
        raise ComplianceError("比赛启动入口仍引用调试模型 typhoon_h480_zzufly")
    _validate_bash_syntax(script_path)
    _validate_ros_substitutions(launch_text, launch_path)
    _validate_launch_official_references(launch_text, launch_path)
    _validate_disallowed_shell_constructs(script_text, "1.sh")
    for line_number, command in logical_shell_commands(script_text):
        if _is_symbolic_link_command(_shell_tokens(command)):
            raise ComplianceError(
                "比赛启动脚本不得创建符号链接：1.sh:{}：{}".format(
                    line_number, command
                )
            )
    _validate_official_shell_references(script_text, "1.sh")


def validate_evidence_path(root, evidence):
    root = _resolve_root(root)
    declared = pathlib.Path(evidence)
    if declared.is_absolute():
        try:
            relative_raw = declared.relative_to(root).as_posix()
        except ValueError as error:
            raise ComplianceError("合规证据必须位于仓库 competition-artifacts 内") from error
    else:
        relative_raw = pathlib.PurePosixPath(str(declared)).as_posix()
    relative = _relative_path(relative_raw, "合规证据路径")
    if len(relative.parts) != 2 or relative.parts[0] != "competition-artifacts":
        raise ComplianceError("合规证据必须位于仓库 competition-artifacts 内")
    _reject_symlink_components(root, relative, "合规证据")
    target = root.joinpath(*relative.parts)
    artifacts = root / "competition-artifacts"
    try:
        resolved_target = target.resolve(strict=False)
        resolved_artifacts = artifacts.resolve(strict=False)
        resolved_target.relative_to(resolved_artifacts)
    except (OSError, ValueError, RuntimeError) as error:
        raise ComplianceError("合规证据路径超出 competition-artifacts：{}".format(target)) from error
    return resolved_target


def write_evidence(root, path, payload):
    try:
        serialized = (json.dumps(payload, indent=2, ensure_ascii=False) + "\n").encode(
            "utf-8"
        )
    except (TypeError, ValueError, UnicodeError) as error:
        raise ComplianceError("无法序列化合规证据：{}".format(error)) from error

    root = _resolve_root(root)
    path = validate_evidence_path(root, path)
    root_fd = None
    artifacts_fd = None
    fresh_artifacts_fd = None
    temporary_fd = None
    temporary_name = None
    published_name = None
    published_identity = None
    try:
        directory_flags = os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW
        root_fd = os.open(str(root), directory_flags)
        artifacts_created = False
        try:
            os.mkdir("competition-artifacts", mode=0o755, dir_fd=root_fd)
            artifacts_created = True
        except FileExistsError:
            pass
        artifacts_fd = os.open(
            "competition-artifacts", directory_flags, dir_fd=root_fd
        )
        if artifacts_created:
            os.fsync(root_fd)

        for _attempt in range(16):
            candidate = ".{}.tmp-{}".format(path.name, secrets.token_hex(8))
            try:
                temporary_fd = os.open(
                    candidate,
                    os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW,
                    0o600,
                    dir_fd=artifacts_fd,
                )
                temporary_name = candidate
                break
            except FileExistsError:
                continue
        if temporary_fd is None:
            raise OSError(errno.EEXIST, "无法创建唯一的证据临时文件")

        view = memoryview(serialized)
        while view:
            written = os.write(temporary_fd, view)
            if written <= 0:
                raise OSError(errno.EIO, "写入合规证据时未取得进展")
            view = view[written:]
        os.fsync(temporary_fd)
        temporary_stat = os.fstat(temporary_fd)
        temporary_identity = (temporary_stat.st_dev, temporary_stat.st_ino)
        os.close(temporary_fd)
        temporary_fd = None

        try:
            os.link(
                temporary_name,
                path.name,
                src_dir_fd=artifacts_fd,
                dst_dir_fd=artifacts_fd,
                follow_symlinks=False,
            )
        except FileExistsError as error:
            raise ComplianceError(
                "合规证据已存在，拒绝覆盖：{}".format(path)
            ) from error
        published_name = path.name
        published_identity = temporary_identity

        os.unlink(temporary_name, dir_fd=artifacts_fd)
        temporary_name = None
        os.fsync(artifacts_fd)

        try:
            fresh_artifacts_fd = os.open(
                "competition-artifacts", directory_flags, dir_fd=root_fd
            )
            held_directory = os.fstat(artifacts_fd)
            fresh_directory = os.fstat(fresh_artifacts_fd)
            held_final = os.stat(
                path.name, dir_fd=artifacts_fd, follow_symlinks=False
            )
            fresh_final = os.stat(
                path.name, dir_fd=fresh_artifacts_fd, follow_symlinks=False
            )
        except OSError as error:
            raise ComplianceError(
                "合规证据目录身份变化或已被替换：{}".format(path.parent)
            ) from error
        if (
            (held_directory.st_dev, held_directory.st_ino)
            != (fresh_directory.st_dev, fresh_directory.st_ino)
            or (held_final.st_dev, held_final.st_ino) != published_identity
            or (fresh_final.st_dev, fresh_final.st_ino) != published_identity
        ):
            raise ComplianceError(
                "合规证据文件或目录身份变化，可能已被替换：{}".format(path)
            )

        published_name = None
        published_identity = None
    except ComplianceError:
        raise
    except OSError as error:
        raise ComplianceError("无法写入合规证据 {}：{}".format(path, error)) from error
    finally:
        if temporary_fd is not None:
            try:
                os.close(temporary_fd)
            except OSError:
                pass
        if temporary_name is not None and artifacts_fd is not None:
            try:
                os.unlink(temporary_name, dir_fd=artifacts_fd)
                os.fsync(artifacts_fd)
            except OSError:
                pass
        if (
            published_name is not None
            and published_identity is not None
            and artifacts_fd is not None
        ):
            try:
                current = os.stat(
                    published_name,
                    dir_fd=artifacts_fd,
                    follow_symlinks=False,
                )
                if (current.st_dev, current.st_ino) == published_identity:
                    os.unlink(published_name, dir_fd=artifacts_fd)
                    os.fsync(artifacts_fd)
            except OSError:
                pass
        for descriptor in (fresh_artifacts_fd, artifacts_fd, root_fd):
            if descriptor is not None:
                try:
                    os.close(descriptor)
                except OSError:
                    pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", required=True, type=pathlib.Path)
    parser.add_argument("--px4-dir", required=True, type=pathlib.Path)
    parser.add_argument("--xtdrone-dir", required=True, type=pathlib.Path)
    parser.add_argument("--gazebo-models-dir", required=True, type=pathlib.Path)
    parser.add_argument("--xtdrone-pythonpath", required=True, type=pathlib.Path)
    parser.add_argument("--manifest", required=True, type=pathlib.Path)
    parser.add_argument("--ownership", required=True, type=pathlib.Path)
    parser.add_argument("--evidence", required=True, type=pathlib.Path)
    args = parser.parse_args()
    root = _resolve_root(args.root)

    present = [path for path in FORBIDDEN if (root / path).exists() or (root / path).is_symlink()]
    if present:
        raise ComplianceError("clean 分支包含禁止目录：{}".format(", ".join(present)))

    manifest_path = canonical_manifest_path(root, args.manifest)
    manifest, manifest_digest = load_manifest_with_digest(manifest_path)
    ownership_path = canonical_ownership_path(root, args.ownership)
    ownership_entries, ownership_digest = load_ownership_with_digest(ownership_path)
    runtime_roots = canonical_runtime_roots(
        {
            "PX4_DIR": args.px4_dir,
            "XTDRONE_DIR": args.xtdrone_dir,
            "GAZEBO_MODELS_DIR": args.gazebo_models_dir,
            "XTDRONE_PYTHONPATH": args.xtdrone_pythonpath,
        }
    )
    manifest = verify_manifest_document(
        manifest,
        runtime_roots,
    )
    verify_required_manifest_identities(manifest)
    actual_versions = verify_versions(manifest, runtime_roots["XTDRONE_DIR"])
    verified_ownership_entries = verify_competition_ownership_entries(
        root, ownership_entries, runtime_roots["XTDRONE_DIR"]
    )
    verify_entrypoints(root)
    evidence_path = validate_evidence_path(root, args.evidence)
    write_evidence(
        root,
        evidence_path,
        build_evidence(
            manifest_digest,
            manifest,
            actual_versions,
            ownership_digest,
            verified_ownership_entries,
            runtime_roots,
        ),
    )
    print("完整静态合规检查通过")


if __name__ == "__main__":
    try:
        main()
    except ComplianceError as error:
        print("完整合规检查失败：{}".format(error), file=sys.stderr)
        print(
            "恢复方法：按 docs/TROUBLESHOOTING.md 恢复对应官方文件或依赖后重试。",
            file=sys.stderr,
        )
        raise SystemExit(2)

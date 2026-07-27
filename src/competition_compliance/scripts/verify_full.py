#!/usr/bin/env python3

import argparse
import json
import os
import pathlib
import re
import shlex
import subprocess
import sys
import xml.etree.ElementTree as ET

from competition_compliance.manifest import (
    sha256_file,
    verify_manifest,
    verify_versions,
)
from competition_compliance.model import ComplianceError


FORBIDDEN = ("src/gazebo_ros_pkgs", "typhoon_h480_zzufly", "src/gimbal")
_BASE_ENTRY_KEYS = {"path", "kind", "source", "version", "license"}
_HASH_PATTERN = re.compile(r"^[0-9a-f]{64}$")
_PLACEHOLDER_TOKENS = (
    "<license>TODO",
    "todo.com",
    "todo.todo",
    "Your Name",
    "your-email",
    "your_email",
    "your.email",
)
_COMMENTED_IDENTITY = re.compile(
    r"<!--(?:(?!-->).)*<(?:author|maintainer)\b", re.DOTALL
)
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
        'if ! "$COMPLIANCE_PYTHON" "$PREPARE_MODEL" --px4-dir "$PX4_DIR" --xtdrone-dir "$XTDRONE_DIR" --manifest "$OFFICIAL_MANIFEST" --mount-config "$SENSOR_MOUNT_CONFIG" --output "$GENERATED_MODEL" >/dev/null; then',
        'start_communication "$XTDRONE_PYTHON" "$XTDRONE_DIR/communication/multirotor_communication.py" || return 1',
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


def load_ownership(path):
    path = pathlib.Path(path)
    try:
        data = json.loads(
            path.read_text(encoding="utf-8"), object_pairs_hook=_unique_object
        )
    except OSError as error:
        raise ComplianceError("无法读取所有权清单 {}：{}".format(path, error)) from error
    except UnicodeError as error:
        raise ComplianceError("所有权清单不是有效 UTF-8 {}：{}".format(path, error)) from error
    except json.JSONDecodeError as error:
        raise ComplianceError("所有权清单 JSON 格式错误 {}：{}".format(path, error)) from error
    if not isinstance(data, dict) or set(data) != {"entries"}:
        raise ComplianceError("所有权清单必须只包含 entries")
    if not isinstance(data["entries"], list):
        raise ComplianceError("所有权清单 entries 必须为数组")
    return data["entries"]


def _validate_entry(entry):
    if not isinstance(entry, dict):
        raise ComplianceError("所有权条目必须为对象")
    kind = entry.get("kind")
    expected_keys = _BASE_ENTRY_KEYS
    if kind == "third-party" and "files" in entry:
        expected_keys = _BASE_ENTRY_KEYS | {"files"}
    if set(entry) != expected_keys:
        raise ComplianceError("所有权条目字段不符合 {} 类型要求".format(kind))
    if kind not in ("team", "third-party"):
        raise ComplianceError("所有权 kind 只能是 team 或 third-party")
    relative = _relative_path(entry["path"], "所有权包路径")
    for key in ("source", "version", "license"):
        value = entry[key]
        if not isinstance(value, str) or not value.strip():
            raise ComplianceError("所有权条目 {} 必须为非空字符串：{}".format(key, entry["path"]))
    files = entry.get("files")
    if files is not None:
        if not isinstance(files, dict) or not files:
            raise ComplianceError("第三方 files 必须为非空哈希对象：{}".format(entry["path"]))
        for path, digest in files.items():
            _relative_path(path, "第三方文件路径")
            if not isinstance(digest, str) or not _HASH_PATTERN.fullmatch(digest):
                raise ComplianceError("第三方文件 sha256 无效：{}/{}".format(entry["path"], path))
    return relative


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


def verify_ownership(root, ownership_path):
    root = _resolve_root(root)
    ownership_file = _repository_file(root, ownership_path, "所有权清单")
    entries = load_ownership(ownership_file)
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
        if entry["kind"] == "team" or entry["source"].startswith("https://github.com/leggedrobotics/"):
            if version != entry["version"]:
                raise ComplianceError("所有权 version 与 package.xml 不一致：{}".format(entry["path"]))
        normalized_licenses = {_normalized_license(value) for value in licenses}
        if _normalized_license(entry["license"]) not in normalized_licenses:
            raise ComplianceError("所有权 license 与 package.xml 不一致：{}".format(entry["path"]))
        try:
            metadata_text = package_xml.read_text(encoding="utf-8")
        except (OSError, UnicodeError) as error:
            raise ComplianceError("无法读取 Catkin 包元数据 {}：{}".format(package_xml, error)) from error
        if any(token in metadata_text for token in _PLACEHOLDER_TOKENS) or _COMMENTED_IDENTITY.search(metadata_text):
            raise ComplianceError("Catkin 包元数据仍含占位内容：{}".format(package_xml))
        for file_path, expected in entry.get("files", {}).items():
            file_relative = _relative_path(file_path, "第三方文件路径")
            _reject_symlink_components(package_dir, file_relative, "第三方文件")
            target = package_dir.joinpath(*file_relative.parts)
            if not target.is_file() or sha256_file(target) != expected:
                raise ComplianceError("第三方文件校验失败：{}".format(target))
    return entries


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
    _validate_launch_official_references(launch_text, launch_path)
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
    if not relative.parts or relative.parts[0] != "competition-artifacts" or len(relative.parts) < 2:
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


def write_evidence(path, payload):
    path = pathlib.Path(path)
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("x", encoding="utf-8") as stream:
            json.dump(payload, stream, indent=2, ensure_ascii=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
    except FileExistsError as error:
        raise ComplianceError("合规证据已存在，拒绝覆盖：{}".format(path)) from error
    except OSError as error:
        raise ComplianceError("无法写入合规证据 {}：{}".format(path, error)) from error


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", required=True, type=pathlib.Path)
    parser.add_argument("--px4-dir", required=True, type=pathlib.Path)
    parser.add_argument("--xtdrone-dir", required=True, type=pathlib.Path)
    parser.add_argument("--manifest", required=True, type=pathlib.Path)
    parser.add_argument("--ownership", required=True, type=pathlib.Path)
    parser.add_argument("--evidence", required=True, type=pathlib.Path)
    args = parser.parse_args()
    root = _resolve_root(args.root)

    present = [path for path in FORBIDDEN if (root / path).exists() or (root / path).is_symlink()]
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
    verify_entrypoints(root)
    evidence_path = validate_evidence_path(root, args.evidence)
    write_evidence(
        evidence_path,
        {
            "status": "pass",
            "versions": actual_versions,
            "checked_files": len(manifest["files"]),
            "critical_official_files_match": True,
        },
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

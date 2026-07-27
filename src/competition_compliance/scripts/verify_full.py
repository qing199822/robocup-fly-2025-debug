#!/usr/bin/env python3

import argparse
import json
import os
import pathlib
import re
import shlex
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
_OFFICIAL_VARIABLES = ("$PX4_DIR", "${PX4_DIR}", "$XTDRONE_DIR", "${XTDRONE_DIR}")


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


def _contains_official_variable(value):
    return any(marker in value for marker in _OFFICIAL_VARIABLES)


def _command_writes_official(line):
    if _contains_official_variable(line) and re.search(r"\bsed\b[^\n]*\s-i(?:\s|$)", line):
        return True
    if re.search(r"(?:^|\s)(?:[0-9&]*>{1,2})\s*[\"']?\$(?:\{)?(?:PX4_DIR|XTDRONE_DIR)", line):
        return True
    try:
        tokens = shlex.split(line, comments=True)
    except ValueError:
        tokens = line.split()
    for command in ("cp", "mv"):
        if command not in tokens:
            continue
        index = tokens.index(command)
        segment = []
        for token in tokens[index + 1 :]:
            if token in (";", "&&", "||"):
                break
            if not token.startswith("-"):
                segment.append(token)
        if segment and _contains_official_variable(segment[-1]):
            return True
    return False


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
    if re.search(r"(?:^|[;&|]\s*|\n\s*)ln\s+-s(?:\s|$)", script_text):
        raise ComplianceError("比赛启动脚本不得创建符号链接")
    for line_number, line in enumerate(script_text.splitlines(), 1):
        if _command_writes_official(line):
            raise ComplianceError("比赛启动脚本试图向官方目录写入（第 {} 行）".format(line_number))


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

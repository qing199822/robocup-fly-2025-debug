import hashlib
import json
import pathlib
import subprocess
from collections.abc import Mapping

from competition_compliance.model import ComplianceError


_BLOCK_SIZE = 1024 * 1024
_ENTRY_KEYS = {"root", "path", "sha256"}
_MANIFEST_KEYS = {"files", "versions"}
_VERSION_KEYS = {
    "gazebo11",
    "ros-noetic-gazebo-ros",
    "ros-noetic-gazebo-ros-pkgs",
    "xtdrone_commit",
}


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ComplianceError("官方清单 JSON 包含重复键：{}".format(key))
        result[key] = value
    return result


def _validate_versions(versions):
    if not isinstance(versions, dict) or set(versions) != _VERSION_KEYS:
        raise ComplianceError("官方清单 versions 必须恰好包含四个规定版本键")
    for name, value in versions.items():
        if not isinstance(value, str) or not value.strip():
            raise ComplianceError("官方版本 {} 必须为非空字符串".format(name))
    commit = versions["xtdrone_commit"]
    if (
        len(commit) != 40
        or any(character not in "0123456789abcdef" for character in commit)
    ):
        raise ComplianceError("XTDrone Git 提交必须为 40 位小写十六进制字符串")


def sha256_file(path):
    digest = hashlib.sha256()
    path = pathlib.Path(path)
    try:
        with path.open("rb") as stream:
            for block in iter(lambda: stream.read(_BLOCK_SIZE), b""):
                digest.update(block)
    except OSError as error:
        raise ComplianceError("无法读取校验文件 {}：{}".format(path, error)) from error
    return digest.hexdigest()


def _validate_manifest_document(data):
    if not isinstance(data, dict) or set(data) != _MANIFEST_KEYS:
        raise ComplianceError("官方清单只能包含 files 和 versions")
    if not isinstance(data["files"], list) or not isinstance(data["versions"], dict):
        raise ComplianceError("官方清单格式无效：files 必须为数组，versions 必须为对象")
    _validate_versions(data["versions"])
    _validate_manifest_files(data["files"])
    return data


def load_manifest_with_digest(path):
    path = pathlib.Path(path)
    try:
        encoded = path.read_bytes()
    except OSError as error:
        raise ComplianceError("无法读取官方清单 {}：{}".format(path, error)) from error
    try:
        text = encoded.decode("utf-8")
    except UnicodeError as error:
        raise ComplianceError(
            "官方清单不是有效的 UTF-8 文件 {}：{}".format(path, error)
        ) from error
    try:
        data = json.loads(text, object_pairs_hook=_unique_object)
    except json.JSONDecodeError as error:
        raise ComplianceError("官方清单 JSON 格式错误 {}：{}".format(path, error)) from error
    return _validate_manifest_document(data), hashlib.sha256(encoded).hexdigest()


def load_manifest(path):
    manifest, _digest = load_manifest_with_digest(path)
    return manifest


def _validate_entry(entry):
    if not isinstance(entry, dict) or set(entry) != _ENTRY_KEYS:
        raise ComplianceError("官方文件条目必须只包含 root、path、sha256")

    if not isinstance(entry["root"], str) or not entry["root"]:
        raise ComplianceError("官方文件条目 root 必须为非空字符串")
    if not isinstance(entry["path"], str) or not entry["path"]:
        raise ComplianceError("官方文件条目 path 必须为非空字符串")
    if "\\" in entry["path"]:
        raise ComplianceError("官方文件路径必须使用规范 POSIX 拼写：{}".format(entry["path"]))
    relative = pathlib.PurePosixPath(entry["path"])
    if (
        relative.is_absolute()
        or ".." in relative.parts
        or relative.as_posix() != entry["path"]
        or entry["path"] == "."
    ):
        raise ComplianceError("官方文件路径必须为规范相对路径：{}".format(entry["path"]))
    digest = entry["sha256"]
    if (
        not isinstance(digest, str)
        or len(digest) != 64
        or any(character not in "0123456789abcdef" for character in digest)
    ):
        raise ComplianceError("官方文件条目 sha256 必须为 64 位小写十六进制字符串")
    return entry["root"], entry["path"]


def _validate_manifest_files(files):
    if not files:
        raise ComplianceError("官方清单 files 不得为空")
    identities = set()
    for entry in files:
        identity = _validate_entry(entry)
        if identity in identities:
            raise ComplianceError(
                "官方清单包含重复文件身份：{}/{}".format(*identity)
            )
        identities.add(identity)


def verify_manifest_document(manifest, roots):
    _validate_manifest_document(manifest)
    if not isinstance(roots, Mapping):
        raise ComplianceError("官方目录参数必须为 root 到目录的映射")

    for entry in manifest["files"]:
        _validate_entry(entry)
        root_name = entry["root"]
        if root_name not in roots:
            raise ComplianceError("缺少官方目录参数：{}".format(root_name))

        relative = pathlib.PurePosixPath(entry["path"])
        try:
            declared_root = pathlib.Path(roots[root_name])
            resolved_root = declared_root.resolve(strict=True)
        except (OSError, TypeError, ValueError, RuntimeError) as error:
            raise ComplianceError(
                "官方目录参数无效 {}：{}".format(root_name, error)
            ) from error
        if not resolved_root.is_dir():
            raise ComplianceError("官方目录参数不是目录：{}".format(declared_root))

        target = declared_root / pathlib.Path(*relative.parts)
        component = declared_root
        for part in relative.parts:
            component = component / part
            if component.is_symlink():
                raise ComplianceError(
                    "官方文件路径包含符号链接：{}".format(entry["path"])
                )
        try:
            resolved_target = target.resolve(strict=True)
        except FileNotFoundError as error:
            raise ComplianceError("找不到官方文件：{}".format(target))
        except (OSError, ValueError, RuntimeError) as error:
            raise ComplianceError(
                "无法解析官方文件 {}：{}".format(target, error)
            ) from error
        try:
            resolved_target.relative_to(resolved_root)
        except ValueError as error:
            raise ComplianceError(
                "官方文件路径超出声明目录：{}".format(entry["path"])
            ) from error
        if not resolved_target.is_file():
            raise ComplianceError("找不到官方文件：{}".format(target))

        actual = sha256_file(resolved_target)
        expected = entry["sha256"]
        if actual != expected:
            raise ComplianceError(
                "官方文件校验失败：{}\n期望：{}\n实际：{}".format(
                    entry["path"], expected, actual
                )
            )
    return manifest


def verify_manifest(path, roots):
    return verify_manifest_document(load_manifest(path), roots)


def validate_output_path(output_path, roots):
    if not isinstance(roots, Mapping):
        raise ComplianceError("官方目录参数必须为 root 到目录的映射")
    try:
        resolved_output = pathlib.Path(output_path).resolve(strict=False)
    except (OSError, TypeError, ValueError, RuntimeError) as error:
        raise ComplianceError("无法解析生成模型路径 {}：{}".format(output_path, error)) from error

    for root_name in ("PX4_DIR", "XTDRONE_DIR"):
        if root_name not in roots:
            raise ComplianceError("缺少官方目录参数：{}".format(root_name))
        try:
            resolved_root = pathlib.Path(roots[root_name]).resolve(strict=True)
        except (OSError, TypeError, ValueError, RuntimeError) as error:
            raise ComplianceError(
                "官方目录参数无效 {}：{}".format(root_name, error)
            ) from error
        if not resolved_root.is_dir():
            raise ComplianceError("官方目录参数不是目录：{}".format(resolved_root))
        try:
            resolved_output.relative_to(resolved_root)
        except ValueError:
            continue
        raise ComplianceError(
            "生成模型路径不得位于官方目录 {} 内：{}".format(
                root_name, resolved_output
            )
        )
    return resolved_output


def collect_versions(xtdrone_dir):
    def run_version_command(command, label):
        try:
            return subprocess.check_output(
                command,
                stderr=subprocess.STDOUT,
                text=True,
            ).strip()
        except FileNotFoundError as error:
            raise ComplianceError(
                "无法执行版本检查 {}：找不到命令 {}".format(label, command[0])
            ) from error
        except subprocess.CalledProcessError as error:
            output = error.output
            if isinstance(output, bytes):
                output = output.decode("utf-8", errors="replace")
            detail = str(output).strip() if output else "无输出"
            raise ComplianceError(
                "版本检查失败 {}：{}".format(label, detail)
            ) from error
        except UnicodeError as error:
            raise ComplianceError(
                "版本检查无法解码 {}（命令 {}）：{}".format(
                    label, command[0], error
                )
            ) from error
        except OSError as error:
            raise ComplianceError(
                "无法执行版本检查 {}（命令 {}）：{}".format(
                    label, command[0], error
                )
            ) from error

    def package_version(name):
        return run_version_command(
            ["dpkg-query", "-W", "-f=${Version}", name], name
        )

    return {
        "gazebo11": package_version("gazebo11"),
        "ros-noetic-gazebo-ros": package_version("ros-noetic-gazebo-ros"),
        "ros-noetic-gazebo-ros-pkgs": package_version(
            "ros-noetic-gazebo-ros-pkgs"
        ),
        "xtdrone_commit": run_version_command(
            ["git", "-C", str(xtdrone_dir), "rev-parse", "HEAD"],
            "XTDrone Git 提交",
        ),
    }


def verify_versions(manifest, xtdrone_dir):
    if not isinstance(manifest, dict) or not isinstance(manifest.get("versions"), dict):
        raise ComplianceError("官方清单 versions 必须为对象")
    expected = manifest["versions"]
    _validate_versions(expected)
    actual = collect_versions(xtdrone_dir)
    if actual != expected:
        raise ComplianceError(
            "官方版本不匹配：期望 {}，实际 {}".format(expected, actual)
        )
    return actual

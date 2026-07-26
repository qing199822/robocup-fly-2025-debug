import hashlib
import json
import pathlib
import subprocess
from collections.abc import Mapping

from competition_compliance.model import ComplianceError


_BLOCK_SIZE = 1024 * 1024
_ENTRY_KEYS = {"root", "path", "sha256"}
_MANIFEST_KEYS = {"files", "versions"}


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


def load_manifest(path):
    path = pathlib.Path(path)
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except OSError as error:
        raise ComplianceError("无法读取官方清单 {}：{}".format(path, error)) from error
    except UnicodeError as error:
        raise ComplianceError(
            "官方清单不是有效的 UTF-8 文件 {}：{}".format(path, error)
        ) from error
    except json.JSONDecodeError as error:
        raise ComplianceError("官方清单 JSON 格式错误 {}：{}".format(path, error)) from error

    if not isinstance(data, dict) or set(data) != _MANIFEST_KEYS:
        raise ComplianceError("官方清单只能包含 files 和 versions")
    if not isinstance(data["files"], list) or not isinstance(data["versions"], dict):
        raise ComplianceError("官方清单格式无效：files 必须为数组，versions 必须为对象")
    return data


def _validate_entry(entry):
    if not isinstance(entry, dict) or set(entry) != _ENTRY_KEYS:
        raise ComplianceError("官方文件条目必须只包含 root、path、sha256")

    if not isinstance(entry["root"], str) or not entry["root"]:
        raise ComplianceError("官方文件条目 root 必须为非空字符串")
    if not isinstance(entry["path"], str) or not entry["path"]:
        raise ComplianceError("官方文件条目 path 必须为非空字符串")
    digest = entry["sha256"]
    if (
        not isinstance(digest, str)
        or len(digest) != 64
        or any(character not in "0123456789abcdef" for character in digest)
    ):
        raise ComplianceError("官方文件条目 sha256 必须为 64 位小写十六进制字符串")


def verify_manifest(path, roots):
    manifest = load_manifest(path)
    if not isinstance(roots, Mapping):
        raise ComplianceError("官方目录参数必须为 root 到目录的映射")

    for entry in manifest["files"]:
        _validate_entry(entry)
        root_name = entry["root"]
        if root_name not in roots:
            raise ComplianceError("缺少官方目录参数：{}".format(root_name))

        relative = pathlib.PurePosixPath(entry["path"])
        if relative.is_absolute() or ".." in relative.parts:
            raise ComplianceError(
                "官方文件路径必须位于声明目录内：{}".format(entry["path"])
            )
        try:
            target = pathlib.Path(roots[root_name]) / pathlib.Path(*relative.parts)
        except (OSError, TypeError, ValueError) as error:
            raise ComplianceError(
                "官方目录参数无效 {}：{}".format(root_name, error)
            ) from error
        if not target.is_file():
            raise ComplianceError("找不到官方文件：{}".format(target))

        actual = sha256_file(target)
        expected = entry["sha256"]
        if actual != expected:
            raise ComplianceError(
                "官方文件校验失败：{}\n期望：{}\n实际：{}".format(
                    entry["path"], expected, actual
                )
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
    actual = collect_versions(xtdrone_dir)
    if actual != expected:
        raise ComplianceError(
            "官方版本不匹配：期望 {}，实际 {}".format(expected, actual)
        )
    return actual

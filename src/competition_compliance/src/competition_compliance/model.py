import dataclasses
import math
import numbers
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
        if any(
            isinstance(value, bool) or not isinstance(value, numbers.Real)
            for value in values
        ):
            raise ComplianceError("Realsense 安装位姿包含非数字值")
        numbers_ = tuple(float(value) for value in values)
        if not all(math.isfinite(value) for value in numbers_):
            raise ComplianceError("Realsense 安装位姿不能包含 NaN 或无穷值")
        return cls(*numbers_)

    def to_sdf(self):
        return " ".join(format(value, ".12g") for value in dataclasses.astuple(self))


def load_mount_pose(path):
    path = pathlib.Path(path)
    try:
        text = path.read_text(encoding="utf-8")
        data = yaml.safe_load(text)
    except OSError as error:
        raise ComplianceError("无法读取安装配置 {}：{}".format(path, error)) from error
    except UnicodeError as error:
        raise ComplianceError(
            "安装配置不是有效的 UTF-8 文件 {}：{}".format(path, error)
        ) from error
    except yaml.YAMLError as error:
        raise ComplianceError("安装配置 YAML 格式错误 {}：{}".format(path, error)) from error
    if not isinstance(data, dict) or set(data) != {"realsense_mount"}:
        raise ComplianceError("安装配置只能包含 realsense_mount")
    return MountPose.from_values(data["realsense_mount"])


def _find_mount(root):
    includes = []
    for include in root.findall("./model/include"):
        uri = include.find("uri")
        if (
            uri is not None
            and (uri.text or "").strip() == "model://realsense_camera"
        ):
            includes.append(include)
    if len(includes) != 1:
        raise ComplianceError("官方模型必须恰好包含一个 Realsense include")

    pose = includes[0].find("pose")
    if pose is None:
        raise ComplianceError("Realsense include 缺少 pose")

    joints = []
    for joint in root.findall("./model/joint"):
        child = joint.find("child")
        if (
            child is not None
            and (child.text or "").strip() == "realsense_camera::link"
        ):
            joints.append(joint)
    if len(joints) != 1:
        raise ComplianceError("官方模型必须恰好包含一个 Realsense 固定关节")

    parent = joints[0].find("parent")
    if (
        joints[0].get("type") != "fixed"
        or parent is None
        or (parent.text or "").strip() != "base_link"
    ):
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


def _validate_sdf_pose(text):
    values = (text or "").split()
    try:
        numbers_ = [float(value) for value in values]
    except (TypeError, ValueError) as error:
        raise ComplianceError("生成模型的 Realsense pose 包含非数字值") from error
    return MountPose.from_values(numbers_)


def generate_model(official_path, output_path, mount_pose):
    official_path = pathlib.Path(official_path)
    output_path = pathlib.Path(output_path)
    if output_path.exists():
        raise ComplianceError("拒绝覆盖已有生成模型：{}".format(output_path))

    tree = _parse_xml(official_path)
    _find_mount(tree.getroot()).text = mount_pose.to_sdf()
    try:
        output_path.parent.mkdir(parents=True, exist_ok=True)
    except OSError as error:
        raise ComplianceError("无法创建模型目录 {}：{}".format(output_path.parent, error)) from error

    try:
        with output_path.open("xb") as output:
            tree.write(output, encoding="utf-8", xml_declaration=True)
    except FileExistsError as error:
        raise ComplianceError("拒绝覆盖已有生成模型：{}".format(output_path)) from error
    except OSError as error:
        raise ComplianceError("无法写入临时模型 {}：{}".format(output_path, error)) from error

    assert_only_mount_pose_changed(official_path, output_path)


def assert_only_mount_pose_changed(official_path, generated_path):
    official_tree = _parse_xml(official_path)
    generated_tree = _parse_xml(generated_path)
    official_mount = _find_mount(official_tree.getroot())
    generated_mount = _find_mount(generated_tree.getroot())
    _validate_sdf_pose(generated_mount.text)
    if _canonical(official_tree.getroot(), official_mount) != _canonical(
        generated_tree.getroot(), generated_mount
    ):
        raise ComplianceError("生成模型除 Realsense 安装 pose 外还存在其他差异")

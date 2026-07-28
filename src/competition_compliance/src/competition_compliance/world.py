import pathlib
import xml.etree.ElementTree as ET

from competition_compliance.model import ComplianceError


ACTOR_COUNT = 6
COLLISION_PLUGIN = "libActorCollisionsPlugin.so"
ROS_PLUGIN = "libros_actor_cmd_pose_plugin.so"


def _parse_xml(path):
    path = pathlib.Path(path)
    try:
        return ET.parse(str(path))
    except OSError as error:
        raise ComplianceError(
            "无法读取 Gazebo world {}：{}".format(path, error)
        ) from error
    except ET.ParseError as error:
        raise ComplianceError(
            "Gazebo world XML 格式错误 {}：{}".format(path, error)
        ) from error


def _actor_plugins(actor, filename):
    return [
        plugin
        for plugin in actor.findall("plugin")
        if plugin.get("filename") == filename
    ]


def _validated_actors(root, collision_count):
    worlds = root.findall("./world")
    if len(worlds) != 1:
        raise ComplianceError("Gazebo world 必须恰好包含一个 world 元素")

    actors = worlds[0].findall("actor")
    if len(actors) != ACTOR_COUNT:
        raise ComplianceError("Gazebo world 必须恰好包含 6 个 actor")

    names = [actor.get("name") for actor in actors]
    if any(not name for name in names) or len(set(names)) != ACTOR_COUNT:
        raise ComplianceError("Gazebo actor 名称必须非空且唯一")

    for actor in actors:
        if len(_actor_plugins(actor, COLLISION_PLUGIN)) != collision_count:
            raise ComplianceError(
                "每个 actor 必须恰好包含 {} 个人物碰撞插件".format(
                    collision_count
                )
            )
        if len(_actor_plugins(actor, ROS_PLUGIN)) != 1:
            raise ComplianceError("每个 actor 必须恰好包含一个人物 ROS 控制插件")
    return actors


def _canonical(element):
    return (
        element.tag,
        tuple(sorted(element.attrib.items())),
        (element.text or "").strip(),
        (element.tail or "").strip(),
        tuple(_canonical(child) for child in list(element)),
    )


def _remove_actor_collision_plugins(root):
    actors = _validated_actors(root, collision_count=1)
    for actor in actors:
        actor.remove(_actor_plugins(actor, COLLISION_PLUGIN)[0])


def assert_only_actor_collisions_removed(official_path, generated_path):
    official_tree = _parse_xml(official_path)
    generated_tree = _parse_xml(generated_path)
    _remove_actor_collision_plugins(official_tree.getroot())
    _validated_actors(generated_tree.getroot(), collision_count=0)
    if _canonical(official_tree.getroot()) != _canonical(generated_tree.getroot()):
        raise ComplianceError("生成 world 除人物碰撞插件外还存在其他差异")


def generate_world(official_path, output_path):
    official_path = pathlib.Path(official_path)
    output_path = pathlib.Path(output_path)
    if output_path.exists():
        raise ComplianceError("拒绝覆盖已有生成 world：{}".format(output_path))

    tree = _parse_xml(official_path)
    _remove_actor_collision_plugins(tree.getroot())
    try:
        output_path.parent.mkdir(parents=True, exist_ok=True)
    except OSError as error:
        raise ComplianceError(
            "无法创建 world 目录 {}：{}".format(output_path.parent, error)
        ) from error

    try:
        output = output_path.open("xb")
    except FileExistsError as error:
        raise ComplianceError(
            "拒绝覆盖已有生成 world：{}".format(output_path)
        ) from error
    except OSError as error:
        raise ComplianceError(
            "无法写入临时 world {}：{}".format(output_path, error)
        ) from error

    try:
        with output:
            tree.write(output, encoding="utf-8", xml_declaration=True)
        assert_only_actor_collisions_removed(official_path, output_path)
    except Exception as error:
        try:
            output_path.unlink()
        except FileNotFoundError:
            pass
        except OSError as cleanup_error:
            raise ComplianceError(
                "无法清理无效临时 world {}：{}".format(
                    output_path, cleanup_error
                )
            ) from cleanup_error
        if isinstance(error, OSError):
            raise ComplianceError(
                "无法写入临时 world {}：{}".format(output_path, error)
            ) from error
        raise

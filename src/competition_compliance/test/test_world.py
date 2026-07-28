#!/usr/bin/env python3

import copy
import pathlib
import tempfile
import unittest
import xml.etree.ElementTree as ET
from unittest import mock

from competition_compliance.model import ComplianceError
from competition_compliance.world import (
    assert_only_actor_collisions_removed,
    generate_world,
)


COLLISION_PLUGIN = "libActorCollisionsPlugin.so"
ROS_PLUGIN = "libros_actor_cmd_pose_plugin.so"


def make_world(path, actor_count=6):
    root = ET.Element("sdf", {"version": "1.6"})
    world = ET.SubElement(root, "world", {"name": "default"})
    ET.SubElement(world, "plugin", {"filename": COLLISION_PLUGIN})
    for index in range(actor_count):
        actor = ET.SubElement(
            world,
            "actor",
            {"name": "actor_{}".format(index)},
        )
        collision = ET.SubElement(
            actor,
            "plugin",
            {"name": "actor_collisions_plugin", "filename": COLLISION_PLUGIN},
        )
        ET.SubElement(
            collision,
            "scaling",
            {"collision": "LeftLeg_collision", "scale": "8 8 1"},
        )
        skin = ET.SubElement(actor, "skin")
        ET.SubElement(skin, "filename").text = "model://walker/walk_0.dae"
        animation = ET.SubElement(actor, "animation", {"name": "walking"})
        ET.SubElement(animation, "filename").text = "model://walker/walk_0.dae"
        ros = ET.SubElement(
            actor,
            "plugin",
            {"name": "actor{}_plugin".format(index), "filename": ROS_PLUGIN},
        )
        ET.SubElement(ros, "init_pose").text = (
            "{} {} 1.25 1.57 0 0".format(index * 10, index)
        )
        ET.SubElement(ros, "animation_factor").text = "5.1"
    ET.ElementTree(root).write(str(path), encoding="utf-8", xml_declaration=True)


def actor_plugins(root, filename):
    return root.findall(
        "./world/actor/plugin[@filename='{}']".format(filename)
    )


class WorldGenerationTest(unittest.TestCase):
    def test_generation_removes_only_actor_collision_plugins(self):
        with tempfile.TemporaryDirectory() as directory:
            source = pathlib.Path(directory) / "official.world"
            output = pathlib.Path(directory) / "generated.world"
            make_world(source)
            before = source.read_bytes()

            generate_world(source, output)

            root = ET.parse(str(output)).getroot()
            self.assertEqual(before, source.read_bytes())
            self.assertEqual(0, len(actor_plugins(root, COLLISION_PLUGIN)))
            self.assertEqual(6, len(actor_plugins(root, ROS_PLUGIN)))
            self.assertEqual(
                1,
                len(
                    root.findall(
                        "./world/plugin[@filename='{}']".format(COLLISION_PLUGIN)
                    )
                ),
            )
            self.assertEqual(
                [
                    "{} {} 1.25 1.57 0 0".format(index * 10, index)
                    for index in range(6)
                ],
                [
                    plugin.findtext("init_pose")
                    for plugin in actor_plugins(root, ROS_PLUGIN)
                ],
            )
            self.assertEqual(
                ["5.1"] * 6,
                [
                    plugin.findtext("animation_factor")
                    for plugin in actor_plugins(root, ROS_PLUGIN)
                ],
            )
            assert_only_actor_collisions_removed(source, output)

    def test_actor_count_must_be_exactly_six(self):
        with tempfile.TemporaryDirectory() as directory:
            source = pathlib.Path(directory) / "official.world"
            output = pathlib.Path(directory) / "generated.world"
            make_world(source, actor_count=5)

            with self.assertRaisesRegex(ComplianceError, "6 个 actor"):
                generate_world(source, output)

            self.assertFalse(output.exists())

    def test_actor_names_must_be_nonempty_and_unique(self):
        mutations = {
            "empty": lambda actors: actors[0].set("name", ""),
            "duplicate": lambda actors: actors[1].set(
                "name", actors[0].get("name")
            ),
        }
        for name, mutate in mutations.items():
            with self.subTest(name=name), tempfile.TemporaryDirectory() as directory:
                source = pathlib.Path(directory) / "official.world"
                output = pathlib.Path(directory) / "generated.world"
                make_world(source)
                tree = ET.parse(str(source))
                mutate(tree.getroot().findall("./world/actor"))
                tree.write(str(source), encoding="utf-8", xml_declaration=True)

                with self.assertRaisesRegex(ComplianceError, "名称"):
                    generate_world(source, output)

                self.assertFalse(output.exists())

    def test_each_actor_requires_exactly_one_collision_and_ros_plugin(self):
        def remove_plugin(actor, filename):
            actor.remove(
                next(
                    plugin
                    for plugin in actor.findall("plugin")
                    if plugin.get("filename") == filename
                )
            )

        def duplicate_plugin(actor, filename):
            plugin = next(
                plugin
                for plugin in actor.findall("plugin")
                if plugin.get("filename") == filename
            )
            actor.append(copy.deepcopy(plugin))

        mutations = {
            "missing_collision": lambda actor: remove_plugin(
                actor, COLLISION_PLUGIN
            ),
            "duplicate_collision": lambda actor: duplicate_plugin(
                actor, COLLISION_PLUGIN
            ),
            "missing_ros": lambda actor: remove_plugin(actor, ROS_PLUGIN),
            "duplicate_ros": lambda actor: duplicate_plugin(actor, ROS_PLUGIN),
        }
        for name, mutate in mutations.items():
            with self.subTest(name=name), tempfile.TemporaryDirectory() as directory:
                source = pathlib.Path(directory) / "official.world"
                output = pathlib.Path(directory) / "generated.world"
                make_world(source)
                tree = ET.parse(str(source))
                mutate(tree.getroot().find("./world/actor"))
                tree.write(str(source), encoding="utf-8", xml_declaration=True)

                with self.assertRaisesRegex(ComplianceError, "插件"):
                    generate_world(source, output)

                self.assertFalse(output.exists())

    def test_existing_output_is_refused_without_overwriting(self):
        with tempfile.TemporaryDirectory() as directory:
            source = pathlib.Path(directory) / "official.world"
            output = pathlib.Path(directory) / "generated.world"
            make_world(source)
            output.write_text("keep-me", encoding="utf-8")

            with self.assertRaisesRegex(ComplianceError, "拒绝覆盖"):
                generate_world(source, output)

            self.assertEqual("keep-me", output.read_text(encoding="utf-8"))

    def test_malformed_xml_is_reported_without_output(self):
        with tempfile.TemporaryDirectory() as directory:
            source = pathlib.Path(directory) / "official.world"
            output = pathlib.Path(directory) / "generated.world"
            source.write_text("<sdf><world>", encoding="utf-8")

            with self.assertRaisesRegex(ComplianceError, "XML"):
                generate_world(source, output)

            self.assertFalse(output.exists())

    def test_post_write_validation_failure_removes_output(self):
        with tempfile.TemporaryDirectory() as directory:
            source = pathlib.Path(directory) / "official.world"
            output = pathlib.Path(directory) / "generated.world"
            make_world(source)
            with mock.patch(
                "competition_compliance.world.assert_only_actor_collisions_removed",
                side_effect=ComplianceError("forced validation failure"),
            ):
                with self.assertRaisesRegex(ComplianceError, "forced"):
                    generate_world(source, output)

            self.assertFalse(output.exists())


if __name__ == "__main__":
    unittest.main()

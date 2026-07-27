#!/usr/bin/env python3

import os
import pathlib
import shutil
import subprocess
import tempfile
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
BUILD_HELPER = ROOT / "scripts" / "build_xtdrone_actor_collisions.sh"
SOURCE_PARTS = ("sitl_config", "gazebo_plugin", "actor_collisions")


def official_source_dir():
    common_git_dir = subprocess.run(
        ["git", "rev-parse", "--git-common-dir"],
        cwd=ROOT,
        check=True,
        stdout=subprocess.PIPE,
        text=True,
    ).stdout.strip()
    common_git_path = pathlib.Path(common_git_dir)
    if not common_git_path.is_absolute():
        common_git_path = ROOT / common_git_path
    project_root = common_git_path.resolve().parent.parent
    return project_root / "XTDrone" / pathlib.Path(*SOURCE_PARTS)


def create_symlinked_source_tree(root, component_index):
    xtdrone = root / "XTDrone"
    real_component = root / "real-component"
    real_component.mkdir()

    if component_index == 0:
        xtdrone.symlink_to(real_component, target_is_directory=True)
        source = real_component / pathlib.Path(*SOURCE_PARTS)
    else:
        link_parent = xtdrone / pathlib.Path(*SOURCE_PARTS[: component_index - 1])
        link_parent.mkdir(parents=True)
        link_path = link_parent / SOURCE_PARTS[component_index - 1]
        link_path.symlink_to(real_component, target_is_directory=True)
        source = real_component / pathlib.Path(*SOURCE_PARTS[component_index:])

    source.mkdir(parents=True, exist_ok=True)
    official_source = official_source_dir()
    for file_name in ("ActorCollisionsPlugin.cc", "ActorCollisionsPlugin.hh"):
        shutil.copy2(official_source / file_name, source / file_name)
    return xtdrone


class ExternalPluginBuildTest(unittest.TestCase):
    def test_helper_has_exact_out_of_tree_build_contract(self):
        self.assertTrue(BUILD_HELPER.is_file(), "外部插件构建助手尚未创建")

        text = BUILD_HELPER.read_text(encoding="utf-8")
        self.assertTrue(text.startswith("#!/bin/bash\nset -euo pipefail\n"))
        self.assertIn('SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"', text)
        self.assertIn('WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"', text)
        self.assertIn('PROJECT_ROOT="$(cd "$WORKSPACE_DIR/.." && pwd)"', text)
        self.assertIn('XTDRONE_DIR="${XTDRONE_DIR:-$PROJECT_ROOT/XTDrone}"', text)
        self.assertIn(
            'SOURCE_DIR="$XTDRONE_DIR/sitl_config/gazebo_plugin/actor_collisions"',
            text,
        )
        self.assertIn('BUILD_DIR="$WORKSPACE_DIR/build/actor_collisions"', text)
        self.assertIn('OUTPUT_DIR="$WORKSPACE_DIR/devel/lib"', text)
        self.assertNotIn('cd "$SOURCE_DIR"', text)
        self.assertIn(
            'cmake -S "$SOURCE_DIR" -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release',
            text,
        )
        self.assertIn('cmake --build "$BUILD_DIR" --parallel', text)
        self.assertIn('chmod 0755 "$temp_output"', text)

    def test_tampered_official_source_is_rejected_before_cmake(self):
        self.assertTrue(BUILD_HELPER.is_file(), "外部插件构建助手尚未创建")

        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            workspace = root / "workspace"
            source = (
                root
                / "XTDrone"
                / "sitl_config"
                / "gazebo_plugin"
                / "actor_collisions"
            )
            fake_bin = root / "bin"
            marker = root / "cmake-called"
            (workspace / "scripts").mkdir(parents=True)
            source.mkdir(parents=True)
            fake_bin.mkdir()
            helper = workspace / "scripts" / BUILD_HELPER.name
            shutil.copy2(BUILD_HELPER, helper)
            (source / "ActorCollisionsPlugin.cc").write_text(
                "tampered source\n", encoding="utf-8"
            )
            (source / "ActorCollisionsPlugin.hh").write_text(
                "tampered header\n", encoding="utf-8"
            )
            fake_cmake = fake_bin / "cmake"
            fake_cmake.write_text(
                '#!/bin/sh\ntouch "$CMAKE_MARKER"\n', encoding="utf-8"
            )
            fake_cmake.chmod(0o755)
            environment = os.environ.copy()
            environment.update(
                {
                    "CMAKE_MARKER": str(marker),
                    "PATH": f"{fake_bin}:/usr/bin:/bin",
                    "XTDRONE_DIR": str(root / "XTDrone"),
                }
            )

            result = subprocess.run(
                [str(helper)],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                env=environment,
                timeout=10,
            )

            self.assertNotEqual(0, result.returncode)
            self.assertIn("哈希", result.stderr)
            self.assertIn("ActorCollisionsPlugin.cc", result.stderr)
            self.assertFalse(marker.exists(), "源码校验失败后不应调用 CMake")

    def test_symlinked_source_path_components_are_rejected_before_cmake(self):
        self.assertTrue(BUILD_HELPER.is_file(), "外部插件构建助手尚未创建")

        components = ("XTDrone", *SOURCE_PARTS)
        for component_index, component_name in enumerate(components):
            with self.subTest(component=component_name), tempfile.TemporaryDirectory() as directory:
                root = pathlib.Path(directory)
                workspace = root / "workspace"
                fake_bin = root / "bin"
                marker = root / "cmake-called"
                (workspace / "scripts").mkdir(parents=True)
                fake_bin.mkdir()
                helper = workspace / "scripts" / BUILD_HELPER.name
                shutil.copy2(BUILD_HELPER, helper)
                xtdrone = create_symlinked_source_tree(root, component_index)
                fake_cmake = fake_bin / "cmake"
                fake_cmake.write_text(
                    '#!/bin/sh\ntouch "$CMAKE_MARKER"\n', encoding="utf-8"
                )
                fake_cmake.chmod(0o755)
                environment = os.environ.copy()
                environment.update(
                    {
                        "CMAKE_MARKER": str(marker),
                        "PATH": f"{fake_bin}:/usr/bin:/bin",
                        "XTDRONE_DIR": str(xtdrone),
                    }
                )

                result = subprocess.run(
                    [str(helper)],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                    env=environment,
                    timeout=10,
                )

                self.assertNotEqual(0, result.returncode)
                self.assertIn(component_name, result.stderr)
                self.assertRegex(result.stderr, "符号链接|不安全")
                self.assertFalse(marker.exists(), "不安全路径不应触发 CMake")


if __name__ == "__main__":
    unittest.main()

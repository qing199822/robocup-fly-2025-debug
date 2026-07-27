#!/usr/bin/env python3

import os
import pathlib
import shutil
import subprocess
import tempfile
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
BUILD_HELPER = ROOT / "scripts" / "build_xtdrone_actor_collisions.sh"


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


if __name__ == "__main__":
    unittest.main()

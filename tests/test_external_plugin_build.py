#!/usr/bin/env python3

import base64
import gzip
import hashlib
import os
import pathlib
import shutil
import subprocess
import tempfile
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
BUILD_HELPER = ROOT / "scripts" / "build_xtdrone_actor_collisions.sh"
SOURCE_PARTS = ("sitl_config", "gazebo_plugin", "actor_collisions")
OFFICIAL_FILES = {
    "ActorCollisionsPlugin.cc": (
        "H4sIAAAAAAAAA51WbW/bRgz+rl/BesAgZUnUdV8GJTXgpm5qzLEDO2nRDYNxlmjrEFmn6U5xnCL/feTpxXJjp+v0xdbdQ/LhcyRP/pEDR3Chsk0ul7EB98KDN69//R3GGaYwVUUeIkzUXBkZavigijQSRqqUrNhwKENMNUZA65iDiRF6mQjpp9o5hk+YazKAN6evwWVAp9rqeGfsYqMKWIkNpMpAoZF8SA0LmSDgQ4iZAZlCqFZZIkVKXNbSxDZO5eWUfXypfKi5EQQXZJDR26INBGEq0vzExmSB76/X61NhCZ+qfOknJVT7w8FFfzTtnxDpyug2TVBryPGfQuaU8HwDIiNSoZgT1USsQeUgljnSnlFMep1LI9PlMWi1MGuRI7uJpDa5nBdmR7OaImXeBpBqIoVObwqDaQfe9aaD6TE7+Ty4+Ti+vYHPvcmkN7oZ9KcwnsDFePR+cDMYj+jtA/RGX+CPwej9MSApRnHwIcs5A6IpWU2MrHRTxB0KC1VS0hmGciFDSi1dFmKJsFT3mKeUEWSYr6TmU9VEMGI3iVxJYytDP8+LAzlHvuP8JNMwKSKEc7lMJaP9lTCx/wlDo/LfTuO42wYtxSPOlZ/FG03V5/cYZDEHIe/UwzQWGb6MulBJIpn+y7ChTO92ER1LoTHX10mxlOyl4ziFZmlSsUJNBUVyWWdnzuWfs0n/cjC96U9mV+P3/eHsenh7ORi5e315juP4P/o4e10Fwd5l13O+Ok//J8q9khEcCDVUInIr4YLgSkWYXJscZiv+Ry0QLYKgn+AKU2PXaYF5APg+XKKhls2UTA3XjbKlIzgO7YuCFuwLvIW5UtoEQbQhmWU4q0xmodDmvAluGXbdMjTNmDLIlch4IIQ1c9ChSPjEFta5JpQ2URCsRHZu/3AbcvfWlcpbJg6Cqlajbu3h7D+bXiuNbKgWC41G19QmKCIeGJz2QX7kbQHcwaQgmdGLyyKedD8KXQnrdiqbjucRhNWt9EMCkHylAcn93ODMgtcxz12X4Z5dKH2U4V7xso3XM9WIcjsN3zJm2wZg+UhjL4Xzc+hM72SWcTpYhrZzXBWmlbCovXbYwgqJaZScNe6qLEoalMUIHw5mwk+oUhrABdYrT9WvlYQbte2sfXLdnbzsMdUi7NOAQ+O+/G0gu7sT6WBFNa62OVRp/cV0/yYvFrBN6DvMMqUPE+PNF3lV5Vq72ZKq6rchxfvfivwjh8U2Nhe+fFw6Nm1Kij8T6g6Csv+tG57J2m3VN/XPgGjb5mE0NQpVcRjbhiqo+jDlG1pvC01bO8aedNnUrU+YVayYneIqMxu3kW5bSnXQ61yFfJ2KJNltXM2R61XLyJo8S21b+EFFhtLbDtY69tc9RduYlsrSortbpXUWC5lGLht58OotNLnR4qGimFcX6HeGbXm5BUEzdOuLt+s2/krd2kwtgkI3XKvJfEefG0VO33YIsbin2c80QNuLvEFyXjU7rxWkXjvpTimCfES3tcKvHn2e7LSRt7d/qqL+RrN69YBmrfwoOleTFMkEE/oSukfun105dhvnl7b5PlvP2+2qsk+enH8BIKyD7bULAAA=",
        "e15f07b4a9cc19db1a05dd1aafd1b81557b2badf728cc28d666500034b34e499",
    ),
    "ActorCollisionsPlugin.hh": (
        "H4sIAAAAAAAAA51UXW/bNhR916+4cB6WBq7U9WlwhwGq4zTCXCuwnHXdBwxKupK4USRLUlG8of99l5SGOGuflpfAvPece87lEZOrCK5grfTJ8LZzcLl+Aa9fffsd5BolFGowFcJelcrxysKNGmTNHFeSUB645RVKizXQORpwHUKqWUX/5soSfkJjCQCv41dw6RsWc2nx4o2nOKkBenYCqRwMFomDW2i4QMDHCrUDLqFSvRacSdIycteFOTNL7Dk+zhyqdIzaGQE0/WrOG4G5WbT/65zTqyQZxzFmQXCsTJuIqdUm22y92RWblyR6Bt1LgdaCwU8DN2S4PAHTJKpiJUkVbARlgLUGqeaUFz0a7rhsl2BV40Zm0NPU3DrDy8E929m/Esn5eQNtjUlYpAVkxQLepkVWLD3Jh+xwm98f4EO636e7Q7YpIN/DOt9dZ4cs39GvG0h3H+HHbHe9BKSN0Rx81MY7IJncbxPrsLoC8ZmERk2SrMaKN7wia7IdWIvQqgc0khyBRtNz62/VksDa0wjecxeSYb/05QdFV0kUXfCGag28S3/ZvM2Pm5/T93fbTXG8296/y3bFMV0f8v06326zwvuYjo+3t8fogmBc4v9A0lBZiaFG+L5lf2GpEkpTr2RyJ4aWy7jrfogiyXq0lANyGXqivyOAJEngt9JwEnzwodQBACj9lVvKmBB8WsJ0UWnllFlOGwlgpf1CmBBzVgjEwFZM+CU2zLf7btCKFq+axqLz2UHK4xzemSgwf2OhVI9nY+O5uj4XQrNGZf70kiahFKPyD6ycjSG/3kDHvAg2OPWSouYbZpYGmRuMDwNzsJhrduGNTQQwdvQkcOe/RnJCQilPLoaUzAcrIxdiJtOMkuY6o4Y2fK3e36xi6pj7fPoC+Hhmi/SLOgSRZuMjo7QizIf0LjwxVMKPCct52sF0rbACPZT0ecJ7VaOYDgni7/XZza4J4szgOUJpQq2+znpJT9Z/CbaK1SHs0xKmkMRPXZoZ1v/K5e9w7L0UuFNcOv+BqACjOkoHofZ1mK2bL0BhCiWiuL4BFNgThY2fGXjgxg1MwIPidRB5qbuTpVd8tZpW4sysiF6oulmtNhNNOKcD/zrD5zfR5+gCZc2bfwDgb2ERKgYAAA==",
        "78db47b17157eeb97676fc0ceecc95662dd1a8018c3730c492962ca431b61c29",
    ),
    "CMakeLists.txt": (
        "H4sIAAAAAAAAA22PMU/DMBSEd/8Kq+rgLB2YurqJW6yGBhyKCsuTa5voKY4DTjJA1f9OCgKkKuu9u3vfmUbXDhoM2AwNRPc+YHSWPQlVymJHbxZLuuaPPAehVKESQl4xWHjTptaVY5X+dMeWKvGwl0pkCcFg/GAd2LHF9G1E17H5acNfxKoAuUvzfSYgk6o8J8RjqKeNuVwprp5/jZ3rWXrHtwLSwwHWOd+UdDY/XUln+pf/12YjsbYWPB6jjh+MX36lrffYYRu6ez9UGGh5y0d6OnlcGJOQXsfK9fBN/FN14Z0uu1ohxTjhC/1Z3ZtmAQAA",
        "f38958df562a9f66f435c42e831f2e2606a86b6b7287ad0eb6b8cbb7e4d03b28",
    ),
}


def write_official_source(source):
    source.mkdir(parents=True, exist_ok=True)
    for file_name, (encoded, expected_hash) in OFFICIAL_FILES.items():
        content = gzip.decompress(base64.b64decode(encoded))
        if hashlib.sha256(content).hexdigest() != expected_hash:
            raise AssertionError(f"invalid embedded fixture: {file_name}")
        (source / file_name).write_bytes(content)


def prepare_case(root):
    workspace = root / "workspace"
    source = root / "XTDrone" / pathlib.Path(*SOURCE_PARTS)
    fake_bin = root / "bin"
    (workspace / "scripts").mkdir(parents=True)
    fake_bin.mkdir()
    helper = workspace / "scripts" / BUILD_HELPER.name
    shutil.copy2(BUILD_HELPER, helper)
    write_official_source(source)
    return workspace, root / "XTDrone", fake_bin, helper


def write_fake_cmake(fake_bin, body):
    fake_cmake = fake_bin / "cmake"
    fake_cmake.write_text(f"#!/bin/bash\nset -euo pipefail\n{body}", encoding="utf-8")
    fake_cmake.chmod(0o755)


def helper_environment(fake_bin, xtdrone, marker, **extra):
    environment = os.environ.copy()
    environment.update(
        {
            "CMAKE_MARKER": str(marker),
            "PATH": f"{fake_bin}:{environment['PATH']}",
            "XTDRONE_DIR": str(xtdrone),
            **{key: str(value) for key, value in extra.items()},
        }
    )
    return environment


def run_helper(helper, environment):
    return subprocess.run(
        [str(helper)],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        env=environment,
        timeout=15,
    )


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
    write_official_source(source)
    return xtdrone


HAPPY_CMAKE = r'''touch "$CMAKE_MARKER"
if [ "${1:-}" = "--build" ]; then
    printf 'actor-plugin\n' > "$2/libActorCollisionsPlugin.so"
fi
'''


class ExternalPluginBuildTest(unittest.TestCase):
    def test_helper_has_exact_out_of_tree_build_contract(self):
        text = BUILD_HELPER.read_text(encoding="utf-8")
        self.assertTrue(text.startswith("#!/bin/bash\nset -euo pipefail\n"))
        self.assertIn('SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"', text)
        self.assertIn('WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"', text)
        self.assertIn('PROJECT_ROOT="$(cd "$WORKSPACE_DIR/.." && pwd)"', text)
        self.assertIn('XTDRONE_DIR="${XTDRONE_DIR:-$PROJECT_ROOT/XTDrone}"', text)
        self.assertIn('SOURCE_DIR="$XTDRONE_DIR/sitl_config/gazebo_plugin/actor_collisions"', text)
        self.assertIn('BUILD_DIR="$WORKSPACE_DIR/build/actor_collisions"', text)
        self.assertIn('OUTPUT_DIR="$WORKSPACE_DIR/devel/lib"', text)
        self.assertNotIn('cd "$SOURCE_DIR"', text)
        self.assertIn('cmake -S "$SOURCE_DIR" -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release', text)
        self.assertIn('cmake --build "$BUILD_DIR" --parallel', text)
        self.assertIn("f38958df562a9f66f435c42e831f2e2606a86b6b7287ad0eb6b8cbb7e4d03b28", text)
        self.assertIn('LOCK_FILE="$WORKSPACE_DIR/build/.actor-collisions.lock"', text)
        self.assertIn("flock", text)

    def test_tampered_plugin_source_is_rejected_before_cmake(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            _, xtdrone, fake_bin, helper = prepare_case(root)
            marker = root / "cmake-called"
            source = xtdrone / pathlib.Path(*SOURCE_PARTS)
            (source / "ActorCollisionsPlugin.cc").write_text("tampered\n", encoding="utf-8")
            write_fake_cmake(fake_bin, 'touch "$CMAKE_MARKER"\n')
            result = run_helper(helper, helper_environment(fake_bin, xtdrone, marker))
            self.assertNotEqual(0, result.returncode)
            self.assertIn("哈希", result.stderr)
            self.assertFalse(marker.exists())

    def test_tampered_cmake_is_rejected_before_cmake(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            _, xtdrone, fake_bin, helper = prepare_case(root)
            marker = root / "cmake-called"
            source = xtdrone / pathlib.Path(*SOURCE_PARTS)
            (source / "CMakeLists.txt").write_text("tampered\n", encoding="utf-8")
            write_fake_cmake(fake_bin, 'touch "$CMAKE_MARKER"\n')
            result = run_helper(helper, helper_environment(fake_bin, xtdrone, marker))
            self.assertNotEqual(0, result.returncode)
            self.assertIn("CMakeLists.txt", result.stderr)
            self.assertIn("哈希", result.stderr)
            self.assertFalse(marker.exists())

    def test_symlinked_source_path_components_are_rejected_before_cmake(self):
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
                write_fake_cmake(fake_bin, 'touch "$CMAKE_MARKER"\n')
                result = run_helper(helper, helper_environment(fake_bin, xtdrone, marker))
                self.assertNotEqual(0, result.returncode)
                self.assertIn(component_name, result.stderr)
                self.assertRegex(result.stderr, "符号链接|不安全")
                self.assertFalse(marker.exists())

    def test_trailing_slash_does_not_hide_xtdrone_symlink(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            workspace = root / "workspace"
            fake_bin = root / "bin"
            marker = root / "cmake-called"
            (workspace / "scripts").mkdir(parents=True)
            fake_bin.mkdir()
            helper = workspace / "scripts" / BUILD_HELPER.name
            shutil.copy2(BUILD_HELPER, helper)
            xtdrone = create_symlinked_source_tree(root, 0)
            write_fake_cmake(fake_bin, HAPPY_CMAKE)
            environment = helper_environment(fake_bin, f"{xtdrone}/", marker)
            result = run_helper(helper, environment)
            self.assertNotEqual(0, result.returncode)
            self.assertRegex(result.stderr, "符号链接|不安全")
            self.assertFalse(marker.exists())

    def test_destination_symlinks_are_rejected_without_external_writes(self):
        cases = ("build", "actor_collisions", "devel", "lib")
        for unsafe_component in cases:
            with self.subTest(component=unsafe_component), tempfile.TemporaryDirectory() as directory:
                root = pathlib.Path(directory)
                workspace, xtdrone, fake_bin, helper = prepare_case(root)
                outside = root / "outside"
                outside.mkdir()
                sentinel = outside / "sentinel"
                sentinel.write_text("unchanged\n", encoding="utf-8")
                if unsafe_component == "build":
                    (workspace / "build").symlink_to(outside, target_is_directory=True)
                elif unsafe_component == "actor_collisions":
                    (workspace / "build").mkdir()
                    (workspace / "build" / "actor_collisions").symlink_to(outside, target_is_directory=True)
                elif unsafe_component == "devel":
                    (workspace / "devel").symlink_to(outside, target_is_directory=True)
                else:
                    (workspace / "devel").mkdir()
                    (workspace / "devel" / "lib").symlink_to(outside, target_is_directory=True)
                marker = root / "cmake-called"
                write_fake_cmake(fake_bin, HAPPY_CMAKE)
                result = run_helper(helper, helper_environment(fake_bin, xtdrone, marker))
                self.assertNotEqual(0, result.returncode)
                self.assertRegex(result.stderr, "符号链接|不安全")
                self.assertFalse(marker.exists())
                self.assertEqual(["sentinel"], sorted(path.name for path in outside.iterdir()))
                self.assertEqual("unchanged\n", sentinel.read_text(encoding="utf-8"))

    def test_happy_publish_supports_workspace_paths_with_spaces(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory) / "case with spaces"
            root.mkdir()
            workspace, xtdrone, fake_bin, helper = prepare_case(root)
            marker = root / "cmake-called"
            write_fake_cmake(fake_bin, HAPPY_CMAKE)
            result = run_helper(helper, helper_environment(fake_bin, xtdrone, marker))
            output = workspace / "devel" / "lib" / "libActorCollisionsPlugin.so"
            self.assertEqual(0, result.returncode, result.stderr)
            self.assertEqual(b"actor-plugin\n", output.read_bytes())
            self.assertEqual(0o755, output.stat().st_mode & 0o777)

    def test_cmake_failure_preserves_old_output_and_leaves_no_temp(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            workspace, xtdrone, fake_bin, helper = prepare_case(root)
            output_dir = workspace / "devel" / "lib"
            output_dir.mkdir(parents=True)
            output = output_dir / "libActorCollisionsPlugin.so"
            output.write_bytes(b"old-plugin\n")
            marker = root / "cmake-called"
            write_fake_cmake(
                fake_bin,
                'touch "$CMAKE_MARKER"\nif [ "${1:-}" = "--build" ]; then exit 7; fi\n',
            )
            result = run_helper(helper, helper_environment(fake_bin, xtdrone, marker))
            self.assertNotEqual(0, result.returncode)
            self.assertEqual(b"old-plugin\n", output.read_bytes())
            self.assertEqual([], list(output_dir.glob(".libActorCollisionsPlugin.so.*")))

    def test_concurrent_builds_are_serialized_through_publish(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            workspace, xtdrone, fake_bin, helper = prepare_case(root)
            marker = root / "cmake-called"
            state = root / "cmake-state"
            state.mkdir()
            write_fake_cmake(
                fake_bin,
                r'''touch "$CMAKE_MARKER"
if ! mkdir "$CMAKE_STATE/active" 2>/dev/null; then
    touch "$CMAKE_STATE/overlap"
    exit 9
fi
cleanup() { rmdir "$CMAKE_STATE/active"; }
trap cleanup EXIT
sleep 0.15
if [ "${1:-}" = "--build" ]; then
    printf 'actor-plugin\n' > "$2/libActorCollisionsPlugin.so"
fi
''',
            )
            environment = helper_environment(
                fake_bin, xtdrone, marker, CMAKE_STATE=state
            )
            processes = [
                subprocess.Popen(
                    [str(helper)],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                    env=environment,
                )
                for _ in range(2)
            ]
            results = [process.communicate(timeout=15) for process in processes]
            self.assertEqual([0, 0], [process.returncode for process in processes], results)
            self.assertFalse((state / "overlap").exists(), results)
            output = workspace / "devel" / "lib" / "libActorCollisionsPlugin.so"
            self.assertEqual(b"actor-plugin\n", output.read_bytes())
            self.assertEqual([], list(output.parent.glob(".libActorCollisionsPlugin.so.*")))

    def test_missing_flock_fails_in_chinese_before_workspace_mutation(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            workspace, xtdrone, _, helper = prepare_case(root)
            fake_bin = root / "minimal-bin"
            fake_bin.mkdir()
            (fake_bin / "dirname").symlink_to("/usr/bin/dirname")
            environment = os.environ.copy()
            environment.update({"PATH": str(fake_bin), "XTDRONE_DIR": str(xtdrone)})
            result = run_helper(helper, environment)
            self.assertNotEqual(0, result.returncode)
            self.assertIn("flock", result.stderr)
            self.assertIn("缺少", result.stderr)
            self.assertFalse((workspace / "build").exists())


if __name__ == "__main__":
    unittest.main()

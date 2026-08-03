#!/usr/bin/env python3

import os
import pathlib
import shutil
import subprocess
import tempfile
import textwrap
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
FULL_VERIFIER = ROOT / "scripts" / "verify_competition_clean.sh"
SMOKE_VERIFIER = ROOT / "scripts" / "smoke_competition_clean.sh"


def write_executable(path, text):
    path.write_text(textwrap.dedent(text).lstrip(), encoding="utf-8")
    path.chmod(0o755)


class VerificationScriptsContractTest(unittest.TestCase):
    def test_full_verifier_runs_pre_tests_build_catkin_and_post_in_order(self):
        text = FULL_VERIFIER.read_text(encoding="utf-8")
        first_verify = text.index('run_compliance_verifier "$STATIC_EVIDENCE"')
        unit_tests = text.index("python3 -m unittest discover -s")
        release_build = text.index("catkin_make -DCMAKE_BUILD_TYPE=Release")
        plugin_build = text.index("build_xtdrone_actor_collisions.sh")
        catkin_tests = text.index("catkin_make run_tests")
        catkin_results = text.index("\ncatkin_test_results\n")
        second_verify = text.index('run_compliance_verifier "$POST_BUILD_EVIDENCE"')
        self.assertEqual(
            sorted(
                (
                    first_verify,
                    unit_tests,
                    release_build,
                    plugin_build,
                    catkin_tests,
                    catkin_results,
                    second_verify,
                )
            ),
            [
                first_verify,
                unit_tests,
                release_build,
                plugin_build,
                catkin_tests,
                catkin_results,
                second_verify,
            ],
        )

    def test_full_verifier_uses_canonical_policy_and_all_runtime_roots(self):
        text = FULL_VERIFIER.read_text(encoding="utf-8")
        for fragment in (
            "verify_full.py",
            "--px4-dir",
            "--xtdrone-dir",
            "--gazebo-models-dir",
            "--xtdrone-pythonpath",
            "config/official_manifest.json",
            "config/ownership.json",
            "static-compliance.json",
            "post-build-compliance.json",
        ):
            self.assertIn(fragment, text)

    def test_ros_environment_is_loaded_before_ros_commands_are_checked(self):
        full_text = FULL_VERIFIER.read_text(encoding="utf-8")
        smoke_text = SMOKE_VERIFIER.read_text(encoding="utf-8")
        self.assertLess(
            full_text.index('source "$ROS_SETUP_FILE"'),
            full_text.index("require_command catkin_make"),
        )
        self.assertLess(
            smoke_text.index('source "$ROS_SETUP_FILE"'),
            smoke_text.index('require_command "$command_name"'),
        )

    def test_catkin_test_logs_stay_inside_team_workspace(self):
        text = FULL_VERIFIER.read_text(encoding="utf-8")
        assignment = text.index('ROS_LOG_DIR="$WORKSPACE_DIR/logs/verification"')
        export = text.index("export ROS_LOG_DIR")
        catkin_tests = text.index("catkin_make run_tests")
        self.assertLess(assignment, export)
        self.assertLess(export, catkin_tests)

    def test_smoke_contract_checks_six_distinct_vehicles(self):
        text = SMOKE_VERIFIER.read_text(encoding="utf-8")
        self.assertIn("for id in $(seq 0 5)", text)
        for suffix in (
            "mavros/state",
            "mavros/local_position/pose",
            "realsense/depth_camera/color/image_raw",
            "realsense/depth_camera/depth/image_raw",
            "realsense/depth_camera/color/camera_info",
            "safety/status",
        ):
            self.assertIn(suffix, text)
        self.assertIn('/typhoon_h480_${id}_communication', text)
        self.assertIn('/typhoon_h480_${id}/safety_filter', text)
        self.assertIn("check_final_control_publishers.py", text)
        self.assertIn("base_link", text)
        self.assertIn("depth_camera_base", text)


class VerificationScriptsBehaviorTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = pathlib.Path(self.temporary.name)

    def tearDown(self):
        self.temporary.cleanup()

    def prepare_full_verifier(self):
        project = self.root / "project"
        workspace = project / "workspace"
        scripts = workspace / "scripts"
        package = workspace / "src" / "competition_compliance"
        fake_bin = self.root / "bin"
        for path in (
            scripts,
            package / "scripts",
            package / "config",
            fake_bin,
            project / "PX4_Firmware",
            project / "XTDrone",
            project / "gazebo_models",
            project / ".xtdrone-python",
        ):
            path.mkdir(parents=True, exist_ok=True)
        shutil.copy2(FULL_VERIFIER, scripts / FULL_VERIFIER.name)
        (package / "scripts" / "verify_full.py").touch()
        (package / "config" / "official_manifest.json").write_text("{}\n")
        (package / "config" / "ownership.json").write_text("{}\n")
        ros_setup = self.root / "ros-setup.bash"
        ros_setup.write_text(': "$ROS_SETUP_PROBE"\n', encoding="utf-8")
        log = self.root / "commands.log"
        catkin_toplevel = self.root / "catkin-toplevel.cmake"
        catkin_toplevel.write_text("# generated catkin toplevel\n", encoding="utf-8")
        write_executable(
            fake_bin / "python3",
            r'''
            #!/bin/bash
            set -euo pipefail
            if [[ "${1:-}" == */verify_full.py ]]; then
                if [ -L "$TEST_WORKSPACE/src/CMakeLists.txt" ]; then
                    echo "source symlink rejected" >&2
                    exit 24
                fi
                evidence=""
                previous=""
                for argument in "$@"; do
                    if [ "$previous" = "--evidence" ]; then evidence="$argument"; fi
                    previous="$argument"
                done
                [ -n "$evidence" ]
                if [ -e "$evidence" ]; then
                    echo "refusing existing evidence" >&2
                    exit 23
                fi
                mkdir -p "$(dirname "$evidence")"
                printf '{"status":"pass"}\n' > "$evidence"
                printf 'verify %s\n' "$(basename "$evidence")" >> "$COMMAND_LOG"
            else
                printf 'unittest %s\n' "$*" >> "$COMMAND_LOG"
            fi
            ''',
        )
        write_executable(
            fake_bin / "catkin_make",
            r'''
            #!/bin/bash
            printf 'catkin_make %s\n' "$*" >> "$COMMAND_LOG"
            if [ "${1:-}" = "-DCMAKE_BUILD_TYPE=Release" ]; then
                ln -s "$CATKIN_TOPLEVEL" "$TEST_WORKSPACE/src/CMakeLists.txt"
            fi
            ''',
        )
        write_executable(
            fake_bin / "catkin_test_results",
            '#!/bin/bash\nprintf "catkin_test_results\\n" >> "$COMMAND_LOG"\n',
        )
        write_executable(
            scripts / "build_xtdrone_actor_collisions.sh",
            '#!/bin/bash\nprintf "plugin_build\\n" >> "$COMMAND_LOG"\n',
        )
        environment = os.environ.copy()
        environment.update(
            {
                "PATH": f"{fake_bin}:{environment['PATH']}",
                "ROS_SETUP_FILE": str(ros_setup),
                "COMMAND_LOG": str(log),
                "CATKIN_TOPLEVEL": str(catkin_toplevel),
                "TEST_WORKSPACE": str(workspace),
            }
        )
        return workspace, environment, log

    def test_full_verifier_is_rerunnable_and_preserves_unrelated_evidence(self):
        workspace, environment, log = self.prepare_full_verifier()
        unrelated = workspace / "competition-artifacts" / "keep-me.json"
        unrelated.parent.mkdir()
        unrelated.write_text("keep\n", encoding="utf-8")
        for _run in range(2):
            result = subprocess.run(
                [str(workspace / "scripts" / FULL_VERIFIER.name)],
                cwd=self.root,
                env=environment,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=10,
            )
            self.assertEqual(0, result.returncode, result.stderr)
        self.assertEqual("keep\n", unrelated.read_text(encoding="utf-8"))
        self.assertEqual(
            [
                "verify static-compliance.json",
                "unittest -m unittest discover -s "
                f"{workspace}/tests -p test_*.py",
                "catkin_make -DCMAKE_BUILD_TYPE=Release",
                "plugin_build",
                "catkin_make run_tests",
                "catkin_test_results",
                "verify post-build-compliance.json",
            ]
            * 2,
            log.read_text(encoding="utf-8").splitlines(),
        )

    def prepare_smoke(
        self,
        failure_kind="",
        tf_output="- Translation: [0.090, 0.000, -0.040]",
    ):
        workspace = self.root / "workspace"
        scripts = workspace / "scripts"
        fake_bin = self.root / "bin"
        (workspace / "devel").mkdir(parents=True)
        scripts.mkdir(parents=True)
        fake_bin.mkdir(parents=True)
        shutil.copy2(SMOKE_VERIFIER, scripts / SMOKE_VERIFIER.name)
        ros_setup = self.root / "ros-setup.bash"
        ros_setup.write_text(': "$ROS_SETUP_PROBE"\n', encoding="utf-8")
        (workspace / "devel" / "setup.bash").write_text(
            ': "$WORKSPACE_SETUP_PROBE"\n', encoding="utf-8"
        )
        calls = self.root / "smoke-calls.log"
        write_executable(
            fake_bin / "rostopic",
            r'''
            #!/bin/bash
            topic="${@: -1}"
            printf 'topic %s\n' "$topic" >> "$SMOKE_CALLS"
            if [ "${FAIL_KIND:-}" = topic ] && [[ "$topic" == *typhoon_h480_2* ]]; then
                exit 1
            fi
            ''',
        )
        write_executable(
            fake_bin / "rosnode",
            r'''
            #!/bin/bash
            printf 'nodes\n' >> "$SMOKE_CALLS"
            for id in $(seq 0 5); do
                if [ "${FAIL_KIND:-}" != node ] || [ "$id" != 2 ]; then
                    echo "/typhoon_h480_${id}_communication"
                fi
                echo "/typhoon_h480_${id}/safety_filter"
            done
            ''',
        )
        write_executable(
            fake_bin / "python3",
            r'''
            #!/bin/bash
            printf 'publisher_guard %s\n' "$*" >> "$SMOKE_CALLS"
            if [ "${FAIL_KIND:-}" = publisher ]; then exit 1; fi
            echo "PASS final control topics have one safety_filter publisher each"
            ''',
        )
        write_executable(
            fake_bin / "rosrun",
            r'''
            #!/bin/bash
            printf 'tf %s\n' "$*" >> "$SMOKE_CALLS"
            if [ "${FAIL_KIND:-}" = tf ]; then exit 1; fi
            printf '%s\n' "$TF_OUTPUT"
            ''',
        )
        environment = os.environ.copy()
        environment.update(
            {
                "PATH": f"{fake_bin}:{environment['PATH']}",
                "ROS_SETUP_FILE": str(ros_setup),
                "LOG_DIR": str(self.root / "reports"),
                "SMOKE_CALLS": str(calls),
                "SMOKE_TIMEOUT_SECONDS": "1",
                "FAIL_KIND": failure_kind,
                "TF_OUTPUT": tf_output,
            }
        )
        return workspace, environment, calls

    def run_smoke(self, failure_kind="", tf_output=None):
        if tf_output is None:
            workspace, environment, calls = self.prepare_smoke(failure_kind)
        else:
            workspace, environment, calls = self.prepare_smoke(
                failure_kind, tf_output
            )
        result = subprocess.run(
            [str(workspace / "scripts" / SMOKE_VERIFIER.name)],
            cwd=self.root,
            env=environment,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=10,
        )
        reports = list((self.root / "reports").glob("smoke-*.log"))
        self.assertEqual(1, len(reports))
        return result, calls.read_text(encoding="utf-8"), reports[0].read_text(
            encoding="utf-8"
        )

    def test_smoke_accepts_ros_noetic_tf_output_and_writes_pass_report(self):
        result, calls, report = self.run_smoke()
        self.assertEqual(0, result.returncode, result.stderr)
        for vehicle_id in range(6):
            self.assertEqual(6, calls.count(f"typhoon_h480_{vehicle_id}/"))
            self.assertIn(
                f"PASS node /typhoon_h480_{vehicle_id}_communication", report
            )
            self.assertIn(
                f"PASS node /typhoon_h480_{vehicle_id}/safety_filter", report
            )
        self.assertIn("publisher_guard", calls)
        self.assertIn(
            "PASS final control topics have one safety_filter publisher each",
            report,
        )
        self.assertIn("tf tf_echo base_link depth_camera_base", calls)
        self.assertTrue(
            report.rstrip().endswith(
                "PASS competition-clean six-vehicle smoke"
            )
        )

    def test_smoke_keeps_accepting_translation_without_list_marker(self):
        result, _calls, report = self.run_smoke(
            tf_output="Translation: [0.090, 0.000, -0.040]"
        )
        self.assertEqual(0, result.returncode, result.stderr)
        self.assertIn("PASS TF base_link -> depth_camera_base", report)

    def test_smoke_missing_topic_fails_fast(self):
        result, calls, report = self.run_smoke("topic")
        self.assertNotEqual(0, result.returncode)
        self.assertIn("FAIL topic /typhoon_h480_2/", report)
        self.assertNotIn("typhoon_h480_3/", calls)
        self.assertNotIn("PASS competition-clean six-vehicle smoke", report)

    def test_smoke_missing_exact_node_fails_fast(self):
        result, calls, report = self.run_smoke("node")
        self.assertNotEqual(0, result.returncode)
        self.assertIn("FAIL node /typhoon_h480_2_communication", report)
        self.assertNotIn("typhoon_h480_3/", calls)
        self.assertNotIn("PASS competition-clean six-vehicle smoke", report)

    def test_smoke_missing_tf_is_nonzero_and_never_reports_success(self):
        result, calls, report = self.run_smoke("tf")
        self.assertNotEqual(0, result.returncode)
        self.assertIn("tf tf_echo base_link depth_camera_base", calls)
        self.assertIn("FAIL TF base_link -> depth_camera_base", report)
        self.assertNotIn("PASS competition-clean six-vehicle smoke", report)

    def test_smoke_invalid_final_publisher_is_nonzero(self):
        result, calls, report = self.run_smoke("publisher")
        self.assertNotEqual(0, result.returncode)
        self.assertIn("publisher_guard", calls)
        self.assertIn("FAIL final control publisher ownership", report)
        self.assertNotIn("PASS competition-clean six-vehicle smoke", report)

    def test_smoke_rejects_symlinked_log_parent_without_external_write(self):
        workspace, environment, _calls = self.prepare_smoke()
        outside = self.root / "outside"
        outside.mkdir()
        linked_parent = self.root / "linked-logs"
        linked_parent.symlink_to(outside, target_is_directory=True)
        environment["LOG_DIR"] = str(linked_parent / "competition-clean")
        result = subprocess.run(
            [str(workspace / "scripts" / SMOKE_VERIFIER.name)],
            cwd=self.root,
            env=environment,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=10,
        )
        self.assertNotEqual(0, result.returncode)
        self.assertIn("符号链接", result.stderr)
        self.assertEqual([], list(outside.iterdir()))


if __name__ == "__main__":
    unittest.main()

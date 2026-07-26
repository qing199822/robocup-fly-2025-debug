#!/usr/bin/env python3

import os
import pathlib
import re
import signal
import subprocess
import tempfile
import textwrap
import time
import unittest


ROOT = pathlib.Path(__file__).parents[1]


class LauncherHarness:
    def __init__(self):
        self._temporary_directory = tempfile.TemporaryDirectory()
        self.root = pathlib.Path(self._temporary_directory.name)
        self.workspace = self.root / "workspace"
        self.state = self.root / "state"
        self.bin = self.root / "bin"
        self.px4 = self.root / "PX4_Firmware"
        self.xtdrone = self.root / "XTDrone"
        self.gazebo_models = self.root / "gazebo_models"
        self.pythonpath = self.root / "pythonpath"
        for directory in (self.workspace, self.state, self.bin):
            directory.mkdir(parents=True)

        (self.workspace / "1.sh").write_text(
            (ROOT / "1.sh").read_text(encoding="utf-8"), encoding="utf-8"
        )
        self._create_required_files()
        self._create_command_stubs()

        self.env = os.environ.copy()
        self.env.update(
            {
                "PATH": f"{self.bin}:/usr/bin:/bin",
                "FAKE_BIN": str(self.bin),
                "STATE_DIR": str(self.state),
                "PX4_DIR": str(self.px4),
                "XTDRONE_DIR": str(self.xtdrone),
                "GAZEBO_MODELS_DIR": str(self.gazebo_models),
                "XTDRONE_PYTHONPATH": str(self.pythonpath),
                "XTDRONE_PYTHON": str(self.bin / "communication-python"),
                "COMPLIANCE_PYTHON": str(self.bin / "preflight-python"),
                "ROS_SETUP_FILE": str(self.root / "ros-setup.bash"),
                "TEE_BIN": str(self.bin / "tee"),
                "READY_TIMEOUT_SECONDS": "5",
                "COMMUNICATION_TIMEOUT_SECONDS": "3",
                "CAMERA_TIMEOUT_SECONDS": "2",
                "CLEANUP_GRACE_SECONDS": "1",
                "HELPER_SURVIVAL_SECONDS": "1",
                "OFFBOARD_WARMUP_SECONDS": "0",
                "SIMULATION_MODE": "success",
                "COMMUNICATION_MODE": "success",
                "CAMERA_MODE": "success",
                "HELPER_MODE": "success",
                "MISSION_MODE": "0",
                "TEE_MODE": "success",
                "IGNORE_TERM": "0",
                "DISPLAY": ":99",
            }
        )

    def _write(self, relative_path, content="", executable=False):
        path = self.root / relative_path
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding="utf-8")
        if executable:
            path.chmod(0o755)
        return path

    def _workspace_file(self, relative_path, content="", executable=False):
        path = self.workspace / relative_path
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding="utf-8")
        if executable:
            path.chmod(0o755)
        return path

    def _create_required_files(self):
        self._write("ros-setup.bash", "true\n")
        self._write(
            "PX4_Firmware/Tools/setup_gazebo.bash",
            'export PATH="$FAKE_BIN:$PATH"\n',
        )
        for path in (
            "PX4_Firmware/build/px4_sitl_default/bin/px4",
            "PX4_Firmware/Tools/sitl_gazebo/worlds/robocup.world",
            "XTDrone/sitl_config/models/walker/walk_0.dae",
            "XTDrone/communication/multirotor_communication.py",
            "XTDrone/sitl_config/models/typhoon_h480_realsense/typhoon_h480_realsense.sdf",
            "XTDrone/sitl_config/models/realsense_camera/realsense_camera.sdf",
            "gazebo_models/cessna/model.sdf",
            "pythonpath/pyquaternion/__init__.py",
        ):
            self._write(path)

        self._workspace_file("devel/setup.bash", 'export PATH="$FAKE_BIN:$PATH"\n')
        self._workspace_file(
            "scripts/graphics_environment.sh",
            "ensure_graphics_environment() { return 0; }\n",
        )
        for path in (
            "robocup_zzufly.launch",
            "src/competition_compliance/scripts/prepare_model.py",
            "src/competition_compliance/config/official_manifest.json",
            "src/competition_compliance/config/sensor_mount.yaml",
            "devel/lib/libActorCollisionsPlugin.so",
            "devel/lib/libros_actor_cmd_pose_plugin.so",
        ):
            self._workspace_file(path)

        helper = textwrap.dedent(
            """\
            #!/bin/bash
            echo "$$" >> "$STATE_DIR/helper_pids"
            if [ "$HELPER_MODE" = fail ]; then
                exit 31
            fi
            if [ "$IGNORE_TERM" = 1 ]; then
                trap '' TERM
            else
                trap 'exit 0' TERM
            fi
            while :; do sleep 1; done
            """
        )
        self._workspace_file("src/yolo/multi_yolo_detecting.sh", helper, True)
        self._workspace_file("src/yolo/multi_solving.sh", helper, True)

    def _create_command_stubs(self):
        self._write(
            "bin/preflight-python",
            textwrap.dedent(
                """\
                #!/bin/bash
                touch "$STATE_DIR/preflight"
                output=""
                while [ "$#" -gt 0 ]; do
                    if [ "$1" = --output ]; then
                        output="$2"
                        shift 2
                    else
                        shift
                    fi
                done
                mkdir -p "$(dirname "$output")"
                : > "$output"
                dirname "$output" > "$STATE_DIR/run_tmp"
                """
            ),
            True,
        )
        self._write(
            "bin/communication-python",
            textwrap.dedent(
                """\
                #!/bin/bash
                echo "$$" >> "$STATE_DIR/communication_pids"
                if [ "$COMMUNICATION_MODE" = fail ]; then
                    exit 29
                fi
                if [ "$COMMUNICATION_MODE" = delayed_fail ]; then
                    sleep 0.4
                    exit 33
                fi
                if [ "$IGNORE_TERM" = 1 ]; then
                    trap '' TERM
                else
                    trap 'exit 0' TERM
                fi
                while :; do sleep 1; done
                """
            ),
            True,
        )
        self._write("bin/rospack", "#!/bin/bash\nexit 0\n", True)
        self._write(
            "bin/rosservice",
            textwrap.dedent(
                """\
                #!/bin/bash
                if [ "$SIMULATION_MODE" = success ]; then
                    echo /gazebo/get_world_properties
                    exit 0
                fi
                exit 1
                """
            ),
            True,
        )
        self._write(
            "bin/rosnode",
            textwrap.dedent(
                """\
                #!/bin/bash
                [ -e "$STATE_DIR/simulation_started" ] || exit 0
                if [ "$COMMUNICATION_MODE" = success ] || [ "$COMMUNICATION_MODE" = delayed_fail ]; then
                    for id in $(seq 0 5); do
                        echo "/typhoon_h480_${id}_communication"
                    done
                fi
                """
            ),
            True,
        )
        self._write(
            "bin/rostopic",
            textwrap.dedent(
                """\
                #!/bin/bash
                if [ "$1" = list ]; then
                    for id in $(seq 0 5); do
                        echo "/typhoon_h480_${id}/mavros/local_position/pose"
                    done
                    exit 0
                fi
                topic="${@: -1}"
                case "$topic" in
                    */mavros/state)
                        echo 'connected: True'
                        ;;
                    */realsense/*)
                        if [ "$CAMERA_MODE" = sleep ]; then
                            sleep 30
                        else
                            echo ready
                        fi
                        ;;
                esac
                """
            ),
            True,
        )
        self._write(
            "bin/roslaunch",
            textwrap.dedent(
                """\
                #!/bin/bash
                if [ "$1" = look_up ]; then
                    touch "$STATE_DIR/mission_started"
                    exit "$MISSION_MODE"
                fi
                touch "$STATE_DIR/simulation_started"
                echo "$$" > "$STATE_DIR/simulation_pid"
                if [ "$SIMULATION_MODE" = fail ]; then
                    exit 23
                fi
                if [ "$IGNORE_TERM" = 1 ]; then
                    trap '' TERM
                else
                    trap 'exit 0' TERM
                fi
                while :; do sleep 1; done
                """
            ),
            True,
        )
        self._write(
            "bin/tee",
            textwrap.dedent(
                """\
                #!/bin/bash
                count=0
                [ ! -f "$STATE_DIR/tee_count" ] || count=$(cat "$STATE_DIR/tee_count")
                count=$((count + 1))
                echo "$count" > "$STATE_DIR/tee_count"
                if [ "$TEE_MODE" = setup_fail ]; then
                    exit 17
                fi
                if [ "$TEE_MODE" = runtime_fail ] && [ "$count" -ge 2 ]; then
                    /bin/cat >/dev/null
                    exit 18
                fi
                if [ "$TEE_MODE" = drain ]; then
                    /bin/cat >/dev/null
                    exit 0
                fi
                exec /usr/bin/tee "$@"
                """
            ),
            True,
        )

    def run(self, *, timeout_seconds=5, args=("6", "mission_down.json"), **env):
        launch_env = self.env.copy()
        launch_env.update({key: str(value) for key, value in env.items()})
        started = time.monotonic()
        output_path = self.state / f"launcher-output-{time.monotonic_ns()}.txt"
        with output_path.open("w", encoding="utf-8") as output_file:
            result = subprocess.run(
                [
                    "/usr/bin/timeout",
                    "--kill-after=1s",
                    f"{timeout_seconds}s",
                    "bash",
                    str(self.workspace / "1.sh"),
                    *args,
                ],
                cwd=self.workspace,
                env=launch_env,
                stdout=output_file,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=timeout_seconds + 3,
            )
        result.stdout = output_path.read_text(encoding="utf-8")
        return result, time.monotonic() - started

    def run_tmp_exists(self):
        marker = self.state / "run_tmp"
        return marker.exists() and pathlib.Path(marker.read_text().strip()).exists()

    def recorded_processes_still_exist(self):
        alive = []
        for marker_name in (
            "simulation_pid",
            "communication_pids",
            "helper_pids",
        ):
            marker = self.state / marker_name
            if not marker.exists():
                continue
            for value in marker.read_text().splitlines():
                try:
                    os.kill(int(value), 0)
                    alive.append(int(value))
                except (ProcessLookupError, ValueError):
                    pass
        return alive

    def close(self):
        for marker_name in (
            "simulation_pid",
            "communication_pids",
            "helper_pids",
        ):
            marker = self.state / marker_name
            if not marker.exists():
                continue
            for value in marker.read_text().splitlines():
                try:
                    pid = int(value)
                    command = pathlib.Path(f"/proc/{pid}/cmdline").read_bytes()
                    cwd = pathlib.Path(f"/proc/{pid}/cwd").resolve()
                    if str(self.root).encode() in command or self.workspace in cwd.parents:
                        os.kill(pid, signal.SIGKILL)
                except (FileNotFoundError, ProcessLookupError, ValueError):
                    pass
        self._temporary_directory.cleanup()


class OneClickLaunchTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.script = (pathlib.Path(__file__).parents[1] / "1.sh").read_text(
            encoding="utf-8"
        )

    def test_cleanup_uses_terminable_signal_for_background_jobs(self):
        script = self.script

        self.assertIn('MISSION_PID=""', script)
        self.assertIn('kill -TERM -- "-$pid"', script)
        self.assertIn('kill -KILL -- "-$pid"', script)
        self.assertIn('CLEANUP_GRACE_SECONDS="${CLEANUP_GRACE_SECONDS:-5}"', script)
        self.assertNotIn('kill -INT -- "-$pid"', script)

    def test_mission_launch_is_owned_by_cleanup(self):
        script = self.script

        self.assertIn('start_owned_group "任务节点" roslaunch look_up', script)
        self.assertIn('MISSION_PID="$LAST_STARTED_PID"', script)
        self.assertIn('wait_for_owned_process "$MISSION_PID"', script)

    def test_simulator_ready_timeout_can_be_configured(self):
        script = self.script

        self.assertIn(
            'READY_TIMEOUT_SECONDS="${READY_TIMEOUT_SECONDS:-180}"',
            script,
            "slow six-vehicle startup must allow a longer timeout via the environment",
        )

    def test_communication_bridge_starts_before_mission(self):
        script = self.script

        bridge = 'start_communication "$XTDRONE_PYTHON"'
        mission = 'roslaunch look_up down_resume.launch'
        self.assertIn(bridge, script)
        self.assertIn(mission, script)
        self.assertLess(
            script.index(bridge),
            script.index(mission),
            "XTDrone communication bridges must start before the mission sends commands",
        )

    def test_fast_preflight_runs_before_every_roslaunch(self):
        script = self.script
        prepare = '"$COMPLIANCE_PYTHON" "$PREPARE_MODEL"'
        simulation_launch = (
            'start_owned_group "六机仿真" roslaunch "$SIMULATION_LAUNCH" '
            'model_file:="$GENERATED_MODEL"'
        )
        self.assertIn(prepare, script)
        self.assertIn(simulation_launch, script)
        roslaunch_positions = [match.start() for match in re.finditer("roslaunch", script)]
        self.assertEqual(2, len(roslaunch_positions))
        self.assertTrue(
            all(script.index(prepare) < position for position in roslaunch_positions)
        )

    def test_startup_never_links_or_requires_old_custom_model(self):
        script = self.script
        self.assertNotIn("MODEL_LINK", script)
        self.assertNotIn("ln -s", script)
        self.assertNotIn("typhoon_h480_zzufly", script)
        self.assertNotIn("single_vehicle_spawn_xtd.launch", script)

    def test_realsense_topics_are_ready_after_communication_and_before_yolo(self):
        script = self.script.split("main() {", 1)[1]
        communication = "wait_for_communication || return 1"
        cameras = "wait_for_cameras || return 1"
        yolo = 'start_helper "$WORKSPACE_DIR/src/yolo" "multi_yolo_detecting.sh"'
        self.assertIn(communication, script)
        self.assertIn(cameras, script)
        self.assertIn(yolo, script)
        self.assertLess(script.index(communication), script.index(cameras))
        self.assertLess(script.index(cameras), script.index(yolo))

    def test_clean_start_does_not_launch_realsense_gimbal_worker(self):
        script = self.script
        self.assertNotIn("multi_gimbal_control.sh", script)
        self.assertNotIn('start_helper "$WORKSPACE_DIR/src/gimbal"', script)

    def test_cleanup_only_removes_validated_tmp_directory(self):
        script = self.script
        self.assertIn('case "$RUN_TMP_DIR" in', script)
        self.assertIn('/tmp/robocup-fly-competition-clean.*)', script)
        self.assertIn('rm -rf -- "$RUN_TMP_DIR"', script)
        self.assertIn(
            'echo "拒绝清理非 competition-clean 临时目录：$RUN_TMP_DIR" >&2',
            script,
        )
        self.assertIn(
            'RUN_TMP_DIR="$(mktemp -d /tmp/robocup-fly-competition-clean.XXXXXX)"',
            script,
        )
        self.assertIn(
            'if ! RUN_TMP_DIR="$(mktemp -d '
            '/tmp/robocup-fly-competition-clean.XXXXXX)"; then',
            script,
        )

    def test_launch_writes_a_workspace_diagnostic_log(self):
        script = self.script
        self.assertIn('LOG_DIR="$WORKSPACE_DIR/logs/competition-clean"', script)
        self.assertIn('RUN_LOG="$(mktemp "$LOG_DIR/launch-', script)
        self.assertIn('mkfifo "$LOGGER_FIFO"', script)
        self.assertNotIn('exec > >(tee -a "$RUN_LOG") 2>&1', script)
        self.assertIn('echo "本次完整启动日志：$RUN_LOG"', script)

    def test_preflight_checks_official_inputs_and_passes_explicit_paths(self):
        script = self.script
        required_files = (
            'require_file "$PREPARE_MODEL"',
            'require_file "$OFFICIAL_MANIFEST"',
            'require_file "$SENSOR_MOUNT_CONFIG"',
            "sitl_config/models/typhoon_h480_realsense/typhoon_h480_realsense.sdf",
            "sitl_config/models/realsense_camera/realsense_camera.sdf",
        )
        for required_file in required_files:
            self.assertIn(required_file, script)

        for argument in (
            '--px4-dir "$PX4_DIR"',
            '--xtdrone-dir "$XTDRONE_DIR"',
            '--manifest "$OFFICIAL_MANIFEST"',
            '--mount-config "$SENSOR_MOUNT_CONFIG"',
            '--output "$GENERATED_MODEL"',
        ):
            self.assertIn(argument, script)
        self.assertNotIn("--skip", script)

    def test_camera_readiness_checks_all_six_drones_and_three_topic_types(self):
        script = self.script
        self.assertIn("all_cameras_ready() {", script)
        function = script.split("all_cameras_ready() {", 1)[1].split("\n}\n", 1)[0]
        self.assertIn("for id in $(seq 0 5)", function)
        expected_topics = (
            '"/typhoon_h480_${id}/realsense/depth_camera/color/image_raw"',
            '"/typhoon_h480_${id}/realsense/depth_camera/depth/image_raw"',
            '"/typhoon_h480_${id}/realsense/depth_camera/color/camera_info"',
        )
        topic_paths = re.findall(
            r'"/typhoon_h480_\$\{id\}/realsense/depth_camera/[^\"]+"', function
        )
        self.assertEqual(list(expected_topics), topic_paths)
        self.assertIn('timeout "$probe_timeout" rostopic echo -n 1 "$topic"', function)
        self.assertIn(
            'CAMERA_TIMEOUT_SECONDS="${CAMERA_TIMEOUT_SECONDS:-60}"', script
        )

    def test_generated_model_is_private_run_output(self):
        script = self.script
        self.assertIn('RUN_TMP_DIR=""', script)
        self.assertIn('GENERATED_MODEL=""', script)
        self.assertIn(
            'GENERATED_MODEL="$RUN_TMP_DIR/typhoon_h480_realsense.sdf"', script
        )


class OneClickLaunchBehaviorTest(unittest.TestCase):
    def setUp(self):
        self.harness = LauncherHarness()

    def tearDown(self):
        self.harness.close()

    def test_invalid_timeouts_fail_before_log_or_preflight_setup(self):
        invalid_values = {
            "READY_TIMEOUT_SECONDS": "0",
            "COMMUNICATION_TIMEOUT_SECONDS": "-1",
            "CAMERA_TIMEOUT_SECONDS": "1.5",
            "CLEANUP_GRACE_SECONDS": "bad",
        }
        for variable, value in invalid_values.items():
            with self.subTest(variable=variable):
                harness = LauncherHarness()
                try:
                    result, elapsed = harness.run(
                        args=("7", "mission_down.json"), **{variable: value}
                    )
                    self.assertNotEqual(0, result.returncode, result.stdout)
                    self.assertLess(elapsed, 1.5, result.stdout)
                    self.assertFalse((harness.state / "preflight").exists())
                    self.assertFalse(
                        (harness.workspace / "logs/competition-clean").exists()
                    )
                finally:
                    harness.close()

    def test_log_directory_failure_is_fail_closed_before_preflight(self):
        log_parent = self.harness.workspace / "logs"
        log_parent.mkdir()
        (log_parent / "competition-clean").write_text("not a directory")
        result, elapsed = self.harness.run(TEE_MODE="drain")

        self.assertNotEqual(0, result.returncode, result.stdout)
        self.assertLess(elapsed, 1.5, result.stdout)
        self.assertFalse((self.harness.state / "preflight").exists())
        self.assertFalse((self.harness.state / "simulation_started").exists())

    def test_concurrent_starts_use_distinct_run_logs(self):
        environment = self.harness.env.copy()
        processes = [
            subprocess.Popen(
                ["bash", str(self.harness.workspace / "1.sh"), "7"],
                cwd=self.harness.workspace,
                env=environment,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
            )
            for _ in range(2)
        ]
        results = [process.communicate(timeout=4) for process in processes]
        self.assertTrue(all(process.returncode != 0 for process in processes), results)
        logs = list((self.harness.workspace / "logs/competition-clean").glob("launch-*.log"))
        self.assertEqual(2, len(logs), results)

    def test_logger_runtime_failure_converts_success_to_failure(self):
        result, elapsed = self.harness.run(TEE_MODE="runtime_fail")

        self.assertNotEqual(0, result.returncode, result.stdout)
        self.assertLess(elapsed, 5, result.stdout)
        self.assertTrue((self.harness.state / "mission_started").exists())
        self.assertFalse(self.harness.run_tmp_exists())
        self.assertEqual(
            [], list((self.harness.workspace / "logs/competition-clean").glob(".logger-*"))
        )

    def test_tee_setup_failure_is_fail_closed_before_preflight(self):
        result, elapsed = self.harness.run(TEE_MODE="setup_fail")

        self.assertNotEqual(0, result.returncode, result.stdout)
        self.assertLess(elapsed, 1.5, result.stdout)
        self.assertFalse((self.harness.state / "preflight").exists())
        self.assertFalse((self.harness.state / "simulation_started").exists())
        self.assertEqual(
            [], list((self.harness.workspace / "logs/competition-clean").glob(".logger-*"))
        )

    def test_camera_timeout_is_a_true_global_deadline(self):
        result, elapsed = self.harness.run(
            CAMERA_MODE="sleep", CAMERA_TIMEOUT_SECONDS="1"
        )

        self.assertNotEqual(0, result.returncode, result.stdout)
        self.assertLess(elapsed, 2.5, result.stdout)
        self.assertFalse((self.harness.state / "mission_started").exists())
        self.assertFalse(self.harness.run_tmp_exists())

    def test_simulator_failure_propagates_without_readiness_timeout(self):
        result, elapsed = self.harness.run(SIMULATION_MODE="fail")

        self.assertNotEqual(0, result.returncode, result.stdout)
        self.assertLess(elapsed, 2, result.stdout)
        self.assertIn("六机仿真", result.stdout)
        self.assertFalse((self.harness.state / "mission_started").exists())
        self.assertFalse(self.harness.run_tmp_exists())

    def test_communication_failure_propagates_without_readiness_timeout(self):
        result, elapsed = self.harness.run(
            timeout_seconds=4, COMMUNICATION_MODE="fail"
        )

        self.assertNotEqual(0, result.returncode, result.stdout)
        self.assertLess(elapsed, 2.5, result.stdout)
        self.assertIn("XTDrone", result.stdout)
        self.assertFalse((self.harness.state / "mission_started").exists())
        self.assertFalse(self.harness.run_tmp_exists())

    def test_helper_failure_prevents_mission_launch(self):
        result, elapsed = self.harness.run(HELPER_MODE="fail")

        self.assertNotEqual(0, result.returncode, result.stdout)
        self.assertLess(elapsed, 4, result.stdout)
        self.assertFalse((self.harness.state / "mission_started").exists())
        self.assertFalse(self.harness.run_tmp_exists())

    def test_communication_failure_during_helper_check_prevents_mission(self):
        result, elapsed = self.harness.run(COMMUNICATION_MODE="delayed_fail")

        self.assertNotEqual(0, result.returncode, result.stdout)
        self.assertLess(elapsed, 4, result.stdout)
        self.assertFalse((self.harness.state / "mission_started").exists())
        self.assertFalse(self.harness.run_tmp_exists())

    def test_nonzero_mission_status_is_preserved(self):
        result, elapsed = self.harness.run(MISSION_MODE="42")

        self.assertEqual(42, result.returncode, result.stdout)
        self.assertLess(elapsed, 5, result.stdout)
        self.assertTrue((self.harness.state / "mission_started").exists())
        self.assertFalse(self.harness.run_tmp_exists())

    def test_term_ignoring_owned_process_is_killed_with_bounded_cleanup(self):
        result, elapsed = self.harness.run(MISSION_MODE="0", IGNORE_TERM="1")

        self.assertEqual(0, result.returncode, result.stdout)
        self.assertLess(elapsed, 5, result.stdout)
        simulator_pid = int(
            (self.harness.state / "simulation_pid").read_text().strip()
        )
        with self.assertRaises(ProcessLookupError):
            os.kill(simulator_pid, 0)
        deadline = time.monotonic() + 0.5
        while self.harness.recorded_processes_still_exist() and time.monotonic() < deadline:
            time.sleep(0.02)
        self.assertEqual([], self.harness.recorded_processes_still_exist())
        self.assertFalse(self.harness.run_tmp_exists())


if __name__ == "__main__":
    unittest.main()

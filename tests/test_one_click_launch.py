#!/usr/bin/env python3

import os
import importlib.util
import pathlib
import re
import signal
import subprocess
import tempfile
import textwrap
import time
import unittest
from unittest import mock


ROOT = pathlib.Path(__file__).parents[1]


def load_process_supervisor_module():
    spec = importlib.util.spec_from_file_location(
        "process_supervisor_under_test", ROOT / "scripts/process_supervisor.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


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
        (self.workspace / "1.sh").chmod(0o755)
        supervisor_source = ROOT / "scripts/process_supervisor.py"
        self._workspace_file(
            "scripts/process_supervisor.py",
            supervisor_source.read_text(encoding="utf-8"),
            True,
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
                "SUPERVISOR_PYTHON": "/usr/bin/python3",
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
                "SETSID_MODE": "success",
                "STATE_MODE": "success",
                "SIMULATION_FAIL_DELAY": "0.4",
                "COMMUNICATION_FAIL_DELAY": "0.4",
                "HELPER_FAIL_DELAY": "0.4",
                "MISSION_DURATION": "3",
                "IGNORE_TERM": "0",
                "PS_TREE_DELAY": "0",
                "AWK_TREE_DELAY": "0",
                "DATE_MODE": "normal",
                "LARGE_TREE_CHILDREN": "20",
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
            if [ "$HELPER_MODE" = delayed_fail ]; then
                sleep "$HELPER_FAIL_DELAY"
                exit 32
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
                    sleep "$COMMUNICATION_FAIL_DELAY"
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
                if [ "$SIMULATION_MODE" = success ] \
                    || [ "$SIMULATION_MODE" = delayed_fail ] \
                    || [ "$SIMULATION_MODE" = detached_child ] \
                    || [ "$SIMULATION_MODE" = detached_child_then_fail ] \
                    || [ "$SIMULATION_MODE" = detached_child_ignore_term ] \
                    || [ "$SIMULATION_MODE" = large_tree ]; then
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
                        if [ "$STATE_MODE" = sleep ]; then
                            sleep 30
                        else
                            echo 'connected: True'
                        fi
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
                    if [ "$MISSION_MODE" = sleep ]; then
                        sleep "$MISSION_DURATION"
                        exit 0
                    fi
                    exit "$MISSION_MODE"
                fi
                touch "$STATE_DIR/simulation_started"
                echo "$$" > "$STATE_DIR/simulation_pid"
                if [ "$SIMULATION_MODE" = fail ]; then
                    exit 23
                fi
                if [ "$SIMULATION_MODE" = delayed_fail ]; then
                    sleep "$SIMULATION_FAIL_DELAY"
                    exit 24
                fi
                if [ "$SIMULATION_MODE" = detached_child ] \
                    || [ "$SIMULATION_MODE" = detached_child_then_fail ] \
                    || [ "$SIMULATION_MODE" = detached_child_ignore_term ]; then
                    /usr/bin/setsid bash -c '
                        if [ "$SIMULATION_MODE" = detached_child_ignore_term ]; then
                            trap "" TERM
                        else
                            trap "exit 0" TERM
                        fi
                        echo "$$" >> "$STATE_DIR/detached_child_pids"
                        while :; do sleep 1; done
                    ' &
                    deadline=$((SECONDS + 2))
                    while [ ! -s "$STATE_DIR/detached_child_pids" ] \
                        && [ "$SECONDS" -lt "$deadline" ]; do
                        sleep 0.01
                    done
                    [ -s "$STATE_DIR/detached_child_pids" ] || exit 25
                    if [ "$SIMULATION_MODE" = detached_child_then_fail ]; then
                        exit 24
                    fi
                fi
                if [ "$SIMULATION_MODE" = large_tree ]; then
                    for unused in $(seq 1 "$LARGE_TREE_CHILDREN"); do
                        sleep 30 &
                        echo "$!" >> "$STATE_DIR/large_tree_pids"
                    done
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
                if [ "$TEE_MODE" = early_exit ] && [ "$count" -ge 2 ]; then
                    sleep 0.3
                    exit 19
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
        self._write(
            "bin/date",
            textwrap.dedent(
                """\
                #!/bin/bash
                if [ "$DATE_MODE" = hang ]; then
                    sleep 10
                    exit 1
                fi
                exec /usr/bin/date "$@"
                """
            ),
            True,
        )
        self._write(
            "bin/awk",
            textwrap.dedent(
                """\
                #!/bin/bash
                if [ "$AWK_TREE_DELAY" != 0 ]; then
                    sleep "$AWK_TREE_DELAY"
                fi
                exec /usr/bin/awk "$@"
                """
            ),
            True,
        )
        self._write(
            "bin/ps",
            textwrap.dedent(
                """\
                #!/bin/bash
                if [ "$1" = -eo ] && [ "$2" = "pid=,ppid=" ] \
                    && [ "$PS_TREE_DELAY" != 0 ]; then
                    sleep "$PS_TREE_DELAY"
                fi
                exec /usr/bin/ps "$@"
                """
            ),
            True,
        )
        self._write(
            "bin/setsid",
            textwrap.dedent(
                """\
                #!/bin/bash
                echo "$$" >> "$STATE_DIR/setsid_pids"
                if [ "$SETSID_MODE" = root_delay ]; then
                    touch "$STATE_DIR/setsid_called"
                    exec /bin/sleep 30
                fi
                if [ "$SETSID_MODE" = delay ]; then
                    sleep 2 &
                    delay_pid=$!
                    echo "$delay_pid" >> "$STATE_DIR/setsid_descendant_pids"
                    touch "$STATE_DIR/setsid_called"
                    wait "$delay_pid"
                else
                    touch "$STATE_DIR/setsid_called"
                fi
                exec /usr/bin/setsid "$@"
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

    def interrupt_during_setsid(
        self,
        *,
        signal_number=signal.SIGTERM,
        signal_process_group=False,
        **env,
    ):
        launch_env = self.env.copy()
        launch_env["SETSID_MODE"] = "delay"
        launch_env.update({key: str(value) for key, value in env.items()})
        output_path = self.state / "interrupt-output.txt"
        started = time.monotonic()
        with output_path.open("w", encoding="utf-8") as output_file:
            process = subprocess.Popen(
                ["bash", str(self.workspace / "1.sh"), "6", "mission_down.json"],
                cwd=self.workspace,
                env=launch_env,
                stdout=output_file,
                stderr=subprocess.STDOUT,
                text=True,
                start_new_session=True,
            )
            deadline = time.monotonic() + 2
            marker = self.state / "setsid_called"
            while not marker.exists() and time.monotonic() < deadline:
                time.sleep(0.01)
            if not marker.exists():
                process.kill()
                process.wait(timeout=1)
                raise AssertionError("setsid stub was not reached")
            child_pid = int((self.state / "setsid_pids").read_text().splitlines()[-1])
            if signal_process_group:
                os.killpg(process.pid, signal_number)
            else:
                os.kill(process.pid, signal_number)
            timed_out = False
            try:
                returncode = process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                timed_out = True
                process.kill()
                returncode = process.wait(timeout=1)
        result = subprocess.CompletedProcess(
            process.args,
            returncode,
            output_path.read_text(encoding="utf-8"),
        )
        return result, time.monotonic() - started, timed_out, child_pid

    def interrupt_with_live_detached_child(self):
        launch_env = self.env.copy()
        launch_env.update(
            {
                "SIMULATION_MODE": "detached_child",
                "MISSION_MODE": "sleep",
                "MISSION_DURATION": "30",
            }
        )
        output_path = self.state / "detached-interrupt-output.txt"
        started = time.monotonic()
        with output_path.open("w", encoding="utf-8") as output_file:
            process = subprocess.Popen(
                ["bash", str(self.workspace / "1.sh"), "6", "mission_down.json"],
                cwd=self.workspace,
                env=launch_env,
                stdout=output_file,
                stderr=subprocess.STDOUT,
                text=True,
                start_new_session=True,
            )
            try:
                detached_marker = self.state / "detached_child_pids"
                mission_marker = self.state / "mission_started"
                deadline = time.monotonic() + 5
                while (
                    (not detached_marker.exists() or not mission_marker.exists())
                    and time.monotonic() < deadline
                ):
                    if process.poll() is not None:
                        break
                    time.sleep(0.01)
                if not detached_marker.exists() or not mission_marker.exists():
                    raise AssertionError("launcher did not reach the live mission state")

                detached_pid = int(detached_marker.read_text().strip())
                detached_pgid = os.getpgid(detached_pid)
                detached_sid = os.getsid(detached_pid)
                os.kill(process.pid, signal.SIGINT)
                returncode = process.wait(timeout=5)
            except BaseException:
                if process.poll() is None:
                    process.terminate()
                    try:
                        process.wait(timeout=2)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=1)
                raise
        result = subprocess.CompletedProcess(
            process.args,
            returncode,
            output_path.read_text(encoding="utf-8"),
        )
        return (
            result,
            time.monotonic() - started,
            detached_pid,
            detached_pgid,
            detached_sid,
        )

    def run_tmp_exists(self):
        marker = self.state / "run_tmp"
        return marker.exists() and pathlib.Path(marker.read_text().strip()).exists()

    def recorded_processes_still_exist(self):
        alive = []
        for marker_name in (
            "simulation_pid",
            "communication_pids",
            "helper_pids",
            "setsid_pids",
            "setsid_descendant_pids",
            "detached_child_pids",
            "large_tree_pids",
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
            "setsid_pids",
            "setsid_descendant_pids",
            "detached_child_pids",
            "large_tree_pids",
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
        self.assertIn('declare -A DIRECT_FALLBACK_START_TIMES=()', script)
        self.assertIn('direct_fallback_is_running "$pid"', script)
        self.assertIn(
            'if direct_fallback_is_running "$pending_unregistered"; then', script
        )
        self.assertNotIn('kill -INT -- "-$pid"', script)

    def test_owned_startup_registers_pending_pid_before_release(self):
        script = self.script

        self.assertIn('PENDING_PID=""', script)
        self.assertIn('PENDING_PID=$!', script)
        self.assertIn('register_owned_process "$PENDING_PID" "$name"', script)
        self.assertIn('touch "$release_file"', script)
        self.assertIn('kill -0 "$owner_pid"', script)
        self.assertIn('[ ! -d "$gate_dir" ]', script)
        self.assertLess(
            script.index('register_owned_process "$PENDING_PID" "$name"'),
            script.index('touch "$release_file"'),
        )

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

    def test_xtdrone_models_precede_px4_models_for_realsense_resolution(self):
        self.assertIn(
            'export GAZEBO_MODEL_PATH="$XTDRONE_DIR/sitl_config/models:'
            '$PX4_DIR/Tools/sitl_gazebo/models:$GAZEBO_MODELS_DIR'
            '${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"',
            self.script,
        )

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
            '--gazebo-models-dir "$GAZEBO_MODELS_DIR"',
            '--xtdrone-pythonpath "$XTDRONE_PYTHONPATH"',
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

    def test_logger_exit_with_open_fifo_is_controlled_and_prevents_mission(self):
        result, elapsed = self.harness.run(TEE_MODE="early_exit", MISSION_MODE="sleep")

        self.assertNotEqual(0, result.returncode, result.stdout)
        self.assertNotIn(result.returncode, (-13, 141), result.stdout)
        self.assertLess(elapsed, 2, result.stdout)
        self.assertFalse((self.harness.state / "mission_started").exists())
        self.assertFalse(self.harness.run_tmp_exists())
        self.assertEqual([], self.harness.recorded_processes_still_exist())

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

    def test_simulator_state_probe_respects_ready_global_deadline(self):
        result, elapsed = self.harness.run(
            READY_TIMEOUT_SECONDS="1", STATE_MODE="sleep"
        )

        self.assertNotEqual(0, result.returncode, result.stdout)
        self.assertLess(elapsed, 1.8, result.stdout)
        self.assertFalse((self.harness.state / "mission_started").exists())
        self.assertFalse(self.harness.run_tmp_exists())

    def test_simulator_failure_propagates_without_readiness_timeout(self):
        result, elapsed = self.harness.run(SIMULATION_MODE="fail")

        self.assertNotEqual(0, result.returncode, result.stdout)
        self.assertLess(elapsed, 2, result.stdout)
        self.assertIn("六机仿真", result.stdout)
        self.assertFalse((self.harness.state / "mission_started").exists())
        self.assertFalse(self.harness.run_tmp_exists())

    def test_simulator_death_after_readiness_prevents_mission(self):
        result, elapsed = self.harness.run(
            SIMULATION_MODE="delayed_fail", SIMULATION_FAIL_DELAY="0.4"
        )

        self.assertNotEqual(0, result.returncode, result.stdout)
        self.assertLess(elapsed, 2, result.stdout)
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

    def test_simulator_death_during_mission_fails_the_run(self):
        result, elapsed = self.harness.run(
            SIMULATION_MODE="delayed_fail",
            SIMULATION_FAIL_DELAY="1.7",
            MISSION_MODE="sleep",
        )

        self.assertNotEqual(0, result.returncode, result.stdout)
        self.assertLess(elapsed, 3, result.stdout)
        self.assertTrue((self.harness.state / "mission_started").exists())
        self.assertFalse(self.harness.run_tmp_exists())

    def test_communication_death_during_mission_fails_the_run(self):
        result, elapsed = self.harness.run(
            COMMUNICATION_MODE="delayed_fail",
            COMMUNICATION_FAIL_DELAY="1.7",
            MISSION_MODE="sleep",
        )

        self.assertNotEqual(0, result.returncode, result.stdout)
        self.assertLess(elapsed, 3, result.stdout)
        self.assertTrue((self.harness.state / "mission_started").exists())
        self.assertFalse(self.harness.run_tmp_exists())

    def test_helper_death_during_mission_fails_the_run(self):
        result, elapsed = self.harness.run(
            HELPER_MODE="delayed_fail",
            HELPER_FAIL_DELAY="1.4",
            MISSION_MODE="sleep",
        )

        self.assertNotEqual(0, result.returncode, result.stdout)
        self.assertLess(elapsed, 3, result.stdout)
        self.assertTrue((self.harness.state / "mission_started").exists())
        self.assertFalse(self.harness.run_tmp_exists())

    def test_signal_before_process_group_creation_has_no_leak(self):
        result, elapsed, timed_out, child_pid = self.harness.interrupt_during_setsid()

        self.assertFalse(timed_out, result.stdout)
        self.assertNotEqual(0, result.returncode, result.stdout)
        self.assertLess(elapsed, 2, result.stdout)
        with self.assertRaises(ProcessLookupError):
            os.kill(child_pid, 0)
        self.assertFalse(self.harness.run_tmp_exists())
        self.assertEqual([], self.harness.recorded_processes_still_exist())

    def test_terminal_process_group_signal_has_no_leak(self):
        result, elapsed, timed_out, child_pid = self.harness.interrupt_during_setsid(
            signal_process_group=True
        )

        self.assertFalse(timed_out, result.stdout)
        self.assertNotEqual(0, result.returncode, result.stdout)
        self.assertLess(elapsed, 2, result.stdout)
        with self.assertRaises(ProcessLookupError):
            os.kill(child_pid, 0)
        self.assertFalse(self.harness.run_tmp_exists())
        self.assertEqual([], self.harness.recorded_processes_still_exist())

    def test_terminal_process_group_sigint_has_no_leak(self):
        result, elapsed, timed_out, child_pid = self.harness.interrupt_during_setsid(
            signal_number=signal.SIGINT,
            signal_process_group=True,
        )

        self.assertFalse(timed_out, result.stdout)
        self.assertEqual(130, result.returncode, result.stdout)
        self.assertLess(elapsed, 2, result.stdout)
        with self.assertRaises(ProcessLookupError):
            os.kill(child_pid, 0)
        self.assertFalse(self.harness.run_tmp_exists())
        self.assertEqual([], self.harness.recorded_processes_still_exist())

    def test_terminal_process_group_sighup_has_no_leak(self):
        result, elapsed, timed_out, child_pid = self.harness.interrupt_during_setsid(
            signal_number=signal.SIGHUP,
            signal_process_group=True,
        )

        self.assertFalse(timed_out, result.stdout)
        self.assertEqual(129, result.returncode, result.stdout)
        self.assertLess(elapsed, 2, result.stdout)
        with self.assertRaises(ProcessLookupError):
            os.kill(child_pid, 0)
        self.assertFalse(self.harness.run_tmp_exists())
        self.assertEqual([], self.harness.recorded_processes_still_exist())

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

    def test_owned_child_in_independent_session_is_cleaned_up(self):
        result, elapsed = self.harness.run(SIMULATION_MODE="detached_child")

        self.assertEqual(0, result.returncode, result.stdout)
        self.assertLess(elapsed, 5, result.stdout)
        detached_pid = int(
            (self.harness.state / "detached_child_pids").read_text().strip()
        )
        with self.assertRaises(ProcessLookupError):
            os.kill(detached_pid, 0)
        self.assertEqual([], self.harness.recorded_processes_still_exist())
        self.assertFalse(self.harness.run_tmp_exists())

    def test_detached_owned_child_is_cleaned_after_root_exits_first(self):
        result, elapsed = self.harness.run(
            SIMULATION_MODE="detached_child_then_fail"
        )

        self.assertNotEqual(0, result.returncode, result.stdout)
        self.assertLess(elapsed, 5, result.stdout)
        detached_pid = int(
            (self.harness.state / "detached_child_pids").read_text().strip()
        )
        with self.assertRaises(ProcessLookupError):
            os.kill(detached_pid, 0)
        self.assertEqual([], self.harness.recorded_processes_still_exist())
        self.assertFalse(self.harness.run_tmp_exists())

    def test_term_ignoring_detached_child_is_killed_before_supervisor_exits(self):
        result, elapsed = self.harness.run(
            SIMULATION_MODE="detached_child_ignore_term"
        )

        self.assertEqual(0, result.returncode, result.stdout)
        self.assertLess(elapsed, 5, result.stdout)
        detached_pid = int(
            (self.harness.state / "detached_child_pids").read_text().strip()
        )
        with self.assertRaises(ProcessLookupError):
            os.kill(detached_pid, 0)
        self.assertEqual([], self.harness.recorded_processes_still_exist())
        self.assertFalse(self.harness.run_tmp_exists())

    def test_reused_root_pid_identity_is_not_signaled(self):
        unrelated = subprocess.Popen(["/bin/sleep", "30"])
        identity_file = self.harness.state / "fake-root-start-time"
        identity_file.write_text("100\n", encoding="ascii")
        try:
            probe = subprocess.run(
                [
                    "bash",
                    "-c",
                    textwrap.dedent(
                        """\
                        source "$LAUNCHER_SCRIPT"
                        process_start_time() { cat "$IDENTITY_FILE"; }
                        register_owned_process "$TARGET_PID" "reused root"
                        printf '200\n' > "$IDENTITY_FILE"
                        signal_owned_target TERM "$TARGET_PID"
                        """
                    ),
                ],
                env={
                    **self.harness.env,
                    "LAUNCHER_SCRIPT": str(self.harness.workspace / "1.sh"),
                    "IDENTITY_FILE": str(identity_file),
                    "TARGET_PID": str(unrelated.pid),
                },
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=2,
            )

            self.assertEqual(0, probe.returncode, probe.stdout)
            time.sleep(0.05)
            self.assertIsNone(
                unrelated.poll(), "cleanup signaled a process with a reused root PID"
            )
        finally:
            if unrelated.poll() is None:
                unrelated.terminate()
            unrelated.wait(timeout=1)

    def test_large_owned_tree_cleanup_is_bounded(self):
        result, elapsed = self.harness.run(
            SIMULATION_MODE="large_tree",
            LARGE_TREE_CHILDREN="100",
            timeout_seconds=5,
        )

        self.assertEqual(0, result.returncode, result.stdout)
        self.assertLess(elapsed, 3, result.stdout)
        self.assertEqual([], self.harness.recorded_processes_still_exist())
        self.assertFalse(self.harness.run_tmp_exists())

    def test_shell_fallback_closure_is_inside_cleanup_deadline(self):
        result, elapsed, timed_out, child_pid = self.harness.interrupt_during_setsid(
            AWK_TREE_DELAY="10", SETSID_MODE="root_delay"
        )

        self.assertFalse(timed_out, result.stdout)
        self.assertNotEqual(0, result.returncode, result.stdout)
        self.assertLess(elapsed, 2.2, result.stdout)
        with self.assertRaises(ProcessLookupError):
            os.kill(child_pid, 0)
        self.assertEqual([], self.harness.recorded_processes_still_exist())
        self.assertFalse(self.harness.run_tmp_exists())

    def test_supervisor_identity_scan_stops_at_monotonic_deadline(self):
        supervisor_module = load_process_supervisor_module()
        supervisor = supervisor_module.ProcessSupervisor(["unused"], 1)
        supervisor.tracked = {100000 + index: 1 for index in range(200)}

        def slow_process_record(_pid):
            time.sleep(0.01)
            return supervisor.supervisor_pid, 1

        started = time.monotonic()
        deadline = started + 0.05
        with mock.patch.object(
            supervisor_module, "process_record", side_effect=slow_process_record
        ), mock.patch.object(supervisor, "discover"):
            supervisor.wait_until_empty(deadline, signal.SIGTERM)

        self.assertLess(time.monotonic() - started, 0.25)

    def test_supervisor_reap_stops_at_monotonic_deadline(self):
        supervisor_module = load_process_supervisor_module()
        supervisor = supervisor_module.ProcessSupervisor(["unused"], 1)
        next_pid = iter(range(200000, 200200))

        def slow_waitpid(_pid, _options):
            time.sleep(0.01)
            try:
                return next(next_pid), 0
            except StopIteration:
                return 0, 0

        started = time.monotonic()
        deadline = started + 0.05
        with mock.patch.object(supervisor, "discover"), mock.patch.object(
            supervisor_module.os, "waitpid", side_effect=slow_waitpid
        ):
            supervisor.wait_until_empty(deadline, signal.SIGTERM)

        self.assertLess(time.monotonic() - started, 0.25)

    def test_supervisor_term_is_sent_once_per_initial_snapshot(self):
        supervisor_module = load_process_supervisor_module()
        supervisor = supervisor_module.ProcessSupervisor(["unused"], 1)
        supervisor.tracked = {123456: 1}

        with mock.patch.object(supervisor, "discover"), mock.patch.object(
            supervisor, "identity_matches", return_value=True
        ), mock.patch.object(supervisor, "reap"), mock.patch.object(
            supervisor, "has_live_tracked", return_value=True
        ), mock.patch.object(supervisor_module.os, "kill") as kill_process:
            supervisor.wait_until_empty(time.monotonic() + 0.08, signal.SIGTERM)

        self.assertEqual(1, kill_process.call_count)

    def test_cleanup_clock_does_not_depend_on_wall_clock_command(self):
        probe = subprocess.run(
            [
                "/usr/bin/timeout",
                "0.25s",
                "bash",
                "-c",
                'source "$LAUNCHER_SCRIPT"; current_milliseconds >/dev/null',
            ],
            env={
                **self.harness.env,
                "DATE_MODE": "hang",
                "LAUNCHER_SCRIPT": str(self.harness.workspace / "1.sh"),
            },
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )

        self.assertEqual(0, probe.returncode, probe.stdout)

    def test_ctrl_c_cleans_detached_owned_child_but_preserves_unrelated_session(self):
        unrelated = subprocess.Popen(
            ["/usr/bin/setsid", "/bin/sleep", "30"],
            stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        try:
            deadline = time.monotonic() + 1
            while time.monotonic() < deadline:
                if (
                    os.getpgid(unrelated.pid) == unrelated.pid
                    and os.getsid(unrelated.pid) == unrelated.pid
                ):
                    break
                time.sleep(0.01)
            self.assertEqual(unrelated.pid, os.getpgid(unrelated.pid))
            self.assertEqual(unrelated.pid, os.getsid(unrelated.pid))

            result, elapsed, detached_pid, detached_pgid, detached_sid = (
                self.harness.interrupt_with_live_detached_child()
            )

            self.assertEqual(detached_pid, detached_pgid)
            self.assertEqual(detached_pid, detached_sid)
            self.assertEqual(130, result.returncode, result.stdout)
            self.assertLess(elapsed, 5, result.stdout)
            with self.assertRaises(ProcessLookupError):
                os.kill(detached_pid, 0)
            self.assertEqual([], self.harness.recorded_processes_still_exist())
            self.assertFalse(self.harness.run_tmp_exists())
            self.assertIsNone(
                unrelated.poll(), "launcher cleanup terminated an unrelated session"
            )
        finally:
            if unrelated.poll() is None:
                unrelated.terminate()
                try:
                    unrelated.wait(timeout=1)
                except subprocess.TimeoutExpired:
                    unrelated.kill()
                    unrelated.wait(timeout=1)


if __name__ == "__main__":
    unittest.main()

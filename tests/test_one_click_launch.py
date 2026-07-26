#!/usr/bin/env python3

import pathlib
import re
import unittest


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
        self.assertIn('kill -TERM "$MISSION_PID"', script)
        self.assertIn('kill -TERM "$SIMULATION_PID"', script)
        self.assertNotIn('kill -INT -- "-$pid"', script)
        self.assertNotIn('kill -INT "$SIMULATION_PID"', script)

    def test_mission_launch_is_owned_by_cleanup(self):
        script = self.script

        mission_launch = (
            'roslaunch look_up down_resume.launch num_drones:="$NUM_DRONES" '
            'mission_filename:="$MISSION_FILE" &\n'
            'MISSION_PID=$!\n'
            'wait "$MISSION_PID"'
        )
        self.assertIn(mission_launch, script)

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
            'roslaunch "$SIMULATION_LAUNCH" model_file:="$GENERATED_MODEL" &\n'
            "SIMULATION_PID=$!"
        )
        self.assertIn(prepare, script)
        self.assertIn(simulation_launch, script)
        roslaunch_positions = [
            match.start() for match in re.finditer(r"^roslaunch ", script, re.MULTILINE)
        ]
        self.assertTrue(roslaunch_positions)
        self.assertTrue(all(script.index(prepare) < pos for pos in roslaunch_positions))

    def test_startup_never_links_or_requires_old_custom_model(self):
        script = self.script
        self.assertNotIn("MODEL_LINK", script)
        self.assertNotIn("ln -s", script)
        self.assertNotIn("typhoon_h480_zzufly", script)
        self.assertNotIn("single_vehicle_spawn_xtd.launch", script)

    def test_realsense_topics_are_ready_after_communication_and_before_yolo(self):
        script = self.script
        communication = "if ! wait_for_communication"
        cameras = "if ! wait_for_cameras"
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
        self.assertIn(
            'RUN_LOG="$LOG_DIR/launch-$(date +%Y%m%d-%H%M%S).log"', script
        )
        self.assertIn('exec > >(tee -a "$RUN_LOG") 2>&1', script)
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
        self.assertIn('timeout 3s rostopic echo -n 1 "$topic"', function)
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


if __name__ == "__main__":
    unittest.main()

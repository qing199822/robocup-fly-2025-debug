#!/usr/bin/env python3

import os
import pathlib
import shutil
import subprocess
import tempfile
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
LAUNCHER = ROOT / "1.sh"
MARKER = "ROBOCUP_OFFICIAL_ROOTS_READONLY"


class OfficialReadonlySandboxTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.text = LAUNCHER.read_text(encoding="utf-8")

    def test_launcher_reexecs_once_with_exact_readonly_binds(self):
        self.assertIn("ensure_official_readonly_sandbox() {", self.text)
        self.assertIn('if [ "${' + MARKER + ':-}" = 1 ]; then', self.text)
        self.assertEqual(1, self.text.count('export ' + MARKER + '=1'))
        self.assertIn('--ro-bind "$PX4_DIR" "$PX4_DIR"', self.text)
        self.assertIn('--ro-bind "$XTDRONE_DIR" "$XTDRONE_DIR"', self.text)
        self.assertIn(
            '--ro-bind "$GAZEBO_MODELS_DIR" "$GAZEBO_MODELS_DIR"', self.text
        )
        self.assertIn(
            '--ro-bind "$XTDRONE_PYTHONPATH" "$XTDRONE_PYTHONPATH"', self.text
        )
        self.assertIn('--json-status-fd "$status_fd"', self.text)
        self.assertIn('/usr/bin/setsid "$bwrap_path"', self.text)
        self.assertIn(
            'kill -s "$signal_name" "$OFFICIAL_SANDBOX_CHILD_PID"', self.text
        )
        self.assertIn(
            'kill -TERM "$OFFICIAL_SANDBOX_CHILD_PID"', self.text
        )
        self.assertIn("INT) sandbox_status=130", self.text)
        self.assertIn('"$SCRIPT_DIR/1.sh" "$@"', self.text)
        self.assertLess(
            self.text.index('ensure_official_readonly_sandbox "$@"'),
            self.text.index("trap 'handle_exit $?' EXIT"),
        )

    def test_real_bwrap_blocks_direct_and_indirect_official_writes(self):
        bwrap = shutil.which("bwrap")
        self.assertIsNotNone(bwrap, "bubblewrap must be installed")
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            px4 = root / "PX4_Firmware"
            xtdrone = root / "XTDrone"
            gazebo_models = root / "gazebo_models"
            pythonpath = root / "pythonpath"
            workspace = root / "workspace"
            for path in (px4, xtdrone, gazebo_models, pythonpath, workspace):
                path.mkdir()
            probe = (
                'if touch "$PX4_DIR/direct" 2>/dev/null; then exit 10; fi; '
                'alias_root="$XTDRONE_DIR"; '
                'if touch "$alias_root/indirect" 2>/dev/null; then exit 11; fi; '
                'if touch "$GAZEBO_MODELS_DIR/direct" 2>/dev/null; then exit 12; fi; '
                'alias_root="$XTDRONE_PYTHONPATH"; '
                'if touch "$alias_root/indirect" 2>/dev/null; then exit 13; fi; '
                'touch "$WORKSPACE_DIR/writable"'
            )
            result = subprocess.run(
                [
                    bwrap,
                    "--die-with-parent",
                    "--dev-bind",
                    "/",
                    "/",
                    "--ro-bind",
                    str(px4),
                    str(px4),
                    "--ro-bind",
                    str(xtdrone),
                    str(xtdrone),
                    "--ro-bind",
                    str(gazebo_models),
                    str(gazebo_models),
                    "--ro-bind",
                    str(pythonpath),
                    str(pythonpath),
                    "--setenv",
                    "PX4_DIR",
                    str(px4),
                    "--setenv",
                    "XTDRONE_DIR",
                    str(xtdrone),
                    "--setenv",
                    "GAZEBO_MODELS_DIR",
                    str(gazebo_models),
                    "--setenv",
                    "XTDRONE_PYTHONPATH",
                    str(pythonpath),
                    "--setenv",
                    "WORKSPACE_DIR",
                    str(workspace),
                    "sh",
                    "-c",
                    probe,
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=10,
            )
            self.assertEqual(0, result.returncode, result.stderr)
            self.assertFalse((px4 / "direct").exists())
            self.assertFalse((xtdrone / "indirect").exists())
            self.assertFalse((gazebo_models / "direct").exists())
            self.assertFalse((pythonpath / "indirect").exists())
            self.assertTrue((workspace / "writable").is_file())

    def test_preset_marker_rejects_unprotected_gazebo_and_python_roots(self):
        bwrap = shutil.which("bwrap")
        self.assertIsNotNone(bwrap, "bubblewrap must be installed")
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            px4 = root / "PX4_Firmware"
            xtdrone = root / "XTDrone"
            gazebo_models = root / "gazebo_models"
            pythonpath = root / "pythonpath"
            for path in (px4, xtdrone, gazebo_models, pythonpath):
                path.mkdir()

            base = [
                bwrap,
                "--die-with-parent",
                "--dev-bind",
                "/",
                "/",
                "--ro-bind",
                str(px4),
                str(px4),
                "--ro-bind",
                str(xtdrone),
                str(xtdrone),
            ]
            environment = os.environ.copy()
            environment.update(
                {
                    MARKER: "1",
                    "PX4_DIR": str(px4),
                    "XTDRONE_DIR": str(xtdrone),
                    "GAZEBO_MODELS_DIR": str(gazebo_models),
                    "XTDRONE_PYTHONPATH": str(pythonpath),
                }
            )
            cases = (
                ([], "GAZEBO_MODELS_DIR"),
                (
                    [
                        "--ro-bind",
                        str(gazebo_models),
                        str(gazebo_models),
                    ],
                    "XTDRONE_PYTHONPATH",
                ),
            )
            for extra_binds, expected_root in cases:
                with self.subTest(expected_root=expected_root):
                    result = subprocess.run(
                        [*base, *extra_binds, str(LAUNCHER), "7"],
                        stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE,
                        text=True,
                        env=environment,
                        timeout=5,
                    )
                    self.assertNotEqual(0, result.returncode)
                    self.assertIn(expected_root, result.stderr)
                    self.assertIn("只读挂载", result.stderr)

    def test_missing_bwrap_fails_before_logger_or_preflight(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            workspace = root / "workspace"
            fake_bin = root / "bin"
            px4 = root / "PX4_Firmware"
            xtdrone = root / "XTDrone"
            for path in (workspace, fake_bin, px4, xtdrone):
                path.mkdir()
            script = workspace / "1.sh"
            script.write_bytes(LAUNCHER.read_bytes())
            script.chmod(0o755)
            (fake_bin / "dirname").symlink_to("/usr/bin/dirname")
            environment = os.environ.copy()
            environment.update(
                {
                    "PATH": str(fake_bin),
                    "PX4_DIR": str(px4),
                    "XTDRONE_DIR": str(xtdrone),
                }
            )
            result = subprocess.run(
                [str(script)],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                env=environment,
                timeout=5,
            )
            self.assertNotEqual(0, result.returncode)
            self.assertIn("sudo apt install bubblewrap", result.stderr)
            self.assertFalse((workspace / "logs").exists())

    def test_preset_marker_does_not_bypass_readonly_mount_proof(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            workspace = root / "workspace"
            px4 = root / "PX4_Firmware"
            xtdrone = root / "XTDrone"
            for path in (workspace, px4, xtdrone):
                path.mkdir()
            script = workspace / "1.sh"
            script.write_bytes(LAUNCHER.read_bytes())
            script.chmod(0o755)
            environment = os.environ.copy()
            environment.update(
                {
                    MARKER: "1",
                    "PX4_DIR": str(px4),
                    "XTDRONE_DIR": str(xtdrone),
                }
            )
            result = subprocess.run(
                [str(script), "7"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                env=environment,
                timeout=5,
            )
            self.assertNotEqual(0, result.returncode)
            self.assertIn("只读挂载", result.stderr)
            self.assertFalse((workspace / "logs").exists())


if __name__ == "__main__":
    unittest.main()

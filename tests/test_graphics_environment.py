#!/usr/bin/env python3

import os
import pathlib
import shutil
import subprocess
import tempfile
import time
import unittest


PROJECT_ROOT = pathlib.Path(__file__).parents[1]
HELPER = PROJECT_ROOT / "scripts" / "graphics_environment.sh"
LAUNCHER = PROJECT_ROOT / "1.sh"


class GraphicsEnvironmentTest(unittest.TestCase):
    def test_ignores_processes_whose_name_only_starts_with_gnome_shell(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = pathlib.Path(temp_dir)
            authority = temp_path / "Xauthority"
            authority.touch()
            sleep = shutil.which("sleep")
            self.assertIsNotNone(sleep)

            desktop_executable = temp_path / "gnome-shell"
            calendar_executable = temp_path / "gnome-shell-calendar-server"
            desktop_executable.symlink_to(sleep)
            calendar_executable.symlink_to(sleep)

            desktop_environment = {
                **os.environ,
                "DISPLAY": ":8",
                "XAUTHORITY": str(authority),
                "XDG_RUNTIME_DIR": "/run/user/1000",
            }
            calendar_environment = {
                key: value
                for key, value in os.environ.items()
                if key
                not in {"DISPLAY", "XAUTHORITY", "XDG_RUNTIME_DIR"}
            }

            desktop = subprocess.Popen(
                [str(desktop_executable), "10"], env=desktop_environment
            )
            self.addCleanup(self._stop_process, desktop)
            time.sleep(0.05)
            calendar = subprocess.Popen(
                [str(calendar_executable), "10"], env=calendar_environment
            )
            self.addCleanup(self._stop_process, calendar)

            command = (
                'unset DISPLAY XAUTHORITY XDG_RUNTIME_DIR; '
                'source "$1"; '
                'ensure_graphics_environment; '
                'printf "%s|%s" "$DISPLAY" "$XAUTHORITY"'
            )
            result = subprocess.run(
                ["bash", "-c", command, "bash", str(HELPER)],
                capture_output=True,
                text=True,
                env={**os.environ, "PATH": os.environ["PATH"]},
            )

            self.assertEqual(0, result.returncode, result.stderr)
            self.assertEqual(f":8|{authority}", result.stdout)

    @staticmethod
    def _stop_process(process):
        if process.poll() is None:
            process.terminate()
            process.wait(timeout=2)

    def test_imports_missing_display_from_desktop_session(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = pathlib.Path(temp_dir)
            authority = temp_path / "Xauthority"
            authority.touch()
            environ = temp_path / "environ"
            environ.write_bytes(
                b"DISPLAY=:7\0"
                + f"XAUTHORITY={authority}\0".encode()
                + b"XDG_RUNTIME_DIR=/run/user/1000\0"
            )

            command = (
                'unset DISPLAY XAUTHORITY XDG_RUNTIME_DIR; '
                'source "$1"; '
                'ensure_graphics_environment "$2"; '
                'printf "%s|%s|%s" "$DISPLAY" "$XAUTHORITY" "$XDG_RUNTIME_DIR"'
            )
            result = subprocess.run(
                ["bash", "-c", command, "bash", str(HELPER), str(environ)],
                check=True,
                capture_output=True,
                text=True,
                env={**os.environ, "PATH": os.environ["PATH"]},
            )

            self.assertEqual(
                f":7|{authority}|/run/user/1000",
                result.stdout,
            )

    def test_launcher_prepares_graphics_before_starting_gazebo(self):
        script = LAUNCHER.read_text()
        prepare = "ensure_graphics_environment"
        launch = 'roslaunch "$SIMULATION_LAUNCH"'

        self.assertIn(prepare, script)
        self.assertIn(launch, script)
        self.assertLess(script.index(prepare), script.index(launch))

    def test_launcher_prepares_graphics_before_entering_readonly_sandbox(self):
        script = LAUNCHER.read_text()
        prepare = "ensure_graphics_environment || return 1"
        enter_sandbox = '/usr/bin/setsid "$bwrap_path"'

        self.assertIn(prepare, script)
        self.assertIn(enter_sandbox, script)
        self.assertLess(script.index(prepare), script.index(enter_sandbox))

    def test_resolved_graphics_environment_reaches_bwrap_child(self):
        bwrap = shutil.which("bwrap")
        self.assertIsNotNone(bwrap)
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = pathlib.Path(temp_dir)
            authority = temp_path / "Xauthority"
            authority.touch()
            environ = temp_path / "environ"
            environ.write_bytes(
                b"DISPLAY=:17\0"
                + f"XAUTHORITY={authority}\0".encode()
                + b"XDG_RUNTIME_DIR=/run/user/1000\0"
            )
            command = (
                'unset DISPLAY XAUTHORITY XDG_RUNTIME_DIR; '
                'source "$1"; '
                'ensure_graphics_environment "$2"; '
                '"$3" --die-with-parent --dev-bind / / '
                'bash -c \'printf "%s|%s|%s" "$DISPLAY" '
                '"$XAUTHORITY" "$XDG_RUNTIME_DIR"\''
            )

            result = subprocess.run(
                ["bash", "-c", command, "bash", str(HELPER), str(environ), bwrap],
                capture_output=True,
                text=True,
                env={**os.environ, "PATH": os.environ["PATH"]},
                timeout=5,
            )

            self.assertEqual(0, result.returncode, result.stderr)
            self.assertEqual(
                f":17|{authority}|/run/user/1000", result.stdout
            )


if __name__ == "__main__":
    unittest.main()

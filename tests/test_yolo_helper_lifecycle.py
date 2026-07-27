#!/usr/bin/env python3

import os
import pathlib
import signal
import subprocess
import tempfile
import textwrap
import time
import unittest


ROOT = pathlib.Path(__file__).parents[1]
SCRIPT = ROOT / "src/yolo/multi_yolo_detecting.sh"


class YoloHelperLifecycleTest(unittest.TestCase):
    def setUp(self):
        self._temporary_directory = tempfile.TemporaryDirectory()
        self.root = pathlib.Path(self._temporary_directory.name)
        self.state = self.root / "state"
        self.state.mkdir()
        self.fake_python = self.root / "fake-yolo-python"
        self.fake_python.write_text(
            textwrap.dedent(
                """\
                #!/usr/bin/python3
                import os
                import pathlib
                import signal
                import sys
                import time

                state = pathlib.Path(os.environ["FAKE_YOLO_STATE"])
                vehicle_id = sys.argv[3]
                (state / ("pid-" + vehicle_id)).write_text(str(os.getpid()))
                proc_root = os.environ.get("YOLO_TEST_PROC_ROOT")
                proc_stat = None
                start_time = 1000 + int(vehicle_id)
                if proc_root:
                    proc_stat = pathlib.Path(proc_root) / str(os.getpid()) / "stat"
                    proc_stat.parent.mkdir(parents=True)
                    proc_stat.write_text(
                        f"{os.getpid()} (fake yolo) S "
                        + " ".join(["0"] * 18)
                        + f" {start_time}\\n"
                    )

                def stop(signum, _frame):
                    (state / ("stopped-" + vehicle_id)).touch()
                    if proc_stat is not None:
                        proc_stat.unlink(missing_ok=True)
                    raise SystemExit(0)

                if os.environ.get("FAKE_YOLO_IGNORE_TERM_ID") == vehicle_id:
                    signal.signal(signal.SIGTERM, signal.SIG_IGN)
                else:
                    signal.signal(signal.SIGTERM, stop)
                signal.signal(signal.SIGINT, stop)
                signal.signal(signal.SIGHUP, stop)

                if os.environ.get("FAKE_YOLO_FAIL_ID") == vehicle_id:
                    deadline = time.monotonic() + 2
                    while len(list(state.glob("pid-*"))) < 6 and time.monotonic() < deadline:
                        time.sleep(0.01)
                    if proc_stat is not None:
                        proc_stat.write_text(
                            f"{os.getpid()} (replacement) S "
                            + " ".join(["0"] * 18)
                            + f" {start_time + 10000}\\n"
                        )
                    raise SystemExit(37)

                signal.pause()
                """
            ),
            encoding="utf-8",
        )
        self.fake_python.chmod(0o755)
        self.process = None
        self.hook = self.root / "after-worker-start"
        self.hook.write_text(
            textwrap.dedent(
                """\
                #!/bin/bash
                parent_pid="$1"
                worker_pid="$2"
                vehicle_id="$3"
                echo "$worker_pid" >> "$FAKE_YOLO_STATE/launched-pids"
                if [ -n "${YOLO_TEST_PROC_ROOT:-}" ]; then
                    while [ ! -f "$YOLO_TEST_PROC_ROOT/$worker_pid/stat" ]; do
                        sleep 0.01
                    done
                fi
                if [ "${FAKE_SIGNAL_AT_ID:-}" = "$vehicle_id" ]; then
                    kill "-${FAKE_SIGNAL_NAME}" "$parent_pid"
                fi
                """
            ),
            encoding="utf-8",
        )
        self.hook.chmod(0o755)

    def tearDown(self):
        if self.process is not None and self.process.poll() is None:
            self.process.kill()
            self.process.wait(timeout=1)
        for marker in self.state.glob("pid-*"):
            try:
                os.kill(int(marker.read_text()), signal.SIGKILL)
            except (ProcessLookupError, ValueError):
                pass
        launched = self.state / "launched-pids"
        if launched.exists():
            for value in launched.read_text().splitlines():
                try:
                    os.kill(int(value), signal.SIGKILL)
                except (ProcessLookupError, ValueError):
                    pass
        self._temporary_directory.cleanup()

    def _start(self, **extra_env):
        env = os.environ.copy()
        env.update(
            {
                "YOLO_PYTHON": str(self.fake_python),
                "FAKE_YOLO_STATE": str(self.state),
                "YOLO_SHUTDOWN_GRACE_SECONDS": "0.5",
                "YOLO_TEST_AFTER_WORKER_START_HOOK": str(self.hook),
            }
        )
        env.update(extra_env)
        self.process = subprocess.Popen(
            ["bash", str(SCRIPT)],
            cwd=ROOT,
            env=env,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )

    def _wait_for_six_nodes(self):
        deadline = time.monotonic() + 3
        while len(list(self.state.glob("pid-*"))) < 6 and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertEqual(6, len(list(self.state.glob("pid-*"))))

    def _assert_recorded_nodes_stopped(self):
        for marker in self.state.glob("pid-*"):
            pid = int(marker.read_text())
            with self.assertRaises(ProcessLookupError, msg=f"YOLO node {pid} survived"):
                os.kill(pid, 0)

    def test_stays_alive_until_term_then_reaps_all_six_nodes(self):
        self._start()
        self._wait_for_six_nodes()

        self.assertIsNone(self.process.poll(), "helper returned while YOLO nodes were alive")
        self.process.terminate()
        returncode = self.process.wait(timeout=3)

        self.assertEqual(143, returncode)
        self.assertEqual(6, len(list(self.state.glob("stopped-*"))))
        self._assert_recorded_nodes_stopped()

    def test_node_failure_stops_siblings_and_propagates_failure(self):
        self._start(FAKE_YOLO_FAIL_ID="2")
        self._wait_for_six_nodes()

        returncode = self.process.wait(timeout=3)

        self.assertEqual(37, returncode)
        self.assertEqual(5, len(list(self.state.glob("stopped-*"))))
        self._assert_recorded_nodes_stopped()

    def test_watchdog_skips_reused_pid_but_kills_owned_term_ignoring_worker(self):
        proc_root = self.root / "proc"
        proc_root.mkdir()
        kill_log = self.state / "kill-log"
        bash_env = self.root / "bash-env"
        bash_env.write_text(
            textwrap.dedent(
                """\
                kill() {
                    printf '%s\\n' "$*" >> "$FAKE_KILL_LOG"
                    if [ "$1" = -0 ] && [ -f "$FAKE_REPLACED_PID_FILE" ] && \
                       [ "$2" = "$(cat "$FAKE_REPLACED_PID_FILE")" ]; then
                        return 0
                    fi
                    builtin kill "$@"
                }
                """
            ),
            encoding="utf-8",
        )
        self._start(
            BASH_ENV=str(bash_env),
            FAKE_KILL_LOG=str(kill_log),
            FAKE_REPLACED_PID_FILE=str(self.state / "pid-2"),
            FAKE_YOLO_FAIL_ID="2",
            FAKE_YOLO_IGNORE_TERM_ID="5",
            YOLO_TEST_PROC_ROOT=str(proc_root),
            YOLO_SHUTDOWN_GRACE_SECONDS="0.1",
        )
        self._wait_for_six_nodes()

        returncode = self.process.wait(timeout=3)
        replaced_pid = (self.state / "pid-2").read_text()
        stubborn_pid = (self.state / "pid-5").read_text()
        kill_calls = kill_log.read_text().splitlines()

        self.assertEqual(37, returncode)
        self.assertNotIn(f"-KILL {replaced_pid}", kill_calls)
        self.assertIn(f"-KILL {stubborn_pid}", kill_calls)
        self._assert_recorded_nodes_stopped()

    def _assert_signal_during_registration_is_clean(self, signal_name, vehicle_id, status):
        self._start(FAKE_SIGNAL_NAME=signal_name, FAKE_SIGNAL_AT_ID=str(vehicle_id))
        returncode = self.process.wait(timeout=3)

        self.assertEqual(status, returncode)
        launched = [
            int(value)
            for value in (self.state / "launched-pids").read_text().splitlines()
        ]
        self.assertGreaterEqual(len(launched), 1)
        for pid in launched:
            with self.assertRaises(ProcessLookupError, msg=f"worker {pid} leaked"):
                os.kill(pid, 0)

    def test_term_during_first_worker_registration_leaves_no_worker(self):
        self._assert_signal_during_registration_is_clean("TERM", 0, 143)

    def test_int_during_middle_worker_registration_leaves_no_worker(self):
        self._assert_signal_during_registration_is_clean("INT", 3, 130)

    def test_hup_during_middle_worker_registration_leaves_no_worker(self):
        self._assert_signal_during_registration_is_clean("HUP", 2, 129)

    def test_term_at_wait_boundary_cleans_all_six_workers(self):
        bash_env = self.root / "wait-boundary-bash-env"
        marker = self.state / "wait-boundary-reached"
        bash_env.write_text(
            textwrap.dedent(
                """\
                wait() {
                    if [ "$1" = -n ] && [ ! -f "$FAKE_WAIT_BOUNDARY_MARKER" ]; then
                        touch "$FAKE_WAIT_BOUNDARY_MARKER"
                        builtin kill -TERM "$$"
                    fi
                    builtin wait "$@"
                }
                """
            ),
            encoding="utf-8",
        )
        self._start(
            BASH_ENV=str(bash_env),
            FAKE_WAIT_BOUNDARY_MARKER=str(marker),
        )

        returncode = self.process.wait(timeout=3)
        launched = [
            int(value)
            for value in (self.state / "launched-pids").read_text().splitlines()
        ]

        self.assertTrue(marker.exists())
        self.assertEqual(143, returncode)
        self.assertEqual(6, len(launched))
        for pid in launched:
            with self.assertRaises(ProcessLookupError, msg=f"worker {pid} leaked"):
                os.kill(pid, 0)


if __name__ == "__main__":
    unittest.main()

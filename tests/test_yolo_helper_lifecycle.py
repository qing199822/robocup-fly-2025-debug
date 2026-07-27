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

                def stop(signum, _frame):
                    (state / ("stopped-" + vehicle_id)).touch()
                    raise SystemExit(0)

                signal.signal(signal.SIGTERM, stop)
                signal.signal(signal.SIGINT, stop)
                signal.signal(signal.SIGHUP, stop)

                if os.environ.get("FAKE_YOLO_FAIL_ID") == vehicle_id:
                    deadline = time.monotonic() + 2
                    while len(list(state.glob("pid-*"))) < 6 and time.monotonic() < deadline:
                        time.sleep(0.01)
                    raise SystemExit(37)

                signal.pause()
                """
            ),
            encoding="utf-8",
        )
        self.fake_python.chmod(0o755)
        self.process = None

    def tearDown(self):
        if self.process is not None and self.process.poll() is None:
            self.process.kill()
            self.process.wait(timeout=1)
        for marker in self.state.glob("pid-*"):
            try:
                os.kill(int(marker.read_text()), signal.SIGKILL)
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


if __name__ == "__main__":
    unittest.main()

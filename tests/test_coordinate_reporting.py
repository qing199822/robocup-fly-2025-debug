#!/usr/bin/env python3

import pathlib
import sys
import unittest


YOLO_DIR = pathlib.Path(__file__).resolve().parents[1] / "src" / "yolo"
sys.path.insert(0, str(YOLO_DIR))

from coordinate_reporting import publish_actor_info_with_heartbeat


class RecordingPublisher:
    def __init__(self, name, calls, error=None):
        self.name = name
        self.calls = calls
        self.error = error

    def publish(self, message):
        self.calls.append((self.name, message))
        if self.error is not None:
            raise self.error


class CoordinateReportingTest(unittest.TestCase):
    def test_actor_report_is_published_before_heartbeat(self):
        calls = []
        actor_message = object()
        heartbeat = object()

        publish_actor_info_with_heartbeat(
            RecordingPublisher("actor", calls),
            actor_message,
            RecordingPublisher("heartbeat", calls),
            heartbeat,
        )

        self.assertEqual(
            [("actor", actor_message), ("heartbeat", heartbeat)], calls
        )

    def test_actor_publish_failure_prevents_heartbeat(self):
        calls = []

        with self.assertRaisesRegex(RuntimeError, "publish failed"):
            publish_actor_info_with_heartbeat(
                RecordingPublisher(
                    "actor", calls, RuntimeError("publish failed")
                ),
                object(),
                RecordingPublisher("heartbeat", calls),
                object(),
            )

        self.assertEqual(["actor"], [name for name, _ in calls])


if __name__ == "__main__":
    unittest.main()

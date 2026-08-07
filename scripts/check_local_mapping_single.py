#!/usr/bin/env python3

import argparse
import math
import re
import sys
import threading
import time


MINIMUM_RATE_HZ = 5.0
MAXIMUM_DEPTH_AGE_S = 0.50
MAXIMUM_OUTPUT_GAP_S = 0.25
FUTURE_DEPTH_TOLERANCE_S = 0.001
SUBSCRIBER_CONNECTION_TIMEOUT_S = 5.0
INPUT_NAMES = ("depth", "camera_info", "global_odom", "bounding_boxes")
FRAME_NAMES = ("health", "static_cloud", "dynamic_cloud", "clearance")
RATE_NAMES = ("health", "static_cloud", "dynamic_cloud")
REQUIRED_SAMPLE_FIELDS = (
    "health_rate_hz",
    "max_depth_age_s",
    "planner_depth_publishers",
    "control_publishers",
    "static_cloud_rate_hz",
    "dynamic_cloud_rate_hz",
    "received_inputs",
    "frame_ids",
    "sample_start_s",
    "sample_end_s",
    "output_receive_times",
)


def _format_metric(value):
    try:
        return f"{float(value):.3f}"
    except (TypeError, ValueError):
        return repr(value)


def _finite_number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def observed_rate_hz(receive_times):
    if len(receive_times) < 2:
        return 0.0
    first = receive_times[0]
    last = receive_times[-1]
    if not _finite_number(first) or not _finite_number(last) or last <= first:
        return 0.0
    return (len(receive_times) - 1) / (last - first)


def depth_age_s(now, stamp):
    if not _finite_number(now) or not _finite_number(stamp):
        return float("inf")
    age = now - stamp
    if age < -FUTURE_DEPTH_TOLERANCE_S:
        return float("inf")
    return max(0.0, age)


def validate_sample(sample):
    errors = [
        f"missing sample field: {field}"
        for field in REQUIRED_SAMPLE_FIELDS
        if field not in sample
    ]
    if errors:
        return errors

    health_rate = sample.get("health_rate_hz")
    if not _finite_number(health_rate) or health_rate < MINIMUM_RATE_HZ:
        errors.append(
            "health_rate_hz must be at least 5.0 "
            f"(got {_format_metric(health_rate)})"
        )

    max_depth_age = sample.get("max_depth_age_s")
    if (
        not _finite_number(max_depth_age)
        or max_depth_age > MAXIMUM_DEPTH_AGE_S
    ):
        errors.append(
            "max_depth_age_s must be at most 0.50 "
            f"(got {_format_metric(max_depth_age)})"
        )

    planner_publishers = sample.get("planner_depth_publishers")
    if planner_publishers != 1:
        errors.append(
            "planner_depth_publishers must equal 1 "
            f"(got {planner_publishers!r})"
        )

    control_publishers_value = sample["control_publishers"]
    if not isinstance(control_publishers_value, (list, tuple, set)):
        errors.append("control_publishers must be a sequence")
        control_publishers_value = []
    control_publishers = sorted(
        str(publisher) for publisher in control_publishers_value
    )
    if control_publishers:
        errors.append(
            f"control_publishers must be empty (got {control_publishers!r})"
        )

    for name in ("static_cloud", "dynamic_cloud"):
        key = f"{name}_rate_hz"
        rate = sample[key]
        if not _finite_number(rate) or rate < MINIMUM_RATE_HZ:
            errors.append(
                f"{key} must be at least 5.0 (got {_format_metric(rate)})"
            )

    received_inputs = sample["received_inputs"]
    if not isinstance(received_inputs, dict):
        errors.append("received_inputs must be a mapping")
    else:
        for name in INPUT_NAMES:
            received = received_inputs.get(name, False)
            if received is False:
                errors.append(f"input {name} was not received")
            elif received is not True:
                errors.append(
                    f"input {name} must be boolean true (got {received!r})"
                )

    frame_ids = sample["frame_ids"]
    if not isinstance(frame_ids, dict):
        errors.append("frame_ids must be a mapping")
    else:
        for name in FRAME_NAMES:
            observed = frame_ids.get(name, "")
            if isinstance(observed, str):
                if observed == "map":
                    continue
                errors.append(
                    f"{name} frame_id must equal map (got {observed!r})"
                )
                continue
            if isinstance(observed, (list, tuple, set)):
                observed_frames = sorted(set(observed))
                if observed_frames == ["map"]:
                    continue
                errors.append(
                    f"{name} frame_ids must equal ['map'] "
                    f"(got {observed_frames!r})"
                )
                continue
            errors.append(
                f"{name} frame_ids must be a string or sequence of strings"
            )

    sample_start = sample["sample_start_s"]
    sample_end = sample["sample_end_s"]
    valid_window = (
        _finite_number(sample_start)
        and _finite_number(sample_end)
        and sample_end > sample_start
    )
    if not valid_window:
        errors.append("sample window must have finite increasing start/end")

    output_receive_times = sample["output_receive_times"]
    if not isinstance(output_receive_times, dict):
        errors.append("output_receive_times must be a mapping")
        return errors

    for name in RATE_NAMES:
        receive_times = output_receive_times.get(name)
        if (
            not isinstance(receive_times, (list, tuple))
            or len(receive_times) < 2
            or any(not _finite_number(value) for value in receive_times)
            or any(
                current < previous
                for previous, current in zip(
                    receive_times, receive_times[1:]
                )
            )
        ):
            errors.append(
                f"{name} receive_times must contain at least 2 "
                "finite ordered timestamps"
            )
            continue
        if not valid_window:
            continue

        first_gap = receive_times[0] - sample_start
        last_gap = sample_end - receive_times[-1]
        maximum_gap = max(
            current - previous
            for previous, current in zip(receive_times, receive_times[1:])
        )
        if first_gap < 0.0 or first_gap > MAXIMUM_OUTPUT_GAP_S + 1e-9:
            errors.append(
                f"{name} first receive gap must be at most 0.250s "
                f"(got {first_gap:.3f}s)"
            )
        if last_gap < 0.0 or last_gap > MAXIMUM_OUTPUT_GAP_S + 1e-9:
            errors.append(
                f"{name} last receive gap must be at most 0.250s "
                f"(got {last_gap:.3f}s)"
            )
        if maximum_gap > MAXIMUM_OUTPUT_GAP_S + 1e-9:
            errors.append(
                f"{name} maximum receive gap must be at most 0.250s "
                f"(got {maximum_gap:.3f}s)"
            )

    return errors


def _is_control_topic(topic):
    control_names = {
        "cmd_vel",
        "cmd_vel_flu",
        "raw_cmd",
        "raw_cmd_vel",
        "cmd",
        "pose_cmd",
        "position_cmd",
        "velocity_command",
    }
    return any(
        part == "mux_inputs"
        or part in control_names
        or part.endswith("cmd_vel")
        or part.startswith("setpoint_")
        for part in topic.split("/")
    )


def wait_for_subscriber_connections(
    subscribers_by_topic,
    timeout_s=SUBSCRIBER_CONNECTION_TIMEOUT_S,
    monotonic=time.monotonic,
    sleep=time.sleep,
):
    if not _finite_number(timeout_s) or timeout_s <= 0.0:
        raise ValueError("subscriber connection timeout must be positive")
    deadline = monotonic() + timeout_s
    while True:
        missing = sorted(
            topic
            for topic, subscriber in subscribers_by_topic.items()
            if subscriber.get_num_connections() < 1
        )
        if not missing:
            return
        now = monotonic()
        if now >= deadline:
            raise RuntimeError(
                "subscriber connection timeout; missing topics: "
                + ", ".join(missing)
            )
        sleep(min(0.05, deadline - now))


class _SampleCollector:
    def __init__(self, rospy, monotonic=time.monotonic):
        self._rospy = rospy
        self._monotonic = monotonic
        self._lock = threading.Lock()
        self.received_inputs = {name: False for name in INPUT_NAMES}
        self.frame_ids = {name: set() for name in FRAME_NAMES}
        self.receive_times = {name: [] for name in RATE_NAMES}
        self.max_depth_age_s = 0.0

    def input_callback(self, name):
        def callback(_message):
            with self._lock:
                self.received_inputs[name] = True

        return callback

    def depth_callback(self, message):
        try:
            now = self._rospy.Time.now().to_sec()
            stamp = message.header.stamp.to_sec()
            age = depth_age_s(now, stamp)
        except Exception:
            age = float("inf")
        with self._lock:
            self.received_inputs["depth"] = True
            self.max_depth_age_s = max(self.max_depth_age_s, age)

    def output_callback(self, name):
        def callback(message):
            with self._lock:
                received_at = self._monotonic()
                self.frame_ids[name].add(message.header.frame_id)
                if name in self.receive_times:
                    self.receive_times[name].append(received_at)

        return callback

    def start_sample(self):
        with self._lock:
            self.received_inputs = {name: False for name in INPUT_NAMES}
            self.frame_ids = {name: set() for name in FRAME_NAMES}
            self.receive_times = {name: [] for name in RATE_NAMES}
            self.max_depth_age_s = 0.0
            return self._monotonic()

    def finish_sample(self, sample_start):
        with self._lock:
            sample_end = self._monotonic()
            receive_times = {
                name: [
                    value
                    for value in self.receive_times[name]
                    if sample_start <= value <= sample_end
                ]
                for name in RATE_NAMES
            }
            sample = {
                "max_depth_age_s": self.max_depth_age_s,
                "received_inputs": dict(self.received_inputs),
                "frame_ids": {
                    name: sorted(values)
                    for name, values in self.frame_ids.items()
                },
                "sample_start_s": sample_start,
                "sample_end_s": sample_end,
                "output_receive_times": receive_times,
            }
            for name in RATE_NAMES:
                sample[f"{name}_rate_hz"] = observed_rate_hz(
                    receive_times[name]
                )
            return sample


def collect_sample(vehicle, duration):
    import rosgraph
    import rospy
    from darknet_ros_msgs.msg import BoundingBoxes
    from nav_msgs.msg import Odometry
    from search_msgs.msg import LocalClearance, PerceptionHealth
    from sensor_msgs.msg import CameraInfo, Image, PointCloud2

    if not re.fullmatch(r"[A-Za-z0-9_]+", vehicle):
        raise ValueError(f"invalid vehicle name: {vehicle!r}")
    if not _finite_number(duration) or duration <= 0.0:
        raise ValueError("duration must be finite and greater than zero")

    rospy.init_node("check_local_mapping_single", anonymous=True)
    master = rosgraph.Master(rospy.get_name())
    master.getPid()

    prefix = f"/{vehicle}"
    topics = {
        "depth": f"{prefix}/realsense/depth_camera/depth/image_raw",
        "camera_info": f"{prefix}/realsense/depth_camera/depth/camera_info",
        "global_odom": f"{prefix}/global_odom",
        "bounding_boxes": f"{prefix}/yolo11n/bounding_boxes",
        "health": f"{prefix}/local_mapping/health",
        "static_cloud": f"{prefix}/local_mapping/static_cloud",
        "dynamic_cloud": f"{prefix}/local_mapping/dynamic_cloud",
        "clearance": f"{prefix}/local_mapping/clearance",
        "planner_depth": f"{prefix}/local_mapping/planner_depth",
    }
    collector = _SampleCollector(rospy)
    subscribers_by_topic = {}
    try:
        subscribers_by_topic = {
            topics["depth"]: rospy.Subscriber(
                topics["depth"], Image, collector.depth_callback, queue_size=10
            ),
            topics["camera_info"]: rospy.Subscriber(
                topics["camera_info"],
                CameraInfo,
                collector.input_callback("camera_info"),
                queue_size=10,
            ),
            topics["global_odom"]: rospy.Subscriber(
                topics["global_odom"],
                Odometry,
                collector.input_callback("global_odom"),
                queue_size=10,
            ),
            topics["bounding_boxes"]: rospy.Subscriber(
                topics["bounding_boxes"],
                BoundingBoxes,
                collector.input_callback("bounding_boxes"),
                queue_size=10,
            ),
            topics["health"]: rospy.Subscriber(
                topics["health"],
                PerceptionHealth,
                collector.output_callback("health"),
                queue_size=100,
            ),
            topics["static_cloud"]: rospy.Subscriber(
                topics["static_cloud"],
                PointCloud2,
                collector.output_callback("static_cloud"),
                queue_size=100,
            ),
            topics["dynamic_cloud"]: rospy.Subscriber(
                topics["dynamic_cloud"],
                PointCloud2,
                collector.output_callback("dynamic_cloud"),
                queue_size=100,
            ),
            topics["clearance"]: rospy.Subscriber(
                topics["clearance"],
                LocalClearance,
                collector.output_callback("clearance"),
                queue_size=100,
            ),
        }
        wait_for_subscriber_connections(subscribers_by_topic)

        sample_start = collector.start_sample()
        deadline = sample_start + duration
        while time.monotonic() < deadline:
            if rospy.is_shutdown():
                raise RuntimeError("ROS shutdown before sampling completed")
            time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))

        sample = collector.finish_sample(sample_start)
        publishers, _, _ = master.getSystemState()
        publishers_by_topic = {
            topic: sorted(set(nodes)) for topic, nodes in publishers
        }
        sample["planner_depth_publishers"] = len(
            publishers_by_topic.get(topics["planner_depth"], [])
        )

        expected_node = f"local_mapping_{vehicle}"
        control_publishers = []
        for topic, nodes in publishers_by_topic.items():
            if not _is_control_topic(topic):
                continue
            for node in nodes:
                if node.rsplit("/", 1)[-1] == expected_node:
                    control_publishers.append(f"{node} on {topic}")
        sample["control_publishers"] = sorted(control_publishers)
        return sample
    finally:
        for subscriber in subscribers_by_topic.values():
            subscriber.unregister()


def build_argument_parser():
    parser = argparse.ArgumentParser(
        description="Check the single-drone local mapping runtime contract."
    )
    parser.add_argument("--vehicle", default="typhoon_h480_0")
    parser.add_argument("--duration", type=float, default=30.0)
    return parser


def main(argv=None):
    args = build_argument_parser().parse_args(argv)
    try:
        sample = collect_sample(args.vehicle, args.duration)
    except Exception as error:
        print(f"FAIL local mapping single-drone contract: {error}", file=sys.stderr)
        return 1

    errors = validate_sample(sample)
    if errors:
        for error in errors:
            print(f"FAIL {error}", file=sys.stderr)
        return 1

    print("PASS local mapping single-drone contract")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

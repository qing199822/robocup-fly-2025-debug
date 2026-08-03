#!/usr/bin/env python3

import argparse
import sys


def validate_publishers(publishers, count, vehicle_type):
    by_topic = dict(publishers)
    errors = []
    for drone_id in range(count):
        topic = f"/xtdrone/{vehicle_type}_{drone_id}/cmd_vel_flu"
        expected = [f"/{vehicle_type}_{drone_id}/safety_filter"]
        actual = sorted(by_topic.get(topic, []))
        if actual != expected:
            errors.append(
                f"vehicle {drone_id}: expected {expected}, got {actual} on {topic}"
            )
    return errors


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--count", type=int, default=6)
    parser.add_argument("--vehicle-type", default="typhoon_h480")
    args = parser.parse_args()

    import rosgraph

    master = rosgraph.Master("/competition_clean_final_publisher_guard")
    publishers, _, _ = master.getSystemState()
    errors = validate_publishers(publishers, args.count, args.vehicle_type)
    if errors:
        for error in errors:
            print(f"FAIL {error}", file=sys.stderr)
        return 1
    print("PASS final control topics have one safety_filter publisher each")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

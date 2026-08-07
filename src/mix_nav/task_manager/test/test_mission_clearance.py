#!/usr/bin/env python3

import json
import unittest
from pathlib import Path


MISSION_FILE = Path(__file__).resolve().parents[1] / "launch" / "mission_down.json"
MISSION_FILES = sorted(MISSION_FILE.parent.glob("mission_*.json"))
HORIZONTAL_CLEARANCE = 2.0
WAYPOINT_SWITCH_ALLOWANCE = 3.0
MAX_FLIGHT_ALTITUDE = 6.0
INTER_VEHICLE_CLEARANCE = 5.0
PATROL_ALTITUDE = 3.5
LAUNCH_BOX = (-25.0, 0.0, -12.0, 12.0)
EPSILON = 1e-9

INITIAL_POSITIONS = {
    "typhoon_h480_0": (-17.0, -3.0, 3.0),
    "typhoon_h480_1": (-14.0, -3.0, 3.0),
    "typhoon_h480_2": (-17.0, 0.0, 3.0),
    "typhoon_h480_3": (-14.0, 0.0, 3.0),
    "typhoon_h480_4": (-17.0, 3.0, 3.0),
    "typhoon_h480_5": (-14.0, 3.0, 3.0),
}

# Conservative world-frame AABBs derived from robocup.world collision
# transforms and model collision meshes; values are rounded outward.
SWITCH_SENSITIVE_OBSTACLES = (
    ("house_1_66", 54.89, 67.83, 14.91, 31.41, 7.69),
    ("house_2_71", 70.34, 83.08, 8.50, 18.30, 7.20),
    ("house_3_68", 90.10, 102.76, 11.74, 16.54, 10.62),
)

# Bounds are calculated from the collision meshes and transforms in robocup.world.
STATIC_OBSTACLES = (
    ("gas_station_73", 51.59, 72.17, -36.63, -6.64, 8.98),
    ("fast_food_93", 14.12, 39.29, -22.22, -5.64, 11.02),
    ("house_1_146", -34.34, -21.40, 17.49, 33.99, 7.69),
    ("house_1_146_clone", 6.15, 19.09, 11.19, 27.70, 7.69),
    ("house_2_125", -5.96, 6.77, -19.82, -10.03, 7.20),
    ("house_3_156", -35.78, -30.98, -25.68, -13.03, 10.62),
    ("house_3_157", -36.40, -23.75, -35.32, -30.52, 10.62),
    ("house_3_158", -28.15, -23.34, -27.65, -15.00, 10.62),
    ("lamp_post_189", -35.70, -34.50, -4.30, -3.10, 7.20),
    ("lamp_post_190", -25.80, -24.60, 3.10, 4.30, 7.20),
    ("lamp_post_191", -3.60, -2.40, -4.30, -3.10, 7.20),
    ("lamp_post_192", 8.40, 9.60, 3.10, 4.30, 7.20),
    ("lamp_post_193", 20.40, 21.60, -4.30, -3.10, 7.20),
    ("lamp_post_194", 26.40, 27.60, 3.10, 4.30, 7.20),
    ("lamp_post_195", 57.40, 58.60, -4.30, -3.10, 7.20),
    ("lamp_post_196", 70.40, 71.60, 3.10, 4.30, 7.20),
    ("lamp_post_197", 83.40, 84.60, -4.30, -3.10, 7.20),
    ("lamp_post_198", 89.90, 91.10, 3.10, 4.30, 7.20),
    *SWITCH_SENSITIVE_OBSTACLES,
)


def load_missions(path):
    with path.open(encoding="utf-8") as mission_stream:
        return json.load(mission_stream)


def mission_segments(mission):
    vehicle_id = mission["vehicle_id"]
    sx, sy, sz = INITIAL_POSITIONS[vehicle_id]
    start = {"x": sx, "y": sy, "z": sz}
    entry = mission.get("entry_waypoints", [])
    patrol = mission["waypoints"]
    route = [start] + entry + patrol[:1]
    segments = list(zip(route, route[1:]))
    segments.extend(zip(patrol, patrol[1:]))
    segments.append((patrol[-1], patrol[0]))
    return segments


def segment_intersects_box(start, end, x_min, x_max, y_min, y_max):
    """Return whether a 2-D segment intersects an axis-aligned box."""
    dx = end["x"] - start["x"]
    dy = end["y"] - start["y"]
    t_min = 0.0
    t_max = 1.0

    for origin, delta, lower, upper in (
        (start["x"], dx, x_min, x_max),
        (start["y"], dy, y_min, y_max),
    ):
        if abs(delta) < EPSILON:
            if origin < lower or origin > upper:
                return False
            continue

        entry = (lower - origin) / delta
        exit_ = (upper - origin) / delta
        if entry > exit_:
            entry, exit_ = exit_, entry
        t_min = max(t_min, entry)
        t_max = min(t_max, exit_)
        if t_min > t_max:
            return False

    return True


def _cross(first, second, third):
    return ((second["x"] - first["x"]) * (third["y"] - first["y"]) -
            (second["y"] - first["y"]) * (third["x"] - first["x"]))


def _on_segment(point, start, end):
    return (min(start["x"], end["x"]) - EPSILON <= point["x"] <=
            max(start["x"], end["x"]) + EPSILON and
            min(start["y"], end["y"]) - EPSILON <= point["y"] <=
            max(start["y"], end["y"]) + EPSILON)


def segments_intersect(first_start, first_end, second_start, second_end):
    orientations = (
        _cross(first_start, first_end, second_start),
        _cross(first_start, first_end, second_end),
        _cross(second_start, second_end, first_start),
        _cross(second_start, second_end, first_end),
    )
    if ((orientations[0] > EPSILON and orientations[1] < -EPSILON or
         orientations[0] < -EPSILON and orientations[1] > EPSILON) and
            (orientations[2] > EPSILON and orientations[3] < -EPSILON or
             orientations[2] < -EPSILON and orientations[3] > EPSILON)):
        return True

    return any((
        abs(orientations[0]) <= EPSILON and
        _on_segment(second_start, first_start, first_end),
        abs(orientations[1]) <= EPSILON and
        _on_segment(second_end, first_start, first_end),
        abs(orientations[2]) <= EPSILON and
        _on_segment(first_start, second_start, second_end),
        abs(orientations[3]) <= EPSILON and
        _on_segment(first_end, second_start, second_end),
    ))


def point_to_segment_distance(point, start, end):
    dx = end["x"] - start["x"]
    dy = end["y"] - start["y"]
    denominator = dx * dx + dy * dy
    if denominator == 0.0:
        return ((point["x"] - start["x"]) ** 2 +
                (point["y"] - start["y"]) ** 2) ** 0.5
    projection = ((point["x"] - start["x"]) * dx +
                  (point["y"] - start["y"]) * dy) / denominator
    projection = max(0.0, min(1.0, projection))
    nearest_x = start["x"] + projection * dx
    nearest_y = start["y"] + projection * dy
    return ((point["x"] - nearest_x) ** 2 +
            (point["y"] - nearest_y) ** 2) ** 0.5


def segment_distance(first_start, first_end, second_start, second_end):
    if segments_intersect(first_start, first_end, second_start, second_end):
        return 0.0
    return min(
        point_to_segment_distance(first_start, second_start, second_end),
        point_to_segment_distance(first_end, second_start, second_end),
        point_to_segment_distance(second_start, first_start, first_end),
        point_to_segment_distance(second_end, first_start, first_end),
    )


def _interpolate(start, end, parameter):
    return {
        coordinate: start[coordinate] +
        parameter * (end[coordinate] - start[coordinate])
        for coordinate in ("x", "y", "z")
    }


def clip_segment_outside_box(start, end, bounds=LAUNCH_BOX):
    """Return only the portions of a segment outside an axis-aligned box."""
    x_min, x_max, y_min, y_max = bounds
    dx = end["x"] - start["x"]
    dy = end["y"] - start["y"]
    t_enter = 0.0
    t_exit = 1.0

    for coefficient, distance in (
        (-dx, start["x"] - x_min),
        (dx, x_max - start["x"]),
        (-dy, start["y"] - y_min),
        (dy, y_max - start["y"]),
    ):
        if abs(coefficient) <= EPSILON:
            if distance < 0.0:
                return [(start, end)]
            continue
        ratio = distance / coefficient
        if coefficient < 0.0:
            t_enter = max(t_enter, ratio)
        else:
            t_exit = min(t_exit, ratio)
        if t_enter > t_exit:
            return [(start, end)]

    outside = []
    if t_enter > EPSILON:
        outside.append((start, _interpolate(start, end, t_enter)))
    if t_exit < 1.0 - EPSILON:
        outside.append((_interpolate(start, end, t_exit), end))
    return outside


def collect_obstacle_collisions(
        missions,
        obstacles=STATIC_OBSTACLES,
        horizontal_clearance=HORIZONTAL_CLEARANCE):
    collisions = []
    for mission in missions:
        vehicle_id = mission["vehicle_id"]
        for index, (start, end) in enumerate(mission_segments(mission), start=1):
            segment_floor = min(start["z"], end["z"])
            for name, x_min, x_max, y_min, y_max, z_max in obstacles:
                if segment_floor > z_max:
                    continue
                if segment_intersects_box(
                    start,
                    end,
                    x_min - horizontal_clearance,
                    x_max + horizontal_clearance,
                    y_min - horizontal_clearance,
                    y_max + horizontal_clearance,
                ):
                    collisions.append(
                        f"{vehicle_id} segment {index} intersects {name}"
                    )
    return collisions


def collect_inter_vehicle_violations(missions):
    routes = {
        mission["vehicle_id"]: [
            outside
            for segment in mission_segments(mission)
            for outside in clip_segment_outside_box(*segment)
        ]
        for mission in missions
    }
    violations = []
    vehicle_ids = sorted(routes)
    for first_index, first_id in enumerate(vehicle_ids):
        for second_id in vehicle_ids[first_index + 1:]:
            for first_segment_index, first_segment in enumerate(routes[first_id], start=1):
                for second_segment_index, second_segment in enumerate(
                        routes[second_id], start=1):
                    distance = segment_distance(
                        first_segment[0], first_segment[1],
                        second_segment[0], second_segment[1],
                    )
                    if distance < INTER_VEHICLE_CLEARANCE - EPSILON:
                        violations.append(
                            f"{first_id} segment {first_segment_index} and "
                            f"{second_id} segment {second_segment_index} have "
                            f"clearance {distance:.3f}m"
                        )
    return violations


class GeometryHelpersTest(unittest.TestCase):
    @staticmethod
    def point(x, y, z=3.5):
        return {"x": x, "y": y, "z": z}

    def test_intersecting_segments_have_zero_distance(self):
        distance = segment_distance(
            self.point(0, 0), self.point(10, 10),
            self.point(0, 10), self.point(10, 0),
        )
        self.assertEqual(0.0, distance)

    def test_collinear_overlapping_segments_have_zero_distance(self):
        distance = segment_distance(
            self.point(0, 0), self.point(10, 0),
            self.point(5, 0), self.point(15, 0),
        )
        self.assertEqual(0.0, distance)

    def test_parallel_segments_enforce_five_metre_boundary(self):
        too_close = segment_distance(
            self.point(0, 0), self.point(10, 0),
            self.point(0, 4.99), self.point(10, 4.99),
        )
        boundary = segment_distance(
            self.point(0, 0), self.point(10, 0),
            self.point(0, 5.0), self.point(10, 5.0),
        )
        self.assertLess(too_close, INTER_VEHICLE_CLEARANCE)
        self.assertGreaterEqual(boundary, INTER_VEHICLE_CLEARANCE)

    def test_launch_box_clipping_keeps_both_outside_pieces(self):
        pieces = clip_segment_outside_box(
            self.point(-30, 0), self.point(5, 0)
        )
        self.assertEqual(2, len(pieces))
        self.assertEqual(-25.0, pieces[0][1]["x"])
        self.assertEqual(0.0, pieces[1][0]["x"])


class MissionClearanceTest(unittest.TestCase):
    def test_all_mission_waypoints_are_strictly_below_six_metres(self):
        violations = []
        for mission_file in MISSION_FILES:
            for mission in load_missions(mission_file):
                points = (mission.get("entry_waypoints", []) +
                          mission["waypoints"])
                for index, waypoint in enumerate(points, start=1):
                    if waypoint["z"] >= MAX_FLIGHT_ALTITUDE:
                        violations.append(
                            f"{mission_file.name}: {mission['vehicle_id']} point "
                            f"{index} has z={waypoint['z']}"
                        )
        self.assertEqual([], violations, "\n" + "\n".join(violations))

    def test_static_baseline_has_complete_six_vehicle_schema(self):
        missions = load_missions(MISSION_FILE)
        self.assertEqual(
            set(INITIAL_POSITIONS),
            {mission["vehicle_id"] for mission in missions},
        )
        for mission in missions:
            self.assertIn("entry_waypoints", mission)
            self.assertGreaterEqual(len(mission["entry_waypoints"]), 1)
            self.assertGreaterEqual(len(mission["waypoints"]), 3)
            for waypoint in mission["entry_waypoints"] + mission["waypoints"]:
                self.assertEqual(PATROL_ALTITUDE, waypoint["z"])

    def test_all_complete_segments_clear_known_static_obstacles(self):
        collisions = collect_obstacle_collisions(load_missions(MISSION_FILE))
        self.assertEqual([], collisions, "\n" + "\n".join(collisions))

    def test_routes_clear_switch_sensitive_buildings_with_arrival_allowance(self):
        collisions = collect_obstacle_collisions(
            load_missions(MISSION_FILE),
            obstacles=SWITCH_SENSITIVE_OBSTACLES,
            horizontal_clearance=(
                HORIZONTAL_CLEARANCE + WAYPOINT_SWITCH_ALLOWANCE
            ),
        )
        self.assertEqual([], collisions, "\n" + "\n".join(collisions))

    def test_different_aircraft_routes_clear_five_metres_outside_launch_box(self):
        violations = collect_inter_vehicle_violations(load_missions(MISSION_FILE))
        self.assertEqual([], violations, "\n" + "\n".join(violations))

    def test_visible_waypoint_entry_resolves_to_runtime_mission(self):
        visible = Path(__file__).resolve().parents[4] / "waypoint" / "mission_down.json"
        self.assertTrue(visible.is_symlink())
        self.assertEqual(MISSION_FILE.resolve(), visible.resolve())


if __name__ == "__main__":
    unittest.main()

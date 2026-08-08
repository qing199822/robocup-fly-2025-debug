#!/usr/bin/env python3

import argparse
import pathlib
import subprocess
import sys


PIN = "92fe9f7227b2da819133eb8e0e8c7fc000f6ae20"
REQUIRED = (
    "src/uav_simulator/Utils/quadrotor_msgs/msg/PositionCommand.msg",
    "src/planner/traj_utils/msg/Bspline.msg",
    "src/planner/plan_manage/launch/single_run_in_sim.launch",
    "src/planner/plan_manage/launch/advanced_param.xml",
)

REQUIRED_FIELDS = {
    "src/uav_simulator/Utils/quadrotor_msgs/msg/PositionCommand.msg": (
        "Header header",
        "geometry_msgs/Point position",
        "geometry_msgs/Vector3 velocity",
        "geometry_msgs/Vector3 acceleration",
        "float64 yaw",
        "float64 yaw_dot",
    ),
    "src/planner/traj_utils/msg/Bspline.msg": (
        "int32 order",
        "time start_time",
        "int64 traj_id",
        "geometry_msgs/Point[] pos_pts",
        "float64[] knots",
    ),
    "src/planner/plan_manage/launch/advanced_param.xml": (
        "grid_map/fx",
        "grid_map/fy",
        "grid_map/cx",
        "grid_map/cy",
    ),
}


def git(repo, *args):
    return subprocess.check_output(
        ["git", "-C", str(repo), *args], text=True
    ).strip()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--ego-dir",
        default="/home/wangtao/robocup_fly/external/ego-planner-swarm",
    )
    args = parser.parse_args()
    root = pathlib.Path(args.ego_dir).resolve()
    failures = []
    if not (root / ".git").is_dir():
        failures.append(f"not a git checkout: {root}")
    else:
        if git(root, "rev-parse", "HEAD") != PIN:
            failures.append("EGO revision differs from pinned revision")
        if git(root, "status", "--porcelain"):
            failures.append("EGO working tree is modified")
    for relative in REQUIRED:
        if not (root / relative).is_file():
            failures.append(f"missing interface: {relative}")
    for relative, fields in REQUIRED_FIELDS.items():
        path = root / relative
        if not path.is_file():
            continue
        source = path.read_text(encoding="utf-8")
        for field in fields:
            if field not in source:
                failures.append(f"missing field in {relative}: {field}")
    if failures:
        print("\n".join(failures), file=sys.stderr)
        return 1
    print(f"EGO_EXTERNAL_OK {PIN}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

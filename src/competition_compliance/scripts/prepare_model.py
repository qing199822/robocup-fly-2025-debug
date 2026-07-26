#!/usr/bin/env python3

import argparse
import pathlib
import sys

from competition_compliance.manifest import (
    validate_output_path,
    verify_manifest,
    verify_versions,
)
from competition_compliance.model import ComplianceError, generate_model, load_mount_pose


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--px4-dir", required=True, type=pathlib.Path)
    parser.add_argument("--xtdrone-dir", required=True, type=pathlib.Path)
    parser.add_argument("--manifest", required=True, type=pathlib.Path)
    parser.add_argument("--mount-config", required=True, type=pathlib.Path)
    parser.add_argument("--output", required=True, type=pathlib.Path)
    args = parser.parse_args()

    manifest = verify_manifest(
        args.manifest,
        {"PX4_DIR": args.px4_dir, "XTDRONE_DIR": args.xtdrone_dir},
    )
    verify_versions(manifest, args.xtdrone_dir)
    pose = load_mount_pose(args.mount_config)
    output = validate_output_path(
        args.output,
        {"PX4_DIR": args.px4_dir, "XTDRONE_DIR": args.xtdrone_dir},
    )
    official = (
        args.xtdrone_dir
        / "sitl_config/models/typhoon_h480_realsense/typhoon_h480_realsense.sdf"
    )
    generate_model(official, output, pose)
    print(pose.to_sdf(), file=sys.stderr)
    print(output)


if __name__ == "__main__":
    try:
        main()
    except ComplianceError as error:
        print("合规自检失败：{}".format(error), file=sys.stderr)
        print(
            "恢复方法：不要修改官方目录；按 docs/TROUBLESHOOTING.md 恢复对应版本后重试。",
            file=sys.stderr,
        )
        raise SystemExit(2)

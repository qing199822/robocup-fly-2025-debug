#!/usr/bin/env python3

import argparse
import pathlib
import sys

from competition_compliance.manifest import validate_output_path
from competition_compliance.model import ComplianceError
from competition_compliance.world import generate_world


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--px4-dir", required=True, type=pathlib.Path)
    parser.add_argument("--xtdrone-dir", required=True, type=pathlib.Path)
    parser.add_argument("--input", required=True, type=pathlib.Path)
    parser.add_argument("--output", required=True, type=pathlib.Path)
    args = parser.parse_args()

    output = validate_output_path(
        args.output,
        {"PX4_DIR": args.px4_dir, "XTDRONE_DIR": args.xtdrone_dir},
    )
    generate_world(args.input, output)
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

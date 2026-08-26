#!/usr/bin/env python3

import argparse
import re
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]

DEFAULT_CONFIG = (
    PACKAGE_ROOT /
    "config/thor7_safety.yaml"
)

DEFAULT_SRDF = (
    PACKAGE_ROOT /
    "config/thor7v3.srdf"
)


def expected_block(safety):
    pose = safety["safe_pose"]

    joints = [
        "joint1",
        "joint2",
        "joint3",
        "joint4",
        "joint5",
        "joint6",
    ]

    lines = [
        '  <group_state name="safe_pose" group="thor7_arm">'
    ]

    for name in joints:
        value = float(pose[name])

        lines.append(
            f'    <joint name="{name}" value="{value:.9f}"/>'
        )

    lines.append("  </group_state>")

    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser()

    mode = parser.add_mutually_exclusive_group(
        required=True
    )

    mode.add_argument(
        "--check",
        action="store_true",
    )

    mode.add_argument(
        "--write",
        action="store_true",
    )

    parser.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_CONFIG,
    )

    parser.add_argument(
        "--srdf",
        type=Path,
        default=DEFAULT_SRDF,
    )

    args = parser.parse_args()

    safety = yaml.safe_load(
        args.config.read_text()
    )

    expected = expected_block(safety)

    text = args.srdf.read_text()

    pattern = re.compile(
        r'  <group_state name="safe_pose" '
        r'group="thor7_arm">.*?'
        r'  </group_state>',
        re.DOTALL,
    )

    matches = pattern.findall(text)

    if len(matches) != 1:
        raise RuntimeError(
            "Expected exactly one thor7_arm safe_pose "
            f"group_state, found {len(matches)}"
        )

    current = matches[0]

    if args.check:
        if current != expected:
            raise SystemExit(
                "SRDF safe_pose differs from thor7_safety.yaml. "
                "Run sync_thor7_safe_pose.py --write."
            )

        print(
            "PASS: SRDF safe_pose matches "
            "thor7_safety.yaml"
        )

        return

    updated = pattern.sub(
        expected,
        text,
        count=1,
    )

    args.srdf.write_text(updated)

    print(
        f"Updated {args.srdf} from {args.config}"
    )


if __name__ == "__main__":
    main()

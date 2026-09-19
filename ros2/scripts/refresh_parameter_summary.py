#!/usr/bin/env python3
"""Refresh the source-derived template-key count without rewriting audited prose."""

import argparse
from pathlib import Path
import re

import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    template = root / "ros2/src/mowgli_bringup/config/mowgli_robot.yaml"
    params = yaml.safe_load(template.read_text(encoding="utf-8"))["mowgli"]["ros__parameters"]
    index = root / "docs/claude/parameters.md"
    old = index.read_text(encoding="utf-8")
    new, count = re.subn(r"All \d+ template keys\.", f"All {len(params)} template keys.", old)
    if count != 1:
        raise SystemExit("Expected exactly one template-key count in parameters.md")
    if args.check:
        if old != new:
            raise SystemExit("Template-key count is stale; run ros2/scripts/refresh_parameter_summary.py")
    else:
        index.write_text(new, encoding="utf-8")
    print(f"Template summary: {len(params)} keys")


if __name__ == "__main__":
    main()

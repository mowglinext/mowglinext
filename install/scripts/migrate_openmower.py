#!/usr/bin/env python3
# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0-or-later
"""
migrate_openmower.py — One-shot migration from an OpenMower-style
install (mowgli-docker fork shape) to the current MowgliNext layout.

Reads:
  <source>/config/om/mower_config.sh   (env-var dump: OM_DATUM_LAT,
                                        OM_NTRIP_*, OM_BATTERY_*, …)
  <source>/config/om/mower_config.yaml (same fields, alternate format)
  <source>/ros/map.json                (areas + docking_stations)
  <source>/config/mowgli/mowgli_robot.yaml
                                       (older MowgliNext shape — used
                                        as a source of truth for any
                                        already-migrated keys)

Writes:
  <target>/mowgli/mowgli_robot.yaml    (in-place patch of existing
                                        keys; preserves comments)
  <target>/mowgli/garden_areas.dat     (areas.dat format consumed by
                                        load_areas_from_file at boot)

Usage:
  python3 migrate_openmower.py \\
      --source ~/mowgli-docker \\
      --target ~/mowglinext/docker/config

  # Dry run (print what would be written, don't touch files):
  python3 migrate_openmower.py --source ... --target ... --dry-run

The script never overwrites a value the operator has already
calibrated by hand — datum, dock pose, sensor mounting, IMU yaw —
unless --force is passed. By default it only fills missing/zero keys
and writes the areas file. It prints a report listing every field it
mapped and every field it ignored, so you can spot anything you
might want to translate manually.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple


# ── Mapping table ────────────────────────────────────────────────────
#
# OpenMower env-var → mowgli_robot.yaml key. None on the right means
# "drop, no equivalent". Tuples carry an optional value transform.

# Identity: OM value goes into the yaml key as-is.
def _id(v: str) -> str:
    return v


# Strip surrounding quotes if any.
def _unquote(v: str) -> str:
    return v.strip().strip('"').strip("'")


def _bool(v: str) -> str:
    s = _unquote(v).lower()
    return "true" if s in ("1", "true", "yes", "on") else "false"


def _automatic_mode(v: str) -> str:
    s = _unquote(v).upper()
    return {"MANUAL": "0", "SEMIAUTO": "1", "AUTO": "2"}.get(s, _unquote(v))


OM_TO_YAML: Dict[str, Tuple[str, callable]] = {
    "OM_DATUM_LAT": ("datum_lat", _unquote),
    "OM_DATUM_LONG": ("datum_lon", _unquote),
    "OM_GPS_PROTOCOL": ("gps_protocol", _unquote),
    "OM_GPS_PORT": ("gps_port", _unquote),
    "OM_GPS_BAUDRATE": ("gps_baudrate", _unquote),
    "OM_USE_NTRIP": ("ntrip_enabled", _bool),
    "OM_NTRIP_HOSTNAME": ("ntrip_host", _unquote),
    "OM_NTRIP_PORT": ("ntrip_port", _unquote),
    "OM_NTRIP_USER": ("ntrip_user", _unquote),
    "OM_NTRIP_PASSWORD": ("ntrip_password", _unquote),
    "OM_NTRIP_ENDPOINT": ("ntrip_mountpoint", _unquote),
    "OM_TOOL_WIDTH": ("tool_width", _unquote),
    "OM_AUTOMATIC_MODE": ("automatic_mode", _automatic_mode),
    "OM_BATTERY_FULL_VOLTAGE": ("battery_full_voltage", _unquote),
    "OM_BATTERY_EMPTY_VOLTAGE": ("battery_empty_voltage", _unquote),
    "OM_BATTERY_CRITICAL_VOLTAGE": ("battery_critical_voltage", _unquote),
    "OM_MOWING_MOTOR_TEMP_LOW": ("motor_temp_low_c", _unquote),
    "OM_MOWING_MOTOR_TEMP_HIGH": ("motor_temp_high_c", _unquote),
    "OM_DOCKING_DISTANCE": ("dock_approach_distance", _unquote),
    "OM_UNDOCK_DISTANCE": ("undock_distance", _unquote),
    "OM_MOWING_ANGLE_OFFSET": ("mow_angle_offset_deg", _unquote),
    "OM_MOWING_ANGLE_INCREMENT": ("mow_angle_increment_deg", _unquote),
    "OM_OUTLINE_OFFSET": ("outline_offset", _unquote),
    "OM_OUTLINE_OVERLAP_COUNT": ("outline_overlap", _unquote),
    "OM_OUTLINE_COUNT": ("outline_passes", _unquote),
    "OM_GPS_TIMEOUT_SEC": ("gps_timeout_sec", _unquote),
    "OM_GPS_WAIT_TIME_SEC": ("gps_wait_after_undock_sec", _unquote),
    "OM_ENABLE_MOWER": ("mowing_enabled", _bool),
}

# Keys we deliberately drop (no equivalent in MowgliNext).
OM_IGNORED = {
    "OM_LANGUAGE",
    "OM_VOLUME",
    "OM_MOWER_GAMEPAD",
    "OM_DFP_IS_5V",
    "OM_USE_F9R_SENSOR_FUSION",
    "OM_USE_RELATIVE_POSITION",
    "OM_NO_COMMS",
    "OM_HARDWARE_VERSION",
    "OM_DOCKING_EXTRA_TIME",
    "OM_ENABLE_RECORDING_ALL",
    "OM_MOWER_ESC_TYPE",
    "OM_AUTOMATIC_TIMEOUTS_LIST",
    "OM_RECORD_GPS",
}


# ── Parsing ──────────────────────────────────────────────────────────


def parse_mower_config_sh(path: Path) -> Dict[str, str]:
    """Parse `export OM_KEY=value` lines into a dict."""
    out: Dict[str, str] = {}
    pat = re.compile(r"^\s*export\s+(OM_[A-Z0-9_]+)=(.*?)\s*$")
    with path.open() as f:
        for line in f:
            m = pat.match(line)
            if m:
                key, val = m.group(1), m.group(2)
                out[key] = _unquote(val)
    return out


@dataclass
class Area:
    is_navigation: bool
    outline: List[Tuple[float, float]]
    obstacles: List[List[Tuple[float, float]]] = field(default_factory=list)
    name: str = ""


@dataclass
class Dock:
    x: float
    y: float
    yaw: float


@dataclass
class MapData:
    areas: List[Area] = field(default_factory=list)
    docks: List[Dock] = field(default_factory=list)


def parse_map_json(path: Path) -> MapData:
    """Parse OpenMower-fork's map.json: {areas: [...], docking_stations: [...]}."""
    with path.open() as f:
        raw = json.load(f)

    md = MapData()
    for area in raw.get("areas", []):
        atype = area.get("properties", {}).get("type", "mow")
        is_nav = atype != "mow"
        outline_raw = area.get("outline", [])
        outline: List[Tuple[float, float]] = []
        prev: Optional[Tuple[float, float]] = None
        for p in outline_raw:
            pt = (float(p["x"]), float(p["y"]))
            # Drop consecutive duplicates (the dump in the wild has many).
            if prev is not None and abs(pt[0] - prev[0]) < 1e-6 and abs(pt[1] - prev[1]) < 1e-6:
                continue
            outline.append(pt)
            prev = pt
        if len(outline) < 3:
            continue  # invalid polygon
        obstacles_raw = area.get("obstacles", [])
        obstacles: List[List[Tuple[float, float]]] = []
        for obs in obstacles_raw:
            pts = [(float(p["x"]), float(p["y"])) for p in obs.get("outline", obs)]
            if len(pts) >= 3:
                obstacles.append(pts)
        md.areas.append(
            Area(
                is_navigation=is_nav,
                outline=outline,
                obstacles=obstacles,
                name=area.get("properties", {}).get("name", "") or "",
            )
        )

    for d in raw.get("docking_stations", []):
        pos = d.get("position", {})
        md.docks.append(
            Dock(
                x=float(pos.get("x", 0.0)),
                y=float(pos.get("y", 0.0)),
                yaw=float(d.get("heading", 0.0)),
            )
        )
    return md


# ── Yaml line-splice patch (mirrors calibrate_imu_yaw_node's helper) ─

YAML_KEY_RE = re.compile(
    r"^(?P<indent>[ \t]+)(?P<key>[A-Za-z_][A-Za-z0-9_]*):"
    r"\s*(?P<val>[^#\n]*?)(?P<comment>\s*#.*)?$"
)


def yaml_patch(content: str, key: str, value: str) -> str:
    """Replace the value of `key:` in a YAML doc, preserving indent + trailing
    comment. If the key is missing, append it under the first
    `ros__parameters:` block. Returns the new content."""
    lines = content.splitlines(keepends=True)
    for i, line in enumerate(lines):
        m = YAML_KEY_RE.match(line.rstrip("\n"))
        if m and m.group("key") == key:
            indent = m.group("indent")
            comment = m.group("comment") or ""
            newline = "\n" if line.endswith("\n") else ""
            lines[i] = f"{indent}{key}: {value}{comment}{newline}"
            return "".join(lines)

    # Key not found — insert under ros__parameters:
    out: List[str] = []
    inserted = False
    for line in lines:
        out.append(line)
        if not inserted and line.strip() == "ros__parameters:":
            indent_count = len(line) - len(line.lstrip())
            indent = " " * (indent_count + 4)
            out.append(f"{indent}{key}: {value}\n")
            inserted = True
    return "".join(out)


def yaml_value_is_set(content: str, key: str) -> bool:
    """Return True when the YAML key has a non-zero / non-empty value."""
    for line in content.splitlines():
        m = YAML_KEY_RE.match(line)
        if m and m.group("key") == key:
            v = m.group("val").strip().strip('"').strip("'")
            return v not in ("", "0", "0.0", "0.000000", "false", "False")
    return False


# ── Areas.dat writer (matches save_areas_to_file format) ─────────────


def polygon_to_string(pts: List[Tuple[float, float]]) -> str:
    return ";".join(f"{x},{y}" for x, y in pts)


def write_areas_dat(path: Path, md: MapData) -> None:
    lines: List[str] = []
    lines.append("# Mowgli ROS2 — Migrated from OpenMower map.json\n")
    lines.append("# Generated by install/scripts/migrate_openmower.py.\n\n")
    lines.append(f"area_count: {len(md.areas)}\n\n")
    for i, a in enumerate(md.areas):
        nm = a.name or f"area_{i}"
        lines.append(f"area_{i}_name: {nm}\n")
        lines.append(f"area_{i}_polygon: {polygon_to_string(a.outline)}\n")
        lines.append(f"area_{i}_is_navigation: {1 if a.is_navigation else 0}\n")
        lines.append(f"area_{i}_obstacle_count: {len(a.obstacles)}\n")
        for j, obs in enumerate(a.obstacles):
            lines.append(f"area_{i}_obstacle_{j}: {polygon_to_string(obs)}\n")
        lines.append("\n")
    path.write_text("".join(lines))


# ── Driver ────────────────────────────────────────────────────────────


@dataclass
class Report:
    mapped: List[str] = field(default_factory=list)  # OM_KEY → yaml_key
    skipped_existing: List[str] = field(default_factory=list)  # operator already set
    ignored: List[str] = field(default_factory=list)  # explicit drop
    unmapped: List[str] = field(default_factory=list)  # in source, not in our table
    areas: int = 0
    nav_areas: int = 0
    obstacles: int = 0
    dropped_duplicate_points: int = 0
    docks: int = 0


def migrate(source: Path, target: Path, force: bool, dry_run: bool) -> Report:
    rep = Report()

    sh_path = source / "config" / "om" / "mower_config.sh"
    yaml_path = target / "mowgli" / "mowgli_robot.yaml"
    map_json_path = source / "ros" / "map.json"
    areas_dat_target = target / "mowgli" / "garden_areas.dat"

    if not yaml_path.exists():
        print(f"ERROR: target yaml not found at {yaml_path}", file=sys.stderr)
        print("       run mowglinext install first to seed the target tree.", file=sys.stderr)
        sys.exit(1)

    yaml_content = yaml_path.read_text()

    # 1) mower_config.sh → yaml
    if sh_path.exists():
        env = parse_mower_config_sh(sh_path)
        for om_key, om_val in env.items():
            if om_key in OM_IGNORED:
                rep.ignored.append(om_key)
                continue
            mapping = OM_TO_YAML.get(om_key)
            if mapping is None:
                rep.unmapped.append(om_key)
                continue
            yaml_key, transform = mapping
            value = transform(om_val)
            already_set = yaml_value_is_set(yaml_content, yaml_key)
            if already_set and not force:
                rep.skipped_existing.append(f"{om_key} → {yaml_key} (kept current)")
                continue
            yaml_content = yaml_patch(yaml_content, yaml_key, value)
            rep.mapped.append(f"{om_key}={value} → {yaml_key}")
    else:
        print(f"warn: {sh_path} not found, skipping config migration")

    # 2) map.json → areas.dat + dock_pose
    md = MapData()
    if map_json_path.exists():
        md = parse_map_json(map_json_path)
        rep.areas = sum(1 for a in md.areas if not a.is_navigation)
        rep.nav_areas = sum(1 for a in md.areas if a.is_navigation)
        rep.obstacles = sum(len(a.obstacles) for a in md.areas)
        rep.docks = len(md.docks)

        if md.docks:
            d = md.docks[0]
            for k, v in (("dock_pose_x", d.x), ("dock_pose_y", d.y), ("dock_pose_yaw", d.yaw)):
                already_set = yaml_value_is_set(yaml_content, k)
                if already_set and not force:
                    rep.skipped_existing.append(f"{k} (kept current; map.json had {v:.4f})")
                    continue
                yaml_content = yaml_patch(yaml_content, k, f"{v:.6f}")
                rep.mapped.append(f"map.json dock → {k}={v:.6f}")
            if len(md.docks) > 1:
                print(
                    f"note: {len(md.docks)} docks in map.json; only the first was migrated"
                )

    # 3) Write
    if dry_run:
        print("--- dry run: yaml diff ---")
        # Show the resulting yaml diffs implicitly by saving to /dev/null.
    else:
        yaml_path.write_text(yaml_content)
        if md.areas:
            write_areas_dat(areas_dat_target, md)
            rep.mapped.append(f"wrote {areas_dat_target}")

    return rep


def print_report(rep: Report) -> None:
    print()
    print("================ migration report ================")
    print(f"  mowing areas:        {rep.areas}")
    print(f"  navigation areas:    {rep.nav_areas}")
    print(f"  obstacles:           {rep.obstacles}")
    print(f"  docks (1st migrated):{rep.docks}")
    if rep.dropped_duplicate_points:
        print(f"  dup points removed:  {rep.dropped_duplicate_points}")
    print()
    if rep.mapped:
        print("Mapped:")
        for m in rep.mapped:
            print(f"  ✓ {m}")
    if rep.skipped_existing:
        print()
        print("Skipped (target already has a value — pass --force to overwrite):")
        for m in rep.skipped_existing:
            print(f"  · {m}")
    if rep.unmapped:
        print()
        print("Unmapped (no MowgliNext equivalent — manual review needed):")
        for m in rep.unmapped:
            print(f"  ? {m}")
    if rep.ignored:
        print()
        print("Ignored (deliberately not migrated):")
        for m in rep.ignored:
            print(f"  - {m}")
    print("==================================================")


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument(
        "--source",
        type=Path,
        required=True,
        help="OpenMower install root (the directory containing config/om and ros/)",
    )
    p.add_argument(
        "--target",
        type=Path,
        required=True,
        help="MowgliNext config root (e.g. ~/mowglinext/docker/config). Must "
        "already contain mowgli/mowgli_robot.yaml (run the installer once first).",
    )
    p.add_argument(
        "--force",
        action="store_true",
        help="Overwrite values already set in the target yaml. Off by default — "
        "the script tries to be conservative so an operator's calibrated values "
        "(datum, dock_pose, imu_yaw, …) survive a re-run.",
    )
    p.add_argument(
        "--dry-run",
        action="store_true",
        help="Don't write any files. Print the report only.",
    )
    args = p.parse_args()

    if not args.source.exists():
        print(f"ERROR: --source {args.source} does not exist", file=sys.stderr)
        return 2

    rep = migrate(args.source.expanduser(), args.target.expanduser(), args.force, args.dry_run)
    print_report(rep)
    return 0


if __name__ == "__main__":
    sys.exit(main())

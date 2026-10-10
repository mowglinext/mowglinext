# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0

import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path


_PKG_DIR = Path(__file__).resolve().parent.parent
_XACRO_FILE = _PKG_DIR / "urdf" / "mowgli.urdf.xacro"


def test_chassis_mass_argument_sets_base_link_inertial_mass() -> None:
    """A configured chassis mass must reach the generated base_link inertia."""
    configured_mass = 12.34
    result = subprocess.run(
        ["xacro", str(_XACRO_FILE), f"chassis_mass_kg:={configured_mass}"],
        check=True,
        capture_output=True,
        text=True,
    )

    robot = ET.fromstring(result.stdout)
    base_link = robot.find("./link[@name='base_link']")
    assert base_link is not None
    mass = base_link.find("./inertial/mass")
    assert mass is not None
    assert float(mass.attrib["value"]) == configured_mass


def test_blade_link_is_attached_by_a_fixed_joint() -> None:
    """Coverage must always be able to resolve the cutting-tool TF."""
    result = subprocess.run(
        ["xacro", str(_XACRO_FILE)],
        check=True,
        capture_output=True,
        text=True,
    )

    robot = ET.fromstring(result.stdout)
    blade_joint = robot.find("./joint[@name='blade_joint']")
    assert blade_joint is not None
    assert blade_joint.attrib["type"] == "fixed"
    parent = blade_joint.find("./parent")
    child = blade_joint.find("./child")
    assert parent is not None
    assert child is not None
    assert parent.attrib["link"] == "base_link"
    assert child.attrib["link"] == "blade_link"


def test_caster_offset_reaches_both_axles_and_auto_preserves_placement() -> None:
    for offset, expected in [(-1.0, 0.45), (0.32, 0.32), (0.0, 0.0), (-0.1, -0.1)]:
        result = subprocess.run(
            ["xacro", str(_XACRO_FILE), "chassis_length:=0.6",
             "chassis_center_x:=0.18", "caster_radius:=0.03",
             f"caster_x_offset:={offset}"],
            check=True, capture_output=True, text=True,
        )
        robot = ET.fromstring(result.stdout)
        for side in ("left", "right"):
            origin = robot.find(f"./joint[@name='front_{side}_caster_joint']/origin")
            assert origin is not None
            assert abs(float(origin.attrib["xyz"].split()[0]) - expected) < 1e-9


def test_chassis_offset_moves_body_only_and_retains_dimensions() -> None:
    snapshots = []
    for offset in (-0.05, 0.0):
        result = subprocess.run(
            ["xacro", str(_XACRO_FILE), f"chassis_z_offset:={offset}",
             "chassis_height:=0.19", "wheel_radius:=0.1"],
            check=True, capture_output=True, text=True,
        )
        robot = ET.fromstring(result.stdout)
        for kind in ("visual", "collision"):
            body = robot.find(f"./link[@name='base_link']/{kind}")
            assert body is not None
            z = float(body.find("origin").attrib["xyz"].split()[2])
            height = float(body.find("geometry/box").attrib["size"].split()[2])
            assert height == 0.19
            assert abs(z - height / 2 - offset) < 1e-9
            assert abs(0.1 + z - height / 2 - (0.1 + offset)) < 1e-9
        snapshots.append({j.attrib["name"]: ET.tostring(j) for j in robot.findall("joint")})
    # Changing shell Z must not relocate base_link, axles, blade or sensors.
    assert snapshots[0] == snapshots[1]

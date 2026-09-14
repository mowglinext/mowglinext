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

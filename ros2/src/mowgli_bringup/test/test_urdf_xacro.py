# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0

import importlib.util
import subprocess

import pytest
import yaml
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


def test_explicit_caster_position_is_fixed_when_chassis_length_changes() -> None:
    for length in (0.54, 0.8):
        result = subprocess.run(
            ["xacro", str(_XACRO_FILE), f"chassis_length:={length}", "caster_x_offset:=0.4"],
            check=True, capture_output=True, text=True,
        )
        robot = ET.fromstring(result.stdout)
        for side in ("left", "right"):
            origin = robot.find(f"./joint[@name='front_{side}_caster_joint']/origin")
            assert origin is not None
            assert float(origin.attrib["xyz"].split()[0]) == 0.4


def test_blade_offsets_preserve_legacy_and_locate_asymmetric_cutting_tool() -> None:
    cases = [({}, (0.195, 0.0)),
             ({"blade_x_offset": 0.0, "blade_y_offset": -0.0575}, (0.195, -0.0575)),
             ({"blade_x_offset": 0.02, "blade_y_offset": 0.04}, (0.215, 0.04))]
    for offsets, expected in cases:
        result = subprocess.run(
            ["xacro", str(_XACRO_FILE), "chassis_center_x:=0.195", "wheel_x_offset:=-0.005",
             *[f"{key}:={value}" for key, value in offsets.items()]],
            check=True, capture_output=True, text=True,
        )
        robot = ET.fromstring(result.stdout)
        origin = robot.find("./joint[@name='blade_joint']/origin")
        xyz = [float(value) for value in origin.attrib["xyz"].split()]
        assert abs(xyz[0] - expected[0]) < 1e-9
        assert abs(xyz[1] - expected[1]) < 1e-9
        # Blade height is unchanged; wheel offset is not added to the blade.
        assert abs(xyz[2] - (-0.1 + 0.05)) < 1e-9


@pytest.mark.parametrize("length,centre,radius", [(0.60, 0.18, 0.03), (0.775, 0.30, 0.045)])
@pytest.mark.parametrize("explicit", [False, True])
@pytest.mark.parametrize("model", ["YardForce500", "YardForce500B", "CUSTOM"])
def test_sparse_upgrade_preserves_geometry(tmp_path, length, centre, radius, explicit, model):
    """Exercise the real template merge, not just standalone xacro defaults."""
    spec = importlib.util.spec_from_file_location(
        "upgrade_robot_config", _PKG_DIR / "launch" / "robot_config_util.py")
    config = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(config)
    params = {"mower_model": model, "chassis_length": length, "chassis_center_x": centre,
              "chassis_height": 0.19, "caster_radius": radius}
    if explicit:
        params.update(caster_x_offset=0.32, chassis_z_offset=-0.04)
    runtime = tmp_path / "robot.yaml"
    runtime.write_text(yaml.safe_dump({"mowgli": {"ros__parameters": params}}))
    merged = config.load_robot_params(str(_PKG_DIR), str(runtime), "mowgli")
    keys = [*params, "caster_x_offset", "chassis_z_offset"]
    result = subprocess.run(
        ["xacro", str(_XACRO_FILE), *[f"{key}:={merged[key]}" for key in dict.fromkeys(keys)]],
        check=True, capture_output=True, text=True)
    robot = ET.fromstring(result.stdout)
    expected_x = 0.32 if explicit else centre + length / 2 - radius
    for side in ("left", "right"):
        origin = robot.find(f"./joint[@name='front_{side}_caster_joint']/origin")
        assert float(origin.attrib["xyz"].split()[0]) == pytest.approx(expected_x)
    for kind in ("visual", "collision"):
        origin = robot.find(f"./link[@name='base_link']/{kind}/origin")
        z = float(origin.attrib["xyz"].split()[2])
        assert z == pytest.approx(0.095 + (-0.04 if explicit else merged["chassis_z_offset"]))

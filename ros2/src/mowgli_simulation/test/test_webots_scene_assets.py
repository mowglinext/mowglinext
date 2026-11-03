# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0

"""Keep the garden's PROTOs and appearance maps usable in an offline SDK."""

from pathlib import Path
import re
import xml.etree.ElementTree as ET


PACKAGE = Path(__file__).resolve().parents[1]


def test_garden_proto_dependencies_are_bundled():
    pending = [PACKAGE / 'worlds_webots' / 'mowgli_garden.wbt']
    seen = set()
    while pending:
        path = pending.pop().resolve()
        if path in seen:
            continue
        seen.add(path)
        source = path.read_text()
        references = re.findall(r'^EXTERNPROTO\s+"([^"]+)"', source, re.M)
        # Floor is a PROTO, not a built-in node: the original undeclared
        # instance was silently skipped, leaving the garden without ground.
        if re.search(r'\bFloor\s*\{', source):
            assert any(Path(ref).stem == 'Floor' for ref in references), (
                f'{path}: Floor requires an explicit local EXTERNPROTO'
            )
        for reference in references:
            assert '://' not in reference, f'{path}: external dependency {reference}'
            dependency = (path.parent / reference).resolve()
            assert dependency.is_relative_to(PACKAGE)
            assert dependency.is_file(), f'{path}: missing {reference}'
            pending.append(dependency)


def test_tire_rubber_texture_dependencies_are_bundled():
    appearance = PACKAGE / 'protos' / 'webots' / 'TireRubber.proto'
    source = appearance.read_text()
    # Upstream's template selects one of these two complete material sets.
    types = re.search(r'field SFString\{([^}]+)\}\s+type', source).group(1)
    suffixes = re.findall(r'_tire_rubber_([a-z_]+\.jpg)', source)
    assert len(suffixes) == 4
    for kind in re.findall(r'"([^"]+)"', types):
        for suffix in suffixes:
            texture = appearance.parent / 'textures' / 'tire_rubber' / (
                f'{kind}_tire_rubber_{suffix}'
            )
            assert texture.is_file(), f'missing appearance map: {texture}'
            assert texture.read_bytes().startswith(b'\xff\xd8'), texture


def test_cutting_tool_frame_is_reachable_in_the_fixed_webots_model():
    robot = ET.parse(PACKAGE / 'urdf_webots' / 'mowgli_webots.urdf').getroot()
    assert len(robot.findall("link[@name='blade_link']")) == 1
    parents = {}
    joints = {}
    for joint in robot.findall('joint'):
        child = joint.find('child').attrib['link']
        assert child not in parents, f'{child}: multiple TF parents'
        parents[child] = joint.find('parent').attrib['link']
        joints[child] = joint
    # map_server stamps the blade position through this static chain. The
    # xacro blade-placement formula uses the fixed Webots dimensions here.
    assert parents['blade_link'] == 'base_link'
    assert parents['base_link'] == 'base_footprint'
    blade = joints['blade_link']
    assert blade.attrib['type'] == 'fixed'
    assert tuple(map(float, blade.find('origin').attrib['xyz'].split())) == (.18, 0., -.043)
    assert tuple(map(float, blade.find('origin').attrib['rpy'].split())) == (0., 0., 0.)
    assert joints['base_link'].attrib['type'] == 'fixed'
    assert tuple(map(float, joints['base_link'].find('origin').attrib['xyz'].split())) == (
        0., 0., .093)

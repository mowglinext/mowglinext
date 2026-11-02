# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0

"""Keep the garden's PROTOs and appearance maps usable in an offline SDK."""

from pathlib import Path
import re


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

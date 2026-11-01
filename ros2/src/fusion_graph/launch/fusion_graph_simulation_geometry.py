# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0-or-later
"""Validate a complete simulator datum/antenna snapshot shared by launch files."""

import json
import math


def resolve_simulation_geometry(spec, use_sim_time):
    """An empty bundle preserves robot configuration; overrides require sim time."""
    if not spec:
        return {}
    if str(use_sim_time).lower() not in ('true', '1'):
        raise ValueError('simulation_geometry requires use_sim_time=true')
    fields = {'datum_lat', 'datum_lon', 'lever_arm_x', 'lever_arm_y'}
    geometry = json.loads(spec)
    if not isinstance(geometry, dict) or set(geometry) != fields:
        raise ValueError(
            'simulation_geometry must contain the complete datum and antenna geometry')
    if any(isinstance(value, bool) or not isinstance(value, (int, float))
           or not math.isfinite(value) for value in geometry.values()):
        raise ValueError('simulation_geometry values must be finite numbers')
    if not -90 < geometry['datum_lat'] < 90 or not -180 <= geometry['datum_lon'] <= 180:
        raise ValueError('simulation_geometry datum is outside WGS84 bounds')
    if geometry['datum_lat'] == 0 and geometry['datum_lon'] == 0:
        raise ValueError('simulation_geometry must specify a datum, not self-seeding')
    return {key: float(value) for key, value in geometry.items()}

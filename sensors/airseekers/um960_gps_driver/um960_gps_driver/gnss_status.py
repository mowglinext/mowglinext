# SPDX-License-Identifier: MIT
"""UM960 parser state -> mowgli_interfaces/GnssStatus field values (pure).

Added for the MowgliNext Airseekers backend: the sidecar publishes the
public ``/gps/status`` contract itself (``gnss_stack: disabled``), so the
fields follow the Universal GNSS bridge's projection
(sensors/gps/universal_gnss_topic_bridge.py): fix type from the receiver's own
solution, quality from the fix type, accuracy from the receiver's sigma (or
the HDOP estimate), satellites, correction age/flow.
"""

from __future__ import annotations

import math
from typing import Dict, Optional, Sequence

# GnssStatus constants (kept numeric so this module needs no ROS).
FIX_TYPE_NO_FIX, FIX_TYPE_GPS_FIX, FIX_TYPE_RTK_FLOAT, FIX_TYPE_RTK_FIXED, \
    FIX_TYPE_DEAD_RECKONING = 0, 1, 2, 3, 4
RTK_MODE_NONE, RTK_MODE_FLOAT, RTK_MODE_FIXED = 1, 2, 3
CAP_RTK_MODE = 1
CAP_HDOP = 2
CAP_VDOP = 4
CAP_HORIZONTAL_ACCURACY = 8
CAP_VERTICAL_ACCURACY = 16
CAP_SATELLITES_USED = 128
CAP_SATELLITES_VISIBLE = 256
CAP_DIFFERENTIAL_CORRECTIONS = 1024
CAP_CORRECTIONS_ACTIVE = 2048
CAP_CORRECTION_AGE = 4096
CAP_CORRECTION_FLOW = 67108864
CORRECTION_FLOW_STATUS = {'idle': 1, 'waiting': 2, 'active': 3, 'stale': 4, 'held': 2}

QUALITY_PERCENT = {FIX_TYPE_NO_FIX: 0.0, FIX_TYPE_GPS_FIX: 25.0, FIX_TYPE_RTK_FLOAT: 50.0,
                   FIX_TYPE_RTK_FIXED: 100.0, FIX_TYPE_DEAD_RECKONING: 10.0}

# Unicore BESTNAV position types.
_RTK_FIXED_TYPES = {'NARROW_INT', 'WIDE_INT', 'L1_INT', 'INS_RTKFIXED', 'FIXEDPOS'}
_RTK_FLOAT_TYPES = {'NARROW_FLOAT', 'L1_FLOAT', 'IONOFREE_FLOAT', 'INS_RTKFLOAT'}
_DIFF_TYPES = {'PSRDIFF', 'SBAS', 'INS_PSRDIFF'}


def fix_type_of(fix: Dict[str, object], has_position: bool) -> int:
    if not has_position:
        return FIX_TYPE_NO_FIX
    pos_type = fix.get('position_type')
    if isinstance(pos_type, str) and pos_type:
        if pos_type in _RTK_FIXED_TYPES:
            return FIX_TYPE_RTK_FIXED
        if pos_type in _RTK_FLOAT_TYPES:
            return FIX_TYPE_RTK_FLOAT
        if pos_type == 'NONE':
            return FIX_TYPE_NO_FIX
        return FIX_TYPE_GPS_FIX
    quality = int(fix.get('quality') or 0)
    return {0: FIX_TYPE_NO_FIX, 4: FIX_TYPE_RTK_FIXED, 5: FIX_TYPE_RTK_FLOAT,
            6: FIX_TYPE_DEAD_RECKONING}.get(quality, FIX_TYPE_GPS_FIX)


def gnss_status_fields(fix: Dict[str, object], has_position: bool,
                       covariance: Optional[Sequence[float]],
                       corrections: Optional[Dict[str, object]] = None,
                       observation_sequence: int = 0) -> Dict[str, object]:
    corrections = corrections or {}
    ft = fix_type_of(fix, has_position)
    out: Dict[str, object] = {
        'backend': 'um960_gps_driver',
        'receiver_vendor': 'Unicore',
        'receiver_model': 'UM960',
        'receiver_firmware': str(fix.get('version') or ''),
        'fix_type': ft,
        'fix_valid': ft in (FIX_TYPE_GPS_FIX, FIX_TYPE_RTK_FLOAT, FIX_TYPE_RTK_FIXED),
        'dead_reckoning': ft == FIX_TYPE_DEAD_RECKONING,
        'quality_percent': QUALITY_PERCENT[ft],
        'rtk_mode': {FIX_TYPE_RTK_FIXED: RTK_MODE_FIXED,
                     FIX_TYPE_RTK_FLOAT: RTK_MODE_FLOAT}.get(ft, RTK_MODE_NONE),
        'position_observation_sequence': int(observation_sequence),
    }
    cap = (CAP_RTK_MODE | CAP_HDOP | CAP_VDOP | CAP_HORIZONTAL_ACCURACY | CAP_VERTICAL_ACCURACY
           | CAP_SATELLITES_USED | CAP_SATELLITES_VISIBLE | CAP_DIFFERENTIAL_CORRECTIONS
           | CAP_CORRECTIONS_ACTIVE | CAP_CORRECTION_AGE | CAP_CORRECTION_FLOW)
    val = CAP_RTK_MODE | CAP_DIFFERENTIAL_CORRECTIONS
    pos_type = fix.get('position_type')
    out['differential_corrections'] = (
        ft in (FIX_TYPE_RTK_FLOAT, FIX_TYPE_RTK_FIXED)
        or (isinstance(pos_type, str) and pos_type in _DIFF_TYPES)
        or int(fix.get('quality') or 0) == 2)
    hdop = float(fix.get('hdop') or 0.0)
    if hdop > 0.0:
        out['hdop'] = hdop
        val |= CAP_HDOP
    vdop = float(fix.get('vdop') or 0.0)
    if vdop > 0.0:
        out['vdop'] = vdop
        val |= CAP_VDOP
    used = int(fix.get('num_sats_used') or fix.get('num_sats') or 0)
    if used:
        out['satellites_used'] = used
        val |= CAP_SATELLITES_USED
    visible = int(fix.get('satellites_in_view') or 0)
    if visible:
        out['satellites_visible'] = visible
        val |= CAP_SATELLITES_VISIBLE
    if has_position and covariance is not None and len(covariance) >= 9:
        h = max(float(covariance[0]), float(covariance[4]))
        if h > 0.0 and math.isfinite(h):
            out['horizontal_accuracy_m'] = math.sqrt(h)
            val |= CAP_HORIZONTAL_ACCURACY
        v = float(covariance[8])
        if v > 0.0 and math.isfinite(v):
            out['vertical_accuracy_m'] = math.sqrt(v)
            val |= CAP_VERTICAL_ACCURACY
    age = corrections.get('corr_age_s')
    if age is None:
        age = fix.get('fix_age')
    if age is not None and math.isfinite(float(age)):
        out['correction_age_s'] = float(age)
        val |= CAP_CORRECTION_AGE
    flow = str(corrections.get('flow') or '')
    if flow in CORRECTION_FLOW_STATUS:
        out['correction_flow_status'] = CORRECTION_FLOW_STATUS[flow]
        val |= CAP_CORRECTION_FLOW
    out['corrections_active'] = flow == 'active'
    if flow:
        val |= CAP_CORRECTIONS_ACTIVE
    out['correction_source'] = str(corrections.get('source') or '')
    out['capability_flags'] = cap
    out['value_flags'] = val
    return out

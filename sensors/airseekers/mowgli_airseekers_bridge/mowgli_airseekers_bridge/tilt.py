# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0
# Ported from the Airseekers ROS 2 port (mower_control/tilt_logic.py); same author, relicensed GPL-3.0.
"""Slope / tilt bands from the IMU attitude (pure Python, no ROS).

In the bridge only the CRITICAL band is acted on: it is an emergency source
(latched, wheels + blade off, cleared by the emergency reset service). The
lower bands are reported in the diagnostics only.

2026-10-09. Why a host-side guard although the MCU already has one: the cutter firmware
(0.6.36) asserts ``lift`` only when the forwarded IMU leaves
pitch [-36.8, +40.1] deg or |roll| > 38.5 deg, i.e. when the mower is already about to roll
or flip, and it LATCHES after ~17 s (clear-estop then works only while the IMU reads level).
Before that point nothing slowed the mower down or kept it off a slope it cannot hold.

Input: roll / pitch of the WIT JY61P orientation. The JY61P fuses accelerometer + gyro
itself; its roll / pitch are gravity referenced (only its yaw is gyro-integrated and not
earth referenced, which this module never uses). REP-103 signs: +roll = left side up,
+pitch = nose DOWN.

Mounting: on this Tron the forwarded attitude reads ~0 / ~0 upright (verified 2026-10-06:
the clear frame released a latched lift with the IMU forwarded unmodified). The vendor
capture showed roll ~ -178 deg (WIT mounted upside down relative to the vendor frame);
``mount_roll_offset_deg`` / ``mount_pitch_offset_deg`` exist for such a board and are
applied as a rotation of the gravity vector (a 180 deg roll offset is exact, not an angle
subtraction that breaks near +-180). Without the right offset an inverted board reads
~180 deg = CRITICAL: the guard fails safe and the node logs the hint.

Bands (separate thresholds per axis; roll = side slope / rollover risk, pitch = up/down):

=========  =====================================================================
ok         nothing
caution    reported only
limit      reported only
critical   bridge emergency: wheels + blade off, latched until reset
=========  =====================================================================

Filtering, so that roots / kerbs do not trigger:

* first-order low-pass (``filter_tau_s``) on the body-frame gravity vector (roll and
  pitch are read from the filtered vector);
* a band is ENTERED only after its condition (|roll| >= roll threshold OR |pitch| >= pitch
  threshold, on the filtered values) held continuously for that band's debounce
  (caution 1.0 s, limit 0.6 s, critical 0.3 s by default). A higher band can be entered
  directly from any lower one; its own debounce counts from when its condition began;
* a band is LEFT only after both axes stayed below (threshold - ``hysteresis_deg``) for
  ``clear_s``; the band then drops to the highest band whose hysteresis-reduced condition
  still holds (no second debounce on the way down);
* no IMU sample for ``stale_s`` -> band 'unknown' (consumers keep their last decision and
  do not act on it; the node reports it).
"""

import math

BANDS = ('ok', 'caution', 'limit', 'critical')
UNKNOWN = 'unknown'
LEVEL = {b: i for i, b in enumerate(BANDS)}
LEVEL[UNKNOWN] = -1

DEFAULTS = {
    # deg, (caution, limit, critical). Critical stays clear of the firmware lift window
    # (|roll| 38.5, pitch -36.8 / +40.1) so the host stops before the MCU latches lift.
    'roll_thresholds_deg': (15.0, 22.0, 30.0),
    'pitch_thresholds_deg': (18.0, 25.0, 32.0),
    'hysteresis_deg': 3.0,
    'debounce_s': (1.0, 0.6, 0.3),     # caution, limit, critical
    'clear_s': 2.0,
    'filter_tau_s': 0.25,
    'stale_s': 0.5,
    'mount_roll_offset_deg': 0.0,
    'mount_pitch_offset_deg': 0.0,
}


def quat_to_roll_pitch(x, y, z, w):
    """Roll, pitch (rad, REP-103 ZYX Euler) of an orientation quaternion."""
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    return roll, math.asin(sp)


def up_vector(roll, pitch, mount_roll=0.0, mount_pitch=0.0):
    """World 'up' as a unit vector in the BODY frame, from the sensor roll / pitch (rad)
    and a fixed mounting correction (rad, added to the sensor angles like the bridge's
    forward_imu_*_offset_deg: roll offset about x, then pitch offset about y).

    Yaw never enters: the result is purely gravity referenced. Working on the vector (not
    on angles) keeps a 180 deg offset exact and lets the low-pass run without the +-180
    wrap (an inverted board jittering between +179 and -179 must not average to 0)."""
    gx = -math.sin(pitch)
    gy = math.sin(roll) * math.cos(pitch)
    gz = math.cos(roll) * math.cos(pitch)
    if mount_roll:
        c, s = math.cos(mount_roll), math.sin(mount_roll)
        gy, gz = c * gy + s * gz, -s * gy + c * gz
    if mount_pitch:
        c, s = math.cos(mount_pitch), math.sin(mount_pitch)
        gx, gz = c * gx - s * gz, s * gx + c * gz
    return gx, gy, gz


def vector_roll_pitch(g):
    """(roll, pitch) in deg of a body-frame up vector (need not be unit length)."""
    gx, gy, gz = g
    return (math.degrees(math.atan2(gy, gz)),
            math.degrees(math.atan2(-gx, math.hypot(gy, gz))))


def gravity_tilt(roll, pitch, mount_roll=0.0, mount_pitch=0.0):
    """Body roll / pitch (rad) after the mounting correction (see :func:`up_vector`)."""
    r, p = vector_roll_pitch(up_vector(roll, pitch, mount_roll, mount_pitch))
    return math.radians(r), math.radians(p)


class TiltBands:
    """Low-pass + band state machine. Feed :meth:`update` with (t, roll, pitch) in rad."""

    def __init__(self, **kw):
        c = dict(DEFAULTS)
        for k, v in kw.items():
            if k not in c:
                raise KeyError('unknown tilt parameter %r' % k)
            c[k] = v
        self.cfg = c
        self.roll_thr = [float(v) for v in c['roll_thresholds_deg']]
        self.pitch_thr = [float(v) for v in c['pitch_thresholds_deg']]
        self.debounce = [float(v) for v in c['debounce_s']]
        if len(self.roll_thr) != 3 or len(self.pitch_thr) != 3 or len(self.debounce) != 3:
            raise ValueError('thresholds / debounce need 3 values (caution, limit, critical)')
        for thr in (self.roll_thr, self.pitch_thr):
            if not thr[0] < thr[1] < thr[2]:
                raise ValueError('thresholds must increase: %s' % thr)
        self.hyst = float(c['hysteresis_deg'])
        self.clear_s = float(c['clear_s'])
        self.tau = float(c['filter_tau_s'])
        self.stale_s = float(c['stale_s'])
        self.mount_roll = math.radians(float(c['mount_roll_offset_deg']))
        self.mount_pitch = math.radians(float(c['mount_pitch_offset_deg']))
        self.reset()

    def reset(self):
        self.level = 0                 # 0 ok .. 3 critical
        self.roll = self.pitch = None  # filtered, deg
        self.raw_roll = self.raw_pitch = None
        self._g = None                 # filtered body-frame up vector
        self.t = None                  # time of the last sample
        self.since = None              # time the current band was entered
        self.axis = ''                 # axis that set the current band
        self._above_since = [None, None, None]   # per band 1..3: condition start
        self._below_since = None       # current band's release condition start

    # ---- per-sample ------------------------------------------------------------------
    def _over(self, k, margin=0.0):
        """Filtered tilt at / over band k's thresholds (k = 1..3) minus margin; -> axis."""
        if abs(self.roll) >= self.roll_thr[k - 1] - margin:
            return 'roll'
        if abs(self.pitch) >= self.pitch_thr[k - 1] - margin:
            return 'pitch'
        return ''

    def update(self, t, roll, pitch):
        """One IMU sample (rad, sensor frame). Returns the band name."""
        g = up_vector(roll, pitch, self.mount_roll, self.mount_pitch)
        self.raw_roll, self.raw_pitch = vector_roll_pitch(g)
        if self._g is None or self.t is None or t - self.t > self.stale_s or t < self.t:
            self._g = g                      # (re)start the filter on the first fresh sample
        else:
            a = 1.0 - math.exp(-(t - self.t) / self.tau) if self.tau > 0 else 1.0
            self._g = tuple(f + a * (n - f) for f, n in zip(self._g, g))
        self.roll, self.pitch = vector_roll_pitch(self._g)
        self.t = t
        if self.since is None:
            self.since = t
        self._step(t)
        return self.band(t)

    def _step(self, t):
        # escalation: highest band whose condition held for its debounce
        target = self.level
        for k in (1, 2, 3):
            if self._over(k):
                if self._above_since[k - 1] is None:
                    self._above_since[k - 1] = t
                if k > target and t - self._above_since[k - 1] >= self.debounce[k - 1]:
                    target = k
            else:
                self._above_since[k - 1] = None
        if target > self.level:
            self._enter(target, self._over(target), t)
            return
        # release: both axes below (threshold - hysteresis) of the current band for clear_s
        if self.level == 0:
            return
        if self._over(self.level, self.hyst):
            self._below_since = None
            return
        if self._below_since is None:
            self._below_since = t
        if t - self._below_since < self.clear_s:
            return
        lower = 0
        for k in range(self.level - 1, 0, -1):
            if self._over(k, self.hyst):
                lower = k
                break
        self._enter(lower, self._over(lower, self.hyst) if lower else '', t)

    def _enter(self, level, axis, t):
        self.level, self.axis, self.since = level, axis, t
        self._below_since = None

    # ---- queries ---------------------------------------------------------------------
    def stale(self, now):
        return self.t is None or now - self.t > self.stale_s

    def band(self, now):
        return UNKNOWN if self.stale(now) else BANDS[self.level]

    def status(self, now):
        """JSON-able tilt summary (bridge diagnostics)."""
        band = self.band(now)

        def rnd(v):
            return None if v is None else round(v, 2)
        return {
            'band': band,
            'level': LEVEL[band],
            'roll_deg': rnd(self.roll),
            'pitch_deg': rnd(self.pitch),
            'raw_roll_deg': rnd(self.raw_roll),
            'raw_pitch_deg': rnd(self.raw_pitch),
            'axis': self.axis,
            'stale': self.stale(now),
            'imu_age_s': None if self.t is None else round(now - self.t, 3),
            'since_s': None if self.since is None else round(now - self.since, 2),
            'thresholds': {'roll': list(self.roll_thr), 'pitch': list(self.pitch_thr),
                           'hysteresis': self.hyst},
        }

    def inverted(self):
        """Filtered attitude says the board is (near) upside down: |roll| > 150 deg."""
        return self.roll is not None and abs(self.roll) > 150.0

# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0
"""The host-side safety model of the Airseekers bridge. Pure: no ROS, no I/O.

On the Mowgli STM32 the firmware is the blade and e-stop authority. The Tron's
vendor MCU only enforces a few things itself (it ignores SpeedData while its
own ``lift`` or ``stop`` flag is set and zeroes the setpoint some time after
host traffic stops); everything else the vendor's host software used to do.
So, as on the OpenMower backend, the bridge is in the actuation path and
holds the same line (sensors/openmower/.../blade_policy.hpp):

* wheels stop on any emergency, a stale MCU link, a stale ``/cmd_vel``, in
  IDLE, and on the dock until the behaviour tree leaves IDLE (undock);
* the blade runs only in AUTONOMOUS / MANUAL_MOWING with a fresh
  HighLevelStatus, mowing_enabled, no emergency and a live link;
* an emergency (stop button, lift, tilt, host request) LATCHES on the host
  and is cleared only by ``/hardware_bridge/emergency_stop`` with
  ``emergency=0`` while no trigger is asserted.

The SpeedData transmit policy (``SpeedTx``) is specific to this MCU: the
drives keep the LAST setpoint while host traffic (our heartbeat) continues,
and ANY SpeedData frame (zeros included) takes the chassis out of its
position hold, which on the dock lets the wheels creep off the contacts. So
the bridge streams the slewed command while moving, ends every motion with
``stop_frames`` explicit zeros, and is otherwise silent.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

HL_NULL, HL_IDLE, HL_AUTONOMOUS, HL_RECORDING, HL_MANUAL_MOWING = 0, 1, 2, 3, 4


# ---------------------------------------------------------------------------
# Blade / wheel gates
# ---------------------------------------------------------------------------
@dataclass
class BladeInputs:
    requested: bool = False        # /hardware_bridge/mower_control mow_enabled
    mowing_enabled: bool = True    # mowgli_robot.yaml dry-run inhibit
    emergency: bool = False        # latched or active emergency
    link_alive: bool = False       # MCU telemetry fresh
    hl_mode: int = HL_NULL
    hl_status_fresh: bool = False  # HighLevelStatus within high_level_status_timeout_s
    update_maintenance: bool = False


def blade_may_run(b: BladeInputs) -> bool:
    if (not b.requested or not b.mowing_enabled or b.emergency or not b.link_alive
            or not b.hl_status_fresh or b.update_maintenance):
        return False
    return b.hl_mode in (HL_AUTONOMOUS, HL_MANUAL_MOWING)


def wheels_may_run(emergency: bool, link_alive: bool, hl_mode: int, cmd_fresh: bool) -> bool:
    if emergency or not link_alive or not cmd_fresh:
        return False
    return hl_mode != HL_IDLE


@dataclass
class DockGuard:
    """Never move while docked until an undock.

    On the dock the behaviour tree sits in IDLE (which already holds the
    wheels). Motion is allowed again only once a FRESH HighLevelStatus says
    something other than IDLE/NULL (the BT starting a mission or manual mode)
    -- the NULL-mode "somebody is driving" fallback does not apply here.
    ``allow_reverse`` false additionally refuses reverse motion while docked:
    the Tron reverses onto its charger, so reversing on the dock only pushes
    into the contacts.
    """
    allow_reverse: bool = False

    def filter(self, docked: bool, hl_mode: int, hl_fresh: bool,
               linear: float, angular: float) -> Tuple[float, float, str]:
        if not docked:
            return linear, angular, ''
        if not hl_fresh or hl_mode in (HL_NULL, HL_IDLE):
            return 0.0, 0.0, 'docked: waiting for an undock (behaviour tree not active)'
        if linear < 0.0 and not self.allow_reverse:
            return 0.0, 0.0, 'docked: reverse refused (would push into the charger)'
        return linear, angular, ''


# ---------------------------------------------------------------------------
# Emergency latch
# ---------------------------------------------------------------------------
@dataclass
class EmergencyInputs:
    stop: bool = False         # MCU SensorInfo.stop (red button)
    lift: bool = False         # MCU SensorInfo.lift (halls + forwarded IMU window)
    tilt: bool = False         # host tilt monitor, critical band
    bumper: bool = False       # MCU SensorInfo bumper (only if configured as emergency)


@dataclass
class EmergencyState:
    active: bool
    latched: bool
    reason: str


class EmergencyLatch:
    """Host emergency latch.

    Any trigger (or a host request) latches. ``request_release`` clears the
    latch only while no trigger is asserted -- a release attempted while the
    stop button is still held is refused and NOT remembered, so letting go of
    the button later does not free the robot by itself.
    """

    def __init__(self, latch_on_boot: bool = True) -> None:
        self._latched = latch_on_boot
        self._host_request = False
        self._reason = 'Latched at start-up (reset the emergency to drive)' if latch_on_boot else ''
        self._last_trigger = EmergencyInputs()

    @staticmethod
    def _trigger_reason(t: EmergencyInputs) -> str:
        parts = []
        if t.stop:
            parts.append('STOP button')
        if t.lift:
            parts.append('Lift detected')
        if t.tilt:
            parts.append('Tilt critical')
        if t.bumper:
            parts.append('Bumper')
        return ', '.join(parts)

    def update(self, t: EmergencyInputs) -> EmergencyState:
        self._last_trigger = t
        reason = self._trigger_reason(t)
        if reason:
            self._latched = True
            self._reason = reason
        if self._host_request:
            self._latched = True
            if not reason:
                self._reason = 'Software emergency stop'
        return self.state()

    def state(self) -> EmergencyState:
        active_reason = self._trigger_reason(self._last_trigger)
        if active_reason:
            reason = active_reason
        elif self._latched:
            reason = 'Latched: %s' % (self._reason or 'emergency')
        else:
            reason = ''
        return EmergencyState(bool(active_reason), self._latched, reason)

    def request_emergency(self) -> None:
        self._host_request = True
        self._latched = True
        self._reason = 'Software emergency stop'

    def request_release(self) -> Tuple[bool, str]:
        trig = self._trigger_reason(self._last_trigger)
        if trig:
            return False, 'release refused: %s still asserted' % trig
        self._host_request = False
        self._latched = False
        self._reason = ''
        return True, 'emergency released'

    @property
    def is_emergency(self) -> bool:
        return self._latched or bool(self._trigger_reason(self._last_trigger))


# ---------------------------------------------------------------------------
# Command shaping
# ---------------------------------------------------------------------------
def limit_slew(target: float, previous: float, dt_s: float, accel: float, decel: float) -> float:
    """Port of mowgli_hardware/cmd_vel_slew.hpp ``limit_motion_command_slew``:
    a zero target stops at once; a sign change decelerates to zero first."""
    if target == 0.0:
        return 0.0
    dt_s = max(0.0, dt_s)
    if previous != 0.0 and math.copysign(1.0, target) != math.copysign(1.0, previous):
        step = decel * dt_s
        return max(0.0, previous - step) if previous > 0.0 else min(0.0, previous + step)
    rate = accel if abs(target) > abs(previous) else decel
    step = rate * dt_s
    return previous + max(-step, min(step, target - previous))


def clamp(v: float, lim: float) -> float:
    return max(-lim, min(lim, v))


@dataclass
class SpeedTx:
    """SpeedData transmit policy, ticked at a fixed rate (``tick``).

    Returns the frames to send this tick as a list of (linear, angular):
    * the slewed command every tick while it is non-zero;
    * on the first all-zero tick after motion: one zero now and
      ``stop_frames - 1`` more, ``stop_spacing_s`` apart;
    * nothing otherwise (idle = silence, which keeps the drives in their
      position hold -- the vendor host's idle behaviour).
    """
    accel_lin: float = 0.30
    decel_lin: float = 0.60
    accel_ang: float = 1.0
    decel_ang: float = 2.0
    stop_frames: int = 3
    stop_spacing_s: float = 0.1
    applied: Tuple[float, float] = (0.0, 0.0)
    moving: bool = False
    _stop_left: int = 0
    _stop_next: float = 0.0
    _last_t: Optional[float] = None
    frames_sent: int = 0
    stop_sequences: int = 0

    def tick(self, now: float, target_lin: float, target_ang: float) -> List[Tuple[float, float]]:
        dt = 0.0 if self._last_t is None else min(0.5, max(0.0, now - self._last_t))
        self._last_t = now
        lin = limit_slew(target_lin, self.applied[0], dt, self.accel_lin, self.decel_lin)
        ang = limit_slew(target_ang, self.applied[1], dt, self.accel_ang, self.decel_ang)
        self.applied = (lin, ang)
        out: List[Tuple[float, float]] = []
        if lin != 0.0 or ang != 0.0:
            self._stop_left = 0
            self.moving = True
            out.append((lin, ang))
        elif self.moving:
            self.moving = False
            self.stop_sequences += 1
            out.append((0.0, 0.0))
            self._stop_left = max(0, self.stop_frames - 1)
            self._stop_next = now + self.stop_spacing_s
        elif self._stop_left > 0 and now >= self._stop_next - 1e-9:
            out.append((0.0, 0.0))
            self._stop_left -= 1
            self._stop_next += self.stop_spacing_s
        self.frames_sent += len(out)
        return out

    @property
    def stopping(self) -> bool:
        return self._stop_left > 0


# ---------------------------------------------------------------------------
# Odometry helpers
# ---------------------------------------------------------------------------
@dataclass
class TickIntegrator:
    """Per-wheel tick counters DERIVED from the MCU's measured body velocity.

    The Tron MCU does not put encoder counts on the wire, only its measured
    body velocity (SpeedData, ~100 Hz). ``/wheel_ticks`` is therefore the
    integral of the per-wheel speeds implied by that velocity, scaled by
    ``ticks_per_meter`` -- documented as derived, not raw encoder data.
    """
    ticks_per_meter: float = 1000.0
    wheel_track: float = 0.48
    dist: List[float] = field(default_factory=lambda: [0.0, 0.0])   # signed metres
    mag: List[float] = field(default_factory=lambda: [0.0, 0.0])    # |metres|
    direction: List[int] = field(default_factory=lambda: [1, 1])

    def feed(self, linear: float, angular: float, dt: float) -> None:
        if dt <= 0.0:
            return
        half = 0.5 * self.wheel_track
        for i, v in enumerate((linear - angular * half, linear + angular * half)):
            d = v * dt
            self.dist[i] += d
            self.mag[i] += abs(d)
            if d > 0.0:
                self.direction[i] = 1
            elif d < 0.0:
                self.direction[i] = 0

    def ticks(self) -> Tuple[int, int]:
        return (int(self.mag[0] * self.ticks_per_meter) & 0xFFFFFFFF,
                int(self.mag[1] * self.ticks_per_meter) & 0xFFFFFFFF)

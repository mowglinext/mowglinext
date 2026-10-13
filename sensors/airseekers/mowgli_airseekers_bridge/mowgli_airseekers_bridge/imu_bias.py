# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0
"""At-rest gyro / accelerometer bias estimate (pure, no ROS).

Python port of sensors/openmower/.../imu_bias_estimator.hpp (same rules, so
both backends calibrate alike): while the robot is provably at rest, N samples
are averaged into offsets subtracted from every later sample. Accel Z is never
corrected (gravity must survive for the fusion).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple


@dataclass
class ImuBias:
    ax: float = 0.0
    ay: float = 0.0
    gx: float = 0.0
    gy: float = 0.0
    gz: float = 0.0
    gz_variance: float = 0.0


class ImuBiasEstimator:
    def __init__(self, samples_required: int = 200) -> None:
        self.samples_required = max(1, int(samples_required))
        self.collecting = False
        self.ready = False
        self.count = 0
        self._sums = [0.0] * 5
        self._sq_gz = 0.0
        self.bias = ImuBias()

    def start(self) -> None:
        self.collecting = True
        self.count = 0
        self._sums = [0.0] * 5
        self._sq_gz = 0.0

    def abort(self) -> None:
        self.collecting = False
        self.count = 0

    def feed(self, acc: Sequence[float], gyro: Sequence[float]) -> bool:
        """True when this sample completed a calibration."""
        if not self.collecting:
            return False
        vals = (acc[0], acc[1], gyro[0], gyro[1], gyro[2])
        if not all(math.isfinite(v) for v in vals):
            return False
        for i, v in enumerate(vals):
            self._sums[i] += v
        self._sq_gz += gyro[2] * gyro[2]
        self.count += 1
        if self.count < self.samples_required:
            return False
        n = float(self.count)
        ax, ay, gx, gy, gz = (s / n for s in self._sums)
        self.bias = ImuBias(ax, ay, gx, gy, gz, max(0.0, self._sq_gz / n - gz * gz))
        self.ready = True
        self.collecting = False
        return True

    def apply(self, acc: Sequence[float], gyro: Sequence[float]
              ) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
        if not self.ready:
            return tuple(acc), tuple(gyro)
        b = self.bias
        return ((acc[0] - b.ax, acc[1] - b.ay, acc[2]),
                (gyro[0] - b.gx, gyro[1] - b.gy, gyro[2] - b.gz))


class RestCalibrator:
    """When to run the estimator (the OpenMower bridge's policy):

    * armed on every dock arrival; starts once the wheels have been still
      for ``still_s`` (not on the contact event itself -- the robot may still
      be creeping onto the contacts); motion aborts the attempt, which is then
      retried;
    * off the dock, if no calibration has completed yet, after
      ``auto_rest_s`` at rest.
    """

    def __init__(self, estimator: ImuBiasEstimator, still_s: float = 1.0,
                 auto_rest_s: float = 15.0) -> None:
        self.est = estimator
        self.still_s = still_s
        self.auto_rest_s = auto_rest_s
        self.dock_pending = False
        self._was_docked = False
        self._still_since: Optional[float] = None

    def step(self, now: float, docked: bool, moving: bool) -> str:
        """Returns '' or an event: 'started', 'aborted'."""
        event = ''
        if docked and not self._was_docked:
            self.dock_pending = True
        if not docked and self._was_docked:
            self.dock_pending = False
        self._was_docked = docked
        if moving:
            self._still_since = None
            if self.est.collecting:
                self.est.abort()
                event = 'aborted'
                if docked:
                    self.dock_pending = True   # retried once still again
            return event
        if self._still_since is None:
            self._still_since = now
        still = now - self._still_since
        if self.est.collecting:
            return ''
        if docked and self.dock_pending and still >= self.still_s:
            self.est.start()
            self.dock_pending = False
            return 'started'
        if not docked and not self.est.ready and still >= self.auto_rest_s:
            self.est.start()
            return 'started'
        return ''

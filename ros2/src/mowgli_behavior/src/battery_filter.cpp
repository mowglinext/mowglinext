// Copyright 2026 Mowgli Project
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

// SPDX-License-Identifier: GPL-3.0

#include "mowgli_behavior/battery_filter.hpp"

#include <algorithm>

namespace mowgli_behavior
{

BatteryVoltageFilter::BatteryVoltageFilter(double tau_s, float min_valid_v)
    : tau_s_(tau_s), min_valid_v_(min_valid_v)
{
}

void BatteryVoltageFilter::reset()
{
  value_.reset();
  last_sec_ = 0.0;
}

std::optional<float> BatteryVoltageFilter::update(float v_raw, double now_sec)
{
  // An invalid sample carries no information, so hold the last filtered value
  // and — deliberately — do NOT advance last_sec_. The gap therefore counts
  // towards the next valid sample's dt, so recovering from a dropout snaps to
  // the fresh reading instead of dragging the pre-dropout value back in.
  if (v_raw < min_valid_v_)
  {
    return value_;
  }

  if (!value_)
  {
    // Bootstrap on the first sane sample. Seeding from a zero-initialised
    // state would otherwise ramp the percent up from 0 % over one time
    // constant every startup.
    value_ = v_raw;
    last_sec_ = now_sec;
    return value_;
  }

  const double dt = now_sec - last_sec_;
  if (dt <= 0.0)
  {
    // Duplicate or non-monotonic clock reading (a stalled /clock before sim
    // time starts ticking, say). Nothing sensible to integrate over.
    return value_;
  }

  // alpha = dt / (tau + dt) — the discrete-time equivalent of a first-order
  // lag with time constant tau_s_, evaluated at whatever rate this sample
  // happened to arrive at. tau <= 0 degenerates to alpha = 1 (pass-through).
  const double alpha = (tau_s_ > 0.0) ? std::clamp(dt / (tau_s_ + dt), 0.0, 1.0) : 1.0;
  value_ = static_cast<float>(alpha * v_raw + (1.0 - alpha) * static_cast<double>(*value_));
  last_sec_ = now_sec;
  return value_;
}

float batteryPercentFromVoltage(float voltage, float v_empty, float v_full)
{
  const float range = v_full - v_empty;
  if (range <= 0.01f)
  {
    return 0.0f;
  }
  const float clamped = std::clamp(voltage, v_empty, v_full);
  return 100.0f * (clamped - v_empty) / range;
}

}  // namespace mowgli_behavior

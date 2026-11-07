// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0

#pragma once

#include <cmath>
#include <cstdint>

namespace mowgli_interfaces
{

inline constexpr std::int64_t kHighLevelFreshNs = 2'000'000'000;

/// Only fresh BT-reported autonomy exempts a target from the soft deadband.
/// Fallback-mutated firmware modes and cmd_vel receipt cannot supply this evidence.
inline bool autonomous_precision_allowed(bool bt_reports_autonomous,
                                         bool have_status,
                                         std::int64_t now_ns,
                                         std::int64_t receipt_ns,
                                         double timeout_s = 2.0)
{
  return bt_reports_autonomous && have_status && receipt_ns > 0 && now_ns >= receipt_ns &&
         static_cast<double>(now_ns - receipt_ns) * 1.0e-9 <= timeout_s;
}

/// No minimum-thrust boost: non-exempt sub-deadband requests become exact STOP.
/// The dust threshold defaults to the hardware bridges' existing input policy.
inline double apply_linear_soft_deadband(double vx,
                                         double minimum,
                                         bool autonomous_precision,
                                         double dust = 1.0e-3)
{
  return !autonomous_precision && std::abs(vx) > dust && std::abs(vx) < minimum ? 0.0 : vx;
}

}  // namespace mowgli_interfaces

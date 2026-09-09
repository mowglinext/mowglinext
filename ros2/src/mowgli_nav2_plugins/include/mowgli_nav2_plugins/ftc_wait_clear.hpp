// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <algorithm>
#include <cmath>

namespace mowgli_nav2_plugins
{

/// Debounce release from FTC's obstacle wait. The path must remain followable
/// continuously for clear_hold_s; any blocked tick discards partial evidence.
/// Returns true while the controller must continue holding zero velocity.
inline bool ObstacleWaitMustContinue(const bool waiting,
                                     const bool path_followable,
                                     const double dt,
                                     const double clear_hold_s,
                                     double& followable_time_s)
{
  if (!waiting)
  {
    followable_time_s = 0.0;
    return false;
  }
  if (!path_followable)
  {
    followable_time_s = 0.0;
    return true;
  }

  if (std::isfinite(dt) && dt > 0.0)
  {
    followable_time_s += dt;
  }
  const double required = std::max(0.0, clear_hold_s);
  if (followable_time_s < required)
  {
    return true;
  }

  followable_time_s = 0.0;
  return false;
}

}  // namespace mowgli_nav2_plugins

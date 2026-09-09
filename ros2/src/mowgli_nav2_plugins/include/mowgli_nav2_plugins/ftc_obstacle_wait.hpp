// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <algorithm>

namespace mowgli_nav2_plugins
{

/// Debounce recovery from a zero-velocity obstacle hold.
///
/// A followable result must persist continuously for clear_hold_s. Any blocked
/// tick resets the accumulated evidence, preventing alternating LiDAR scans
/// from producing a stop/go command on every controller cycle.
inline bool ObstacleWaitReadyToResume(bool followable,
                                      double dt,
                                      double clear_hold_s,
                                      double& continuous_followable_s)
{
  if (!followable)
  {
    continuous_followable_s = 0.0;
    return false;
  }

  continuous_followable_s += std::max(0.0, dt);
  return continuous_followable_s >= std::max(0.0, clear_hold_s);
}

}  // namespace mowgli_nav2_plugins

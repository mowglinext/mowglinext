// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <cmath>

namespace fusion_graph
{

constexpr double kClearGraphMaxLinearSpeedMps = 0.02;
constexpr double kClearGraphMaxAngularSpeedRadps = 0.05;

// Clearing the graph drops map->odom until a fresh position + heading seed is
// available. Only expose that discontinuity while the mission is idle and the
// chassis is stationary.
inline bool ClearGraphAllowed(bool high_level_state_known,
                              bool high_level_idle,
                              double linear_speed_mps,
                              double angular_speed_radps)
{
  return high_level_state_known && high_level_idle && std::isfinite(linear_speed_mps) &&
         std::isfinite(angular_speed_radps) &&
         std::abs(linear_speed_mps) <= kClearGraphMaxLinearSpeedMps &&
         std::abs(angular_speed_radps) <= kClearGraphMaxAngularSpeedRadps;
}

}  // namespace fusion_graph

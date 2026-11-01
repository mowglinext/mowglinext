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
#pragma once

#include <cmath>

namespace mowgli_behavior
{
// Paired with coverage_entry_goal_checker in nav2_params_base.yaml. These are
// entry conditions, not coverage endpoint tolerances or physical pose accuracy.
inline constexpr double kCoverageEntryXYM = 0.03;
inline constexpr double kCoverageEntryYawRad = 0.30;
inline constexpr double kCoverageEntryPoseMaxAgeSec = 0.30;

inline bool coverageEntryReached(
    double x, double y, double yaw, double goal_x, double goal_y, double goal_yaw, double age_sec)
{
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw) || !std::isfinite(goal_x) ||
      !std::isfinite(goal_y) || !std::isfinite(goal_yaw) || !std::isfinite(age_sec) ||
      age_sec < 0 || age_sec > kCoverageEntryPoseMaxAgeSec)
    return false;
  const double yaw_error = std::atan2(std::sin(yaw - goal_yaw), std::cos(yaw - goal_yaw));
  return std::hypot(x - goal_x, y - goal_y) <= kCoverageEntryXYM &&
         std::abs(yaw_error) <= kCoverageEntryYawRad;
}
}  // namespace mowgli_behavior

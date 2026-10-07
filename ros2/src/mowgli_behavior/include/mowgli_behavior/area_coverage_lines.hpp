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
//
// Which swath angle and perimeter winding an area is actually planned with.
//
// A mowing area may carry its own mow angle and perimeter winding (MapArea
// has_mow_angle / has_ring_direction, set from the Map page's "mowing lines"
// preview). They are opt-in overrides of the robot-wide settings, so the plan
// for an area that overrides nothing is exactly the plan it always was:
//
//   * angle: the area's value when it overrides (a negative one pins the area
//     to AUTO even when the robot-wide angle is fixed), else the robot-wide
//     mow_angle_deg (the blackboard value; negative = auto);
//   * winding: the area's value when it overrides, else the goal says "no
//     override" and coverage_server falls back to its live ring_direction
//     parameter, as before.
//
//   * start point: the operator's start point when the area has one, else none — the
//     planner then starts where it always did. It always starts on the outermost ring.
//
// A value that cannot be right (a NaN / out-of-range angle, an unknown winding)
// is ignored and the robot-wide setting applies — a corrupt
// override must never turn into a plan the operator never asked for.
//
// Pure / message-only: unit-tested in test_area_coverage_lines.cpp.
#pragma once

#include <cmath>
#include <cstdint>

#include <mowgli_interfaces/msg/map_area.hpp>

namespace mowgli_behavior
{

struct CoverageLineChoice
{
  /// Swath heading for the plan_coverage goal (degrees; < 0 = auto).
  double mow_angle_deg{-1.0};
  /// false = leave the goal's ring direction to coverage_server's live parameter.
  bool override_ring_direction{false};
  /// 0 planner default / 1 clockwise / 2 counter-clockwise; meaningful only when overriding.
  int32_t ring_direction{0};
  /// The operator chose where the route starts: a map-frame point the planner snaps onto the
  /// OUTERMOST headland ring. false = the planner's own start.
  bool has_start_point{false};
  double start_x{0.0};
  double start_y{0.0};
};

[[nodiscard]] inline CoverageLineChoice ResolveCoverageLines(
    const mowgli_interfaces::msg::MapArea& area, double robot_wide_angle_deg)
{
  CoverageLineChoice choice;
  choice.mow_angle_deg = robot_wide_angle_deg;
  if (area.has_mow_angle && std::isfinite(area.mow_angle_deg) && area.mow_angle_deg < 180.0)
  {
    // Negative = auto, kept as the one canonical auto value the goal expects.
    choice.mow_angle_deg = area.mow_angle_deg < 0.0 ? -1.0 : area.mow_angle_deg;
  }
  if (area.has_ring_direction && area.ring_direction <= 2)
  {
    choice.override_ring_direction = true;
    choice.ring_direction = area.ring_direction;
  }
  // A start point is passed through only when it is a real pair of coordinates: a NaN
  // would otherwise reach the planner, which treats it as no hint, but the BT should not
  // depend on that.
  if (area.has_start_point && std::isfinite(area.start_x) && std::isfinite(area.start_y))
  {
    choice.has_start_point = true;
    choice.start_x = area.start_x;
    choice.start_y = area.start_y;
  }
  return choice;
}

}  // namespace mowgli_behavior

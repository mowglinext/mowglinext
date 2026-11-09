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
/**
 * @file map_replace_gate.hpp
 * @brief When a whole-map REPLACE must wait — pure, no ROS deps.
 *
 * A GUI "Save map" is clear_map -> add_area x N -> save_areas. It throws the
 * whole map away before the new one is complete: the keepout mask goes empty (the garden keepout
 * layer then fills the GLOBAL costmap lethal, so transit and docking plans fail), the mow-progress
 * grid is wiped, every add_area auto-saves a PARTIAL areas.dat, and the area-list generation moves
 * under the behaviour tree. A mower that is undocking, mowing, in transit or driving home is
 * standing on that map.
 *
 * HIGH_LEVEL_STATE_AUTONOMOUS is the BT's own "a mission is running" state
 * (undock, transit, mowing, recovery and RETURNING_HOME — see
 * docs/claude/high-level-api.md). A mid-mission charge hold and a STOP hold
 * publish IDLE, so "Stop, edit the map, Start" works: progress survives, the
 * robot is not moving, and the replace is allowed.
 *
 * Fail-OPEN on missing or stale status: refusing then would make the map
 * uneditable on a bench, in a map-only bring-up or after a BT crash. That is a
 * known gap, not a proof that nothing is running: a status stream that stalls
 * for more than kHighLevelStatusMaxAgeS while a mission continues lets a
 * replace through. The gate protects a running mission from an operator edit;
 * it is not a safety interlock for motion.
 * Incremental edits (add_area on its own, obstacle accept/discard, corridor
 * edits) are NOT gated: they do not drop the map. ~/load_areas (the GUI's
 * backup restore) is NOT gated YET either: that flow overwrites areas.dat
 * BEFORE calling it, so a refusal would leave the disk and the running map
 * disagreeing; it needs a pre-check or a rollback in the GUI first.
 */

#pragma once

#include <cstdint>
#include <optional>
#include <string>

namespace mowgli_map
{

/// Mirrors mowgli_interfaces/msg/HighLevelStatus HIGH_LEVEL_STATE_AUTONOMOUS.
/// Plain integer so this header stays free of generated-message includes;
/// area_manager.cpp static_asserts it against the .msg.
inline constexpr uint8_t kHighLevelStateAutonomous = 2;

/// A status older than this is not evidence of a running mission. The BT
/// republishes at 1 Hz, so this tolerates several lost samples.
inline constexpr double kHighLevelStatusMaxAgeS = 5.0;

/**
 * @param status_received  a HighLevelStatus has ever arrived
 * @param status_age_s     seconds since the last one (ignored if none)
 * @param state            its numeric `state`
 * @return the operator-facing reason the replace must wait, or nullopt.
 */
inline std::optional<std::string> MapReplaceRejection(bool status_received,
                                                      double status_age_s,
                                                      uint8_t state)
{
  if (!status_received || status_age_s > kHighLevelStatusMaxAgeS)
    return std::nullopt;
  if (state != kHighLevelStateAutonomous)
    return std::nullopt;
  return std::string(
      "the mower is running a mission (undocking, mowing or driving home): replacing the whole "
      "map now would wipe its keepout mask and mow progress under it. Press Stop to pause, "
      "save the map, then resume.");
}

}  // namespace mowgli_map

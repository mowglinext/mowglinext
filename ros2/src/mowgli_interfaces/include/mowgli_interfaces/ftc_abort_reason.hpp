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

#include <string>

namespace mowgli_interfaces::ftc_abort_reason
{

// Marker prefix mowgli_nav2_plugins/FTCController stamps onto the
// FollowCoveragePath action's error_msg when it aborts a strip because of an
// obstacle it could not skirt or drive through — a lethal cell in the LOCAL
// costmap against the robot's real oriented footprint (root CLAUDE.md
// Invariant 5's "count the body exactly once" — the model that actually
// produced the abort).
//
// Issue #743: mowgli_behavior/FollowStrip's OWN obstacle confirmation
// (tryStartDetour) re-derives "is this really an obstacle" from a DIFFERENT
// source — a GLOBAL costmap snapshot with a disc body approximation —
// because it needs a stable, bounded view to search a resume point in (the
// local costmap is a rolling window that would scroll during the search).
// The two can disagree: a LiDAR-observed, undrawn obstacle that is lethal
// locally but not (yet) in the global snapshot, or one whose geometry only
// blocks the oriented body, not the disc. When they disagreed, tryStartDetour
// used to decline the detour and fall through to the NON-OBSTACLE recovery
// (skip a fixed distance forward and re-dispatch) — sending the robot back at
// an obstacle that is still there, having booked that span as passed.
//
// This marker is a structured, single-sourced signal BOTH sides own and
// agree on — NOT a parse of nav2's own exception wording. transit_failure.hpp
// bans that for the NavigateToPose path specifically because that text is
// NAV2's own, which can reword silently on an upgrade and produce a WRONG
// verdict with no build-time warning. Here the text is OURS:
// mowgli_nav2_plugins writes it, mowgli_behavior reads it, both are
// versioned together in this repo, and the constant lives in mowgli_interfaces
// (like coverage_geometry.hpp's kSegmentTransitGapM) precisely so the two
// packages cannot drift apart. It only ever ADDS a way to confirm an
// obstacle — the existing global-costmap search is untouched and still runs;
// a target confirmed by EITHER source is treated as an obstacle.
//
// Only stamped onto FTCController's genuinely obstacle-caused throws (the
// obstacle_lookahead collision check and the WEDGED wait-timeout abort) —
// never a TF-lookup failure or the crash/collision detector, which are not
// "an obstacle blocked the path forward" in the sense tryStartDetour needs.
inline constexpr const char* kObstacleAbortMarker = "FTC_OBSTACLE_ABORT: ";

/// True when `error_msg` (a FollowCoveragePath action result's error_msg)
/// carries the marker — i.e. FTCController itself reported this abort as
/// obstacle-caused, independent of any costmap re-derivation.
inline bool HasObstacleAbortMarker(const std::string& error_msg)
{
  return error_msg.rfind(kObstacleAbortMarker, 0) == 0;
}

}  // namespace mowgli_interfaces::ftc_abort_reason

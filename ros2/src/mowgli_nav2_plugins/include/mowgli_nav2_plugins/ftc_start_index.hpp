// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Which index of a freshly dispatched plan FTC should begin tracking from.
// Pure, so it is unit-testable without ROS/nav2 (see test_ftc_start_index.cpp)
// — same shape as ftc_stall.hpp.
//
// ── Why this exists ─────────────────────────────────────────────────────────
// setPlan used to run an UNBOUNDED nearest-point search over the whole plan and
// start tracking wherever it landed. Coverage headland rings are CLOSED —
// start == end to the millimetre — so index 0 and index N-1 describe the SAME
// POINT and the search is genuinely ambiguous; floating-point noise picks the
// winner. When the last index won, FTC drove a few poses, reported the goal
// reached, and FollowStrip recorded the whole ring as MOWED. Observed on the
// robot on both 2026-08-24 runs:
//
//     new path with 436 poses, start=(1.95,9.50), end=(1.95,9.50)
//     setPlan with 436 points, starting at idx=432   -> 99 % of the ring skipped
//     setPlan with 805 points, starting at idx=372   -> 46 % skipped
//
// Both were then logged "reached 99-100 % of path - treating as MOWED", so the
// un-mowed ground was recorded as done and skipped_swaths still read 0. Nothing
// ever came back for it.
//
// Resume is not this function's job. FollowStrip owns it: it trims the path at
// its resume cursor BEFORE dispatch, and decides whether to transit blade-off
// first by measuring to poses.front() — index 0 (coverage_nodes.cpp:444, :644).
// Snapping to some other index here silently disagreed with that decision.

#pragma once

#include <cstddef>
#include <utility>
#include <vector>

namespace mowgli_nav2_plugins
{

/// Index to begin tracking from.
///
/// Default (`snap_to_nearest == false`) is always 0: the caller has already
/// placed the robot at the plan start, or asked Nav2 to.
///
/// The legacy nearest-point behaviour is kept behind the flag for sites that
/// need it, with one correction — ties break toward the EARLIER index, so a
/// closed ring starts at its beginning rather than its end. `>` rather than
/// `>=` on the comparison is the whole fix for the ambiguity.
///
/// @param snap_to_nearest  enable the legacy nearest-point search
/// @param plan             plan points as (x, y) in the robot's frame
/// @param rx,ry            robot position in the same frame
inline std::size_t ChooseStartIndex(bool snap_to_nearest,
                                    const std::vector<std::pair<double, double>>& plan,
                                    double rx,
                                    double ry)
{
  if (!snap_to_nearest || plan.empty())
  {
    return 0;
  }

  std::size_t best = 0;
  double best_dist_sq = -1.0;
  for (std::size_t i = 0; i < plan.size(); ++i)
  {
    const double dx = plan[i].first - rx;
    const double dy = plan[i].second - ry;
    const double d = dx * dx + dy * dy;
    // Strictly-less keeps the FIRST of any equidistant run, so a closed ring
    // resolves to its start rather than its end.
    if (best_dist_sq < 0.0 || d < best_dist_sq)
    {
      best_dist_sq = d;
      best = i;
    }
  }
  return best;
}

}  // namespace mowgli_nav2_plugins

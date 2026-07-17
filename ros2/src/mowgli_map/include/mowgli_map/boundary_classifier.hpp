// Copyright (C) 2024 Cedric <cedric@mowgli.dev>
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

// Pure geofence blade-kill classifier, factored out of
// MapServerNode::check_boundary_violation so it is unit-testable without ROS
// (see progress_tracker.cpp and test_boundary_classifier.cpp). Firmware has
// no polygon knowledge — this classification is the ONLY geofence
// enforcement in the system, so its two margins are deliberately asymmetric:
// "soft" (still recoverable, debounced against single-tick TF noise) vs
// "lethal" (blade/motor hazard — stop immediately, never debounced).

#pragma once

#include <limits>

namespace mowgli_map
{

struct BoundaryClassification
{
  bool soft = false;    // published on /boundary_violation
  bool lethal = false;  // published on /lethal_boundary_violation
};

// Classify one boundary sample and advance the debounce state in place.
// `consecutive_outside_samples` is the caller-owned debounce counter — reset
// to 0 the moment the robot is back inside or within the soft margin, and
// incremented (saturating) on every soft-outside sample. `soft` only
// asserts once the counter reaches `debounce_samples`; `lethal` asserts
// immediately off the same sample — deliberately not debounced, because a
// robot that is genuinely `lethal_margin_m` outside needs the blade to stop
// *now*, not after N more samples.
inline BoundaryClassification ClassifyBoundary(bool inside_any,
                                               double min_edge_dist_m,
                                               double soft_margin_m,
                                               double lethal_margin_m,
                                               int debounce_samples,
                                               int& consecutive_outside_samples)
{
  const bool soft_outside = !inside_any && (min_edge_dist_m > soft_margin_m);
  if (soft_outside)
  {
    if (consecutive_outside_samples < std::numeric_limits<int>::max())
    {
      ++consecutive_outside_samples;
    }
  }
  else
  {
    consecutive_outside_samples = 0;
  }

  BoundaryClassification result;
  result.soft = soft_outside && (consecutive_outside_samples >= debounce_samples);
  result.lethal = !inside_any && (min_edge_dist_m > lethal_margin_m);
  return result;
}

}  // namespace mowgli_map

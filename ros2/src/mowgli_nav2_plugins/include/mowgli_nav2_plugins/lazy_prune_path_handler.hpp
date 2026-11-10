// Copyright (C) 2026 Mowgli Project
//
// SPDX-License-Identifier: GPL-3.0 AND Apache-2.0
//
// Portions of this file are derived from Nav2
// (nav2_controller/plugins/feasible_path_handler.{hpp,cpp}, Nav2 1.5.2) and keep its notices:
//
//   Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
//   Copyright (c) 2023 Dexory
//   Copyright (c) 2023 Open Navigation LLC
//
//   Licensed under the Apache License, Version 2.0 (the "License");
//   you may not use this file except in compliance with the License.
//   You may obtain a copy of the License at
//
//       http://www.apache.org/licenses/LICENSE-2.0
//
//   Unless required by applicable law or agreed to in writing, software
//   distributed under the License is distributed on an "AS IS" BASIS,
//   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//   See the License for the specific language governing permissions and
//   limitations under the License.
//
// Modified for MowgliNext: pruning keeps the passed poses behind an offset (see below).
//
// LazyPrunePathHandler — Nav2's FeasiblePathHandler with O(1) amortized pruning.
//
// controller_server asks its path handler, every control cycle, for the part of
// the plan near the robot (findPlanSegment) and then has it drop the poses the
// robot has passed (transformLocalPlan → prunePlan). Upstream (Nav2 1.5.2) drops
// them with vector::erase(begin, closest), which moves EVERY remaining pose each
// cycle. A coverage sub-path is one goal of up to ~400 000 poses (a 2000 m² lawn
// at 0.03 m), so that erase alone cost 3.9 ms per cycle on x86 at 400k poses —
// more than half of controller_server's own per-cycle path work, on top of FTC
// (test/bench/cycle_bench.cpp).
//
// This subclass keeps the passed poses in place behind an offset and erases them
// in bulk only once they are more than half of the vector. Everything else is
// the upstream algorithm on the same pose sequence, so every result is
// identical: the logical plan [begin + pruned_, end) is always exactly
// upstream's pruned global_plan_up_to_constraint_. Pinned by a differential
// test against the real upstream plugin (test/test_lazy_prune_path_handler.cpp).
//
// The findPlanSegment / transformLocalPlan bodies follow
// nav2_controller/plugins/feasible_path_handler.cpp (Nav2 1.5.2); parameters and
// getTransformedGoal are inherited unchanged. With enforce_path_inversion or
// enforce_path_rotation the plan is compacted every cycle, so those modes keep
// upstream's per-cycle cost.

#pragma once

#include <cstddef>

#include "nav2_controller/plugins/feasible_path_handler.hpp"

namespace mowgli_nav2_plugins
{

class LazyPrunePathHandler : public nav2_controller::FeasiblePathHandler
{
public:
  void setPlan(const nav_msgs::msg::Path& path) override;

  nav2_core::PathSegment findPlanSegment(const geometry_msgs::msg::PoseStamped& pose) override;

  nav_msgs::msg::Path transformLocalPlan(const nav2_core::PathIterator& closest_point,
                                         const nav2_core::PathIterator& pruned_plan_end) override;

private:
  /// Erase the passed poses now. Caller holds mutex_.
  void compact();

  /// Leading poses of global_plan_up_to_constraint_ the robot has passed and
  /// upstream would already have erased.
  std::size_t pruned_{0};
};

}  // namespace mowgli_nav2_plugins

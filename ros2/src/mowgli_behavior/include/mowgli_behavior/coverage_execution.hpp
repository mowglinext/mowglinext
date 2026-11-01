// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <algorithm>
#include <cstddef>
#include <map>
#include <set>

namespace mowgli_behavior
{

/// Accounting against ORIGINAL unit indices, independent of its recovery resume cursor.
/// Execution means observed coverage-goal progress, never physical cutting evidence.
struct UnitCoverageExecution
{
  std::size_t accounted_frontier = 0;
  std::size_t executed_poses = 0;
  /// Conservative: replay does not reclaim a known gap; explicit coverage reset does.
  bool has_gap = false;
};

using UnitCoverageExecutionMap = std::map<std::size_t, UnitCoverageExecution>;

/// Credit only newly observed coverage beyond the frontier; replay cannot refill retry budgets.
inline void accountCoverageAdvance(UnitCoverageExecution& execution, std::size_t original_local)
{
  if (original_local > execution.accounted_frontier)
  {
    execution.executed_poses += original_local - execution.accounted_frontier;
    execution.accounted_frontier = original_local;
  }
}

/// Commit reached coverage before consuming a skipped span without giving it execution credit.
inline void accountCoverageSkip(UnitCoverageExecution& execution,
                                std::size_t original_reached,
                                std::size_t original_resume)
{
  accountCoverageAdvance(execution, original_reached);
  if (original_resume > execution.accounted_frontier)
  {
    execution.has_gap = true;
    execution.accounted_frontier = std::max(execution.accounted_frontier, original_resume);
  }
}

/// Finite execution credit across all units, including units already processed.
inline std::size_t executedAreaPoses(const UnitCoverageExecutionMap* units)
{
  std::size_t sum = 0;
  if (units != nullptr)
  {
    for (const auto& [unit, execution] : *units)
      sum += execution.executed_poses;
  }
  return sum;
}

/// Processed units with known gap-free provenance under the current action contract.
/// An absent ledger or legacy area provenance cannot certify full coverage.
inline std::size_t fullyCoveredUnitCount(const std::set<std::size_t>& processed,
                                         const UnitCoverageExecutionMap* units,
                                         bool unverified = false)
{
  if (unverified || units == nullptr)
    return 0;
  std::size_t count = 0;
  for (const std::size_t unit : processed)
  {
    const auto it = units->find(unit);
    if (it != units->end() && !it->second.has_gap)
      ++count;
  }
  return count;
}

}  // namespace mowgli_behavior

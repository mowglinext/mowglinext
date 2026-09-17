// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Lateral-offset PROFILE planner for FTC's obstacle avoidance.
//
// The legacy avoidance picks ONE scalar offset for the whole lookahead window:
// the side is decided from a single pose, latched, and the offset grown until
// the entire window is clear — so the robot shifts sideways as a block instead
// of going AROUND anything, and "needed > max_lateral_deviation" (WEDGED) fires
// on geometry a curved skirt would pass (field 2026-09-17: ~100 consecutive
// reverse-escapes at the same spot, three sessions in a row).
//
// This planner borrows what makes DWB / MPPI good at obstacles — evaluate whole
// candidate trajectories against separate critics and keep the best — WITHOUT
// what got MPPI reverted on this robot: it does not sample velocities and does
// not replace the tracker. The candidates live in PATH space (Frenet frame):
// a lattice of (station along the path) x (lateral offset), searched by dynamic
// programming. FTC's PID keeps tracking; only the offset it is asked to hold at
// the carrot changes. Deterministic: same costmap, same answer.
//
//   hard critics  : node blocked (real footprint on a lethal cell, or outside
//                   the mowing zone)           -> node removed
//                   |d(offset)/d(station)| <= max_slope -> edge removed
//   soft critics  : un-mowed area   (|offset| * ds)
//                   smoothness      (|delta offset|)
//                   side stability  (offset on the side opposite to the one the
//                                    previous plan committed to)
//                   return to line  (|offset| at the end of the horizon)
//
// The module is costmap-free: the caller supplies `blocked(station, offset)`,
// which keeps it unit-testable on synthetic obstacle fields.

#pragma once

#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <vector>

namespace mowgli_nav2_plugins
{

struct OffsetLatticeCfg
{
  /// Lateral resolution of the lattice, metres.
  double offset_step{0.05};
  /// Largest |offset| a candidate may use, metres.
  double max_offset{1.5};
  /// Weight of the un-mowed area, per m^2.
  double w_area{1.0};
  /// Weight of each metre of lateral change (keeps the profile smooth).
  double w_smooth{0.2};
  /// Extra weight, per m^2, for area on the side OPPOSITE to `preferred_sign`.
  /// A soft latch: the side stays put unless the other one is clearly cheaper.
  double w_side_switch{2.0};
  /// Weight of the offset left at the end of the horizon, per metre.
  double w_terminal{0.5};
};

struct OffsetLatticeResult
{
  /// False when no collision-free profile exists inside the lattice.
  bool feasible{false};
  /// One lateral offset per station (same size as the `stations` input).
  std::vector<double> offsets;
  double cost{0.0};

  double MaxAbsOffset() const
  {
    double m = 0.0;
    for (const double o : offsets)
    {
      m = std::max(m, std::fabs(o));
    }
    return m;
  }
};

/// Station spacing that lets the profile change by exactly one lattice step per
/// station at the steepest allowed slope. The caller resamples its path window
/// to (about) this spacing; a finer window would make every lateral move
/// infeasible, a coarser one wastes resolution.
inline double OffsetLatticeStationSpacing(double offset_step, double max_slope)
{
  return (max_slope > 0.0) ? offset_step / max_slope : offset_step;
}

/// Plan the cheapest collision-free offset profile.
///
/// @param stations        cumulative arc length of each station, metres,
///                        non-decreasing, stations[0] is where the carrot is NOW
/// @param start_offset    offset currently applied (the profile must start there:
///                        station 0 is never collision-checked, the robot is
///                        where it is)
/// @param preferred_sign  +1 / -1 for the side the previous plan committed to,
///                        0 for none
/// @param blocked         hard critic: true when the body cannot be at
///                        (station index, lateral offset)
inline OffsetLatticeResult PlanOffsetProfile(
    const std::vector<double>& stations,
    double start_offset,
    int preferred_sign,
    const std::function<bool(std::size_t, double)>& blocked,
    const OffsetLatticeCfg& cfg = {})
{
  OffsetLatticeResult result;
  if (stations.empty() || cfg.offset_step <= 0.0 || cfg.max_offset < 0.0)
  {
    return result;
  }

  const int half = static_cast<int>(std::floor(cfg.max_offset / cfg.offset_step + 1e-9));
  const int width = 2 * half + 1;
  const auto offset_of = [&](int k)
  {
    return static_cast<double>(k - half) * cfg.offset_step;
  };
  const int start_k =
      std::min(width - 1,
               std::max(0, half + static_cast<int>(std::lround(start_offset / cfg.offset_step))));

  constexpr double kInf = std::numeric_limits<double>::infinity();
  const std::size_t n = stations.size();
  std::vector<std::vector<double>> cost(n,
                                        std::vector<double>(static_cast<std::size_t>(width), kInf));
  std::vector<std::vector<int>> parent(n, std::vector<int>(static_cast<std::size_t>(width), -1));
  cost[0][static_cast<std::size_t>(start_k)] = 0.0;

  for (std::size_t i = 1; i < n; ++i)
  {
    const double ds = std::max(0.0, stations[i] - stations[i - 1]);
    for (int k = 0; k < width; ++k)
    {
      // One lattice step per station: the slope limit is enforced by the
      // station spacing the caller chose (OffsetLatticeStationSpacing).
      double best = kInf;
      int best_parent = -1;
      for (int pk = std::max(0, k - 1); pk <= std::min(width - 1, k + 1); ++pk)
      {
        const double c = cost[i - 1][static_cast<std::size_t>(pk)];
        if (c == kInf)
        {
          continue;
        }
        const double move = std::fabs(offset_of(k) - offset_of(pk));
        const double candidate = c + cfg.w_smooth * move;
        if (candidate < best)
        {
          best = candidate;
          best_parent = pk;
        }
      }
      if (best_parent < 0)
      {
        continue;
      }
      const double offset = offset_of(k);
      if (blocked(i, offset))
      {
        continue;
      }
      double node = cfg.w_area * std::fabs(offset) * ds;
      const int sign = (offset > 0.0) ? 1 : ((offset < 0.0) ? -1 : 0);
      if (preferred_sign != 0 && sign != 0 && sign != preferred_sign)
      {
        node += cfg.w_side_switch * std::fabs(offset) * ds;
      }
      cost[i][static_cast<std::size_t>(k)] = best + node;
      parent[i][static_cast<std::size_t>(k)] = best_parent;
    }
  }

  // Cheapest end node, counting what is left to come back from. Ties go to the
  // smaller |offset|, then to the preferred side, so the answer is stable.
  int end_k = -1;
  double end_cost = kInf;
  for (int k = 0; k < width; ++k)
  {
    const double c = cost[n - 1][static_cast<std::size_t>(k)];
    if (c == kInf)
    {
      continue;
    }
    const double total = c + cfg.w_terminal * std::fabs(offset_of(k));
    const bool better =
        total < end_cost - 1e-12 || (std::fabs(total - end_cost) <= 1e-12 && end_k >= 0 &&
                                     std::fabs(offset_of(k)) < std::fabs(offset_of(end_k)));
    if (end_k < 0 || better)
    {
      end_k = k;
      end_cost = total;
    }
  }
  if (end_k < 0)
  {
    return result;
  }

  result.feasible = true;
  result.cost = end_cost;
  result.offsets.assign(n, 0.0);
  int k = end_k;
  for (std::size_t i = n; i-- > 0;)
  {
    result.offsets[i] = offset_of(k);
    if (i > 0)
    {
      k = parent[i][static_cast<std::size_t>(k)];
    }
  }
  return result;
}

}  // namespace mowgli_nav2_plugins

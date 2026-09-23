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
// The costmap side of FTC's offset lattice: which (pose, offset) nodes the
// body cannot occupy, and the degrading solve (reaction slack -> no slack ->
// ignore the stations under the body, each over a shrinking horizon) around
// the pure DP in ftc_offset_lattice.hpp.
//
// Kept out of FTCController so the exact production decision can be replayed
// offline on recorded costmaps and pinned by tests without a node: the
// controller only supplies the plan window (already in the local costmap
// frame), the costmaps and the parameters.

#pragma once

#include <cstddef>
#include <functional>
#include <limits>
#include <optional>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mowgli_nav2_plugins/ftc_offset_lattice.hpp"
#include "mowgli_nav2_plugins/obstacle_deviation.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace mowgli_nav2_plugins
{

/// Plan poses the lattice is solved over, resampled at the station spacing.
struct LatticeWindow
{
  /// Plan index of each resampled pose: `lead` behind the carrot, the carrot,
  /// then the horizon ahead.
  std::vector<std::size_t> pose_idx;
  /// Position of the carrot inside `pose_idx` (= number of poses behind it).
  std::size_t carrot_pos{0};
  /// A pivot corner ends the horizon: its station must be planned at ZERO offset.
  bool corner_is_last_station{false};
};

/// Resample `plan` from `lead_m` behind `carrot_idx` to `horizon_m` ahead of it,
/// within the leg [leg_first, leg_last] (a pivot corner ends a leg: the heading,
/// and so every lateral offset, is discontinuous across it). A pivot `corner`
/// reached by the horizon walk becomes the last station.
LatticeWindow ResampleLatticeWindow(const std::vector<geometry_msgs::msg::PoseStamped>& plan,
                                    std::size_t carrot_idx,
                                    std::size_t leg_first,
                                    std::size_t leg_last,
                                    std::optional<std::size_t> corner,
                                    double ds,
                                    double lead_m,
                                    double horizon_m);

struct LatticeSolverCfg
{
  OffsetLatticeCfg lattice;
  /// Nominal station spacing (OffsetLatticeStationSpacing); used as the mean
  /// spacing when the window holds a single station.
  double station_spacing_m{0.05};
  /// How far the robot trails the carrot (CarrotMaxLead).
  double lead_m{0.30};
  /// avoidance_reaction_m: slack the skirt must be in place before an obstacle.
  double reaction_m{0.5};
  /// avoidance_min_horizon_m: a blockage beyond this only cuts the horizon.
  double min_horizon_m{1.0};
};

/// Why a lattice node is blocked.
enum class LatticeBlock : signed char
{
  kFree = 0,
  /// The body polygon covers a lethal cell of the local costmap.
  kObstacle = 1,
  /// Off the planned line, the body axis crosses the zone (global cells >= 99).
  kZone = 2,
};

struct LatticeSolution
{
  OffsetLatticeResult plan;
  /// 1 = with reaction slack, 2 = without, 3 = ignoring the stations under the
  /// body. Meaningful only when `plan.feasible`.
  int level{0};
  /// Stations actually planned (the horizon may have been cut).
  std::size_t planned_stations{0};
};

/// One tick's lattice problem. Holds a REFERENCE to the local costmap: build it,
/// solve and drop it inside one tick, under the costmap lock the caller already
/// holds (FTCController::updateLateralDeviation); never keep it across ticks.
class LatticeSolver
{
public:
  /// @param costmap  local costmap (master grid), in the frame of `poses`
  /// @param guard    zone guard (global costmap), applied off the line only
  /// @param body     base-frame footprint, already widened by the clearance margin
  /// @param poses    the window's poses in the costmap frame (LatticeWindow order)
  /// @param carrot_pos position of the carrot in `poses`
  /// @param corner_is_last_station the last pose is a pivot corner (zero offset)
  LatticeSolver(const nav2_costmap_2d::Costmap2D& costmap,
                const BoundaryGuard& guard,
                const ObstacleDeviation::Footprint& body,
                std::vector<geometry_msgs::msg::PoseStamped> poses,
                std::size_t carrot_pos,
                bool corner_is_last_station,
                const LatticeSolverCfg& cfg);

  /// Cumulative arc length of each station; station 0 is the carrot.
  const std::vector<double>& Stations() const
  {
    return stations_;
  }

  /// Station index of a pivot corner that ends the horizon, or max().
  std::size_t CornerStation() const
  {
    return corner_station_;
  }

  /// Number of stations covering `metres` of path (mean station spacing).
  std::size_t StationsIn(double metres) const;

  /// Cheapest collision-free profile starting at `start_offset`. `only_side`
  /// != 0 forbids every offset on the other side (hard).
  LatticeSolution Solve(double start_offset, int preferred_sign, int only_side);

  /// Memoised node test on a single pose of the window.
  LatticeBlock PoseBlock(std::size_t pose, double offset);

  /// Node test the DP uses: `station` at `offset` over its span of poses
  /// (`ahead` stations of reaction slack; stations <= `grace` always free).
  LatticeBlock SpanBlock(std::size_t station, double offset, std::size_t ahead, std::size_t grace);

  const std::vector<geometry_msgs::msg::PoseStamped>& Poses() const
  {
    return poses_;
  }

  std::size_t CarrotPos() const
  {
    return carrot_pos_;
  }

private:
  const nav2_costmap_2d::Costmap2D& costmap_;
  BoundaryGuard guard_;
  ObstacleDeviation::Footprint body_;
  std::vector<geometry_msgs::msg::PoseStamped> poses_;
  std::size_t carrot_pos_;
  LatticeSolverCfg cfg_;
  std::vector<double> stations_;
  std::size_t corner_station_{std::numeric_limits<std::size_t>::max()};
  int half_{0};
  std::size_t width_{1};
  std::vector<signed char> memo_;
  double axis_rear_{0.0};
  double axis_front_{0.0};
};

/// The lattice problem FTC solves on the first tick it FOLLOWS from plan index
/// `carrot_idx` with no offset applied — e.g. right after a turn-fallback rejoin
/// (ftc_turn_fallback.hpp): true when it finds a profile, false when it would be
/// WEDGED again. `leg_first`/`leg_last`/`corner` are the pivot leg and next
/// pivot corner of `carrot_idx` (ftc_pivot.hpp PivotLeg / NextPivotCorner);
/// `to_costmap` maps a plan pose into the costmap frame.
bool LatticeFeasibleFrom(
    const nav2_costmap_2d::Costmap2D& costmap,
    const BoundaryGuard& guard,
    const ObstacleDeviation::Footprint& body,
    const std::vector<geometry_msgs::msg::PoseStamped>& plan,
    std::size_t carrot_idx,
    std::size_t leg_first,
    std::size_t leg_last,
    std::optional<std::size_t> corner,
    const std::function<geometry_msgs::msg::PoseStamped(const geometry_msgs::msg::PoseStamped&)>&
        to_costmap,
    const LatticeSolverCfg& cfg,
    double horizon_m);

}  // namespace mowgli_nav2_plugins

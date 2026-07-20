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

#include "mowgli_behavior/coverage_nodes.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "action_msgs/msg/goal_status.hpp"
#include "mowgli_behavior/coverage_persistence.hpp"
#include "tf2/exceptions.h"

namespace mowgli_behavior
{

namespace
{

/// Plan-geometry fingerprint of the drivable units (FNV-1a 64-bit over every
/// unit's pose count and each pose position quantized to mm). Any change of mow
/// angle, swath ordering, or sub-path split — all of which can preserve the
/// total pose COUNT — changes the hash, so it is a sound staleness key for the
/// resume cursor. Deterministic: F2C is deterministic for a fixed area+params,
/// and mm-quantization absorbs any residual float noise across re-plans/restart.
uint64_t hashPlanGeometry(const std::vector<nav_msgs::msg::Path>& units)
{
  uint64_t h = 1469598103934665603ULL;  // FNV-1a 64-bit offset basis
  auto mix = [&h](uint64_t v)
  {
    for (int b = 0; b < 8; ++b)
    {
      h ^= (v & 0xFFULL);
      h *= 1099511628211ULL;  // FNV-1a 64-bit prime
      v >>= 8;
    }
  };
  auto q = [](double x) -> uint64_t
  {
    // Round to mm, then reinterpret the signed integer as unsigned for hashing.
    return static_cast<uint64_t>(static_cast<int64_t>(std::llround(x * 1000.0)));
  };
  mix(units.size());
  for (const auto& u : units)
  {
    mix(u.poses.size());
    for (const auto& p : u.poses)
    {
      mix(q(p.pose.position.x));
      mix(q(p.pose.position.y));
    }
  }
  return h;
}

}  // namespace

void refreshSwathProgress(BTContext& ctx, uint32_t area_idx, std::size_t unit_count)
{
  // Denominator = drivable units in the plan; numerator = units recorded mowed
  // so far for this area. operator[] default-inserts an empty set for a not-yet-
  // seen area (matching the existing area_completed_swaths access pattern), so a
  // fresh area reads 0 completed. Display-only — no blade/motion effect.
  ctx.total_swaths = static_cast<int>(unit_count);
  ctx.completed_swaths = static_cast<int>(ctx.area_completed_swaths[area_idx].size());
}

float coveragePercentFromCursor(std::size_t absolute_cursor, std::size_t total_poses)
{
  if (total_poses == 0)
  {
    return 0.0f;
  }
  const float pct = 100.0f * static_cast<float>(absolute_cursor) / static_cast<float>(total_poses);
  return std::clamp(pct, 0.0f, 100.0f);
}

// ===========================================================================
// FollowStrip — execute the coverage plan as ONE CONTINUOUS joined path
// ===========================================================================

BT::NodeStatus FollowStrip::onStart()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  // ONE CONTINUOUS PATH, A→Z. The coverage server's full_path is the rings +
  // swaths CONNECTED by forward turn-around arcs (coverage_server →
  // buildContinuousPath): a single CUSP-FREE, in-bounds polyline. We drive it as
  // ONE FollowCoveragePath goal — no per-segment dispatch, no transit, no cuts.
  //   * No sharp ~180° reversal anywhere → MPPI doesn't dither/spin; it tracks
  //     the smooth arcs (helped by the backported arc-length fix, PR #6055) AND
  //     keeps its dynamic obstacle avoidance (deviate around, return to path).
  //     enforce_path_inversion is OFF (nothing to crop). Re-mowing on the turn
  //     loops is accepted.
  //   * cusp-free + in-bounds are guaranteed by test_coverage_planning
  //     (CoverageContinuousPath) on the real area; a future area that breaks it
  //     fails that test first.
  // TransitToStrip (boundary-aware) already drove the robot to the path start,
  // so FollowStrip dispatches the full path as one goal. Fall back to joining the
  // raw segments only if the connected path is somehow missing.
  swaths_.clear();
  swath_base_.clear();
  resume_start_idx_ = 0;
  path_progress_idx_ = 0;
  total_path_poses_ = 0;
  area_idx_ = (ctx->current_area >= 0) ? static_cast<uint32_t>(ctx->current_area) : 0u;

  // Build the drivable UNITS. Prefer the hole-free continuous sub-paths (#333):
  // FollowStrip drives each with MPPI and bridges the gap between consecutive
  // units with a blade-off Nav2 transit that routes around the obstacle. A
  // hole-free field yields exactly one sub-path (== the single continuous path).
  // Fall back to the single continuous full_path, or to joining the raw segments
  // if neither is present.
  std::vector<nav_msgs::msg::Path> units;
  if (!ctx->current_strip_subpaths.empty())
  {
    for (const auto& sp : ctx->current_strip_subpaths)
    {
      if (sp.poses.size() >= 2)
      {
        units.push_back(sp);
      }
    }
  }
  if (units.empty())
  {
    nav_msgs::msg::Path full_path;
    if (ctx->current_strip_path.poses.size() >= 2)
    {
      full_path = ctx->current_strip_path;
    }
    else
    {
      const auto& segs = ctx->current_strip_segments;
      if (segs.empty())
      {
        RCLCPP_ERROR(ctx->node->get_logger(), "FollowStrip: no coverage path/segments in context");
        return BT::NodeStatus::FAILURE;
      }
      full_path.header = segs.front().header;
      for (const auto& seg : segs)
      {
        full_path.poses.insert(full_path.poses.end(), seg.poses.begin(), seg.poses.end());
      }
      if (full_path.poses.empty())
      {
        RCLCPP_ERROR(ctx->node->get_logger(), "FollowStrip: coverage path empty");
        return BT::NodeStatus::FAILURE;
      }
      RCLCPP_WARN(
          ctx->node->get_logger(),
          "FollowStrip: no continuous path — fell back to %zu joined raw segments (%zu poses)",
          segs.size(),
          full_path.poses.size());
    }
    units.push_back(std::move(full_path));
  }

  // Prefix-sum base offsets (from ORIGINAL unit sizes) and the concatenation
  // length — the resume cursor is an index into this concatenation.
  swath_base_.assign(units.size(), 0);
  {
    std::size_t acc = 0;
    for (std::size_t i = 0; i < units.size(); ++i)
    {
      swath_base_[i] = acc;
      acc += units[i].poses.size();
    }
    total_path_poses_ = acc;
  }

  // Staleness guard for disk-loaded resume state: F2C is deterministic for a
  // fixed area+params, so a persisted plan whose GEOMETRY no longer matches the
  // freshly re-planned path means the area geometry or coverage params changed
  // since the interrupted (possibly pre-restart) session. Resuming a stale cursor
  // / skipping stale swath indices against a DIFFERENT path is UNSAFE — it can
  // re-enable the blade at the wrong location — so discard this area's persisted
  // resume state and mow it fresh from the start.
  //
  // The key is a full plan-geometry fingerprint (hashPlanGeometry), NOT the pose
  // COUNT alone: the AUTO mow-angle tie-break (longest-edge; a tiny polygon edit
  // can flip which edge wins) and the sub-path split can both yield a
  // geometrically different concatenation with the SAME pose count, which a
  // count-only guard would wrongly accept as resumable. (Within a live session
  // the fingerprint always matches, so this is a no-op there; it only bites on a
  // cross-dispatch/cross-restart load after the plan geometry changed.)
  const uint64_t plan_fingerprint = hashPlanGeometry(units);
  {
    auto fpit = ctx->area_plan_fingerprint.find(area_idx_);
    const bool has_persisted_state = ctx->area_resume_pose_index.count(area_idx_) > 0 ||
                                     ctx->area_completed_swaths.count(area_idx_) > 0;
    if (fpit != ctx->area_plan_fingerprint.end() && fpit->second != plan_fingerprint &&
        has_persisted_state)
    {
      RCLCPP_WARN(ctx->node->get_logger(),
                  "FollowStrip: area %u persisted plan fingerprint 0x%016llx != re-plan 0x%016llx "
                  "(%zu poses) — area/params changed, discarding stale resume state and mowing "
                  "fresh",
                  area_idx_,
                  static_cast<unsigned long long>(fpit->second),
                  static_cast<unsigned long long>(plan_fingerprint),
                  total_path_poses_);
      ctx->area_resume_pose_index.erase(area_idx_);
      ctx->area_completed_swaths.erase(area_idx_);
      ctx->completed_areas.erase(area_idx_);
      saveCoverageResumeState(*ctx);
    }
  }
  ctx->area_plan_fingerprint[area_idx_] = plan_fingerprint;
  ctx->area_path_pose_count[area_idx_] = total_path_poses_;

  // RESUME: if an earlier pass was interrupted mid-path (recharge / preempt /
  // controller abort / restart), map the persisted absolute cursor (index into
  // the concatenation of all units) to (unit k, local offset). Units 0..k-1 are
  // marked done so the skip-loop advances past them, and unit k is trimmed so we
  // resume mid-unit. F2C is deterministic, so the re-planned units are identical
  // and the cursor is stable. For a single unit this reduces to trimming the
  // already-driven prefix. Guard against a stale/last-pose cursor.
  {
    auto it = ctx->area_resume_pose_index.find(area_idx_);
    if (it != ctx->area_resume_pose_index.end() && it->second > 0 &&
        it->second + 2 < total_path_poses_)
    {
      const std::size_t cursor = it->second;
      std::size_t k = 0;
      std::size_t local = cursor;
      while (k < units.size() && local >= units[k].poses.size())
      {
        local -= units[k].poses.size();
        ++k;
      }
      if (k < units.size())
      {
        for (std::size_t j = 0; j < k; ++j)
        {
          ctx->area_completed_swaths[area_idx_].insert(j);  // fully-driven units
        }
        if (local > 0 && local + 2 < units[k].poses.size())
        {
          resume_start_idx_ = local;
          nav_msgs::msg::Path trimmed;
          trimmed.header = units[k].header;
          trimmed.poses.assign(units[k].poses.begin() + static_cast<std::ptrdiff_t>(local),
                               units[k].poses.end());
          units[k] = std::move(trimmed);
        }
        RCLCPP_INFO(ctx->node->get_logger(),
                    "FollowStrip: RESUMING area %u at pose %zu/%zu (%.0f%% already driven) — "
                    "unit %zu/%zu",
                    area_idx_,
                    cursor,
                    total_path_poses_,
                    100.0 * static_cast<double>(cursor) / static_cast<double>(total_path_poses_),
                    k + 1,
                    units.size());
      }
    }
  }
  if (units.size() > 1)
  {
    RCLCPP_INFO(ctx->node->get_logger(),
                "FollowStrip: following %zu hole-free sub-paths (blade-off Nav2 transit between)",
                units.size());
  }
  else
  {
    RCLCPP_INFO(ctx->node->get_logger(),
                "FollowStrip: following ONE continuous coverage path (%zu poses)",
                units.front().poses.size());
  }
  for (auto& u : units)
  {
    swaths_.push_back(std::move(u));
  }
  swaths_skipped_ = 0;
  transit_active_ = false;
  transit_pending_ = false;
  swath_goal_sent_ = false;
  // Swath-completion model (replaces the mow_progress cell grid): record this
  // area's swath count and resume at the first swath NOT already mowed. F2C is
  // deterministic for a fixed area+params, so indices are stable across the
  // re-plan that a recharge/preempt resume triggers.
  ctx->area_swath_count[area_idx_] = swaths_.size();
  // Seed the GUI's live swath progress from the START of the pass (total > 0,
  // completed = whatever this area already had mowed on a resume) so the
  // percentage renders immediately instead of only at the terminal branch.
  refreshSwathProgress(*ctx, area_idx_, swaths_.size());
  swath_idx_ = 0;
  {
    const auto& done = ctx->area_completed_swaths[area_idx_];
    while (swath_idx_ < swaths_.size() && done.count(swath_idx_) > 0)
    {
      ++swath_idx_;
    }
    if (swath_idx_ >= swaths_.size())
    {
      // Every swath already mowed this session — nothing left for this area.
      // total/completed swaths were already seeded by refreshSwathProgress above
      // (both == swaths_.size() here); just finalise the percentage.
      ctx->completed_areas.insert(area_idx_);
      saveCoverageResumeState(*ctx);
      ctx->coverage_percent = 100.0f;
      RCLCPP_INFO(ctx->node->get_logger(),
                  "FollowStrip: area %u already fully mowed (%zu/%zu swaths) — nothing to do",
                  area_idx_,
                  done.size(),
                  swaths_.size());
      return BT::NodeStatus::SUCCESS;
    }
  }

  if (!follow_client_)
  {
    follow_client_ = rclcpp_action::create_client<Nav2FollowPath>(ctx->node, "/follow_path");
  }
  if (!nav_client_)
  {
    nav_client_ = rclcpp_action::create_client<Nav2Navigate>(ctx->node, "/navigate_to_pose");
  }
  if (!coverage_plan_pub_)
  {
    // Match the goal-checker's plan_topic QoS exactly (reliable +
    // transient_local, depth 1) so the PathProgressGoalChecker always has
    // the active segment.
    coverage_plan_pub_ = ctx->node->create_publisher<nav_msgs::msg::Path>(
        "/controller_server/FollowCoveragePath/global_plan", rclcpp::QoS(1).transient_local());
  }
  if (!follow_client_->wait_for_action_server(std::chrono::seconds(5)))
  {
    RCLCPP_ERROR(ctx->node->get_logger(), "FollowStrip: follow_path not available");
    return BT::NodeStatus::FAILURE;
  }

  setBladeEnabled(true);
  blade_start_time_ = std::chrono::steady_clock::now();
  goal_sent_ = false;

  RCLCPP_INFO(ctx->node->get_logger(),
              "FollowStrip: area %u, %zu segments (%zu already done); "
              "blade enabled, waiting %.1fs for spinup",
              area_idx_,
              swaths_.size(),
              ctx->area_completed_swaths[area_idx_].size(),
              kBladeSpinupDelaySec);

  // Seed the smooth GUI percent for THIS area: 0 % for a fresh area, or the
  // resumed fraction if resuming mid-path. Resets the value per area so it does
  // not carry the previous area's 100 % into the next area's first ticks.
  ctx->coverage_percent = livePercent();

  return BT::NodeStatus::RUNNING;
}

double FollowStrip::distanceToSegmentStart(const std::shared_ptr<BTContext>& ctx) const
{
  if (swath_idx_ >= swaths_.size() || swaths_[swath_idx_].poses.empty())
  {
    return 0.0;
  }
  try
  {
    const auto tf = ctx->tf_buffer->lookupTransform("map", "base_footprint", tf2::TimePointZero);
    const auto& start = swaths_[swath_idx_].poses.front().pose.position;
    return std::hypot(start.x - tf.transform.translation.x, start.y - tf.transform.translation.y);
  }
  catch (const tf2::TransformException&)
  {
    // No pose → take the safe path (boundary-aware transit).
    return std::numeric_limits<double>::max();
  }
}

void FollowStrip::updateProgress(const std::shared_ptr<BTContext>& ctx)
{
  if (swath_idx_ >= swaths_.size())
  {
    return;
  }
  const auto& poses = swaths_[swath_idx_].poses;
  if (poses.empty())
  {
    return;
  }
  double rx, ry;
  try
  {
    const auto tf = ctx->tf_buffer->lookupTransform("map", "base_footprint", tf2::TimePointZero);
    rx = tf.transform.translation.x;
    ry = tf.transform.translation.y;
  }
  catch (const tf2::TransformException&)
  {
    return;  // no pose this tick — keep the last cursor
  }
  // Monotonic, bounded forward nearest-pose search from the current cursor. The
  // path can be thousands of poses, so we only scan a forward window (the robot
  // can't have jumped far in one tick) — O(window), cheap to call every tick.
  constexpr std::size_t kSearchWindow = 400;
  const std::size_t end = std::min(poses.size(), path_progress_idx_ + kSearchWindow);
  double best_d2 = std::numeric_limits<double>::max();
  std::size_t best = path_progress_idx_;
  for (std::size_t i = path_progress_idx_; i < end; ++i)
  {
    const auto& p = poses[i].pose.position;
    const double d2 = (p.x - rx) * (p.x - rx) + (p.y - ry) * (p.y - ry);
    if (d2 < best_d2)
    {
      best_d2 = d2;
      best = i;
    }
  }
  if (best > path_progress_idx_)
  {
    path_progress_idx_ = best;
  }
}

float FollowStrip::livePercent() const
{
  // Cursor is an index into the CONCATENATION of all units: the current unit's
  // base offset + how far into that (possibly trimmed) unit we got. Monotonic as
  // the robot advances across sub-paths, so the percentage climbs smoothly.
  const std::size_t base = (swath_idx_ < swath_base_.size()) ? swath_base_[swath_idx_] : 0;
  const std::size_t absolute = base + resume_start_idx_ + path_progress_idx_;
  return coveragePercentFromCursor(absolute, total_path_poses_);
}

void FollowStrip::persistResumeCursor(const std::shared_ptr<BTContext>& ctx)
{
  if (total_path_poses_ == 0)
  {
    return;
  }
  // Cursor is an index into the CONCATENATION of all units: the current unit's
  // base offset + how far into that (possibly trimmed) unit we got.
  const std::size_t base = (swath_idx_ < swath_base_.size()) ? swath_base_[swath_idx_] : 0;
  const std::size_t absolute = base + resume_start_idx_ + path_progress_idx_;
  const double pct = coveragePercentFromCursor(absolute, total_path_poses_);

  // Near-complete acceptance. The coverage_goal_checker already treats
  // >= 95 % monotonic path traversal as goal-reached; if a reactive guard
  // (a phantom obstacle promotion, or a localization-drift boundary flicker at
  // the outermost ring) halts FollowStrip AFTER that much of the path is driven,
  // the area is effectively mowed. Re-dispatching it just re-plans and re-drives
  // into the SAME edge trip — an infinite loop that never advances to GoHome.
  // Instead, mark every swath of this area done so GetNextUnmowedArea retires it
  // and the BT proceeds to the next area / dock.
  constexpr double kAreaCompleteProgressPct = 95.0;
  if (pct >= kAreaCompleteProgressPct)
  {
    for (std::size_t s = 0; s < swaths_.size(); ++s)
    {
      ctx->area_completed_swaths[area_idx_].insert(s);
    }
    ctx->completed_areas.insert(area_idx_);
    ctx->area_resume_pose_index.erase(area_idx_);  // done — do not resume
    ctx->coverage_percent = 100.0f;
    RCLCPP_INFO(ctx->node->get_logger(),
                "FollowStrip: area %u interrupted at pose %zu/%zu (%.0f%%) — >= %.0f%% driven, "
                "accepting as MOWED (guard trip near field edge); advancing to GoHome/dock "
                "instead of re-dispatching",
                area_idx_,
                absolute,
                total_path_poses_,
                pct,
                kAreaCompleteProgressPct);
    saveCoverageResumeState(*ctx);
    return;
  }

  ctx->area_resume_pose_index[area_idx_] = absolute;
  ctx->coverage_percent = static_cast<float>(pct);
  RCLCPP_INFO(ctx->node->get_logger(),
              "FollowStrip: area %u interrupted at pose %zu/%zu (%.0f%%) — resume cursor saved",
              area_idx_,
              absolute,
              total_path_poses_,
              static_cast<double>(ctx->coverage_percent));
  // Persist to disk so the resume survives a full process/container restart,
  // not just the in-RAM BT halt (the whole point of issue #334).
  saveCoverageResumeState(*ctx);
}

bool FollowStrip::sendFollowGoal(const std::shared_ptr<BTContext>& ctx)
{
  if (!follow_client_ || swath_idx_ >= swaths_.size())
  {
    return false;
  }
  // Mowing resumes on this segment — make sure the blade is on (it may have
  // been switched off for a preceding inter-segment transit).
  setBladeEnabled(true);

  Nav2FollowPath::Goal goal;
  goal.path = swaths_[swath_idx_];
  goal.controller_id = "FollowCoveragePath";
  goal.goal_checker_id = "coverage_goal_checker";

  // Publish the segment on the coverage controller's global_plan topic BEFORE
  // dispatching the goal, so the PathProgressGoalChecker has the plan in hand
  // by the time the controller starts ticking (MPPI doesn't republish it).
  if (coverage_plan_pub_)
  {
    coverage_plan_pub_->publish(goal.path);
  }

  follow_handle_.reset();
  follow_future_ = follow_client_->async_send_goal(goal);
  swath_goal_sent_ = true;

  RCLCPP_INFO(ctx->node->get_logger(),
              "FollowStrip: sent segment %zu/%zu (%zu poses) to the coverage controller",
              swath_idx_ + 1,
              swaths_.size(),
              goal.path.poses.size());
  return true;
}

bool FollowStrip::sendCurrentSwath(const std::shared_ptr<BTContext>& ctx)
{
  if (swath_idx_ >= swaths_.size())
  {
    return false;
  }
  // When the segment start is far away (resume mid-list, a skipped segment,
  // or a concave field whose serpentine hops across a notch), it MUST be reached
  // with a BLADE-OFF Nav2 transit via the boundary-aware global planner — we do
  // not mow the transit ("navigation area"), and FTC would otherwise cut
  // cross-country toward the plan, blade-on, potentially through out-of-bounds
  // area. This gap check is STRUCTURAL for blade safety: past kSegmentTransitGap
  // there is no code path that may drive to the start with the blade on.
  const double gap = distanceToSegmentStart(ctx);
  if (gap > kSegmentTransitGap)
  {
    // SAFETY: force the blade OFF first, unconditionally, before any dispatch or
    // early return below — nothing may cross this gap blade-on.
    setBladeEnabled(false);

    // If navigate_to_pose isn't ready yet, do NOT fall through to a blade-on
    // FollowPath. Hold with the blade off and retry the transit each tick
    // (bounded by kTransitServerWaitSec in onRunning); onRunning skips the swath
    // if the server never comes up.
    if (!nav_client_ || !nav_client_->action_server_is_ready())
    {
      if (!transit_pending_)
      {
        transit_pending_ = true;
        transit_wait_start_ = std::chrono::steady_clock::now();
        RCLCPP_WARN(ctx->node->get_logger(),
                    "FollowStrip: segment %zu/%zu starts %.2fm away but navigate_to_pose is "
                    "not ready — blade OFF, holding for the transit server (no blade-on crossing)",
                    swath_idx_ + 1,
                    swaths_.size(),
                    gap);
      }
      return true;
    }

    // sendFollowGoal re-enables the blade when the robot reaches the segment and
    // mowing resumes.
    transit_pending_ = false;
    Nav2Navigate::Goal nav_goal;
    nav_goal.pose = swaths_[swath_idx_].poses.front();
    nav_goal.pose.header.frame_id = "map";
    nav_goal.pose.header.stamp = ctx->node->get_clock()->now();
    nav_handle_.reset();
    nav_future_ = nav_client_->async_send_goal(nav_goal);
    transit_active_ = true;
    swath_goal_sent_ = true;
    RCLCPP_INFO(ctx->node->get_logger(),
                "FollowStrip: segment %zu/%zu starts %.2fm away — blade off, transit first",
                swath_idx_ + 1,
                swaths_.size(),
                gap);
    return true;
  }
  // Segment start is within kSegmentTransitGap (adjacent-swath close, ~one
  // op_width): FTC closes it blade-on, an accepted small re-mow. Never a large
  // cross-country crossing.
  transit_pending_ = false;
  return sendFollowGoal(ctx);
}

BT::NodeStatus FollowStrip::onRunning()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  // Advance to the next swath; finish (SUCCESS/FAILURE) when none remain.
  // A swath is SKIPPED on goal-reject/abort rather than failing the whole
  // area — robust coverage; gaps are reclaimed on the next pass (and, under
  // MPPI, in-controller avoidance makes aborts rare). The area only FAILS if
  // every swath was skipped (nothing got mowed).
  auto advance = [&]() -> BT::NodeStatus
  {
    ++swath_idx_;
    // Moving to a fresh unit: its progress starts at 0 and it is untrimmed (only
    // the one resumed unit carries a trim offset). Without this reset the stale
    // cursor from the previous unit would corrupt the next unit's progress and
    // the persisted resume index.
    path_progress_idx_ = 0;
    resume_start_idx_ = 0;
    // Skip any swaths already mowed in an earlier pass (resume).
    const auto& done = ctx->area_completed_swaths[area_idx_];
    while (swath_idx_ < swaths_.size() && done.count(swath_idx_) > 0)
    {
      ++swath_idx_;
    }
    // Refresh the GUI's live swath progress on every boundary (the swath that
    // just completed was inserted into area_completed_swaths before advance()
    // ran), so completed_swaths climbs during the pass rather than only at its
    // end. total_swaths stays == swaths_.size() throughout this pass.
    refreshSwathProgress(*ctx, area_idx_, swaths_.size());
    if (swath_idx_ < swaths_.size())
    {
      sendCurrentSwath(ctx);
      return BT::NodeStatus::RUNNING;
    }
    setBladeEnabled(false);
    // Coverage % is the MAX of the per-segment done fraction and the continuous
    // resume cursor's fraction (set on a mid-path abort/halt). On full success
    // done_pct=100 dominates; on a partial abort done_pct=0 but the resume cursor
    // carries the real progress — so GetNextUnmowedArea sees the area advancing
    // and does not abandon it (and the next pass resumes from the cursor).
    const float done_pct = swaths_.empty() ? 100.0f
                                           : 100.0f * static_cast<float>(done.size()) /
                                                 static_cast<float>(swaths_.size());
    float resume_pct = 0.0f;
    auto rit = ctx->area_resume_pose_index.find(area_idx_);
    if (rit != ctx->area_resume_pose_index.end() && total_path_poses_ > 0)
    {
      resume_pct = 100.0f * static_cast<float>(rit->second) / static_cast<float>(total_path_poses_);
    }
    ctx->coverage_percent = std::max(done_pct, resume_pct);
    // Mark the area complete once every swath is mowed (skipped swaths don't
    // count as done — they roll over to the next pass).
    if (done.size() >= swaths_.size())
    {
      ctx->completed_areas.insert(area_idx_);
      saveCoverageResumeState(*ctx);
    }
    if (swaths_skipped_ >= swaths_.size())
    {
      RCLCPP_WARN(ctx->node->get_logger(),
                  "FollowStrip: all %zu swaths skipped — area %u not mowable now",
                  swaths_.size(),
                  area_idx_);
      return BT::NodeStatus::FAILURE;
    }
    RCLCPP_INFO(ctx->node->get_logger(),
                "FollowStrip: area %u pass done (%zu/%zu swaths mowed, %zu skipped this pass)",
                area_idx_,
                done.size(),
                swaths_.size(),
                swaths_skipped_);
    return BT::NodeStatus::SUCCESS;
  };

  // Wait for blade spin-up, then dispatch the first segment.
  if (!goal_sent_)
  {
    auto elapsed = std::chrono::steady_clock::now() - blade_start_time_;
    if (elapsed < std::chrono::duration<double>(kBladeSpinupDelaySec))
      return BT::NodeStatus::RUNNING;
    goal_sent_ = true;
    sendCurrentSwath(ctx);
    return BT::NodeStatus::RUNNING;
  }

  // A required blade-off transit could not be dispatched because navigate_to_pose
  // wasn't ready. The blade is already OFF (sendCurrentSwath forced it). Retry the
  // dispatch each tick until the server is up; if it never comes within the wait
  // budget, SKIP this swath (it rolls to the next pass) rather than ever driving
  // to a >kSegmentTransitGap start blade-on. Placed before the transit/follow
  // handlers because neither is in flight while a dispatch is pending.
  if (transit_pending_)
  {
    if (std::chrono::steady_clock::now() - transit_wait_start_ >
        std::chrono::duration<double>(kTransitServerWaitSec))
    {
      RCLCPP_ERROR(ctx->node->get_logger(),
                   "FollowStrip: navigate_to_pose never became ready for the segment %zu/%zu "
                   "transit — skipping swath (blade stays OFF, not mowed this pass)",
                   swath_idx_ + 1,
                   swaths_.size());
      transit_pending_ = false;
      ++swaths_skipped_;
      return advance();
    }
    sendCurrentSwath(ctx);
    return BT::NodeStatus::RUNNING;
  }

  // Inter-segment transit in flight: when it completes, dispatch the segment
  // itself; if it fails, skip the segment (it stays un-mowed for a next pass).
  if (transit_active_)
  {
    if (!nav_handle_)
    {
      if (nav_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
        return BT::NodeStatus::RUNNING;
      nav_handle_ = nav_future_.get();
      if (!nav_handle_)
      {
        RCLCPP_WARN(ctx->node->get_logger(),
                    "FollowStrip: segment %zu transit goal rejected — skipping",
                    swath_idx_ + 1);
        transit_active_ = false;
        ++swaths_skipped_;
        return advance();
      }
      return BT::NodeStatus::RUNNING;
    }
    const auto nav_status = nav_handle_->get_status();
    if (nav_status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)
    {
      transit_active_ = false;
      nav_handle_.reset();
      sendFollowGoal(ctx);
      return BT::NodeStatus::RUNNING;
    }
    if (nav_status == action_msgs::msg::GoalStatus::STATUS_ABORTED ||
        nav_status == action_msgs::msg::GoalStatus::STATUS_CANCELED)
    {
      RCLCPP_WARN(ctx->node->get_logger(),
                  "FollowStrip: segment %zu transit failed — skipping",
                  swath_idx_ + 1);
      transit_active_ = false;
      nav_handle_.reset();
      ++swaths_skipped_;
      return advance();
    }
    return BT::NodeStatus::RUNNING;
  }

  if (!follow_handle_)
  {
    if (follow_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
      return BT::NodeStatus::RUNNING;
    follow_handle_ = follow_future_.get();
    if (!follow_handle_)
    {
      RCLCPP_WARN(ctx->node->get_logger(),
                  "FollowStrip: segment %zu goal rejected — skipping",
                  swath_idx_ + 1);
      ++swaths_skipped_;
      return advance();
    }
  }

  auto status = follow_handle_->get_status();

  // Track how far along the continuous path the robot has driven, every tick,
  // so an abort/halt can persist an accurate resume cursor.
  updateProgress(ctx);

  // Smooth live coverage percent for the GUI, refreshed on every following tick
  // (not only on abort/pass-end). Monotonic within the area; reset per area by
  // the onStart seed. This is the PRIMARY GUI %; the swath X/Y counters remain a
  // secondary readout.
  ctx->coverage_percent = livePercent();

  if (status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)
  {
    // Whole path done — clear the resume cursor so a later re-selection doesn't
    // trim from a stale mid-path index.
    ctx->area_resume_pose_index.erase(area_idx_);
    // Record this segment as mowed for the area (survives a resume re-plan).
    ctx->area_completed_swaths[area_idx_].insert(swath_idx_);
    saveCoverageResumeState(*ctx);
    RCLCPP_INFO(ctx->node->get_logger(),
                "FollowStrip: segment %zu/%zu completed (area %u)",
                swath_idx_ + 1,
                swaths_.size(),
                area_idx_);
    follow_handle_.reset();
    return advance();
  }

  if (status == action_msgs::msg::GoalStatus::STATUS_ABORTED ||
      status == action_msgs::msg::GoalStatus::STATUS_CANCELED)
  {
    // Progress WITHIN the current unit (resume_start_idx_ = trim offset,
    // path_progress_idx_ = furthest pose reached in the trimmed unit) as a
    // fraction of that unit's FULL length — so "near the end" is judged per unit,
    // not against the whole concatenation (which would never fire on an early
    // sub-path).
    const std::size_t unit_reached = resume_start_idx_ + path_progress_idx_;
    const std::size_t unit_full =
        resume_start_idx_ + (swath_idx_ < swaths_.size() ? swaths_[swath_idx_].poses.size() : 0);
    const double frac =
        unit_full > 0 ? static_cast<double>(unit_reached) / static_cast<double>(unit_full) : 0.0;
    // Reached the end of the unit: FTC parks ~max_goal_distance_error short of
    // the final pose, so the goal-checker can't fire and the progress_checker
    // aborts (err 105) at ~100 % tracked. The unit IS mowed — treat it as
    // COMPLETE (clear the resume cursor, record it done) instead of skipping it,
    // which previously discarded the near-100 % cursor and re-mowed from scratch.
    // See kPathCompleteFraction.
    if (frac >= kPathCompleteFraction)
    {
      RCLCPP_INFO(ctx->node->get_logger(),
                  "FollowStrip: segment %zu/%zu reached %.0f%% of path then aborted near the goal "
                  "(area %u) — treating as MOWED (FTC parks short of the final pose)",
                  swath_idx_ + 1,
                  swaths_.size(),
                  100.0 * frac,
                  area_idx_);
      ctx->area_resume_pose_index.erase(area_idx_);
      ctx->area_completed_swaths[area_idx_].insert(swath_idx_);
      saveCoverageResumeState(*ctx);
      follow_handle_.reset();
      return advance();
    }
    RCLCPP_WARN(ctx->node->get_logger(),
                "FollowStrip: unit %zu/%zu aborted/canceled at %.0f%% of the unit (area %u) — "
                "saving resume cursor",
                swath_idx_ + 1,
                swaths_.size(),
                100.0 * frac,
                area_idx_);
    // Persist where we got to so the next dispatch resumes here instead of
    // re-mowing from the start, and so the partial coverage_percent keeps
    // GetNextUnmowedArea from abandoning a still-progressing area.
    persistResumeCursor(ctx);
    follow_handle_.reset();
    ++swaths_skipped_;
    return advance();
  }

  return BT::NodeStatus::RUNNING;
}

void FollowStrip::onHalted()
{
  // Preempt (recharge, e-stop, command change) mid-path: capture how far we got
  // and persist the resume cursor so the next dispatch continues from here
  // rather than re-mowing the whole area from the start.
  if (follow_handle_ && total_path_poses_ > 0)
  {
    auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
    updateProgress(ctx);
    persistResumeCursor(ctx);
  }
  if (follow_handle_)
  {
    follow_client_->async_cancel_goal(follow_handle_);
  }
  follow_handle_.reset();
  if (nav_handle_ && nav_client_)
  {
    nav_client_->async_cancel_goal(nav_handle_);
  }
  nav_handle_.reset();
  transit_active_ = false;
  transit_pending_ = false;
  setBladeEnabled(false);
}

void FollowStrip::setBladeEnabled(bool enabled)
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  if (!blade_client_)
  {
    blade_client_ = ctx->node->create_client<mowgli_interfaces::srv::MowerControl>(
        "/hardware_bridge/mower_control");
  }
  if (!blade_client_->wait_for_service(std::chrono::milliseconds(200)))
    return;

  auto req = std::make_shared<mowgli_interfaces::srv::MowerControl::Request>();
  req->mow_enabled = enabled ? 1u : 0u;
  blade_client_->async_send_request(req);
}

// ===========================================================================
// TransitToStrip — navigate to strip start using Nav2
// ===========================================================================

BT::NodeStatus TransitToStrip::onStart()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  // Nothing to transit to if there is no coverage path (e.g. the area is
  // already fully mowed → PlanCoverageArea returned an empty path with a
  // stale transit goal). Skip cleanly; FollowStrip will handle the empty
  // path via its own AreaUnreachable fallthrough.
  if (ctx->current_strip_path.poses.empty())
  {
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_INFO(ctx->node->get_logger(),
              "TransitToStrip: goal frame='%s' pos=(%.2f, %.2f)",
              ctx->current_transit_goal.header.frame_id.c_str(),
              ctx->current_transit_goal.pose.position.x,
              ctx->current_transit_goal.pose.position.y);

  if (!nav_client_)
  {
    nav_client_ = rclcpp_action::create_client<Nav2Navigate>(ctx->node, "/navigate_to_pose");
  }
  if (!nav_client_->wait_for_action_server(std::chrono::seconds(5)))
  {
    RCLCPP_ERROR(ctx->node->get_logger(), "TransitToStrip: navigate_to_pose not available");
    return BT::NodeStatus::FAILURE;
  }

  Nav2Navigate::Goal goal;
  goal.pose = ctx->current_transit_goal;

  nav_handle_.reset();
  nav_future_ = nav_client_->async_send_goal(goal);

  RCLCPP_INFO(ctx->node->get_logger(),
              "TransitToStrip: navigating to (%.2f, %.2f)",
              goal.pose.pose.position.x,
              goal.pose.pose.position.y);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TransitToStrip::onRunning()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (!nav_handle_)
  {
    if (nav_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
      return BT::NodeStatus::RUNNING;
    nav_handle_ = nav_future_.get();
    if (!nav_handle_)
    {
      RCLCPP_WARN(ctx->node->get_logger(), "TransitToStrip: goal rejected");
      return BT::NodeStatus::FAILURE;
    }
  }

  auto status = nav_handle_->get_status();

  if (status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)
  {
    RCLCPP_INFO(ctx->node->get_logger(), "TransitToStrip: arrived at strip start");
    nav_handle_.reset();
    return BT::NodeStatus::SUCCESS;
  }

  if (status == action_msgs::msg::GoalStatus::STATUS_ABORTED ||
      status == action_msgs::msg::GoalStatus::STATUS_CANCELED)
  {
    RCLCPP_WARN(ctx->node->get_logger(), "TransitToStrip: navigation failed");
    nav_handle_.reset();
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void TransitToStrip::onHalted()
{
  if (nav_handle_)
  {
    nav_client_->async_cancel_goal(nav_handle_);
  }
  nav_handle_.reset();
}

// ===========================================================================
// DetourAroundObstacle — short side-step via global planner so the robot
// gets out from in front of an obstacle that aborted the strip.
// ===========================================================================

BT::NodeStatus DetourAroundObstacle::onStart()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  double forward_m = 0.8;
  double lateral_m = 0.6;
  getInput("forward_m", forward_m);
  getInput("lateral_m", lateral_m);

  // Read current pose from map → base_footprint. If TF isn't ready, bail —
  // the BT will fall through to SkipStrip and we don't risk sending a
  // stale-pose-based goal.
  geometry_msgs::msg::TransformStamped t_map_base;
  try
  {
    t_map_base = ctx->tf_buffer->lookupTransform("map",
                                                 "base_footprint",
                                                 tf2::TimePointZero,
                                                 tf2::durationFromSec(0.2));
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_WARN(ctx->node->get_logger(), "DetourAroundObstacle: TF lookup failed: %s", ex.what());
    return BT::NodeStatus::FAILURE;
  }

  // Yaw from quaternion: standard ZYX Euler extraction. Avoids pulling
  // in tf2_geometry_msgs just for tf2::getYaw().
  const auto& q = t_map_base.transform.rotation;
  const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  const double cy = std::cos(yaw);
  const double sy = std::sin(yaw);

  // Body-frame (forward, lateral) → map frame, added to current position.
  // Lateral positive = left (right-hand-rule with z-up).
  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.header.stamp = ctx->node->now();
  goal.pose.position.x = t_map_base.transform.translation.x + cy * forward_m - sy * lateral_m;
  goal.pose.position.y = t_map_base.transform.translation.y + sy * forward_m + cy * lateral_m;
  goal.pose.position.z = 0.0;

  // Keep the same heading. The global planner adjusts the path heading;
  // we just don't want to hand Nav2 a wildly different goal yaw.
  goal.pose.orientation = t_map_base.transform.rotation;

  RCLCPP_INFO(ctx->node->get_logger(),
              "DetourAroundObstacle: goal=(%.2f, %.2f) "
              "from (%.2f, %.2f), forward=%.2f lateral=%.2f",
              goal.pose.position.x,
              goal.pose.position.y,
              t_map_base.transform.translation.x,
              t_map_base.transform.translation.y,
              forward_m,
              lateral_m);

  if (!nav_client_)
  {
    nav_client_ = rclcpp_action::create_client<Nav2Navigate>(ctx->node, "/navigate_to_pose");
  }
  if (!nav_client_->wait_for_action_server(std::chrono::seconds(2)))
  {
    RCLCPP_ERROR(ctx->node->get_logger(), "DetourAroundObstacle: navigate_to_pose not available");
    return BT::NodeStatus::FAILURE;
  }

  Nav2Navigate::Goal nav_goal;
  nav_goal.pose = goal;

  nav_handle_.reset();
  nav_future_ = nav_client_->async_send_goal(nav_goal);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DetourAroundObstacle::onRunning()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (!nav_handle_)
  {
    if (nav_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
      return BT::NodeStatus::RUNNING;
    nav_handle_ = nav_future_.get();
    if (!nav_handle_)
    {
      RCLCPP_WARN(ctx->node->get_logger(), "DetourAroundObstacle: goal rejected");
      return BT::NodeStatus::FAILURE;
    }
  }

  const auto status = nav_handle_->get_status();
  if (status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)
  {
    RCLCPP_INFO(ctx->node->get_logger(), "DetourAroundObstacle: detour complete");
    nav_handle_.reset();
    return BT::NodeStatus::SUCCESS;
  }
  if (status == action_msgs::msg::GoalStatus::STATUS_ABORTED ||
      status == action_msgs::msg::GoalStatus::STATUS_CANCELED)
  {
    RCLCPP_WARN(ctx->node->get_logger(), "DetourAroundObstacle: detour navigation failed");
    nav_handle_.reset();
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

void DetourAroundObstacle::onHalted()
{
  if (nav_handle_)
  {
    nav_client_->async_cancel_goal(nav_handle_);
  }
  nav_handle_.reset();
}

// ===========================================================================
// GetNextUnmowedArea — iterate areas, find first with strips remaining
// ===========================================================================

BT::NodeStatus GetNextUnmowedArea::onStart()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  auto helper = ctx->helper_node;

  if (!client_)
  {
    client_ = helper->create_client<mowgli_interfaces::srv::GetMowingArea>(
        "/map_server_node/get_mowing_area");
  }

  if (!client_->service_is_ready())
  {
    RCLCPP_ERROR(ctx->node->get_logger(),
                 "GetNextUnmowedArea: get_mowing_area service not available");
    return BT::NodeStatus::FAILURE;
  }

  // Reset per-run state
  getInput<uint32_t>("max_areas", max_areas_);
  current_area_idx_ = 0;
  areas_queried_ = 0;
  areas_complete_ = 0;
  probe_retries_ = 0;
  // Clear any stale completion flag from a previous run so a transient failure
  // this run can never be read as "all areas complete". Unlocked: see the
  // context_mutex doc comment in bt_context.hpp — this field is BT-tick-
  // thread-only.
  ctx->coverage_all_complete = false;

  // Honor a one-shot user-selected target area (set by ~/start_in_area).
  // We start the iteration from the requested index AND clip max_areas_ to
  // (target + 1) so the BT mows just that area and exits MowingSequence,
  // instead of rolling over to the next area. The optional is consumed
  // here so subsequent COMMAND_START runs use the normal ordering.
  if (ctx->target_area_index.has_value())
  {
    const int target = *ctx->target_area_index;
    if (target >= 0)
    {
      current_area_idx_ = static_cast<uint32_t>(target);
      max_areas_ = current_area_idx_ + 1;
      // Explicit single-area re-mow: clear any stale completed/attempted flag
      // for THIS target so the skip loop below cannot advance past it. Without
      // this, re-selecting an area already mown this session (sets are only
      // cleared by EndSession) makes the skip loop overshoot max_areas_ →
      // "all areas completed" → FAILURE → the requested area never mows.
      // Guarded to the explicit-target path only, so normal COMMAND_START
      // iteration still honors completed/attempted skipping.
      ctx->completed_areas.erase(current_area_idx_);
      ctx->attempted_areas.erase(current_area_idx_);
      RCLCPP_INFO(ctx->node->get_logger(),
                  "GetNextUnmowedArea: targeted run — mowing only area %u (single-area mode)",
                  current_area_idx_);
    }
    ctx->target_area_index.reset();
  }

  // Skip any area that has already burned its attempt budget this
  // session (BTContext::kMaxAreaAttempts dispatches of PlanCoverageArea
  // + FollowStrip). An area is added to attempted_areas only after
  // either completing successfully (strips_remaining == 0) or
  // exhausting its budget — so a single boundary-recovery preemption
  // does NOT permanently disable the area. attempted_areas is cleared
  // by EndSession at session end.
  while (current_area_idx_ < max_areas_ && (ctx->attempted_areas.count(current_area_idx_) > 0 ||
                                            ctx->completed_areas.count(current_area_idx_) > 0))
  {
    RCLCPP_INFO(ctx->node->get_logger(),
                "GetNextUnmowedArea: area %u already %s this session, skipping",
                current_area_idx_,
                ctx->completed_areas.count(current_area_idx_) > 0 ? "completed" : "attempted");
    current_area_idx_++;
  }
  if (current_area_idx_ >= max_areas_)
  {
    RCLCPP_INFO(ctx->node->get_logger(),
                "GetNextUnmowedArea: all areas already completed/attempted this session");
    ctx->coverage_all_complete = true;  // genuine completion → MOWING_COMPLETE
    return BT::NodeStatus::FAILURE;
  }

  // Fire off the first async existence probe.
  auto request = std::make_shared<mowgli_interfaces::srv::GetMowingArea::Request>();
  request->index = current_area_idx_;
  pending_future_.emplace(client_->async_send_request(request));
  call_start_ = std::chrono::steady_clock::now();

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GetNextUnmowedArea::onRunning()
{
  // Check if current async call has completed
  if (pending_future_->future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
  {
    // Still waiting — check 2s timeout
    if (std::chrono::steady_clock::now() - call_start_ > std::chrono::seconds(2))
    {
      auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
      // A transient service blip must NOT abort the run, and must NEVER be read
      // as "all areas complete" (ctx->coverage_all_complete stays false). Re-probe
      // the same index a bounded number of times before giving up.
      if (probe_retries_ < kMaxProbeRetries)
      {
        ++probe_retries_;
        RCLCPP_WARN(ctx->node->get_logger(),
                    "GetNextUnmowedArea: get_mowing_area timed out for area %u after 2s — "
                    "re-probing (retry %u/%u)",
                    current_area_idx_,
                    probe_retries_,
                    kMaxProbeRetries);
        if (client_->service_is_ready())
        {
          auto request = std::make_shared<mowgli_interfaces::srv::GetMowingArea::Request>();
          request->index = current_area_idx_;
          pending_future_.emplace(client_->async_send_request(request));
        }
        call_start_ = std::chrono::steady_clock::now();
        return BT::NodeStatus::RUNNING;
      }
      RCLCPP_ERROR(ctx->node->get_logger(),
                   "GetNextUnmowedArea: get_mowing_area timed out for area %u after %u retries — "
                   "FAILURE (transient service error, NOT mowing-complete)",
                   current_area_idx_,
                   kMaxProbeRetries);
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  return processResponse();
}

// Advance current_area_idx_ past any already-completed/attempted areas and
// fire the next existence probe. Returns RUNNING (probe in flight) or FAILURE
// (no candidate area remains).
BT::NodeStatus GetNextUnmowedArea::advanceAndProbe()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  current_area_idx_++;
  while (current_area_idx_ < max_areas_ && (ctx->attempted_areas.count(current_area_idx_) > 0 ||
                                            ctx->completed_areas.count(current_area_idx_) > 0))
  {
    current_area_idx_++;
  }
  if (current_area_idx_ >= max_areas_)
  {
    RCLCPP_INFO(ctx->node->get_logger(),
                "GetNextUnmowedArea: all %u area(s) complete",
                areas_complete_);
    ctx->coverage_all_complete = true;  // genuine completion → MOWING_COMPLETE
    return BT::NodeStatus::FAILURE;
  }
  auto request = std::make_shared<mowgli_interfaces::srv::GetMowingArea::Request>();
  request->index = current_area_idx_;
  pending_future_.emplace(client_->async_send_request(request));
  call_start_ = std::chrono::steady_clock::now();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GetNextUnmowedArea::processResponse()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  auto response = pending_future_->future.get();
  // A probe completed (no timeout) — reset the transient-retry budget so each
  // stuck probe gets its own retries.
  probe_retries_ = 0;

  if (!response->success)
  {
    // Index past the last defined area — nothing more to check.
    if (areas_queried_ == 0)
    {
      // No areas defined at all — a CONFIG error, not a normal completion.
      // Leave coverage_all_complete=false so this routes to the failure dock,
      // not MOWING_COMPLETE.
      RCLCPP_WARN(ctx->node->get_logger(),
                  "GetNextUnmowedArea: no mowing areas defined in map_server "
                  "(get_mowing_area returned success=false at index %u). "
                  "Record an area via the GUI before starting mowing.",
                  current_area_idx_);
    }
    else
    {
      RCLCPP_INFO(ctx->node->get_logger(),
                  "GetNextUnmowedArea: all %u area(s) complete",
                  areas_complete_);
      ctx->coverage_all_complete = true;  // genuine completion → MOWING_COMPLETE
    }
    return BT::NodeStatus::FAILURE;
  }

  areas_queried_++;

  // Navigation-only areas are transit corridors, NOT mowing targets — they
  // carry is_navigation_area=true and must never be selected for coverage (the
  // blades would run inside a zone the operator marked nav-only). map_server
  // still returns these (success=true) because the obstacle tracker needs their
  // geometry, so the skip lives here on the selection side. Mark the area
  // attempted so subsequent probes don't re-evaluate it, then advance.
  if (response->area.is_navigation_area)
  {
    ctx->attempted_areas.insert(current_area_idx_);
    RCLCPP_INFO(ctx->node->get_logger(),
                "GetNextUnmowedArea: area %u is navigation-only ('%s') — skipping (not mowed)",
                current_area_idx_,
                response->area.name.c_str());
    return advanceAndProbe();
  }

  // Completion is now the swath-completion model: an area is done when
  // FollowStrip has mowed every swath F2C produced for it (recorded in
  // ctx->completed_areas). The onStart/advance skip-loops already exclude
  // completed_areas, so reaching here normally means "has remaining work" —
  // but re-check in case it completed between probes. Unlocked (see the
  // context_mutex doc comment in bt_context.hpp): this used to take
  // context_mutex here and release it before calling advanceAndProbe()
  // (itself locking), which is exactly the kind of nested-lock pattern that
  // deadlocks a non-recursive mutex the moment someone "simplifies" the two
  // scopes together — task #15 removed the lock instead of trying to keep
  // that fragile in-out-in sequencing correct.
  const bool already_complete = ctx->completed_areas.count(current_area_idx_) > 0;
  if (already_complete)
  {
    areas_complete_++;
    RCLCPP_INFO(ctx->node->get_logger(),
                "GetNextUnmowedArea: area %u already complete",
                current_area_idx_);
    return advanceAndProbe();
  }

  // Area has remaining work. Guard against an area that can never finish
  // (e.g. every swath aborts): count CONSECUTIVE dispatches that added no
  // newly-completed swath toward kMaxAreaAttempts. The high-water mark is
  // the size of ctx->area_completed_swaths[area] — if it grew since the last
  // dispatch, the previous FollowStrip pass mowed at least one more swath, so
  // reset the counter. This replaces the old cell-coverage-percent high-water.
  std::size_t done_swaths = 0;
  {
    auto it = ctx->area_completed_swaths.find(current_area_idx_);
    if (it != ctx->area_completed_swaths.end())
    {
      done_swaths = it->second.size();
    }
  }
  auto& n = ctx->area_attempt_count[current_area_idx_];
  auto last_it = ctx->area_last_coverage.find(current_area_idx_);
  const bool made_progress = (last_it == ctx->area_last_coverage.end()) ||
                             (static_cast<float>(done_swaths) > last_it->second + 0.5f);
  if (made_progress)
  {
    ctx->area_last_coverage[current_area_idx_] = static_cast<float>(done_swaths);
    n = 0;
  }
  n++;
  if (n >= BTContext::kMaxAreaAttempts)
  {
    ctx->attempted_areas.insert(current_area_idx_);
    RCLCPP_WARN(ctx->node->get_logger(),
                "GetNextUnmowedArea: area %u hit max attempts (%u), giving up with "
                "%zu swath(s) completed",
                current_area_idx_,
                n,
                done_swaths);
    return advanceAndProbe();
  }

  setOutput("area_index", current_area_idx_);
  ctx->current_area = static_cast<int>(current_area_idx_);
  RCLCPP_INFO(ctx->node->get_logger(),
              "GetNextUnmowedArea: area %u selected (%zu swath(s) done so far) "
              "— dispatch attempt %u/%u",
              current_area_idx_,
              done_swaths,
              n,
              BTContext::kMaxAreaAttempts);
  return BT::NodeStatus::SUCCESS;
}

void GetNextUnmowedArea::onHalted()
{
  // Nothing to cancel — service calls complete on their own.
  // State will be reset in onStart() on next invocation.
}

// ===========================================================================
// PlanCoverageArea — plan_coverage (mowgli_coverage explicit segments)
// ===========================================================================

PlanCoverageArea::PlanCoverage::Goal PlanCoverageArea::buildGoal(
    const mowgli_interfaces::msg::MapArea& area) const
{
  PlanCoverage::Goal goal;
  goal.outer_boundary = area.area;
  for (const auto& hole : area.obstacles)
  {
    if (hole.points.size() >= 3)
    {
      goal.obstacles.push_back(hole);
    }
  }
  // < 0 = auto (the server picks the swath-count-minimising angle). The
  // coverage geometry (operation_width, headland, insets) lives in the
  // coverage server's parameters, injected at launch from mowgli_robot.yaml.
  goal.mow_angle_deg = -1.0;
  return goal;
}

namespace
{

/// Compute axis-aligned bbox + signed area + perimeter of a ROS Polygon.
/// Used to log diagnostics on each ComputeCoveragePath piece so we can
/// see whether F2C choked on a degenerate shape (sliver, self-intersect,
/// zero area, etc.).
struct PolygonStats
{
  double min_x{0}, min_y{0}, max_x{0}, max_y{0};
  double signed_area{0};
  double perimeter{0};
};

PolygonStats polygon_stats(const geometry_msgs::msg::Polygon& poly)
{
  PolygonStats s;
  const size_t n = poly.points.size();
  if (n < 3)
  {
    return s;
  }
  s.min_x = s.max_x = poly.points[0].x;
  s.min_y = s.max_y = poly.points[0].y;
  for (size_t i = 0; i < n; ++i)
  {
    const auto& a = poly.points[i];
    const auto& b = poly.points[(i + 1) % n];
    s.min_x = std::min(s.min_x, static_cast<double>(a.x));
    s.max_x = std::max(s.max_x, static_cast<double>(a.x));
    s.min_y = std::min(s.min_y, static_cast<double>(a.y));
    s.max_y = std::max(s.max_y, static_cast<double>(a.y));
    s.signed_area += static_cast<double>(a.x) * b.y - static_cast<double>(b.x) * a.y;
    s.perimeter += std::hypot(static_cast<double>(b.x) - a.x, static_cast<double>(b.y) - a.y);
  }
  s.signed_area *= 0.5;
  return s;
}

}  // namespace

BT::NodeStatus PlanCoverageArea::onStart()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  // Reset per-tick state.
  area_ = mowgli_interfaces::msg::MapArea{};
  goal_handle_.reset();
  phase_start_ = std::chrono::steady_clock::now();

  uint32_t area_index = 0;
  getInput<uint32_t>("area_index", area_index);

  if (!srv_client_)
  {
    auto helper = ctx->helper_node;
    srv_client_ = helper->create_client<mowgli_interfaces::srv::GetMowingArea>(
        "/map_server_node/get_mowing_area");
  }
  if (!srv_client_->service_is_ready())
  {
    RCLCPP_ERROR(ctx->node->get_logger(), "PlanCoverageArea: get_mowing_area service not ready");
    return BT::NodeStatus::FAILURE;
  }

  if (!action_client_)
  {
    action_client_ = rclcpp_action::create_client<PlanCoverage>(ctx->node, "/plan_coverage");
  }
  if (!action_client_->wait_for_action_server(std::chrono::seconds(2)))
  {
    RCLCPP_ERROR(ctx->node->get_logger(), "PlanCoverageArea: plan_coverage action not available");
    return BT::NodeStatus::FAILURE;
  }

  auto request = std::make_shared<mowgli_interfaces::srv::GetMowingArea::Request>();
  request->index = area_index;
  srv_future_.emplace(srv_client_->async_send_request(request));
  phase_ = Phase::QueryRemaining;

  RCLCPP_INFO(ctx->node->get_logger(),
              "PlanCoverageArea: querying full mowing area %u",
              area_index);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus PlanCoverageArea::onRunning()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  // ── Phase: wait for ~/get_mowing_area response (full area + obstacles) ───
  if (phase_ == Phase::QueryRemaining)
  {
    if (!srv_future_)
    {
      return BT::NodeStatus::FAILURE;
    }
    if (srv_future_->future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
    {
      if (std::chrono::steady_clock::now() - phase_start_ > std::chrono::seconds(3))
      {
        RCLCPP_ERROR(ctx->node->get_logger(), "PlanCoverageArea: get_mowing_area timed out (3s)");
        srv_future_.reset();
        return BT::NodeStatus::FAILURE;
      }
      return BT::NodeStatus::RUNNING;
    }

    auto resp = srv_future_->future.get();
    srv_future_.reset();
    if (!resp->success)
    {
      RCLCPP_ERROR(ctx->node->get_logger(),
                   "PlanCoverageArea: get_mowing_area failed for the requested area");
      return BT::NodeStatus::FAILURE;
    }
    // Defensive: never plan coverage for a navigation-only zone (the blades
    // would run inside a transit corridor). GetNextUnmowedArea already skips
    // these on the selection side; this guards a directly-selected nav index.
    if (resp->area.is_navigation_area)
    {
      RCLCPP_WARN(ctx->node->get_logger(),
                  "PlanCoverageArea: area '%s' is navigation-only — refusing to plan coverage",
                  resp->area.name.c_str());
      return BT::NodeStatus::FAILURE;
    }
    // Plan the FULL area (outer ring + obstacle holes) ONCE. Resume is
    // segment-based: the plan is deterministic for a fixed polygon + params,
    // and FollowStrip skips segment indices already in
    // ctx->area_completed_swaths.
    area_ = resp->area;
    RCLCPP_INFO(ctx->node->get_logger(),
                "PlanCoverageArea: planning FULL area (%zu boundary pts, %zu obstacles)",
                area_.area.points.size(),
                area_.obstacles.size());
    phase_ = Phase::Dispatch;
  }

  // ── Phase: dispatch plan_coverage for the area ────────────────────────────
  if (phase_ == Phase::Dispatch)
  {
    // Pre-dispatch geometry log: any planner failure will be diagnosable from
    // these numbers (sliver / self-intersection / hole inversion / ...).
    const auto outer_stats = polygon_stats(area_.area);
    const double bbox_w = outer_stats.max_x - outer_stats.min_x;
    const double bbox_h = outer_stats.max_y - outer_stats.min_y;
    RCLCPP_INFO(ctx->node->get_logger(),
                "PlanCoverageArea: dispatching — outer %zu pts, bbox=%.2fx%.2fm, "
                "signed_area=%.2fm² (CCW=%c), perimeter=%.2fm, %zu holes",
                area_.area.points.size(),
                bbox_w,
                bbox_h,
                outer_stats.signed_area,
                outer_stats.signed_area > 0 ? 'Y' : 'N',
                outer_stats.perimeter,
                area_.obstacles.size());
    for (size_t h = 0; h < area_.obstacles.size(); ++h)
    {
      const auto hs = polygon_stats(area_.obstacles[h]);
      RCLCPP_INFO(ctx->node->get_logger(),
                  "PlanCoverageArea:   hole %zu — %zu pts, bbox=%.2fx%.2fm, "
                  "signed_area=%.3fm² (%s)",
                  h,
                  area_.obstacles[h].points.size(),
                  hs.max_x - hs.min_x,
                  hs.max_y - hs.min_y,
                  hs.signed_area,
                  hs.signed_area < 0 ? "CW=hole-correct" : "CCW=hole-flipped!");
    }

    auto goal = buildGoal(area_);
    goal_handle_.reset();
    goal_future_ = action_client_->async_send_goal(goal);
    phase_ = Phase::WaitingForGoal;
    phase_start_ = std::chrono::steady_clock::now();
  }

  // ── Phase: wait for goal handle ──────────────────────────────────────────
  if (phase_ == Phase::WaitingForGoal)
  {
    if (goal_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
    {
      if (std::chrono::steady_clock::now() - phase_start_ > std::chrono::seconds(3))
      {
        RCLCPP_ERROR(ctx->node->get_logger(), "PlanCoverageArea: goal handshake timeout");
        return BT::NodeStatus::FAILURE;
      }
      return BT::NodeStatus::RUNNING;
    }
    goal_handle_ = goal_future_.get();
    if (!goal_handle_)
    {
      RCLCPP_ERROR(ctx->node->get_logger(), "PlanCoverageArea: goal rejected");
      return BT::NodeStatus::FAILURE;
    }
    result_future_ = action_client_->async_get_result(goal_handle_);
    phase_ = Phase::WaitingForResult;
    phase_start_ = std::chrono::steady_clock::now();
  }

  // ── Phase: wait for the segments result ──────────────────────────────────
  if (phase_ == Phase::WaitingForResult)
  {
    if (result_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
    {
      // The simple planner (no OR-Tools, no decomposition) finishes in well
      // under a second even on a large field; 12 s is a generous backstop.
      if (std::chrono::steady_clock::now() - phase_start_ > std::chrono::seconds(60))
      {
        RCLCPP_ERROR(ctx->node->get_logger(), "PlanCoverageArea: result timeout (60s)");
        return BT::NodeStatus::FAILURE;
      }
      return BT::NodeStatus::RUNNING;
    }
    auto wrapped = result_future_.get();
    // drivable_subpaths is what FollowStrip actually drives (see the class
    // comment above and FollowStrip::onStart) — segments is GUI/resume
    // bookkeeping only (task #16: PlanCoverage.action's segments field is
    // NOT what execution consumes, and never has been since the FTC/
    // continuous-subpath rewrite; failing here on segments.empty() while
    // drivable_subpaths is genuinely populated would reject a valid plan).
    // Guard the specific entry's pose count too — buildContinuousSubPaths
    // could in principle hand back a degenerate single-pose sub-path,
    // and FollowStrip's own "sp.poses.size() >= 2" filter documents that
    // this is a real possibility, not a hypothetical one.
    const bool has_drivable_path = !wrapped.result->drivable_subpaths.empty() &&
                                   wrapped.result->drivable_subpaths.front().poses.size() >= 2;
    if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED || !wrapped.result->success ||
        !has_drivable_path)
    {
      RCLCPP_WARN(ctx->node->get_logger(),
                  "PlanCoverageArea: plan_coverage failed (code=%d, drivable_subpaths=%zu): %s",
                  static_cast<int>(wrapped.code),
                  wrapped.result ? wrapped.result->drivable_subpaths.size() : 0u,
                  wrapped.result ? wrapped.result->message.c_str() : "");
      return BT::NodeStatus::FAILURE;
    }

    // Store the EXPLICIT segments (GUI/resume bookkeeping), the concatenated
    // path (GUI / empty-checks), and the hole-free drivable SUB-PATHS that
    // FollowStrip actually follows (issue #333 — one per hole-free field).
    ctx->current_strip_segments = wrapped.result->segments;
    ctx->current_strip_path = wrapped.result->full_path;
    ctx->current_strip_subpaths = wrapped.result->drivable_subpaths;

    // Publish the full plan for the GUI/Foxglove (latched). The per-segment
    // FollowCoveragePath/global_plan (goal checker) is a separate topic.
    if (!full_plan_pub_)
    {
      full_plan_pub_ =
          ctx->node->create_publisher<nav_msgs::msg::Path>("/coverage/full_plan",
                                                           rclcpp::QoS(1).transient_local());
    }
    full_plan_pub_->publish(ctx->current_strip_path);

    // Transit goal = the start of the sub-path FollowStrip will actually
    // drive first (NOT segments.front() — segments is bookkeeping and, per
    // the guard above, could theoretically diverge from drivable_subpaths).
    // TransitToStrip (Nav2 Smac, obstacle/boundary-aware) drives the robot
    // there blade-off; FollowStrip then mows from exactly that pose.
    ctx->current_transit_goal = wrapped.result->drivable_subpaths.front().poses.front();
    ctx->current_transit_goal.header = wrapped.result->full_path.header;

    RCLCPP_INFO(ctx->node->get_logger(),
                "PlanCoverageArea: %u ring(s) + %u swath(s) = %zu segments "
                "(%zu drivable sub-path(s)), %.1fm total (planned in %.0fms)",
                wrapped.result->ring_count,
                wrapped.result->swath_count,
                wrapped.result->segments.size(),
                wrapped.result->drivable_subpaths.size(),
                wrapped.result->total_distance,
                1e3 * wrapped.result->planning_time_s);
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

void PlanCoverageArea::onHalted()
{
  if (goal_handle_ && action_client_)
  {
    action_client_->async_cancel_goal(goal_handle_);
  }
  goal_handle_.reset();
  srv_future_.reset();
}

}  // namespace mowgli_behavior

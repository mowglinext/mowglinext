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

#include "mowgli_behavior/escape_nodes.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>

namespace mowgli_behavior
{

namespace
{

/// Seconds between two steady_clock instants, or 0 when `then` was never set.
double AgeSeconds(const std::chrono::steady_clock::time_point& now,
                  const std::chrono::steady_clock::time_point& then)
{
  if (then.time_since_epoch().count() == 0)
  {
    return 0.0;
  }
  return std::chrono::duration<double>(now - then).count();
}

}  // namespace

// ---------------------------------------------------------------------------
// Command plumbing
// ---------------------------------------------------------------------------

void EscapeStartBlocked::publishForward(const std::shared_ptr<BTContext>& ctx, double vx)
{
  geometry_msgs::msg::TwistStamped cmd{};
  cmd.header.stamp = ctx->node->now();
  cmd.header.frame_id = "base_footprint";
  cmd.twist.linear.x = vx;
  // Straight-line nudge only. A turn while standing on a lethal cell sweeps the
  // chassis through cells we have even less information about.
  cmd.twist.angular.z = 0.0;
  pub_->publish(cmd);

  // The escape's own commands must never become "the last motion" — see
  // BTContext::last_motion_suppress_until.
  ctx->last_motion_suppress_until =
      std::chrono::steady_clock::now() +
      std::chrono::milliseconds(static_cast<int64_t>(kSignalHoldoffSec * 1000.0));
}

void EscapeStartBlocked::finish(const std::shared_ptr<BTContext>& ctx, const char* reason)
{
  if (running_)
  {
    // One explicit zero so the navigation lane does not sit on the last non-zero
    // command for its twist_mux timeout after we are done.
    publishForward(ctx, 0.0);
    RCLCPP_WARN(ctx->node->get_logger(),
                "EscapeStartBlocked: %s after %.2f m commanded over %.2f s (budget %.2f m / "
                "%.1f s). NOT field-verified — watch the robot",
                reason,
                state_.travelled,
                state_.elapsed,
                cfg_.distance,
                cfg_.timeout_s);
  }
  running_ = false;
  direction_ = EscapeDirection::kUnknown;
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

BT::NodeStatus EscapeStartBlocked::onStart()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  const auto now = std::chrono::steady_clock::now();

  state_ = StartBlockedEscapeState{};
  direction_ = EscapeDirection::kUnknown;
  running_ = false;

  // SanitizeEscapeCfg is already applied when the parameters are loaded; re-run
  // it so a hand-poked context (tests, future callers) still cannot exceed the
  // compiled ceilings.
  cfg_ = SanitizeEscapeCfg(ctx->start_blocked_escape_cfg);

  // ---- Gate: the arming token from IsCoverageStartBlocked (#495) -----------
  // Consumed unconditionally, so one blocked pass can produce at most one
  // escape attempt no matter what the rest of the branch does.
  const bool armed =
      ctx->start_blocked_escape_armed && AgeSeconds(now, ctx->start_blocked_escape_armed_time) <=
                                             BTContext::kStartBlockedEscapeArmMaxAgeSec;
  ctx->start_blocked_escape_armed = false;

  // ---- Blade: verified off, not merely requested off -----------------------
  double blade_age = 0.0;
  bool blade_fresh = false;
  bool blade_off = false;
  {
    std::lock_guard<std::mutex> lock(ctx->context_mutex);
    blade_age = AgeSeconds(now, ctx->last_status_time);
    blade_fresh =
        ctx->last_status_time.time_since_epoch().count() != 0 && blade_age <= kBladeStateMaxAgeSec;
    // Both signals must agree: mow_enabled is the bridge's commanded latch,
    // mower_esc_status is the blade controller's own activity report.
    blade_off = !ctx->latest_status.mow_enabled && ctx->latest_status.mower_esc_status == 0u;
  }

  // ---- Direction: opposite the last commanded motion ------------------------
  LastMotionSignal signal;
  {
    std::lock_guard<std::mutex> lock(ctx->context_mutex);
    signal.valid = ctx->last_motion_valid;
    signal.vx = ctx->last_motion_cmd_vx;
    signal.age_s = AgeSeconds(now, ctx->last_motion_time);
  }

  const EscapePreconditions pre{armed, blade_fresh, blade_off};
  const EscapeVerdict verdict = EscapeDecide(cfg_, pre, signal);

  if (!EscapeMoves(verdict))
  {
    // SUCCESS, not FAILURE: the rest of the #495 recovery branch (clear
    // costmaps, wait, retry, eventually retire + dock) must run unchanged.
    RCLCPP_WARN(ctx->node->get_logger(),
                "EscapeStartBlocked: standing down (%s) — commanding NO motion. "
                "armed=%d blade_off=%d blade_age=%.1fs dir_signal=%s vx=%.3f age=%.1fs. "
                "Falling through to the non-motion recovery (clear costmaps, retry)",
                EscapeVerdictName(verdict),
                static_cast<int>(armed),
                static_cast<int>(blade_fresh && blade_off),
                blade_age,
                signal.valid ? "yes" : "never-seen",
                signal.vx,
                signal.age_s);
    return BT::NodeStatus::SUCCESS;
  }

  if (!pub_)
  {
    // collision_monitor's cmd_vel_in_topic. See the class comment for why the
    // escape deliberately enters the SAME filter chain as every other
    // autonomous motion instead of writing closer to the wire.
    pub_ = ctx->node->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel_nav", 10);
  }

  direction_ = EscapeVerdictDirection(verdict);
  last_tick_ = now;
  running_ = true;

  RCLCPP_WARN(ctx->node->get_logger(),
              "EscapeStartBlocked: START_OCCUPIED from our own pose — nudging %s (opposite the "
              "last commanded motion vx=%.3f m/s, %.1fs ago) at %.2f m/s, bounded to %.2f m / "
              "%.1f s. Blade verified off; command routes through collision_monitor and the "
              "LOWEST twist_mux lane, so every safety layer still overrides it",
              direction_ == EscapeDirection::kForward ? "FORWARD" : "REVERSE",
              signal.vx,
              signal.age_s,
              cfg_.speed,
              cfg_.distance,
              cfg_.timeout_s);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus EscapeStartBlocked::onRunning()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  const auto now = std::chrono::steady_clock::now();

  // Re-verify the blade EVERY tick. A blade that turns on, or a status stream
  // that dies, ends the manoeuvre immediately.
  bool blade_ok = false;
  {
    std::lock_guard<std::mutex> lock(ctx->context_mutex);
    const double age = AgeSeconds(now, ctx->last_status_time);
    blade_ok = ctx->last_status_time.time_since_epoch().count() != 0 &&
               age <= kBladeStateMaxAgeSec && !ctx->latest_status.mow_enabled &&
               ctx->latest_status.mower_esc_status == 0u;
  }
  if (!blade_ok)
  {
    finish(ctx, "ABORTED — blade no longer verified off");
    return BT::NodeStatus::SUCCESS;
  }

  // Clamp dt so a scheduling hiccup (or a clock that went backwards) cannot
  // charge or refund a large slice of the distance budget in one tick.
  const double dt = std::clamp(AgeSeconds(now, last_tick_), 0.0, kMaxTickDtSec);
  last_tick_ = now;

  const double vx = EscapeStep(cfg_, state_, direction_, dt);
  if (vx == 0.0 || EscapeDone(cfg_, state_))
  {
    // Either bound is spent — stop here rather than issue one more command.
    finish(ctx,
           state_.travelled >= cfg_.distance ? "reached the DISTANCE bound"
                                             : "reached the TIME bound");
    return BT::NodeStatus::SUCCESS;
  }

  publishForward(ctx, vx);
  return BT::NodeStatus::RUNNING;
}

void EscapeStartBlocked::onHalted()
{
  // A halt (guard fired, emergency, session ended) must leave the wire at zero
  // rather than let the navigation lane coast on the last escape command.
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  finish(ctx, "HALTED by the tree");
}

}  // namespace mowgli_behavior

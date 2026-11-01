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
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>

#include "mowgli_interfaces/motion_authorization.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

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

EscapeStartBlocked::EscapeStartBlocked(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config)
{
  auto ctx = config.blackboard->get<std::shared_ptr<BTContext>>("context");
  const auto storage = geometry_;
  geometry_sub_ = ctx->node->create_subscription<visualization_msgs::msg::MarkerArray>(
      "/map_server_node/transit_geometry",
      rclcpp::QoS(1).transient_local().reliable(),
      [storage](visualization_msgs::msg::MarkerArray::ConstSharedPtr msg)
      {
        const auto next = mowgli_interfaces::motion::Snapshot::parse(*msg);
        std::lock_guard<std::mutex> lock(storage->mutex);
        if (!storage->snapshot || next->valid != storage->snapshot->valid ||
            next->identity != storage->snapshot->identity)
        {
          ++storage->generation;
        }
        storage->snapshot = next;
      });
}

bool EscapeStartBlocked::readPose(const std::shared_ptr<BTContext>& ctx,
                                  mowgli_interfaces::motion::Pose& pose)
{
  if (!ctx->tf_buffer)
  {
    return false;
  }
  try
  {
    const auto transform =
        ctx->tf_buffer->lookupTransform("map", "base_footprint", tf2::TimePointZero);
    const double age = (ctx->node->now() - rclcpp::Time(transform.header.stamp,
                                                        ctx->node->get_clock()->get_clock_type()))
                           .seconds();
    // fusion_graph deliberately leads live TF by 0.1 s in simulation. Bound
    // that lead as well as old transforms; static/zero-stamp pose is refused.
    if (rclcpp::Time(transform.header.stamp).nanoseconds() == 0 || age > 0.3 || age < -0.15)
    {
      return false;
    }
    const auto& q = transform.transform.rotation;
    const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (!std::isfinite(norm) || std::abs(norm - 1.0) > 0.01)
    {
      return false;
    }
    pose = {transform.transform.translation.x, transform.transform.translation.y, tf2::getYaw(q)};
    return std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.yaw);
  }
  catch (const tf2::TransformException&)
  {
    return false;
  }
}

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
  authorized_identity_.clear();
  authorized_generation_ = 0;
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
  measured_travel_ = 0.0;
  authorized_identity_.clear();

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
  footprint_.clear();
  if (ctx->node->has_parameter("motion_footprint"))
  {
    const auto parameter = ctx->node->get_parameter("motion_footprint");
    const auto values = parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY
                            ? parameter.as_double_array()
                            : std::vector<double>{};
    if (values.size() >= 6 && values.size() <= 64 && values.size() % 2 == 0)
    {
      for (std::size_t i = 0; i < values.size(); i += 2)
      {
        footprint_.push_back({values[i], values[i + 1]});
      }
    }
  }
  std::lock_guard<std::mutex> geometry_lock(geometry_->mutex);
  const auto snapshot = geometry_->snapshot;
  const double vx = direction_ == EscapeDirection::kForward ? cfg_.speed : -cfg_.speed;
  if (!snapshot || !readPose(ctx, start_pose_) ||
      !snapshot->permits(start_pose_, footprint_, vx > 0 ? 1.0 : -1.0, 0.0, cfg_.distance, false))
  {
    direction_ = EscapeDirection::kUnknown;
    RCLCPP_WARN(ctx->node->get_logger(),
                "EscapeStartBlocked: no valid geometry, chassis, pose or complete escape sweep; "
                "falling through to non-motion recovery");
    return BT::NodeStatus::SUCCESS;
  }
  authorized_identity_ = snapshot->identity;
  authorized_generation_ = geometry_->generation;
  last_pose_ = start_pose_;
  started_ = now;
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

  if (!running_)
  {
    return BT::NodeStatus::SUCCESS;
  }

  // Bind the bounded operation to exactly the geometry admitted onStart. A
  // revocation followed by restoration of identical rings still ends this
  // action; a new armed attempt must authorize again.
  std::lock_guard<std::mutex> geometry_lock(geometry_->mutex);
  const auto snapshot = geometry_->snapshot;
  mowgli_interfaces::motion::Pose pose{};
  if (!snapshot || !snapshot->valid || snapshot->identity != authorized_identity_ ||
      geometry_->generation != authorized_generation_ || !readPose(ctx, pose))
  {
    finish(ctx, "ABORTED - geometry revoked/changed or pose unavailable");
    return BT::NodeStatus::SUCCESS;
  }
  const double dx = pose.x - start_pose_.x, dy = pose.y - start_pose_.y;
  const double sign = direction_ == EscapeDirection::kForward ? 1.0 : -1.0;
  const double along = sign * (dx * std::cos(start_pose_.yaw) + dy * std::sin(start_pose_.yaw));
  const double cross = -dx * std::sin(start_pose_.yaw) + dy * std::cos(start_pose_.yaw);
  const double yaw_delta = std::remainder(pose.yaw - start_pose_.yaw, 2.0 * M_PI);
  measured_travel_ += std::hypot(pose.x - last_pose_.x, pose.y - last_pose_.y);
  last_pose_ = pose;
  const double elapsed = AgeSeconds(now, started_);
  const double tick_dt = AgeSeconds(now, last_tick_);
  const double remaining = std::max(0.0, cfg_.distance - std::max(along, state_.travelled));
  if (elapsed >= cfg_.timeout_s || tick_dt > kMaxTickDtSec || std::abs(cross) > 0.05 ||
      std::abs(yaw_delta) > 0.15 || along < -0.05 || along >= cfg_.distance ||
      measured_travel_ >= cfg_.distance ||
      !snapshot->permits(pose, footprint_, sign, 0.0, remaining, false))
  {
    finish(ctx, "ABORTED - bounded escape envelope or budget exhausted");
    return BT::NodeStatus::SUCCESS;
  }

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
  const double dt = std::clamp(tick_dt, 0.0, kMaxTickDtSec);
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

  // The last issued twist can outlive the next BT tick. Certify that held
  // command as well as the intended complete escape, and taper near the bound.
  // EscapeStep still charges the full configured speed, conservatively ending
  // a tapering nudge before either its commanded or measured distance can grow
  // beyond the original budget. Updating downstream timeouts requires updating
  // this envelope too; final mux/hardware authorization is a separate contract.
  const double command_remaining =
      std::max(0.0, cfg_.distance - std::max({along, measured_travel_, state_.travelled}));
  const double command_vx =
      sign * std::min(cfg_.speed, command_remaining / kCommandHoldEnvelopeSec);
  if (command_vx == 0.0 ||
      !snapshot->permits(pose, footprint_, command_vx, 0.0, kCommandHoldEnvelopeSec, false))
  {
    finish(ctx, "ABORTED - held command has no authorized sweep");
    return BT::NodeStatus::SUCCESS;
  }
  publishForward(ctx, command_vx);
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

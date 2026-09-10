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

#include <chrono>
#include <memory>
#include <string>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "mowgli_behavior/bt_context.hpp"
#include "mowgli_behavior/start_blocked_escape.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mowgli_behavior
{

// ---------------------------------------------------------------------------
// EscapeStartBlocked
// ---------------------------------------------------------------------------

/// Bounded open-loop nudge off a start pose the Nav2 planner refuses to plan
/// from (issue #487). The ONLY new physical motion in the #487 workstream.
///
/// ── What it does ───────────────────────────────────────────────────────────
/// Drives OPPOSITE the last motion the robot was actually commanded to make,
/// for at most `start_blocked_escape_distance` metres and at most
/// `start_blocked_escape_timeout_s` seconds, at
/// `start_blocked_escape_speed` m/s. Both bounds are enforced; whichever is hit
/// first ends the manoeuvre. The direction reasoning, the stand-downs, and the
/// compiled ceilings all live in start_blocked_escape.hpp, which is pure and
/// unit-tested (test_start_blocked_escape.cpp).
///
/// ── Where the command goes, and why ────────────────────────────────────────
/// It publishes TwistStamped on **/cmd_vel_nav**, which is
/// collision_monitor's `cmd_vel_in_topic`. So the escape travels the SAME path
/// as every other autonomous motion on this robot:
///
///     EscapeStartBlocked → /cmd_vel_nav → collision_monitor
///                        → /cmd_vel_monitored → twist_mux (lane "navigation",
///                          priority 10, the LOWEST) → /cmd_vel
///                        → hardware_bridge → STM32
///
/// That placement is deliberate and is the safety argument for the whole node:
///   * collision_monitor still applies — a predicted contact zeroes the escape
///     exactly as it zeroes a controller command;
///   * twist_mux is not bypassed, and the escape sits on the LOWEST-priority
///     lane, so teleop (20), tuning (30) and the emergency lane (100) all
///     override it instantly;
///   * the STM32 remains the sole emergency-stop authority, untouched;
///   * if the Nav2 lifecycle group is paused (idle_nav2_suspend),
///     collision_monitor is inactive and the escape simply never reaches the
///     wheels — fail-safe by construction.
///
/// It is emphatically NOT in hardware_bridge_node, where the dig escape
/// (Invariant 16) lives. That one belongs there because it must cover EVERY
/// motion lane and must be un-overridable by cmd_vel. This one is the opposite
/// on both counts: it is a coverage-BT recovery gated on a BT-only signal, and
/// it MUST remain overridable by the safety layers above it. Putting it in the
/// bridge would add a second motion SOURCE to the layer whose job is to only
/// ever REDUCE motion.
///
/// ── When it may fire ───────────────────────────────────────────────────────
/// Only on a confirmed START_OCCUPIED-with-zero-progress pass. The gate is
/// ctx->start_blocked_escape_armed, which IsCoverageStartBlocked (issue #495)
/// is the sole writer of. The token is CONSUMED here whatever the outcome, and
/// a token older than BTContext::kStartBlockedEscapeArmMaxAgeSec is refused.
///
/// ── Stand-downs ────────────────────────────────────────────────────────────
/// Returns SUCCESS having commanded NOTHING when the escape is disabled, the
/// arming token is missing or stale, the blade is not VERIFIED off, or the
/// direction signal is unknown / stale / below the deadband. SUCCESS rather
/// than FAILURE on purpose: the rest of the #495 recovery branch (clear
/// costmaps, wait, retry, and eventually retire the area and dock) must run
/// unchanged, so every stand-down degrades to exactly the pre-escape
/// behaviour.
///
/// The blade check is re-run on EVERY tick, not just at entry: if the blade
/// state goes stale or reads enabled mid-manoeuvre, the escape stops
/// immediately and publishes zero.
class EscapeStartBlocked : public BT::StatefulActionNode
{
public:
  EscapeStartBlocked(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  /// Max age of a /hardware_bridge/status message that still counts as
  /// describing the blade NOW [s]. The bridge republishes on every STM32 status
  /// packet (several hertz), so 2 s is many messages of margin while still
  /// refusing a stream that has actually died. A compiled floor rather than a
  /// parameter: this is a safety precondition, not a tuning knob.
  static constexpr double kBladeStateMaxAgeSec = 2.0;

  /// Upper clamp on the per-tick dt used to integrate the distance budget [s].
  /// A scheduling hiccup must not be able to charge several tenths of a metre
  /// to the budget in one tick (which would end the escape early — safe) nor,
  /// with a negative clock step, un-charge it (which would not).
  static constexpr double kMaxTickDtSec = 0.5;

  /// How long after the last escape command the /cmd_vel direction tracker
  /// keeps ignoring samples [s]. Without this the escape's OWN commands become
  /// the "last motion", so a second escape on the next blocked pass would drive
  /// straight back into the cell the first one left — an oscillation, not a
  /// recovery.
  static constexpr double kSignalHoldoffSec = 1.0;

private:
  /// Publish one TwistStamped on /cmd_vel_nav (zero angular — this is a
  /// straight-line nudge, never a turn).
  void publishForward(const std::shared_ptr<BTContext>& ctx, double vx);
  /// Publish a single zero command and log the manoeuvre summary.
  void finish(const std::shared_ptr<BTContext>& ctx, const char* reason);

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
  StartBlockedEscapeCfg cfg_{};
  StartBlockedEscapeState state_{};
  EscapeDirection direction_{EscapeDirection::kUnknown};
  std::chrono::steady_clock::time_point last_tick_{};
  bool running_{false};
};

}  // namespace mowgli_behavior

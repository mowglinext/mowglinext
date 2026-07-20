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

#include "mowgli_behavior/calibration_nodes.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "mowgli_interfaces/motion_yaw_fit.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace mowgli_behavior
{

namespace
{

// fit_motion_yaw moved to mowgli_interfaces::motion_yaw_fit::FitMotionYaw
// (task #47) — mowgli_localization/calibrate_imu_yaw_node.cpp's dock-yaw
// reverse maneuver now reuses the exact same total-least-squares fit
// instead of an endpoint-only atan2.
using mowgli_interfaces::motion_yaw_fit::FitMotionYaw;

// -------------------------------------------------------------------------
// dock_pose_yaw is NO LONGER persisted from here. CalibrateHeadingFromUndock
// keeps only its runtime /fusion_graph_node/set_pose seed (a live
// localization aid, below); the ONE canonical persist writer for dock_pose
// is now map_server's on_set_docking_point (yaw_source=MOTION), driven by the
// one-click dock-calibration action (CLAUDE.md Invariant 6 collapse). The
// former per-undock EMA writeback (read_dock_pose_yaw_from_yaml + ema_yaw +
// persist_dock_pose_yaw, all via robot_yaml_scalar::PersistScalar) was
// removed so there is exactly one file writer.
// -------------------------------------------------------------------------

// Fill the covariance block for a yaw-plus-xy seed: tight trust on the
// states we want to set, effectively infinite variance on the states we
// want the filter to keep from its prior.
void set_seed_covariance(geometry_msgs::msg::PoseWithCovarianceStamped& msg, double yaw_var)
{
  msg.pose.covariance.fill(0.0);
  msg.pose.covariance[0] = 0.01;  // x
  msg.pose.covariance[7] = 0.01;  // y
  msg.pose.covariance[14] = 1e6;  // z (keep prior)
  msg.pose.covariance[21] = 1e6;  // roll (keep prior)
  msg.pose.covariance[28] = 1e6;  // pitch (keep prior)
  msg.pose.covariance[35] = yaw_var;
}

}  // namespace

// ---------------------------------------------------------------------------
// RecordUndockStart
// ---------------------------------------------------------------------------

BT::NodeStatus RecordUndockStart::tick()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  ctx->undock_start_x = ctx->gps_x;
  ctx->undock_start_y = ctx->gps_y;
  ctx->undock_start_recorded = true;
  // Clear the per-undock GPS buffer and seed it with the current sample
  // so CalibrateHeadingFromUndock can line-fit the whole BackUp
  // trajectory. The GPS subscriber will append further samples (with
  // 5 cm dedup) while undock_start_recorded stays true.
  ctx->undock_gps_samples.clear();
  ctx->undock_gps_samples.emplace_back(ctx->gps_x, ctx->gps_y);
  RCLCPP_INFO(ctx->node->get_logger(),
              "RecordUndockStart: pos=(%.3f, %.3f)",
              ctx->undock_start_x,
              ctx->undock_start_y);
  return BT::NodeStatus::SUCCESS;
}

// ---------------------------------------------------------------------------
// CalibrateHeadingFromUndock
// ---------------------------------------------------------------------------

BT::NodeStatus CalibrateHeadingFromUndock::tick()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (!ctx->undock_start_recorded)
  {
    // RecordUndockStart never fired (e.g., fresh retry after a failed
    // undock where the node was halted). Report SUCCESS rather than
    // FAILURE so the rest of the sequence runs; the dock_yaw injection
    // (hardware_bridge → ekf_map via dock_yaw_to_set_pose) is still
    // active and will have seeded the filter from the charging state.
    RCLCPP_WARN(ctx->node->get_logger(),
                "CalibrateHeadingFromUndock: no undock_start recorded, "
                "skipping (relying on dock_yaw seed).");
    return BT::NodeStatus::SUCCESS;
  }

  // Minimum GPS displacement to refine yaw. At RTK-Fixed σ≈7 mm,
  // σ_yaw = atan2(2σ_pos, displacement). 0.20 m → σ ≈ 4° — far tighter
  // than the dock_yaw seed σ=10° floor, so even a partial undock with
  // wheel slip on the ramp still gives us a useful yaw refinement based
  // on the robot's actual motion rather than yesterday's calibration.
  double min_displacement = 0.20;
  getInput("min_displacement_m", min_displacement);

  const double dx = ctx->gps_x - ctx->undock_start_x;
  const double dy = ctx->gps_y - ctx->undock_start_y;
  const double dist = std::hypot(dx, dy);

  if (dist < min_displacement)
  {
    const bool still_on_dock = ctx->latest_power.charger_enabled;
    ctx->undock_start_recorded = false;
    if (still_on_dock)
    {
      // BackUp reported complete but GPS barely moved AND the dock still
      // charges — robot is genuinely stuck on the dock latch. Fail so
      // UndockSequence fails and the outer ReactiveSequence retries.
      RCLCPP_WARN(ctx->node->get_logger(),
                  "CalibrateHeadingFromUndock: displacement %.3fm below min %.3fm "
                  "AND is_charging=true — robot stuck on dock, retrying undock.",
                  dist,
                  min_displacement);
      return BT::NodeStatus::FAILURE;
    }
    // Partial undock: GPS moved less than expected (wheel slip on the dock
    // ramp is common) but charging has dropped, so the robot IS off the
    // dock. The displacement is too short to refine yaw reliably — trust
    // the dock_yaw injected by dock_yaw_to_set_pose while still on the
    // dock and continue with the rest of the session.
    RCLCPP_INFO(ctx->node->get_logger(),
                "CalibrateHeadingFromUndock: partial undock (%.3fm < %.3fm) but "
                "charging dropped — keeping dock_yaw seed, skipping refinement.",
                dist,
                min_displacement);
    ctx->yaw_seeded_this_session = true;
    return BT::NodeStatus::SUCCESS;
  }

  // Derive the chassis yaw from the BackUp trajectory. Two strategies:
  //   * line_fit (preferred): total-least-squares through every GPS
  //     sample buffered during the BackUp. σ_yaw shrinks roughly as
  //     1/√n and uses the FULL baseline, so a 1.5 m undock at RTK-Fixed
  //     with ~30 samples reaches σ ≈ 0.05–0.1°, well under the 0.23°
  //     endpoint-only ceiling. Requires ≥ 4 samples spanning ≥ 0.5 m.
  //   * endpoint (fallback): atan2(-dy, -dx) on start vs. current
  //     position. σ ≈ atan(2·σ_GPS / dist).
  // Whichever produces the result, the value is the chassis heading at
  // the END of the BackUp — by construction the chassis didn't rotate
  // during the straight reverse so it equals the dock_pose_yaw, which
  // is what we want to persist.
  const auto& samples = ctx->undock_gps_samples;
  const bool have_line_fit_samples =
      samples.size() >= 4u && std::hypot(samples.back().first - samples.front().first,
                                         samples.back().second - samples.front().second) >= 0.5;
  double yaw = 0.0;
  double sigma_yaw = 0.0;
  const char* method = "endpoint";
  if (have_line_fit_samples)
  {
    const auto [motion_yaw, motion_sigma] = FitMotionYaw(samples);
    // Robot heading points opposite the motion (BackUp reverses).
    yaw = motion_yaw + M_PI;
    while (yaw > M_PI)
      yaw -= 2.0 * M_PI;
    while (yaw < -M_PI)
      yaw += 2.0 * M_PI;
    sigma_yaw = std::max(motion_sigma, 0.001);  // floor at ~0.06°
    method = "line_fit";
  }
  else
  {
    // Endpoint fallback. Motion vector (dx, dy) points OPPOSITE the
    // robot's heading so heading = atan2(-dy, -dx).
    yaw = std::atan2(-dy, -dx);
    sigma_yaw = std::atan2(2.0 * 0.007, std::max(dist, 0.05));
  }
  const double yaw_var = std::max(sigma_yaw * sigma_yaw, 5e-4);  // floor σ≈1.3°

  if (!set_pose_pub_)
  {
    // TRANSIENT_LOCAL matches fusion_graph_node's set_pose subscriber.
    // The calibration seed is a one-shot, fire-and-forget message:
    // VOLATILE on either side races subscription discovery (~tens of ms
    // after node up) and silently drops the seed, leaving fusion_graph
    // anchored at the persisted dock pose while the robot drives away.
    auto qos = rclcpp::QoS(1).reliable().transient_local();
    set_pose_pub_ = ctx->node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/fusion_graph_node/set_pose", qos);
  }

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);

  geometry_msgs::msg::PoseWithCovarianceStamped seed{};
  seed.header.stamp = ctx->node->now();
  seed.header.frame_id = "map";
  seed.pose.pose.position.x = ctx->gps_x;
  seed.pose.pose.position.y = ctx->gps_y;
  seed.pose.pose.orientation.x = q.x();
  seed.pose.pose.orientation.y = q.y();
  seed.pose.pose.orientation.z = q.z();
  seed.pose.pose.orientation.w = q.w();
  set_seed_covariance(seed, yaw_var);
  set_pose_pub_->publish(seed);

  // NO YAML writeback here (Invariant 6 single-writer collapse). This node
  // only publishes the live /set_pose seed above; the persisted dock_pose_yaw
  // is owned solely by map_server's on_set_docking_point (yaw_source=MOTION),
  // written by the one-click dock-calibration action. The former per-undock
  // EMA writeback was removed to keep exactly one file writer.
  ctx->undock_start_recorded = false;
  ctx->yaw_seeded_this_session = true;

  RCLCPP_INFO(ctx->node->get_logger(),
              "CalibrateHeadingFromUndock: dist=%.3fm yaw=%.2f° σ=%.2f° "
              "method=%s n=%zu pos=(%.3f, %.3f) [set_pose seed only]",
              dist,
              yaw * 180.0 / M_PI,
              sigma_yaw * 180.0 / M_PI,
              method,
              samples.size(),
              ctx->gps_x,
              ctx->gps_y);
  return BT::NodeStatus::SUCCESS;
}

// ---------------------------------------------------------------------------
// SeedYawFromMotion
// ---------------------------------------------------------------------------

void SeedYawFromMotion::publish_forward(const rclcpp::Node::SharedPtr& node, double speed)
{
  geometry_msgs::msg::TwistStamped cmd{};
  cmd.header.stamp = node->now();
  cmd.header.frame_id = "base_footprint";
  cmd.twist.linear.x = speed;
  cmd_pub_->publish(cmd);
}

void SeedYawFromMotion::publish_zero(const rclcpp::Node::SharedPtr& node)
{
  publish_forward(node, 0.0);
}

BT::NodeStatus SeedYawFromMotion::onStart()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (ctx->yaw_seeded_this_session)
  {
    // Already seeded this session — don't re-drive on ReactiveSequence
    // halt/resume cycles.
    RCLCPP_DEBUG(ctx->node->get_logger(),
                 "SeedYawFromMotion: yaw already seeded this session, "
                 "skipping forward drive.");
    return BT::NodeStatus::SUCCESS;
  }

  if (!cmd_pub_)
  {
    cmd_pub_ = ctx->node->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel_teleop", 10);
  }
  if (!set_pose_pub_)
  {
    // TRANSIENT_LOCAL matches fusion_graph_node's set_pose subscriber.
    // The calibration seed is a one-shot, fire-and-forget message:
    // VOLATILE on either side races subscription discovery (~tens of ms
    // after node up) and silently drops the seed, leaving fusion_graph
    // anchored at the persisted dock pose while the robot drives away.
    auto qos = rclcpp::QoS(1).reliable().transient_local();
    set_pose_pub_ = ctx->node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/fusion_graph_node/set_pose", qos);
  }

  distance_m_ = 1.0;
  speed_ms_ = 0.2;
  timeout_sec_ = 30.0;
  min_displacement_m_ = 0.5;
  getInput("distance_m", distance_m_);
  getInput("speed_ms", speed_ms_);
  getInput("timeout_sec", timeout_sec_);
  getInput("min_displacement_m", min_displacement_m_);

  if (!ctx->gps_is_fixed)
  {
    RCLCPP_WARN(ctx->node->get_logger(),
                "SeedYawFromMotion: no RTK fix at start (fix_type=%u), "
                "aborting seed.",
                ctx->gps_fix_type);
    return BT::NodeStatus::FAILURE;
  }

  x0_ = ctx->gps_x;
  y0_ = ctx->gps_y;
  start_time_ = ctx->node->now();

  RCLCPP_INFO(ctx->node->get_logger(),
              "SeedYawFromMotion: start pos=(%.3f, %.3f), drive %.2fm fwd at %.2fm/s",
              x0_,
              y0_,
              distance_m_,
              speed_ms_);
  publish_forward(ctx->node, speed_ms_);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SeedYawFromMotion::onRunning()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (ctx->latest_emergency.active_emergency || ctx->latest_emergency.latched_emergency)
  {
    publish_zero(ctx->node);
    RCLCPP_WARN(ctx->node->get_logger(),
                "SeedYawFromMotion: emergency during seed drive, aborting.");
    return BT::NodeStatus::FAILURE;
  }

  const double elapsed = (ctx->node->now() - start_time_).seconds();
  if (elapsed > timeout_sec_)
  {
    publish_zero(ctx->node);
    RCLCPP_WARN(ctx->node->get_logger(),
                "SeedYawFromMotion: timeout after %.1fs, aborting.",
                elapsed);
    return BT::NodeStatus::FAILURE;
  }

  const double dx = ctx->gps_x - x0_;
  const double dy = ctx->gps_y - y0_;
  const double dist = std::hypot(dx, dy);

  if (dist < distance_m_)
  {
    publish_forward(ctx->node, speed_ms_);
    return BT::NodeStatus::RUNNING;
  }

  publish_zero(ctx->node);

  if (dist < min_displacement_m_)
  {
    RCLCPP_WARN(ctx->node->get_logger(),
                "SeedYawFromMotion: displacement %.3fm below min %.3fm, "
                "seed invalid.",
                dist,
                min_displacement_m_);
    return BT::NodeStatus::FAILURE;
  }

  const double yaw = std::atan2(dy, dx);

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);

  geometry_msgs::msg::PoseWithCovarianceStamped seed{};
  seed.header.stamp = ctx->node->now();
  seed.header.frame_id = "map";
  seed.pose.pose.position.x = ctx->gps_x;
  seed.pose.pose.position.y = ctx->gps_y;
  seed.pose.pose.orientation.x = q.x();
  seed.pose.pose.orientation.y = q.y();
  seed.pose.pose.orientation.z = q.z();
  seed.pose.pose.orientation.w = q.w();
  set_seed_covariance(seed, 5e-3);  // ~4° σ
  set_pose_pub_->publish(seed);

  ctx->yaw_seeded_this_session = true;

  RCLCPP_INFO(ctx->node->get_logger(),
              "SeedYawFromMotion: dist=%.3fm yaw_seed=%.1f° pos=(%.3f, %.3f) — "
              "set_pose published.",
              dist,
              yaw * 180.0 / M_PI,
              ctx->gps_x,
              ctx->gps_y);
  return BT::NodeStatus::SUCCESS;
}

void SeedYawFromMotion::onHalted()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  if (cmd_pub_)
  {
    publish_zero(ctx->node);
  }
}

}  // namespace mowgli_behavior

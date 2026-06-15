// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// FusionGraphNode — seed/init + output publishing (odom + TF + diagnostics). (The node
// implementation is split across several translation units to keep each file within the project's
// 600-line budget; all share fusion_graph_node.hpp + fusion_graph_node_util.hpp.)

#include <chrono>
#include <cmath>
#include <limits>
#include <thread>

#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "fusion_graph/fusion_graph_node.hpp"
#include "fusion_graph/fusion_graph_node_util.hpp"

namespace fusion_graph
{

bool FusionGraphNode::TrySeedInitialPose()
{
  if (graph_->IsInitialized())
    return true;
  if (!seed_xy_ || !seed_yaw_)
    return false;
  // When the seed came from an RTK-Fixed fix, override the prior to
  // match the measurement quality. 5 mm is conservative w.r.t the
  // F9P's typical RTK-Fixed σ ~3 mm; tight enough that the wheel
  // between-factors can't pull the first few nodes off the GPS
  // anchor, but loose enough to absorb a few mm of antenna lever-arm
  // residual.
  std::optional<double> prior_override;
  if (seed_xy_rtk_fixed_)
    prior_override = 0.005;
  graph_->Initialize(gtsam::Pose2(seed_xy_->x(), seed_xy_->y(), *seed_yaw_),
                     this->now().seconds(),
                     prior_override);
  t_map_odom_anchor_valid_ = false;
  RCLCPP_INFO(get_logger(),
              "fusion_graph: initialized at (%.3f, %.3f, %.3f rad)%s",
              seed_xy_->x(),
              seed_xy_->y(),
              *seed_yaw_,
              seed_xy_rtk_fixed_ ? " [RTK-Fixed seed, σ=5mm prior]" : " [non-Fixed seed]");
  return true;
}

void FusionGraphNode::PublishLocalOdom()
{
  // odom→base_footprint TF + /odometry/filtered from the dead-reckoning
  // state. Replaces ekf_odom_node. Streams every OnTimer tick so the
  // local frame is ready before the graph itself initializes (Nav2's
  // local costmap and FTCController both rely on this TF, and they
  // can come up before any GPS fix has landed).
  const rclcpp::Time now = this->now();
  // Forward-stamp the TF by tf_publish_lead_s_ so Nav2 controller_server
  // lookups at clock()->now() land in tf2 interpolation territory instead of
  // throwing ExtrapolationException. The pose is propagated forward by the same
  // lead via ForwardStampPose so the future stamp is an honest prediction —
  // without that, a future-stamped current pose injects wz·lead of yaw error
  // during pivots. (On hardware the dedicated TF thread owns this broadcast;
  // this inline path runs only in observer mode / when the thread is disabled.)
  const rclcpp::Time stamp = now + rclcpp::Duration::from_seconds(tf_publish_lead_s_);

  double ex_x = dr_x_;
  double ex_y = dr_y_;
  double ex_yaw = dr_yaw_;
  ForwardStampPose(tf_publish_lead_s_, dr_last_gz_, dr_last_vx_eff_, ex_x, ex_y, ex_yaw);

  geometry_msgs::msg::TransformStamped t_odom_base;
  t_odom_base.header.stamp = stamp;
  t_odom_base.header.frame_id = odom_frame_;
  t_odom_base.child_frame_id = base_frame_;
  t_odom_base.transform.translation.x = ex_x;
  t_odom_base.transform.translation.y = ex_y;
  t_odom_base.transform.translation.z = 0.0;
  t_odom_base.transform.rotation = QuatFromYaw(ex_yaw);
  // Only the PRIMARY localizer may own odom→base_footprint — same single-
  // publisher rule as map→odom below. In observer mode (A/B against another
  // odom source) we still publish /odometry/filtered for comparison but must
  // NOT broadcast the TF, or two nodes would fight over the same transform.
  // When the dedicated TF thread is running it owns this broadcast (single
  // writer, monotonic stamps) — skip the inline send.
  if (primary_mode_ && !tf_thread_.joinable())
  {
    tf_broadcaster_->sendTransform(t_odom_base);
  }

  const geometry_msgs::msg::Quaternion q_msg = QuatFromYaw(dr_yaw_);
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = now;
  odom.header.frame_id = odom_frame_;
  odom.child_frame_id = base_frame_;
  odom.pose.pose.position.x = dr_x_;
  odom.pose.pose.position.y = dr_y_;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = q_msg;
  odom.twist.twist.linear.x = wheel_vx_;
  odom.twist.twist.linear.y = 0.0;
  // Dead reckoning has unbounded drift — leave pose covariance loose
  // and let Nav2 trust the graph's /odometry/filtered_map for absolute
  // positioning. Tight roll/pitch/z so 2D consumers don't see NaN.
  for (auto& v : odom.pose.covariance)
    v = 0.0;
  odom.pose.covariance[0] = 0.05;  // x
  odom.pose.covariance[7] = 0.05;  // y
  odom.pose.covariance[14] = 1e-9;  // z
  odom.pose.covariance[21] = 1e-9;  // roll
  odom.pose.covariance[28] = 1e-9;  // pitch
  odom.pose.covariance[35] = 0.02;  // yaw — gyro short-term σ ≈ 0.14 rad
  pub_local_odom_->publish(odom);
}

void FusionGraphNode::PublishOutputs(const TickOutput& out)
{
  // Extrapolate the last-node pose through current odom integration
  // so the published map-frame outputs reflect motion that happened
  // since the snapshot was taken. Without this, /odometry/filtered_map
  // and the map→odom TF stay glued to out.pose for up to
  // stationary_node_period_s (5 s by default) → robot looks frozen
  // in viz, then teleports when the next Tick lands.
  const gtsam::Pose2 dr_now(dr_x_, dr_y_, dr_yaw_);
  const gtsam::Pose2 extrapolated_map_base =
      t_map_odom_anchor_valid_ ? t_map_odom_anchor_.compose(dr_now) : out.pose;

  // 1. nav_msgs/Odometry on /odometry/filtered_map.
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = this->now();
  odom.header.frame_id = map_frame_;
  odom.child_frame_id = base_frame_;
  odom.pose.pose.position.x = extrapolated_map_base.x();
  odom.pose.pose.position.y = extrapolated_map_base.y();
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = QuatFromYaw(extrapolated_map_base.theta());

  // Pose covariance is 6x6 row-major: x, y, z, roll, pitch, yaw.
  for (auto& v : odom.pose.covariance)
    v = 0.0;
  odom.pose.covariance[0] = out.covariance(0, 0);
  odom.pose.covariance[1] = out.covariance(0, 1);
  odom.pose.covariance[5] = out.covariance(0, 2);
  odom.pose.covariance[6] = out.covariance(1, 0);
  odom.pose.covariance[7] = out.covariance(1, 1);
  odom.pose.covariance[11] = out.covariance(1, 2);
  odom.pose.covariance[30] = out.covariance(2, 0);
  odom.pose.covariance[31] = out.covariance(2, 1);
  odom.pose.covariance[35] = out.covariance(2, 2);
  // Z, roll, pitch — clamped, give them tiny variance so consumers
  // don't choke on zero.
  odom.pose.covariance[14] = 1e-9;
  odom.pose.covariance[21] = 1e-9;
  odom.pose.covariance[28] = 1e-9;
  pub_odom_->publish(odom);

  // Rebaseline the high-rate extrapolator (item #15). The fast
  // publisher will project yaw forward from this pose until the
  // next fusion tick.
  pose_extrap_.OnFusionPose(this->now().seconds(), out.pose);

  // 1b. /imu/fg_yaw — yaw-only sensor_msgs/Imu (cov_yaw, others 1e6
  //     to disable). Published in both primary and observer mode so
  //     ekf_map_node can fuse it as imu2 absolute yaw without
  //     creating a feedback loop (fusion_graph never reads
  //     /odometry/filtered_map).
  sensor_msgs::msg::Imu yaw_msg;
  yaw_msg.header.stamp = odom.header.stamp;
  yaw_msg.header.frame_id = base_frame_;
  yaw_msg.orientation = QuatFromYaw(extrapolated_map_base.theta());
  // Roll/pitch covariances marked huge so robot_localization ignores
  // them; only orientation_covariance[8] (yaw variance) is real.
  for (auto& v : yaw_msg.orientation_covariance)
    v = 0.0;
  yaw_msg.orientation_covariance[0] = 1e6;
  yaw_msg.orientation_covariance[4] = 1e6;
  yaw_msg.orientation_covariance[8] = std::max(out.covariance(2, 2), 1e-6);
  // We don't fuse angular velocity / linear acceleration here; mark
  // them all as -1 covariance to tell consumers the field is invalid.
  yaw_msg.angular_velocity_covariance[0] = -1.0;
  yaw_msg.linear_acceleration_covariance[0] = -1.0;
  pub_fg_yaw_->publish(yaw_msg);

  // 2. TF map -> odom. Skipped in observer mode so the active
  //    map-frame primary (typically ekf_map_node) keeps single
  //    ownership of the map→odom transform. The /odometry/filtered_map
  //    publish above still happens — downstream consumers that
  //    explicitly route to /fusion_graph/odometry (via launch remap)
  //    can use it for diagnostics or A/B comparison.
  if (!primary_mode_)
    return;

  // The dedicated TF thread owns the map→odom broadcast when running
  // (single writer, monotonic stamps) — skip the inline send.
  if (tf_thread_.joinable())
    return;

  // The map→odom TF is the CONSTANT anchor captured at the moment of
  // the last graph node creation (see fusion_graph_node.hpp). Using
  // out.pose × inv(dr_now) instead would zero out current odom
  // motion in the composition map→base = map→odom × odom→base, so
  // the robot would freeze at the snapshot pose between Ticks.
  tf2::Transform T_map_odom;
  if (t_map_odom_anchor_valid_)
  {
    T_map_odom.setOrigin(tf2::Vector3(t_map_odom_anchor_.x(), t_map_odom_anchor_.y(), 0.0));
    tf2::Quaternion q_map_odom;
    q_map_odom.setRPY(0.0, 0.0, t_map_odom_anchor_.theta());
    T_map_odom.setRotation(q_map_odom);
  }
  else
  {
    // Fallback for the brief OnTimer just after init but before the
    // anchor has been set (caller already gates on LatestSnapshot, so
    // we shouldn't reach here in practice). Identity is safer than
    // out.pose × inv(dr_now) — at least the robot doesn't get stuck.
    T_map_odom.setIdentity();
  }

  geometry_msgs::msg::TransformStamped t_map_odom;
  // Forward-stamp by tf_publish_lead_s_ so Nav2 controller_server /
  // RotationShim queries at clock_->now() find a TF in the buffer that
  // is >= the request time and tf2 interpolates back instead of raising
  // ExtrapolationException. Default 0 (real hardware); sim sets ~0.1s.
  t_map_odom.header.stamp = this->now() + rclcpp::Duration::from_seconds(tf_publish_lead_s_);
  t_map_odom.header.frame_id = map_frame_;
  t_map_odom.child_frame_id = odom_frame_;
  t_map_odom.transform = tf2::toMsg(T_map_odom);
  tf_broadcaster_->sendTransform(t_map_odom);
}

void FusionGraphNode::TfBroadcastLoop()
{
  // See tf_broadcast_rate_hz_ in the header for the rationale. This
  // loop owns ALL TF broadcasting in primary mode — PublishLocalOdom
  // and PublishOutputs skip their inline sendTransform when the
  // thread is running (tf_thread_.joinable()), so there is exactly
  // one writer per TF leg and stamps stay monotonic.
  const auto period = std::chrono::duration<double>(1.0 / tf_broadcast_rate_hz_);
  while (!tf_thread_stop_.load(std::memory_order_acquire))
  {
    std::this_thread::sleep_for(period);
    if (tf_thread_stop_.load(std::memory_order_acquire))
    {
      break;
    }

    double x;
    double y;
    double yaw;
    double gz;
    double vx_eff;
    bool anchor_valid;
    gtsam::Pose2 anchor;
    {
      std::lock_guard<std::mutex> lock(tf_state_mu_);
      x = dr_x_;
      y = dr_y_;
      yaw = dr_yaw_;
      gz = dr_last_gz_;
      vx_eff = dr_last_vx_eff_;
      anchor_valid = t_map_odom_anchor_valid_;
      anchor = t_map_odom_anchor_;
    }

    // Honest forward-stamp: advance the DR pose by the lead so the now()+lead
    // stamp predicts the pose at that instant. The rotation happens in the
    // odom→base leg (dr_yaw_); the map→odom anchor is time-invariant, so only
    // odom→base is propagated. Removes the wz·lead heading error during pivots.
    ForwardStampPose(tf_publish_lead_s_, gz, vx_eff, x, y, yaw);

    const rclcpp::Time stamp = this->now() + rclcpp::Duration::from_seconds(tf_publish_lead_s_);

    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    transforms.reserve(2);

    geometry_msgs::msg::TransformStamped t_odom_base;
    t_odom_base.header.stamp = stamp;
    t_odom_base.header.frame_id = odom_frame_;
    t_odom_base.child_frame_id = base_frame_;
    t_odom_base.transform.translation.x = x;
    t_odom_base.transform.translation.y = y;
    t_odom_base.transform.translation.z = 0.0;
    t_odom_base.transform.rotation = QuatFromYaw(yaw);
    transforms.push_back(t_odom_base);

    // map→odom only once the graph has produced its first node — the
    // constant anchor is the single source of truth (see the anchor
    // comment block in the header).
    if (anchor_valid)
    {
      tf2::Transform T_map_odom;
      T_map_odom.setOrigin(tf2::Vector3(anchor.x(), anchor.y(), 0.0));
      tf2::Quaternion q_map_odom;
      q_map_odom.setRPY(0.0, 0.0, anchor.theta());
      T_map_odom.setRotation(q_map_odom);

      geometry_msgs::msg::TransformStamped t_map_odom;
      t_map_odom.header.stamp = stamp;
      t_map_odom.header.frame_id = map_frame_;
      t_map_odom.child_frame_id = odom_frame_;
      t_map_odom.transform = tf2::toMsg(T_map_odom);
      transforms.push_back(t_map_odom);
    }

    tf_broadcaster_->sendTransform(transforms);
  }
}
}  // namespace fusion_graph

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
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/exceptions.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "fusion_graph/anchor_slew.hpp"
#include "fusion_graph/covariance_frame.hpp"
#include "fusion_graph/fusion_graph_node.hpp"
#include "fusion_graph/fusion_graph_node_util.hpp"

namespace fusion_graph
{
namespace
{
std::vector<geometry_msgs::msg::TransformStamped> MotionTransforms(
    const rclcpp::Time& source_stamp,
    const rclcpp::Time& forecast_stamp,
    const gtsam::Pose2& source_pose,
    const gtsam::Pose2& predicted_pose,
    const gtsam::Pose2& anchor,
    bool anchor_valid,
    const std::string& map_frame,
    const std::string& odom_frame,
    const std::string& base_frame)
{
  std::vector<geometry_msgs::msg::TransformStamped> transforms;
  transforms.reserve(4);
  const auto append = [&](const rclcpp::Time& stamp, const gtsam::Pose2& pose)
  {
    if (anchor_valid)
    {
      geometry_msgs::msg::TransformStamped map_odom;
      map_odom.header.stamp = stamp;
      map_odom.header.frame_id = map_frame;
      map_odom.child_frame_id = odom_frame;
      map_odom.transform.translation.x = anchor.x();
      map_odom.transform.translation.y = anchor.y();
      map_odom.transform.rotation = QuatFromYaw(anchor.theta());
      transforms.push_back(map_odom);
    }
    geometry_msgs::msg::TransformStamped odom_base;
    odom_base.header.stamp = stamp;
    odom_base.header.frame_id = odom_frame;
    odom_base.child_frame_id = base_frame;
    odom_base.transform.translation.x = pose.x();
    odom_base.transform.translation.y = pose.y();
    odom_base.transform.rotation = QuatFromYaw(pose.theta());
    transforms.push_back(odom_base);
  };
  // Reinsert the known source estimate at its own epoch before the unchanged
  // future forecast. Existing predicted knots between them are not revised.
  append(source_stamp, source_pose);
  if (forecast_stamp != source_stamp)
    append(forecast_stamp, predicted_pose);
  return transforms;
}
}  // namespace

bool FusionGraphNode::TrySeedInitialPose()
{
  if (graph_->IsInitialized())
    return true;
  if (pending_explicit_seed_)
  {
    if (!integrated_source_ns_ ||
        !InitializeAtSource(*pending_explicit_seed_, *integrated_source_ns_))
      return false;
    pending_explicit_seed_.reset();
    return true;
  }
  if (!seed_xy_ || !seed_yaw_ || !seed_source_ns_ || !seed_yaw_source_ns_ ||
      seed_yaw_source_epoch_ != integration_epoch_)
    return false;
  const auto dr_at_position = DrPoseAtSource(integration_epoch_, *seed_source_ns_);
  const auto dr_at_heading = DrPoseAtSource(integration_epoch_, *seed_yaw_source_ns_);
  if (!dr_at_position || !dr_at_heading)
    return false;
  // A candidate received before the motion baseline keeps its original
  // payload/noise. Retry admission once source support becomes available.
  if (seed_yaw_candidate_ && !seed_yaw_observation_id_ &&
      graph_->QueueYawAtEpoch(*seed_yaw_,
                              seed_yaw_candidate_->sigma,
                              seed_yaw_candidate_->robust,
                              *seed_yaw_source_ns_,
                              seed_yaw_candidate_->id,
                              integration_epoch_))
    seed_yaw_observation_id_ = seed_yaw_candidate_->id;
  if (seed_yaw_candidate_)
  {
    // Original likelihoods share the consumed source history. Missing proof
    // stays pending, including for equal source stamps; a derived pose prior
    // cannot substitute for a heading the input owner has not admitted.
    const auto result =
        graph_->InitializeSourceObservationPair(this->now().nanoseconds(),
                                                *seed_source_ns_,
                                                integration_epoch_,
                                                static_cast<uint64_t>(*seed_source_ns_),
                                                seed_yaw_candidate_->id);
    if (result == SourceBootstrapResult::Failed)
      RCLCPP_ERROR_THROTTLE(get_logger(),
                            *get_clock(),
                            5000,
                            "fusion_graph: source initialization failed; admitted observations "
                            "remain pending");
    if (result != SourceBootstrapResult::Applied)
      return false;
    seed_yaw_observation_id_ = seed_yaw_candidate_->id;
  }
  else
  {
    // Explicit clear can carry a heading prior at a verified consumed boundary.
    // Transport that prior through consumed rotation, retaining the new GNSS
    // likelihood once rather than manufacturing a second position prior.
    const double yaw_at_position = *seed_yaw_ + dr_at_heading->between(*dr_at_position).theta();
    if (graph_->InitializeSourcePositionWithRetimedHeading(this->now().nanoseconds(),
                                                           *seed_source_ns_,
                                                           integration_epoch_,
                                                           static_cast<uint64_t>(*seed_source_ns_),
                                                           yaw_at_position) !=
        SourceBootstrapResult::Applied)
      return false;
  }
  t_map_odom_anchor_valid_ = false;
  const auto initialized = graph_->LatestSnapshot();
  RCLCPP_INFO(get_logger(),
              "fusion_graph: initialized at (%.3f, %.3f, %.3f rad) from admitted GNSS and %s%s",
              initialized->pose.x(),
              initialized->pose.y(),
              initialized->pose.theta(),
              seed_yaw_candidate_ ? "heading observations" : "a carried heading prior",
              seed_xy_rtk_fixed_ ? " [RTK-Fixed seed]" : " [non-Fixed seed]");
  return true;
}

std::optional<FusionGraphNode::MotionPublishState> FusionGraphNode::CaptureMotionPublishState()
{
  std::lock_guard<std::mutex> lock(tf_state_mu_);
  if (!integrated_source_ns_)
    return std::nullopt;
  const auto prediction = gtsam::Pose2::Expmap(
      gtsam::Vector3(dr_last_vx_eff_ * tf_publish_lead_s_, 0.0, dr_last_gz_ * tf_publish_lead_s_));
  const auto source_pose = ContinuousDrPoseLocked();
  return MotionPublishState{*integrated_source_ns_,
                            source_pose,
                            source_pose.compose(prediction),
                            t_map_odom_anchor_,
                            t_map_odom_anchor_valid_,
                            dr_last_vx_eff_,
                            dr_last_gz_};
}

bool FusionGraphNode::FastPoseSourceIsCurrent(double now_s) const
{
  if (!fast_pose_source_ns_ || fast_pose_source_epoch_ != integration_epoch_ ||
      !integrated_source_ns_ || *fast_pose_source_ns_ > *integrated_source_ns_ ||
      !t_map_odom_anchor_valid_ || !pose_extrap_.HasFusionPose())
    return false;
  // Match the helper's existing 200 ms constant-rate prediction limit. Once
  // that source support expires, its baseline cannot be relabelled as now.
  const double dt = now_s - static_cast<double>(*fast_pose_source_ns_) * 1e-9;
  return dt >= 0.0 && dt <= 0.2;
}

void FusionGraphNode::PublishLocalOdom()
{
  const auto state = CaptureMotionPublishState();
  if (!state)
    return;
  const auto stamp = rclcpp::Time(state->source_ns, get_clock()->get_clock_type()) +
                     rclcpp::Duration::from_seconds(tf_publish_lead_s_);
  const double ex_x = state->predicted_pose.x();
  const double ex_y = state->predicted_pose.y();
  const double ex_yaw = state->predicted_pose.theta();

  // In inline mode publish both TF legs from this same captured state.
  // The dedicated thread remains the sole TF writer when enabled.
  if (primary_mode_ && !tf_thread_.joinable())
  {
    const auto source_stamp = rclcpp::Time(state->source_ns, get_clock()->get_clock_type());
    tf_broadcaster_->sendTransform(MotionTransforms(source_stamp,
                                                    stamp,
                                                    state->source_pose,
                                                    state->predicted_pose,
                                                    state->anchor,
                                                    state->anchor_valid,
                                                    map_frame_,
                                                    odom_frame_,
                                                    base_frame_));
  }

  const geometry_msgs::msg::Quaternion q_msg = QuatFromYaw(ex_yaw);
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = odom_frame_;
  odom.child_frame_id = base_frame_;
  odom.pose.pose.position.x = ex_x;
  odom.pose.pose.position.y = ex_y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = q_msg;
  odom.twist.twist.linear.x = state->vx;
  odom.twist.twist.linear.y = 0.0;
  // Bias-corrected gyro yaw rate — the SAME rate dead-reckoning integrates into
  // dr_yaw_. Leaving this 0 made /odometry/filtered unusable as Nav2's
  // controller_server.odom_topic (MPPI seeds rollouts / RPP scales lookahead
  // from twist.angular.z — the documented "controllers had zero velocity
  // feedback" gap that forced odom_topic onto raw /wheel_odom). Same executor
  // thread as the dr_* writers, so no lock needed here.
  odom.twist.twist.angular.z = state->gz;
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
  for (auto& v : odom.twist.covariance)
    v = 0.0;
  odom.twist.covariance[0] = 0.02;  // vx — wheel-odom velocity noise
  odom.twist.covariance[35] = 4e-4;  // wz — gyro rate noise (σ ≈ 0.02 rad/s)
  pub_local_odom_->publish(odom);
}

void FusionGraphNode::PublishOutputs(const TickOutput& out)
{
  const auto state = CaptureMotionPublishState();
  if (!state || !state->anchor_valid)
    return;
  const auto extrapolated_map_base = state->anchor.compose(state->predicted_pose);

  // 1. nav_msgs/Odometry on /odometry/filtered_map.
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = rclcpp::Time(state->source_ns, get_clock()->get_clock_type()) +
                      rclcpp::Duration::from_seconds(tf_publish_lead_s_);
  odom.header.frame_id = map_frame_;
  odom.child_frame_id = base_frame_;
  odom.pose.pose.position.x = extrapolated_map_base.x();
  odom.pose.pose.position.y = extrapolated_map_base.y();
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = QuatFromYaw(extrapolated_map_base.theta());

  // Pose covariance is 6x6 row-major: x, y, z, roll, pitch, yaw, and — like
  // every other field in this message — it is expressed in header.frame_id
  // (map). GTSAM hands back the marginal in the pose's LOCAL tangent frame,
  // so it must be rotated by the pose's heading before it goes out; the wheel
  // between-factor is non-holonomic (sigma_x >> sigma_y), which makes the
  // body-frame matrix strongly anisotropic and the difference material.
  // See covariance_frame.hpp.
  const Eigen::Matrix3d cov_map =
      BodyToMapCovariance(out.covariance, extrapolated_map_base.theta());

  for (auto& v : odom.pose.covariance)
    v = 0.0;
  odom.pose.covariance[0] = cov_map(0, 0);
  odom.pose.covariance[1] = cov_map(0, 1);
  odom.pose.covariance[5] = cov_map(0, 2);
  odom.pose.covariance[6] = cov_map(1, 0);
  odom.pose.covariance[7] = cov_map(1, 1);
  odom.pose.covariance[11] = cov_map(1, 2);
  odom.pose.covariance[30] = cov_map(2, 0);
  odom.pose.covariance[31] = cov_map(2, 1);
  odom.pose.covariance[35] = cov_map(2, 2);
  // Z, roll, pitch — clamped, give them tiny variance so consumers
  // don't choke on zero.
  odom.pose.covariance[14] = 1e-9;
  odom.pose.covariance[21] = 1e-9;
  odom.pose.covariance[28] = 1e-9;
  pub_odom_->publish(odom);

  // Rebaseline the high-rate extrapolator (item #15). The fast
  // publisher will project yaw forward from this pose until the
  // next fusion tick.
  pose_extrap_.OnFusionPose(static_cast<double>(state->source_ns) * 1e-9,
                            state->anchor.compose(state->source_pose));
  fast_pose_source_ns_ = state->source_ns;
  fast_pose_source_epoch_ = integration_epoch_;

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

  // Inline TF publication is paired in PublishLocalOdom so the source and
  // forecast legs share one captured anchor and motion state.
}

gtsam::Pose2 FusionGraphNode::SlewPublishedAnchor(const gtsam::Pose2& target,
                                                  bool anchor_valid,
                                                  double dt)
{
  // An odom re-base (OnTimer) sets force_pub_resync_: the target anchor just
  // jumped by a coordinated amount that leaves map→base UNCHANGED, so the
  // published anchor must snap to it (not ramp) or map→base would move. Drop
  // pub_valid so AnchorSlewStep snaps on this cycle.
  if (force_pub_resync_.exchange(false, std::memory_order_acq_rel))
    t_map_odom_pub_valid_ = false;

  // Thin wrapper over the pure AnchorSlewStep (anchor_slew.hpp). Single
  // writer of t_map_odom_pub_ per run mode (TF thread when the broadcast
  // thread runs, executor otherwise) → no lock taken here.
  const AnchorSlewCfg cfg{anchor_slew_enabled_,
                          anchor_max_lin_slew_mps_,
                          anchor_max_ang_slew_radps_,
                          anchor_snap_dist_m_,
                          anchor_snap_yaw_rad_};
  double px = t_map_odom_pub_.x();
  double py = t_map_odom_pub_.y();
  double pyaw = t_map_odom_pub_.theta();
  double ox = 0.0;
  double oy = 0.0;
  double oyaw = 0.0;
  AnchorSlewStep(t_map_odom_pub_valid_,
                 px,
                 py,
                 pyaw,
                 anchor_valid,
                 target.x(),
                 target.y(),
                 target.theta(),
                 dt,
                 cfg,
                 ox,
                 oy,
                 oyaw);
  t_map_odom_pub_ = gtsam::Pose2(px, py, pyaw);
  return gtsam::Pose2(ox, oy, oyaw);
}

void FusionGraphNode::TfBroadcastLoop()
{
  // See tf_broadcast_rate_hz_ in the header for the rationale. This
  // loop owns ALL TF broadcasting in primary mode — PublishLocalOdom
  // and PublishOutputs skip their inline sendTransform when the
  // thread is running (tf_thread_.joinable()), so there is exactly
  // one writer per TF leg; source knots intentionally backfill forecasts.
  const auto period = std::chrono::duration<double>(1.0 / tf_broadcast_rate_hz_);
  while (!tf_thread_stop_.load(std::memory_order_acquire))
  {
    std::this_thread::sleep_for(period);
    if (tf_thread_stop_.load(std::memory_order_acquire))
    {
      break;
    }

    const auto state = CaptureMotionPublishState();
    if (!state)
      continue;
    const auto source_stamp = rclcpp::Time(state->source_ns, get_clock()->get_clock_type());
    const auto forecast_stamp = source_stamp + rclcpp::Duration::from_seconds(tf_publish_lead_s_);
    tf_broadcaster_->sendTransform(MotionTransforms(source_stamp,
                                                    forecast_stamp,
                                                    state->source_pose,
                                                    state->predicted_pose,
                                                    state->anchor,
                                                    state->anchor_valid,
                                                    map_frame_,
                                                    odom_frame_,
                                                    base_frame_));
  }
}
}  // namespace fusion_graph

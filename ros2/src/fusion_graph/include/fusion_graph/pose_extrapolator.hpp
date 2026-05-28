// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// PoseExtrapolator — constant-rate forward propagation of a Pose2
// between iSAM2 graph ticks.
//
// fusion_graph publishes /odometry/filtered_map at 50 Hz today. For
// display-latency-sensitive consumers (operator GUI, foxglove panels)
// the 20 ms gap between iSAM2 ticks shows up as a perceptible yaw
// lag during pivots — IMU is available at 91 Hz, but the graph
// optimiser doesn't expose pose between ticks.
//
// This helper class lives between the IMU callback and the high-rate
// republisher: it caches the latest fusion-published pose and the
// latest IMU gyro_z sample, and exposes Extrapolate(query_t) that
// returns the pose with yaw projected forward by gyro_z·dt.
//
// Position is NOT extrapolated. Wheel velocity isn't part of the
// inputs here, and the small position change between ticks (<2 cm
// at 0.3 m/s × 20 ms) is well under typical display tolerances —
// while yaw can swing 5-10° at the same rate during pivots, which
// IS visible.
//
// Single-threaded — caller must serialize OnFusionPose / OnImuGyro /
// Extrapolate. fusion_graph_node uses a single rclcpp executor, so
// no locking inside.

#pragma once

#include <cstdint>
#include <optional>

#include <gtsam/geometry/Pose2.h>

namespace fusion_graph
{

class PoseExtrapolator
{
public:
  PoseExtrapolator() = default;

  // Record a fresh fusion-published pose. Subsequent Extrapolate
  // calls take this as the new baseline. Timestamp is in seconds
  // (steady_clock or ROS clock — caller decides, only matters that
  // it agrees with OnImuGyro stamps).
  void OnFusionPose(double timestamp_s, const gtsam::Pose2& pose)
  {
    last_fusion_stamp_s_ = timestamp_s;
    last_fusion_pose_ = pose;
  }

  // Update the latest gyro_z rate (rad/s). Constant-rate model: we
  // assume gyro_z stays at this value until the next sample. At
  // 91 Hz IMU and ≤ 20 ms extrapolation window we'll integrate at
  // most 1-2 samples' worth, so the constant assumption costs at
  // most ~5 % yaw error on aggressive pivots.
  void OnImuGyro(double timestamp_s, double wz_rad_per_s)
  {
    last_gyro_stamp_s_ = timestamp_s;
    last_gyro_wz_ = wz_rad_per_s;
  }

  // Return the extrapolated pose at query_timestamp_s. Returns
  // nullopt if no fusion pose has been seen yet. Position is the
  // last fusion position unchanged; yaw is fusion_yaw + gyro * dt
  // where dt = query - last_fusion_stamp. Negative dt (query in the
  // past) returns the unmodified fusion pose.
  std::optional<gtsam::Pose2> Extrapolate(double query_timestamp_s) const
  {
    if (!last_fusion_pose_)
      return std::nullopt;

    const double dt = query_timestamp_s - last_fusion_stamp_s_;
    if (dt <= 0.0 || !last_gyro_stamp_s_)
      return last_fusion_pose_;

    // Sanity: cap forward extrapolation at 200 ms. If the caller
    // has stalled and dt is larger than that, the gyro-constant
    // assumption is no longer trustworthy; just return the baseline.
    constexpr double kMaxExtrapolationS = 0.2;
    if (dt > kMaxExtrapolationS)
      return last_fusion_pose_;

    const double dyaw = last_gyro_wz_ * dt;
    return gtsam::Pose2(last_fusion_pose_->x(),
                        last_fusion_pose_->y(),
                        last_fusion_pose_->theta() + dyaw);
  }

  // Diagnostic accessors — used by /fusion_graph/diagnostics + tests.
  bool HasFusionPose() const
  {
    return last_fusion_pose_.has_value();
  }
  double LastFusionStamp() const
  {
    return last_fusion_stamp_s_;
  }
  double LastGyroWz() const
  {
    return last_gyro_wz_;
  }

private:
  std::optional<gtsam::Pose2> last_fusion_pose_;
  double last_fusion_stamp_s_ = 0.0;
  std::optional<double> last_gyro_stamp_s_;
  double last_gyro_wz_ = 0.0;
};

}  // namespace fusion_graph

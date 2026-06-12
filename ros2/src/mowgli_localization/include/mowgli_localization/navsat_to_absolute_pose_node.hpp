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

// SPDX-License-Identifier: GPL-3.0
/**
 * @file navsat_to_absolute_pose_node.hpp
 * @brief Converts sensor_msgs/NavSatFix → mowgli_interfaces/AbsolutePose.
 *
 * Bridges generic ROS2 GNSS fixes into the Mowgli AbsolutePose message.
 *
 * Performs WGS84 → local ENU projection using a configurable datum origin.
 * Universal GNSS is the single owner of the typed /gps/status contract, so
 * this node only consumes /gps/fix and publishes /gps/absolute_pose plus the
 * robot_localization-friendly /gps/pose_cov twin.
 *
 * Subscribed topics:
 *   /gps/fix   sensor_msgs/msg/NavSatFix
 *
 * Published topics:
 *   /gps/absolute_pose    mowgli_interfaces/msg/AbsolutePose
 *   /gps/pose_cov         geometry_msgs/msg/PoseWithCovarianceStamped
 */

#pragma once

#include <cmath>
#include <memory>

#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "mowgli_interfaces/msg/absolute_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace mowgli_localization
{

class NavSatToAbsolutePoseNode : public rclcpp::Node
{
public:
  explicit NavSatToAbsolutePoseNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~NavSatToAbsolutePoseNode() override = default;

private:
  void declare_parameters();
  void create_publishers();
  void create_subscribers();
  void create_services();

  void on_navsat_fix(sensor_msgs::msg::NavSatFix::ConstSharedPtr msg);
  void on_set_datum(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  /**
   * @brief Project WGS84 lat/lon to local ENU (x=east, y=north) relative to datum.
   *
   * Uses an equirectangular approximation which is accurate to ~1 cm within
   * 10 km of the datum origin — more than sufficient for a garden mower.
   */
  void wgs84_to_enu(double lat, double lon, double& east, double& north) const;

  // Parameters
  double datum_lat_{0.0};
  double datum_lon_{0.0};
  double cos_datum_lat_{1.0};  ///< Precomputed cos(datum_lat) for projection

  // Latest GPS fix for the set_datum service.
  sensor_msgs::msg::NavSatFix last_fix_;
  bool has_fix_{false};

  // ROS handles
  rclcpp::Publisher<mowgli_interfaces::msg::AbsolutePose>::SharedPtr pose_pub_;
  /// Standard-msg twin of the AbsolutePose topic. robot_localization's EKF
  /// pose0 input expects PoseWithCovarianceStamped; AbsolutePose is a
  /// Mowgli-specific type and not subscribable by the EKF.
  ///
  /// UNLIKE pose_pub_ this publishes BASE FRAME (base_footprint) position —
  /// not antenna position. Lever arm is subtracted using the latest
  /// map→base_footprint TF for the yaw at GPS-fix time. Required so the
  /// EKF tracks the robot body and not the 30-cm antenna circle traced
  /// during pure rotation.
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_cov_pub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr set_datum_srv_;

  /// TF listener to resolve base_footprint↔gps_link (static from URDF,
  /// gives the lever arm) and map↔base_footprint (dynamic from ekf_map,
  /// gives current yaw in the same world frame as the GNSS fix). First-
  /// successful lookup latches the lever arm; yaw is looked up fresh each
  /// fix and propagated conservatively into pose covariance.
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  bool lever_arm_known_{false};
  double lever_arm_x_{0.0};
  double lever_arm_y_{0.0};

  /// Static yaw uncertainty used for lever-arm covariance propagation.
  /// We deliberately do NOT subscribe to ekf_map's own σ²_yaw because
  /// (a) over-confident EKF states would shrink the inflation to ~zero
  /// and reintroduce the over-trust feedback we are trying to prevent,
  /// and (b) over-conservative bootstrap states would balloon pose_cov
  /// to "ignore me" precision, removing the implicit yaw-anchoring that
  /// tight pose_cov provides through xy↔yaw cross-covariance during
  /// COG-poor windows. A constant 3° sigma adds ~1.6 cm of position σ
  /// to RTK's 5 mm — realistic for the lever-arm uncertainty without
  /// destroying EKF anchoring.
  double lever_arm_yaw_sigma_{0.0524};  ///< 3° = 0.0524 rad

  /// Defensive guard on /gps/pose_cov covariance. RTK Fixed nominally
  /// reports σ ≈ 3-10 mm via NAV-COV; if the receiver's own reported
  /// accuracy exceeds inflation_threshold (default 25 mm), we multiply
  /// the published variance by inflation_factor² so the EKF down-weights
  /// the sample instead of trusting a degraded "Fixed" fix. Beyond
  /// reject_threshold (default 0.5 m) we skip publishing /gps/pose_cov
  /// entirely — the EKF should not see the sample at all. The raw
  /// /gps/absolute_pose still publishes for the GUI / BT consumers.
  /// These guards exist because the F9P can keep reporting carr_soln=2
  /// (Fixed) through environmental degradation (multipath, low elevation,
  /// stale base) with the position drifting tens of cm even when the
  /// receiver claims sub-cm accuracy is not the danger — but its own
  /// reported NAV-COV growing past the threshold is the safe signal.
  double pos_accuracy_inflation_threshold_m_{0.025};
  double pos_accuracy_inflation_factor_{10.0};
  double pos_accuracy_reject_threshold_m_{0.500};
};

}  // namespace mowgli_localization

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
 * @file navsat_to_absolute_pose_node.cpp
 * @brief Converts sensor_msgs/NavSatFix to mowgli_interfaces/AbsolutePose.
 *
 * WGS84 → local ENU projection uses equirectangular approximation:
 *   east  = (lon - datum_lon) * cos(datum_lat) * METERS_PER_DEG
 *   north = (lat - datum_lat) * METERS_PER_DEG
 *
 * This is accurate to ~1 cm within 10 km of the datum, which is more than
 * sufficient for a garden robot mower operating within a few hundred metres.
 *
 * NavSatFix status mapping to AbsolutePose flags:
 *   STATUS_FIX              → FLAG_GPS_RTK (generic fix)
 *   STATUS_SBAS_FIX         → FLAG_GPS_RTK_FLOAT
 *   STATUS_GBAS_FIX         → FLAG_GPS_RTK_FIXED
 *   covariance_type UNKNOWN → FLAG_GPS_DEAD_RECKONING
 *
 * Universal GNSS owns the typed /gps/status contract. This node only projects
 * /gps/fix into the Mowgli-local absolute-pose topics consumed by the GUI,
 * behavior tree, and localization stack.
 */

#include "mowgli_localization/navsat_to_absolute_pose_node.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

namespace mowgli_localization
{

/// WGS84 equatorial radius in metres.
static constexpr double EARTH_RADIUS_M = 6378137.0;

/// Degrees to radians.
static constexpr double DEG_TO_RAD = M_PI / 180.0;

/// Metres per degree of latitude (approximate, at the equator).
static constexpr double METERS_PER_DEG = EARTH_RADIUS_M * DEG_TO_RAD;

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

NavSatToAbsolutePoseNode::NavSatToAbsolutePoseNode(const rclcpp::NodeOptions& options)
    : Node("navsat_to_absolute_pose", options)
{
  declare_parameters();
  cos_datum_lat_ = std::cos(datum_lat_ * DEG_TO_RAD);
  create_publishers();
  create_subscribers();
  create_services();

  // TF listener for lever-arm resolution. base_footprint→gps_link comes
  // from the URDF (static); map→base_footprint comes from ekf_map_node
  // (composed through ekf_odom_node's odom→base_footprint) and gives us
  // current yaw.
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  RCLCPP_INFO(get_logger(),
              "NavSatToAbsolutePoseNode started — datum: [%.7f, %.7f]",
              datum_lat_,
              datum_lon_);
}

// ---------------------------------------------------------------------------
// Initialisation
// ---------------------------------------------------------------------------

void NavSatToAbsolutePoseNode::declare_parameters()
{
  datum_lat_ = declare_parameter<double>("datum_lat", 0.0);
  datum_lon_ = declare_parameter<double>("datum_lon", 0.0);

  // Heading-from-COG yaw uncertainty (rad). Was a hardcoded member documented
  // as tunable; now actually a declared parameter (default 3° = 0.0524 rad).
  lever_arm_yaw_sigma_ = declare_parameter<double>("lever_arm_yaw_sigma", 0.0524);

  // Defensive guards on /gps/pose_cov — see header for rationale.
  pos_accuracy_inflation_threshold_m_ =
      declare_parameter<double>("pos_accuracy_inflation_threshold_m", 0.025);
  pos_accuracy_inflation_factor_ =
      declare_parameter<double>("pos_accuracy_inflation_factor", 10.0);
  pos_accuracy_reject_threshold_m_ =
      declare_parameter<double>("pos_accuracy_reject_threshold_m", 0.500);
}

void NavSatToAbsolutePoseNode::create_publishers()
{
  pose_pub_ =
      create_publisher<mowgli_interfaces::msg::AbsolutePose>("/gps/absolute_pose", rclcpp::QoS(10));
  // robot_localization-compatible twin: standard PoseWithCovarianceStamped
  // so ekf_map can subscribe as pose0 input. Published on every fix update
  // in on_navsat_fix alongside the AbsolutePose message.
  pose_cov_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/gps/pose_cov",
                                                                                  rclcpp::QoS(10));
}

void NavSatToAbsolutePoseNode::create_subscribers()
{
  fix_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "/gps/fix",
      rclcpp::QoS(10),
      [this](sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
      {
        on_navsat_fix(msg);
      });
}

// ---------------------------------------------------------------------------
// Callback
// ---------------------------------------------------------------------------

void NavSatToAbsolutePoseNode::create_services()
{
  set_datum_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/set_datum",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res)
      {
        on_set_datum(req, res);
      });
}

void NavSatToAbsolutePoseNode::on_set_datum(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (!has_fix_)
  {
    response->success = false;
    response->message = "No GPS fix received yet";
    return;
  }

  // Require a NavSatFix status that the current backend maps to RTK-fixed
  // quality before accepting the current position as a new datum.
  if (last_fix_.status.status < sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX)
  {
    response->success = false;
    response->message =
        "No RTK fixed quality — current status: " + std::to_string(last_fix_.status.status);
    return;
  }

  datum_lat_ = last_fix_.latitude;
  datum_lon_ = last_fix_.longitude;
  cos_datum_lat_ = std::cos(datum_lat_ * DEG_TO_RAD);

  RCLCPP_INFO(get_logger(),
              "Datum updated to [%.8f, %.8f] from current GPS position",
              datum_lat_,
              datum_lon_);

  std::ostringstream oss;
  oss << std::fixed << std::setprecision(8) << datum_lat_ << "," << datum_lon_;
  response->success = true;
  response->message = oss.str();
}

// ---------------------------------------------------------------------------
// GPS fix callback
// ---------------------------------------------------------------------------

void NavSatToAbsolutePoseNode::on_navsat_fix(sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
{
  // Store latest fix for set_datum service.
  last_fix_ = *msg;
  has_fix_ = true;

  using AbsPose = mowgli_interfaces::msg::AbsolutePose;
  using NavSat = sensor_msgs::msg::NavSatFix;
  using NavStatus = sensor_msgs::msg::NavSatStatus;

  // Discard if no fix at all.
  if (msg->status.status == NavStatus::STATUS_NO_FIX)
  {
    return;
  }

  // Project WGS84 → local ENU.
  double east = 0.0;
  double north = 0.0;
  wgs84_to_enu(msg->latitude, msg->longitude, east, north);

  // Build AbsolutePose. Stamp with the GPS fix time, not wall-clock now(), so
  // the pose isn't tagged with the GPS pipeline latency (which misassociates it
  // in time downstream). Fall back to now() only if the driver left the stamp
  // unset (sec==0 && nanosec==0).
  AbsPose out;
  out.header.stamp = (msg->header.stamp.sec != 0 || msg->header.stamp.nanosec != 0)
                         ? rclcpp::Time(msg->header.stamp)
                         : now();
  out.header.frame_id = "map";
  out.source = AbsPose::SOURCE_GPS;

  // Map NavSatFix status to AbsolutePose flags.
  switch (msg->status.status)
  {
    case NavStatus::STATUS_GBAS_FIX:
      out.flags = AbsPose::FLAG_GPS_RTK | AbsPose::FLAG_GPS_RTK_FIXED;
      break;
    case NavStatus::STATUS_SBAS_FIX:
      out.flags = AbsPose::FLAG_GPS_RTK | AbsPose::FLAG_GPS_RTK_FLOAT;
      break;
    case NavStatus::STATUS_FIX:
      out.flags = AbsPose::FLAG_GPS_RTK;
      break;
    default:
      out.flags = AbsPose::FLAG_GPS_DEAD_RECKONING;
      break;
  }

  // Position in local ENU frame. Note: ROS REP-103 map frame is
  // x=east (or forward), y=north (or left). We store east→x, north→y
  // which matches the ENU convention used by robot_localization.
  out.pose.pose.position.x = east;
  out.pose.pose.position.y = north;
  out.pose.pose.position.z = msg->altitude;

  // No orientation from GPS fix alone.
  out.pose.pose.orientation.w = 1.0;
  out.orientation_valid = 0;
  out.motion_vector_valid = 0;

  // Position accuracy from covariance diagonal (metres, 1-sigma).
  // NavSatFix covariance is [lat, lon, alt] in m² (ENU if type is known).
  if (msg->position_covariance_type != NavSat::COVARIANCE_TYPE_UNKNOWN)
  {
    // Take the mean of lat/lon variance as horizontal accuracy.
    const double lat_var = msg->position_covariance[0];
    const double lon_var = msg->position_covariance[4];
    out.position_accuracy = static_cast<float>(std::sqrt((lat_var + lon_var) / 2.0));
  }
  else
  {
    out.position_accuracy = 10.0f;  // Unknown — large default.
    out.flags = AbsPose::FLAG_GPS_DEAD_RECKONING;
  }

  // Resolve the GPS lever arm once from TF (URDF static). Retry silently
  // every fix until TF is up — no log noise if ros2 is still booting.
  if (!lever_arm_known_)
  {
    try
    {
      auto tf = tf_buffer_->lookupTransform("base_footprint", "gps_link", tf2::TimePointZero);
      lever_arm_x_ = tf.transform.translation.x;
      lever_arm_y_ = tf.transform.translation.y;
      lever_arm_known_ = true;
      RCLCPP_INFO(get_logger(),
                  "GPS lever arm resolved from TF base_footprint→gps_link: "
                  "(%.3f, %.3f) m",
                  lever_arm_x_,
                  lever_arm_y_);
    }
    catch (const tf2::TransformException&)
    {
      // Not yet available; will retry next fix.
    }
  }

  // Look up the current map→base_footprint TF for the world-frame yaw.
  //
  // The lever-arm rotation must use the SAME world frame the GPS sample
  // lives in — i.e. the map frame, anchored to GPS. Using odom yaw
  // instead drifts away from map yaw on every rotation (gyro scale
  // error accumulates in odom alone, while map yaw stays anchored by
  // /gps/pose_cov + /imu/cog_heading). After 1-2 in-place rotations the
  // odom↔map yaw delta reaches ~10° and the R(yaw_odom)·offset
  // correction starts amplifying the antenna arc instead of cancelling
  // it (empirically: /gps/pose_cov residual grew 10cm → 21cm between
  // back-to-back 360° spins on 2026-04-26, with sweep > raw antenna
  // radius).
  //
  // Using the SAME EKF's own map yaw to lever-arm-correct an observation
  // we then feed back to that EKF would naively be a feedback loop. The
  // proper Kalman-filter treatment is to propagate yaw uncertainty
  // through the (nonlinear) lever-arm Jacobian into the pose_cov
  // covariance — see the inflation block below. With that in place, the
  // EKF correctly down-weights pose_cov when its own yaw is uncertain,
  // and there is no runaway because the feedback is mediated by the
  // filter's own σ²_yaw which is bounded.
  //
  // If the TF is not yet available, fall back to publishing the raw
  // antenna position — matches legacy /gps/absolute_pose behavior.
  double base_x = east;
  double base_y = north;
  double cos_yaw = 1.0;  // captured for covariance inflation below
  double sin_yaw = 0.0;
  bool lever_arm_applied = false;
  if (lever_arm_known_)
  {
    try
    {
      auto tf = tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
      tf2::Quaternion q(tf.transform.rotation.x,
                        tf.transform.rotation.y,
                        tf.transform.rotation.z,
                        tf.transform.rotation.w);
      double roll, pitch, yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      // antenna_enu = base_enu + R(yaw) · lever_arm_body
      // → base_enu = antenna_enu - R(yaw) · lever_arm_body
      cos_yaw = std::cos(yaw);
      sin_yaw = std::sin(yaw);
      const double delta_x = cos_yaw * lever_arm_x_ - sin_yaw * lever_arm_y_;
      const double delta_y = sin_yaw * lever_arm_x_ + cos_yaw * lever_arm_y_;
      base_x = east - delta_x;
      base_y = north - delta_y;
      lever_arm_applied = true;
    }
    catch (const tf2::TransformException&)
    {
      // map→base_footprint TF not yet published; keep raw antenna pos.
    }
  }

  // Publish /gps/absolute_pose with lever-arm-corrected base_footprint
  // position so downstream consumers (cog_to_imu, GUI, BT) see the robot
  // body position, not the raw antenna position.
  out.pose.pose.position.x = base_x;
  out.pose.pose.position.y = base_y;
  pose_pub_->publish(out);

  // RTK-aware gate: skip standalone (no RTK) fixes — those have σ ~ 1-3 m
  // and jump erratically under multipath. RTK Float (σ ~ 20 cm) is fused
  // because its actual position_accuracy is known and the EKF weights it
  // appropriately; without Float updates the map-frame position freezes
  // during the long Float windows under tree cover, leaving the controller
  // to act on stale poses.
  if (msg->status.status != NavStatus::STATUS_GBAS_FIX &&
      msg->status.status != NavStatus::STATUS_SBAS_FIX)
  {
    return;
  }

  // Defensive guard on the receiver's own accuracy: even with carr_soln=2
  // (RTK Fixed) the F9P can produce false fixes during environmental
  // degradation. NAV-COV reports σ growing past nominal in those cases
  // — use that signal to either down-weight or drop the sample before
  // the EKF latches onto a bad pose.
  if (out.position_accuracy > pos_accuracy_reject_threshold_m_)
  {
    RCLCPP_WARN_THROTTLE(get_logger(),
                         *get_clock(),
                         5000,
                         "Dropping /gps/pose_cov: receiver reports σ=%.0f mm > reject threshold %.0f mm",
                         static_cast<double>(out.position_accuracy) * 1000.0,
                         pos_accuracy_reject_threshold_m_ * 1000.0);
    return;
  }

  // Standard-msg twin for robot_localization consumption. Pose is the
  // BASE FRAME position (antenna minus lever arm rotated by current yaw).
  // Covariance diagonal built from position_accuracy; frame_id=map so
  // ekf_map honors it correctly.
  geometry_msgs::msg::PoseWithCovarianceStamped twin;
  twin.header.stamp = out.header.stamp;
  twin.header.frame_id = "map";
  twin.pose.pose.position.x = base_x;
  twin.pose.pose.position.y = base_y;
  twin.pose.pose.position.z = msg->altitude;
  twin.pose.pose.orientation.w = 1.0;
  double var_x = static_cast<double>(out.position_accuracy) * out.position_accuracy;
  double var_y = var_x;
  double cov_xy = 0.0;

  // Inflate variance when receiver-reported σ exceeds the trust threshold.
  // Receiver still claims Fixed/Float but its own NAV-COV / h_acc is
  // growing past nominal — give the EKF a hint by squaring the inflation
  // factor onto the variance so kalman gain shrinks proportionally.
  if (out.position_accuracy > pos_accuracy_inflation_threshold_m_)
  {
    const double inflate_sq =
        pos_accuracy_inflation_factor_ * pos_accuracy_inflation_factor_;
    var_x *= inflate_sq;
    var_y *= inflate_sq;
    RCLCPP_DEBUG_THROTTLE(get_logger(),
                          *get_clock(),
                          2000,
                          "Inflating /gps/pose_cov ×%.0f²: receiver σ=%.1f mm > threshold %.1f mm",
                          pos_accuracy_inflation_factor_,
                          static_cast<double>(out.position_accuracy) * 1000.0,
                          pos_accuracy_inflation_threshold_m_ * 1000.0);
  }

  // Lever-arm covariance propagation — see header doc and the lookup block
  // above. The base position is base = antenna - R(ψ)·L, so its uncertainty
  // gets a contribution from σ²_ψ via the Jacobian J = ∂base/∂ψ:
  //
  //   J = [+sin(ψ)·L_x + cos(ψ)·L_y,
  //        -cos(ψ)·L_x + sin(ψ)·L_y]
  //   Σ_xy_added = J · σ²_ψ · Jᵀ (rank-1 inflation along the lever-arm sweep)
  //
  // The yaw variance used for the inflation is a static parameter
  // (lever_arm_yaw_sigma_, default 3°) — conservative and constant.
  // An earlier comment claimed the variance was sourced dynamically
  // from /odometry/filtered_map.covariance[35] (the EKF's own yaw
  // confidence), but no such subscription exists in this node — the
  // dynamic-tracking path was never wired. The static 3° σ slightly
  // under-trusts GPS during yaw-converged phases (cov inflation
  // larger than strictly necessary) but is never less safe than the
  // dynamic version would have been.
  if (lever_arm_applied)
  {
    const double Jx = sin_yaw * lever_arm_x_ + cos_yaw * lever_arm_y_;
    const double Jy = -cos_yaw * lever_arm_x_ + sin_yaw * lever_arm_y_;
    const double yaw_var = lever_arm_yaw_sigma_ * lever_arm_yaw_sigma_;
    var_x += Jx * Jx * yaw_var;
    var_y += Jy * Jy * yaw_var;
    cov_xy = Jx * Jy * yaw_var;
  }

  twin.pose.covariance[0] = var_x;  // xx
  twin.pose.covariance[1] = cov_xy;  // xy
  twin.pose.covariance[6] = cov_xy;  // yx (symmetric)
  twin.pose.covariance[7] = var_y;  // yy
  twin.pose.covariance[14] = var_x * 4.0;  // z — looser, two_d_mode ignores
  twin.pose.covariance[21] = 1.0e3;  // roll — "unknown"
  twin.pose.covariance[28] = 1.0e3;  // pitch — "unknown"
  twin.pose.covariance[35] = 1.0e3;  // yaw  — "unknown"
  pose_cov_pub_->publish(twin);
}

// ---------------------------------------------------------------------------
// WGS84 → ENU projection
// ---------------------------------------------------------------------------

void NavSatToAbsolutePoseNode::wgs84_to_enu(double lat,
                                            double lon,
                                            double& east,
                                            double& north) const
{
  east = (lon - datum_lon_) * cos_datum_lat_ * METERS_PER_DEG;
  north = (lat - datum_lat_) * METERS_PER_DEG;
}

}  // namespace mowgli_localization

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mowgli_localization::NavSatToAbsolutePoseNode>());
  rclcpp::shutdown();
  return 0;
}

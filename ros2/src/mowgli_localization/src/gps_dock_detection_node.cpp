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
 * @file gps_dock_detection_node.cpp
 * @brief Publish a GPS-derived dock "detection" so opennav_docking can
 *        approach the cradle off RTK-Fixed GPS instead of the corruptible
 *        map→odom factor-graph TF.
 *
 * ── Why this node exists ────────────────────────────────────────────────
 * On return-to-dock, fusion_graph (the GTSAM iSAM2 localizer that owns BOTH
 * map→odom AND odom→base_footprint) can come back CORRUPTED after reloading
 * its persisted graph (yaw flips, RTK overrides, a metre-scale jump). When
 * that happens the robot's map-frame TF pose is ~2 m off the true dock.
 *
 * opennav_docking (SimpleChargingDock + graceful controller, fixed_frame=odom,
 * base_frame=base_footprint) derives the robot↔dock geometry from TF. With
 * the default `use_external_detection_pose: false`, docking_server snapshots
 * map→odom EXACTLY ONCE at dockRobot() start (`transform(dock_pose, odom)`)
 * and then SimpleChargingDock::getRefinedPose() is a pure pass-through — the
 * dock target is frozen in odom at that one (possibly corrupt) snapshot. The
 * robot then spins in place / drives to the wrong spot and the action times
 * out. That single map→odom snapshot is the ONLY place graph corruption
 * enters the approach.
 *
 * Meanwhile /gps/absolute_pose is RTK-Fixed and cm-accurate at the cradle.
 *
 * ── What this node does ─────────────────────────────────────────────────
 * With `use_external_detection_pose: true`, getRefinedPose() instead reads
 * the latest message on the plugin's relative topic `detected_dock_pose`
 * (resolves to /detected_dock_pose — relative names resolve against the
 * docking_server NAMESPACE "/", not its node name), transforms it into the
 * fixed_frame (odom), filters it, and uses it as the live dock target. So if
 * we publish the TRUE dock location on that topic, the robot↔dock geometry
 * comes from US (GPS), not from the corrupt map→odom snapshot.
 *
 * Crucially we publish the detection ALREADY EXPRESSED IN `odom`: that way
 * getRefinedPose() sees detected.frame == pose.frame and SKIPS its internal
 * TF (it only transforms when frames differ). Publishing in `map` instead
 * would make opennav re-apply the corrupt map→odom and re-poison the target.
 *
 * The odom-frame dock pose is computed WITHOUT touching map→odom:
 *
 *   POSITION (yaw-free, exact):
 *     p_dock_odom = p_robot_odom + (p_dock_map − p_robot_gps)
 *   The map↔odom rotation cancels identically here: the dock's displacement
 *     from the robot is a world vector, ENU in both map and odom (both frames
 *     share the ENU axis convention; map→odom is a pure SE(2) offset), and we
 *     re-anchor it at the robot's odom position. p_robot_gps is the RTK-Fixed,
 *     lever-arm-corrected base position; p_robot_odom is the continuous DR
 *     position (TF odom→base_footprint). NO map→odom, NO yaw needed.
 *
 *   ORIENTATION (graph-independent map→odom yaw offset):
 *     yaw_dock_odom = dock_pose_yaw − Δ,   Δ = yaw_map→odom
 *   We estimate Δ from /imu/cog_heading (GPS course-over-ground yaw in map,
 *     graph-INDEPENDENT) minus the robot's odom yaw at the same instant:
 *     Δ = yaw_cog_map − yaw_robot_odom. COG is only valid while moving forward
 *     (which the robot is, on the dock approach). Before any COG sample we use
 *     the last good Δ, falling back to Δ=0 (map/odom heading aligned — true
 *     right after a clean boot/seed). This keeps the seat heading off the
 *     corruptible map→odom too, so a 180° graph yaw-flip cannot point the dock
 *     the wrong way.
 *
 * opennav re-transforms the odom-frame dock pose into base_footprint every
 * control loop via the (trusted, continuous) odom→base_footprint TF.
 *
 * ── RTK gate + Float fallback ───────────────────────────────────────────
 * Only RTK-Fixed /gps/absolute_pose samples drive a fresh detection. On
 * Float/no-fix we keep RE-PUBLISHING the LAST GOOD detection (it is anchored
 * in the continuous odom frame, so it stays geometrically valid as the robot
 * moves; opennav re-derives base-frame geometry from it each loop). We only
 * go silent if we have NEVER had a Fixed sample — then getRefinedPose() times
 * out (external_detection_timeout) and docking_server falls back to failing
 * the action, exactly as it would with no detector. The whole feature is
 * additionally behind the `use_gps_dock_detection` launch flag; when that is
 * off, `use_external_detection_pose` stays false and the legacy graph-TF
 * approach runs unchanged. This is the reversible escape hatch.
 *
 * fusion_graph keeps owning all TF — this node publishes NO TF and writes NO
 * graph state; it only publishes a PoseStamped on detected_dock_pose.
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "mowgli_interfaces/msg/absolute_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/LinearMath/Transform.hpp"
#include "tf2/LinearMath/Vector3.hpp"
#include "tf2/utils.hpp"  // tf2::getYaw
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"  // tf2::toMsg(Quaternion)
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace mowgli_localization
{

namespace
{
/// Convert an (x, y, yaw) pose to a planar tf2::Transform.
tf2::Transform make_planar_transform(double x, double y, double yaw)
{
  tf2::Transform t;
  t.setOrigin(tf2::Vector3(x, y, 0.0));
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  t.setRotation(q);
  return t;
}
}  // namespace

class GpsDockDetectionNode : public rclcpp::Node
{
public:
  explicit GpsDockDetectionNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : Node("gps_dock_detection", options)
  {
    // Calibrated dock pose in the map frame (single source of truth =
    // mowgli_robot.yaml; flowed in by navigation.launch.py).
    dock_pose_x_ = declare_parameter<double>("dock_pose_x", 0.0);
    dock_pose_y_ = declare_parameter<double>("dock_pose_y", 0.0);
    dock_pose_yaw_ = declare_parameter<double>("dock_pose_yaw", 0.0);

    // Frames. fixed_frame MUST match docking_server.fixed_frame (odom) so the
    // detection lands in the frame getRefinedPose() expects and skips its
    // internal TF. base_frame MUST match docking_server.base_frame.
    fixed_frame_ = declare_parameter<std::string>("fixed_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");

    // Republish cadence (Hz). opennav's external_detection_timeout default is
    // 1.0 s, so anything comfortably faster than 1 Hz keeps the detection
    // "fresh" between RTK-Fixed GPS samples (GPS is ~5-10 Hz).
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 10.0);

    // Require RTK-Fixed for a fresh detection. Set false to also accept Float
    // (NOT recommended here — Float is decimetre-level; the dock-localization
    // plan notes GPS may Float at some cradles, in which case the legacy
    // graph-TF path via use_gps_dock_detection:=false is the right choice).
    require_rtk_fixed_ = declare_parameter<bool>("require_rtk_fixed", true);

    detection_pub_ =
        create_publisher<geometry_msgs::msg::PoseStamped>("detected_dock_pose", rclcpp::QoS(1));

    abs_pose_sub_ = create_subscription<mowgli_interfaces::msg::AbsolutePose>(
        "/gps/absolute_pose",
        rclcpp::QoS(10),
        [this](mowgli_interfaces::msg::AbsolutePose::ConstSharedPtr msg)
        {
          on_absolute_pose(msg);
        });

    // COG yaw (GPS course-over-ground, map frame, graph-INDEPENDENT) →
    // estimate the map→odom yaw offset Δ off the corruptible graph. SensorData
    // QoS to match cog_to_imu's publisher (best-effort).
    cog_sub_ =
        create_subscription<sensor_msgs::msg::Imu>("/imu/cog_heading",
                                                   rclcpp::SensorDataQoS(),
                                                   [this](sensor_msgs::msg::Imu::ConstSharedPtr msg)
                                                   {
                                                     on_cog_heading(msg);
                                                   });

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    const auto period = std::chrono::duration<double>(1.0 / std::max(0.1, publish_rate_hz_));
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                               [this]()
                               {
                                 on_timer();
                               });

    RCLCPP_INFO(get_logger(),
                "gps_dock_detection started — dock_map=[%.3f, %.3f, %.3f] fixed=%s base=%s "
                "require_rtk_fixed=%s rate=%.1f Hz. Publishing detected_dock_pose "
                "(→ /detected_dock_pose).",
                dock_pose_x_,
                dock_pose_y_,
                dock_pose_yaw_,
                fixed_frame_.c_str(),
                base_frame_.c_str(),
                require_rtk_fixed_ ? "true" : "false",
                publish_rate_hz_);
  }

private:
  void on_absolute_pose(mowgli_interfaces::msg::AbsolutePose::ConstSharedPtr msg)
  {
    using AbsPose = mowgli_interfaces::msg::AbsolutePose;

    // RTK-Fixed gate. Float / DR / no-fix do NOT refresh the detection; we
    // keep republishing the last good one (see on_timer / class doc).
    const bool is_fixed = (msg->flags & AbsPose::FLAG_GPS_RTK_FIXED) != 0;
    if (require_rtk_fixed_ && !is_fixed)
    {
      return;
    }

    // Robot's continuous-DR pose in odom (TF odom→base_footprint). This is the
    // half of the graph that never jumps; map→odom does NOT enter here.
    tf2::Transform robot_odom;
    if (!lookup_robot_odom(robot_odom))
    {
      return;  // odom→base_footprint not available yet; try next sample.
    }

    // POSITION: re-anchor the world robot→dock displacement at the robot's odom
    // position. map→odom is NOT rotation-free — odom is zeroed at the robot's
    // start heading, so a yaw offset Δ (often tens of degrees) separates map and
    // odom. The displacement is computed in map (from GPS) and MUST be rotated by
    // −Δ into odom; otherwise, far from the dock (large displacement during the
    // approach) the target is flung to the wrong side. Δ = map_to_odom_yaw_, the
    // SAME offset the orientation below uses, so the two stay consistent.
    const double dx = dock_pose_x_ - msg->pose.pose.position.x;  // dock − robot, in map
    const double dy = dock_pose_y_ - msg->pose.pose.position.y;
    const double c = std::cos(map_to_odom_yaw_);  // R(−Δ): map displacement → odom
    const double s = std::sin(map_to_odom_yaw_);
    const double dx_odom = c * dx + s * dy;
    const double dy_odom = -s * dx + c * dy;
    const double dock_x_odom = robot_odom.getOrigin().x() + dx_odom;
    const double dock_y_odom = robot_odom.getOrigin().y() + dy_odom;

    // ORIENTATION: map dock yaw minus the (graph-independent) map→odom yaw Δ.
    // Δ comes from /imu/cog_heading (see on_cog_heading); default 0 until a COG
    // sample arrives (map/odom heading aligned right after a clean seed).
    const double dock_yaw_odom = dock_pose_yaw_ - map_to_odom_yaw_;

    last_dock_odom_ = make_planar_transform(dock_x_odom, dock_y_odom, dock_yaw_odom);
    have_detection_ = true;
  }

  // Track the map→odom yaw offset Δ from GPS course-over-ground, WITHOUT the
  // factor graph: Δ = yaw_cog_map − yaw_robot_odom, sampled at COG arrival.
  void on_cog_heading(sensor_msgs::msg::Imu::ConstSharedPtr msg)
  {
    tf2::Transform robot_odom;
    if (!lookup_robot_odom(robot_odom))
    {
      return;
    }
    tf2::Quaternion q(msg->orientation.x,
                      msg->orientation.y,
                      msg->orientation.z,
                      msg->orientation.w);
    const double yaw_cog_map = tf2::getYaw(q);
    const double yaw_robot_odom = tf2::getYaw(robot_odom.getRotation());
    // Normalize to (−π, π].
    double delta = yaw_cog_map - yaw_robot_odom;
    while (delta > M_PI)
      delta -= 2.0 * M_PI;
    while (delta <= -M_PI)
      delta += 2.0 * M_PI;
    map_to_odom_yaw_ = delta;
    if (!have_yaw_offset_)
    {
      have_yaw_offset_ = true;
      RCLCPP_INFO(get_logger(),
                  "Acquired graph-independent map→odom yaw offset Δ=%.1f° from COG; dock "
                  "detection heading is now off the factor graph.",
                  delta * 180.0 / M_PI);
    }
  }

  void on_timer()
  {
    if (!have_detection_)
    {
      // Never had a Fixed sample → publish nothing. getRefinedPose() then
      // times out and docking_server fails the action (legacy no-detector
      // behaviour). The use_gps_dock_detection launch flag is the operator's
      // escape hatch to the graph-TF path.
      return;
    }

    geometry_msgs::msg::PoseStamped out;
    out.header.stamp = now();  // fresh stamp every tick (timeout gate)
    out.header.frame_id = fixed_frame_;  // odom → getRefinedPose skips its TF
    out.pose.position.x = last_dock_odom_.getOrigin().x();
    out.pose.position.y = last_dock_odom_.getOrigin().y();
    out.pose.position.z = 0.0;
    out.pose.orientation = tf2::toMsg(last_dock_odom_.getRotation());
    detection_pub_->publish(out);
  }

  bool lookup_robot_odom(tf2::Transform& out)
  {
    geometry_msgs::msg::TransformStamped tf;
    try
    {
      tf = tf_buffer_->lookupTransform(fixed_frame_, base_frame_, tf2::TimePointZero);
    }
    catch (const tf2::TransformException&)
    {
      return false;
    }
    out.setOrigin(tf2::Vector3(tf.transform.translation.x,
                               tf.transform.translation.y,
                               tf.transform.translation.z));
    out.setRotation(tf2::Quaternion(tf.transform.rotation.x,
                                    tf.transform.rotation.y,
                                    tf.transform.rotation.z,
                                    tf.transform.rotation.w));
    return true;
  }

  // Parameters
  double dock_pose_x_{0.0};
  double dock_pose_y_{0.0};
  double dock_pose_yaw_{0.0};
  std::string fixed_frame_{"odom"};
  std::string base_frame_{"base_footprint"};
  double publish_rate_hz_{10.0};
  bool require_rtk_fixed_{true};

  // State
  tf2::Transform last_dock_odom_;
  bool have_detection_{false};
  // map→odom yaw offset Δ from COG (graph-independent). 0 until first COG.
  double map_to_odom_yaw_{0.0};
  bool have_yaw_offset_{false};

  // ROS
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr detection_pub_;
  rclcpp::Subscription<mowgli_interfaces::msg::AbsolutePose>::SharedPtr abs_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr cog_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace mowgli_localization

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mowgli_localization::GpsDockDetectionNode>());
  rclcpp::shutdown();
  return 0;
}

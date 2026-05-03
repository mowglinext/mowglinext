// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: GPL-3.0
//
// dock_yaw_to_set_pose_node.cpp
//
// C++ port of scripts/dock_yaw_to_set_pose.py — bridges /gnss/heading
// (sensor_msgs/Imu, emitted by hardware_bridge_node every 1 s while
// is_charging is true) into a set_pose on /ekf_map_node/set_pose and
// /set_pose for the robot_localization backend.
//
// Behaviour: rising edge / continuous-while-charging, 1 Hz throttle.
// Dock yaw and its sigma are read as ROS parameters (declared in
// mowgli_robot.yaml — single source of truth for dock pose).

#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "mowgli_interfaces/msg/absolute_pose.hpp"
#include "mowgli_interfaces/msg/status.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace mowgli_localization
{

class DockYawToSetPoseNode : public rclcpp::Node
{
public:
  DockYawToSetPoseNode() : Node("dock_yaw_to_set_pose")
  {
    rclcpp::QoS qos_reliable(rclcpp::KeepLast(10));
    qos_reliable.reliable();
    qos_reliable.durability_volatile();

    rclcpp::QoS qos_sensor(rclcpp::KeepLast(10));
    qos_sensor.best_effort();
    qos_sensor.durability_volatile();

    // The set_pose seed fires once (rising edge of is_charging) and the
    // map-frame localizer has to receive it to bootstrap. With VOLATILE
    // pub QoS, a subscriber that hasn't finished discovery loses the
    // message — observed on 2026-05-03 after a force-recreate when
    // fusion_graph_node started later than dock_yaw_to_set_pose. Use
    // TRANSIENT_LOCAL with depth-1 so a late-joining subscriber that
    // also opts in to TRANSIENT_LOCAL gets the last seed automatically.
    // Subscribers with default VOLATILE QoS keep receiving real-time
    // publishes as before (TL pub ↔ VOLATILE sub is compatible per DDS).
    rclcpp::QoS qos_seed(rclcpp::KeepLast(1));
    qos_seed.reliable();
    qos_seed.transient_local();

    sub_status_ = create_subscription<mowgli_interfaces::msg::Status>(
        "/hardware_bridge/status",
        qos_reliable,
        [this](mowgli_interfaces::msg::Status::ConstSharedPtr msg)
        {
          on_status(*msg);
        });
    sub_heading_ =
        create_subscription<sensor_msgs::msg::Imu>("/gnss/heading",
                                                   qos_reliable,
                                                   [this](sensor_msgs::msg::Imu::ConstSharedPtr msg)
                                                   {
                                                     on_heading(msg);
                                                   });
    sub_gps_ = create_subscription<mowgli_interfaces::msg::AbsolutePose>(
        "/gps/absolute_pose",
        qos_sensor,
        [this](mowgli_interfaces::msg::AbsolutePose::ConstSharedPtr msg)
        {
          on_gps(msg);
        });

    pub_map_ =
        create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/ekf_map_node/set_pose",
                                                                        qos_seed);
    pub_odom_ =
        create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/set_pose", qos_seed);
    // Dual-publish to fusion_graph_node: works regardless of which
    // localizer is the active map-frame primary. The unused publisher
    // costs ~zero (no subscribers).
    pub_fg_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/fusion_graph_node/set_pose", qos_seed);

    yaw_var_ = declare_parameter<double>("seed_yaw_variance", 0.1);

    // Dock yaw + sigma come from mowgli_robot.yaml via the launch wrapper.
    // Calibration (calibrate_imu_yaw_node) and manual GUI overrides
    // (map_server_node /set_docking_point) write back to that file, so
    // the param value is always the latest persisted dock yaw.
    const double dock_yaw_rad = declare_parameter<double>("dock_pose_yaw", 0.0);
    const double dock_yaw_sigma_rad = declare_parameter<double>("dock_pose_yaw_sigma_rad", 0.035);
    cfg_yaw_rad_ = dock_yaw_rad;
    // Floor at variance 0.03 (~10°) so a tiny configured sigma does not
    // pin the EKF too hard on the first seed.
    cfg_yaw_var_ = std::max(dock_yaw_sigma_rad * dock_yaw_sigma_rad, 0.03);
    RCLCPP_INFO(get_logger(),
                "dock_yaw_to_set_pose started — yaw=%.2f° (σ=%.2f°) from "
                "mowgli_robot.yaml. Waits for rising edge of is_charging.",
                dock_yaw_rad * 180.0 / M_PI,
                dock_yaw_sigma_rad * 180.0 / M_PI);
  }

private:
  void on_heading(sensor_msgs::msg::Imu::ConstSharedPtr msg)
  {
    latest_heading_ = msg;
    if (need_to_publish_)
      try_publish();
  }

  void on_gps(mowgli_interfaces::msg::AbsolutePose::ConstSharedPtr msg)
  {
    latest_gps_ = msg;
    if (need_to_publish_)
      try_publish();
  }

  void on_status(const mowgli_interfaces::msg::Status& msg)
  {
    const bool is_charging = msg.is_charging;
    bool fire = false;

    if (!last_is_charging_known_ && is_charging)
    {
      RCLCPP_INFO(get_logger(), "boot detected docked state → seeding pose once");
      fire = true;
    }
    else if (is_charging && last_is_charging_known_ && !last_is_charging_)
    {
      RCLCPP_INFO(get_logger(), "charging rising edge → seeding pose once");
      fire = true;
    }
    else if (last_is_charging_known_ && last_is_charging_ && !is_charging)
    {
      RCLCPP_INFO(get_logger(), "charging dropped → seed no longer asserted");
    }

    last_is_charging_ = is_charging;
    last_is_charging_known_ = true;

    // Edge-only seed: fire once on boot-while-docked or on dock arrival,
    // then stay silent until the robot leaves and returns. Continuous
    // 1 Hz publishing was breaking fusion_graph_node — every set_pose
    // adds a tight PriorFactor on the latest graph node, so a 1 Hz
    // stream pinned the pose at the dock and prevented the graph from
    // tracking actual motion. ekf_map_node tolerated the continuous
    // stream (state reset semantics) but didn't need it either.
    if (fire)
    {
      need_to_publish_ = true;
      try_publish();
    }
  }

  void try_publish()
  {
    if (!latest_gps_)
      return;

    const auto now_mono = std::chrono::steady_clock::now();
    const double now_s = std::chrono::duration<double>(now_mono.time_since_epoch()).count();
    if (now_s - last_publish_time_ < min_publish_period_)
      return;
    last_publish_time_ = now_s;

    geometry_msgs::msg::Quaternion yaw_quat;
    yaw_quat.w = std::cos(cfg_yaw_rad_ / 2.0);
    yaw_quat.z = std::sin(cfg_yaw_rad_ / 2.0);
    const double yaw_var = cfg_yaw_var_;

    std::array<double, 36> cov{};
    cov[0] = 0.01;
    cov[7] = 0.01;
    cov[14] = 1e6;
    cov[21] = 1e6;
    cov[28] = 1e6;
    cov[35] = yaw_var;

    geometry_msgs::msg::PoseWithCovarianceStamped map_seed;
    map_seed.header.stamp = now();
    map_seed.header.frame_id = "map";
    map_seed.pose.pose.position.x = latest_gps_->pose.pose.position.x;
    map_seed.pose.pose.position.y = latest_gps_->pose.pose.position.y;
    map_seed.pose.pose.orientation = yaw_quat;
    map_seed.pose.covariance = cov;
    pub_map_->publish(map_seed);
    pub_fg_->publish(map_seed);  // same payload, fusion_graph reads it

    geometry_msgs::msg::PoseWithCovarianceStamped odom_seed;
    odom_seed.header.stamp = map_seed.header.stamp;
    odom_seed.header.frame_id = "odom";
    odom_seed.pose.pose.position.x = 0.0;
    odom_seed.pose.pose.position.y = 0.0;
    odom_seed.pose.pose.orientation = yaw_quat;
    odom_seed.pose.covariance = cov;
    pub_odom_->publish(odom_seed);

    need_to_publish_ = false;

    const double yaw =
        std::atan2(2.0 * yaw_quat.w * yaw_quat.z, 1.0 - 2.0 * yaw_quat.z * yaw_quat.z);
    RCLCPP_INFO(get_logger(),
                "published dock set_pose: map=(%.3f, %.3f) yaw=%.1f°, "
                "odom=(0, 0) yaw=%.1f°",
                map_seed.pose.pose.position.x,
                map_seed.pose.pose.position.y,
                yaw * 180.0 / M_PI,
                yaw * 180.0 / M_PI);
  }

  rclcpp::Subscription<mowgli_interfaces::msg::Status>::SharedPtr sub_status_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_heading_;
  rclcpp::Subscription<mowgli_interfaces::msg::AbsolutePose>::SharedPtr sub_gps_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_map_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_odom_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_fg_;

  bool last_is_charging_{false};
  bool last_is_charging_known_{false};
  sensor_msgs::msg::Imu::ConstSharedPtr latest_heading_;
  mowgli_interfaces::msg::AbsolutePose::ConstSharedPtr latest_gps_;
  bool need_to_publish_{false};
  double last_publish_time_{0.0};
  double min_publish_period_{1.0};
  double yaw_var_{0.1};

  double cfg_yaw_rad_{0.0};
  double cfg_yaw_var_{0.03};
};

}  // namespace mowgli_localization

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mowgli_localization::DockYawToSetPoseNode>());
  rclcpp::shutdown();
  return 0;
}

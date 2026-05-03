// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: GPL-3.0
//
// costmap_scan_filter_node.cpp
//
// Conditional LiDAR range filter for the local_costmap obstacle_layer.
//
// The local_costmap historically blanked all returns under 0.65 m via
// `obstacle_min_range` so the dock body (which sits ~45 cm in front of
// base_footprint when docked) wouldn't be marked LETHAL — the BackUp
// recovery's collision checker reads local_costmap/costmap_raw, so a
// dock-as-obstacle aborted undock with "Collision Ahead" (error 714).
//
// The side effect was a 0.10–0.65 m blind ring around the robot during
// mowing: collision_monitor (which polls /scan directly) would zero
// cmd_vel on a contact, but the local_costmap had no idea anything was
// there, so FTCController kept commanding forward and Nav2 never
// triggered a recovery. A 414 s PolygonStop wedge was observed on
// 2026-05-03.
//
// Fix: rewrite the radial blanking as a stateful filter on a separate
// topic (/scan_costmap). Returns within `dock_blank_range` are pushed
// to +inf (so obstacle_layer treats them as "no return") only while
// the robot is charging or for `post_undock_blank_sec` seconds after
// charging drops — exactly the window during which the dock might be
// in front of the LiDAR. Otherwise the scan passes through untouched
// and obstacle_min_range can be lowered to 0.10 m, closing the blind
// ring during normal mowing.
//
// collision_monitor still subscribes to /scan unfiltered.

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "mowgli_interfaces/msg/status.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace mowgli_localization
{

class CostmapScanFilterNode : public rclcpp::Node
{
public:
  CostmapScanFilterNode() : Node("costmap_scan_filter")
  {
    dock_blank_range_ = declare_parameter<double>("dock_blank_range", 0.70);
    post_undock_blank_sec_ = declare_parameter<double>("post_undock_blank_sec", 5.0);
    const std::string input_topic = declare_parameter<std::string>("input_topic", "/scan");
    const std::string output_topic =
        declare_parameter<std::string>("output_topic", "/scan_costmap");
    const std::string status_topic =
        declare_parameter<std::string>("status_topic", "/hardware_bridge/status");

    rclcpp::QoS qos_sensor = rclcpp::SensorDataQoS();
    rclcpp::QoS qos_reliable(rclcpp::KeepLast(10));
    qos_reliable.reliable();
    qos_reliable.durability_volatile();

    pub_scan_ = create_publisher<sensor_msgs::msg::LaserScan>(output_topic, qos_sensor);

    sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
        input_topic,
        qos_sensor,
        [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) { on_scan(*msg); });

    sub_status_ = create_subscription<mowgli_interfaces::msg::Status>(
        status_topic,
        qos_reliable,
        [this](mowgli_interfaces::msg::Status::ConstSharedPtr msg) { on_status(*msg); });

    RCLCPP_INFO(get_logger(),
                "costmap_scan_filter started — %s -> %s, dock_blank_range=%.2f m, "
                "post_undock_blank_sec=%.1f s. Filter active while is_charging "
                "or for post_undock_blank_sec after charging drops.",
                input_topic.c_str(),
                output_topic.c_str(),
                dock_blank_range_,
                post_undock_blank_sec_);
  }

  // --- Pure logic exposed for unit tests ---------------------------------

  /// Apply the radial blank to a copy of @p in. Returns the result.
  /// `blank_active` is the cached output of `is_blank_active()` — passed
  /// in so the test can drive the state machine without a clock.
  static sensor_msgs::msg::LaserScan filter_scan(const sensor_msgs::msg::LaserScan& in,
                                                 double dock_blank_range,
                                                 bool blank_active)
  {
    sensor_msgs::msg::LaserScan out = in;
    if (!blank_active)
      return out;
    const float threshold = static_cast<float>(dock_blank_range);
    const float inf = std::numeric_limits<float>::infinity();
    for (auto& r : out.ranges)
    {
      if (std::isfinite(r) && r < threshold)
        r = inf;
    }
    return out;
  }

private:
  void on_status(const mowgli_interfaces::msg::Status& msg)
  {
    const bool is_charging = msg.is_charging;
    if (last_is_charging_known_ && last_is_charging_ && !is_charging)
    {
      // Falling edge — start the post-undock grace window.
      charging_dropped_at_ = now();
      RCLCPP_INFO(get_logger(),
                  "charging dropped — extending dock-blank for %.1f s",
                  post_undock_blank_sec_);
    }
    last_is_charging_ = is_charging;
    last_is_charging_known_ = true;
  }

  void on_scan(const sensor_msgs::msg::LaserScan& msg)
  {
    pub_scan_->publish(filter_scan(msg, dock_blank_range_, is_blank_active()));
  }

  bool is_blank_active() const
  {
    if (!last_is_charging_known_)
    {
      // Be conservative until we've heard from hardware_bridge: keep the
      // dock blank in case we're booting docked.
      return true;
    }
    if (last_is_charging_)
      return true;
    if (charging_dropped_at_.nanoseconds() == 0)
      return false;
    const double since_drop = (now() - charging_dropped_at_).seconds();
    return since_drop >= 0.0 && since_drop < post_undock_blank_sec_;
  }

  // --- Subscriptions / publishers ---------------------------------------

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
  rclcpp::Subscription<mowgli_interfaces::msg::Status>::SharedPtr sub_status_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_scan_;

  // --- Parameters --------------------------------------------------------

  double dock_blank_range_{0.70};
  double post_undock_blank_sec_{5.0};

  // --- Charging-state machine -------------------------------------------

  bool last_is_charging_{false};
  bool last_is_charging_known_{false};
  rclcpp::Time charging_dropped_at_{0, 0, RCL_ROS_TIME};
};

}  // namespace mowgli_localization

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mowgli_localization::CostmapScanFilterNode>());
  rclcpp::shutdown();
  return 0;
}

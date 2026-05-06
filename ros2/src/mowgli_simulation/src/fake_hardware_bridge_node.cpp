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

/**
 * @file fake_hardware_bridge_node.cpp
 * @brief Fake hardware bridge for simulation.
 *
 * Provides stub services and topics that the behavior tree expects from the
 * real hardware_bridge_node, so simulation runs without "service unavailable"
 * warnings.
 *
 * Services:
 *   - /hardware_bridge/mower_control (MowerControl) — always succeeds
 *
 * Publishers:
 *   - /hardware_bridge/status   (mowgli_interfaces/Status)   — simulated idle
 *   - /hardware_bridge/power    (mowgli_interfaces/Power)     — simulated full battery
 *   - /hardware_bridge/emergency (mowgli_interfaces/Emergency) — no emergency
 */

#include <cmath>
#include <memory>
#include <mutex>

#include "mowgli_interfaces/msg/emergency.hpp"
#include "mowgli_interfaces/msg/power.hpp"
#include "mowgli_interfaces/msg/status.hpp"
#include "mowgli_interfaces/srv/emergency_stop.hpp"
#include "mowgli_interfaces/srv/mower_control.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/imu.hpp"

class FakeHardwareBridgeNode : public rclcpp::Node
{
public:
  FakeHardwareBridgeNode() : Node("fake_hardware_bridge")
  {
    // Service: mower_control
    mower_control_srv_ = create_service<mowgli_interfaces::srv::MowerControl>(
        "/hardware_bridge/mower_control",
        [this](const std::shared_ptr<mowgli_interfaces::srv::MowerControl::Request> req,
               std::shared_ptr<mowgli_interfaces::srv::MowerControl::Response> res)
        {
          mow_enabled_ = (req->mow_enabled != 0);
          res->success = true;
          RCLCPP_INFO(get_logger(), "Fake mower_control: mow_enabled=%u", req->mow_enabled);
        });

    // Service: emergency_stop
    emergency_stop_srv_ = create_service<mowgli_interfaces::srv::EmergencyStop>(
        "/hardware_bridge/emergency_stop",
        [this](const std::shared_ptr<mowgli_interfaces::srv::EmergencyStop::Request> req,
               std::shared_ptr<mowgli_interfaces::srv::EmergencyStop::Response> res)
        {
          if (req->emergency != 0u)
          {
            RCLCPP_INFO(get_logger(), "Fake emergency_stop: emergency activated");
            sim_emergency_active_ = true;
          }
          else
          {
            RCLCPP_INFO(get_logger(), "Fake emergency_stop: emergency released");
            sim_emergency_active_ = false;
          }
          res->success = true;
        });

    // Dock position (origin) and proximity threshold
    declare_parameter<double>("dock_x", 0.0);
    declare_parameter<double>("dock_y", 0.0);
    declare_parameter<double>("dock_pose_yaw", 0.0);
    declare_parameter<double>("dock_proximity", 0.3);
    dock_x_ = get_parameter("dock_x").as_double();
    dock_y_ = get_parameter("dock_y").as_double();
    dock_yaw_ = get_parameter("dock_pose_yaw").as_double();
    dock_proximity_ = get_parameter("dock_proximity").as_double();

    // Publishers
    status_pub_ = create_publisher<mowgli_interfaces::msg::Status>("/hardware_bridge/status",
                                                                   rclcpp::QoS(10));
    power_pub_ =
        create_publisher<mowgli_interfaces::msg::Power>("/hardware_bridge/power", rclcpp::QoS(10));
    emergency_pub_ =
        create_publisher<mowgli_interfaces::msg::Emergency>("/hardware_bridge/emergency",
                                                            rclcpp::QoS(10));
    battery_state_pub_ =
        create_publisher<sensor_msgs::msg::BatteryState>("/battery_state", rclcpp::QoS(10));
    // Real hardware_bridge publishes ~/dock_heading at 1 Hz while charging
    // (remapped via mowgli.launch.py to /gnss/heading). Mirror that here so
    // calibrate_imu_yaw_node and dock_yaw_to_set_pose see the same data
    // path in sim as on real hardware.
    dock_heading_pub_ = create_publisher<sensor_msgs::msg::Imu>(
        "/gnss/heading", rclcpp::QoS(10));

    // Subscribe to map-frame pose so the dock-proximity test compares like-
    // for-like with dock_x/dock_y (which are map-frame coordinates). The
    // earlier subscription to /wheel_odom (odom frame) silently broke after
    // CalibrateHeadingFromUndock re-seeded the map→odom transform: the
    // robot's odom-frame position no longer matched its map-frame position,
    // so near_dock stayed false even when the robot was physically docked,
    // opennav_docking timed out waiting for charge to start, the BT halted
    // DockRobot, BoundaryGuard's BackUp recovery shoved the robot south by
    // 0.5 m, and the cycle repeated until the robot ended up outside the
    // polygon and stuck in "Start occupied".
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered_map",
        rclcpp::SensorDataQoS(),
        [this](const nav_msgs::msg::Odometry::SharedPtr msg)
        {
          std::lock_guard<std::mutex> lock(odom_mutex_);
          robot_x_ = msg->pose.pose.position.x;
          robot_y_ = msg->pose.pose.position.y;
          odom_received_ = true;
        });

    // Publish at 10 Hz to match the real bridge's effective Status emission
    // rate (firmware aggregates LL packets at ~47 Hz, the BT consumes
    // Status at its tick_rate=10 Hz). 1 Hz was too slow — the rising
    // edge of is_charging (which gates the EKF dock-yaw seed,
    // costmap_scan_filter, calibrate_imu_yaw, etc.) was detected with
    // up to 1 s latency, racing other startup nodes.
    timer_ = create_wall_timer(std::chrono::milliseconds(100),
                               [this]()
                               {
                                 publish_fake_data();
                               });

    RCLCPP_INFO(get_logger(), "Fake hardware bridge started (simulation mode)");
  }

private:
  void publish_fake_data()
  {
    auto now = this->now();

    // Determine if robot is near the dock
    bool near_dock = false;
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      if (odom_received_)
      {
        double dx = robot_x_ - dock_x_;
        double dy = robot_y_ - dock_y_;
        near_dock = std::sqrt(dx * dx + dy * dy) < dock_proximity_;
      }
      else
      {
        // Before first odom, assume docked (robot starts at dock)
        near_dock = true;
      }
    }

    mowgli_interfaces::msg::Status status;
    status.stamp = now;
    // Field parity with the real hardware_bridge_node so downstream
    // diagnostics, MQTT bridge, and BT logic see the same picture in sim.
    status.mower_status = mowgli_interfaces::msg::Status::MOWER_STATUS_OK;
    status.raspberry_pi_power = true;
    status.esc_power = !sim_emergency_active_;
    status.rain_detected = false;
    status.sound_module_available = false;
    status.sound_module_busy = false;
    status.ui_board_available = true;
    status.mow_enabled = mow_enabled_;
    status.mower_esc_status = 0;
    status.mower_esc_temperature = 25.0f;
    status.mower_esc_current = mow_enabled_ ? 0.5f : 0.0f;
    status.mower_motor_temperature = mow_enabled_ ? 30.0f : 25.0f;
    status.mower_motor_rpm = mow_enabled_ ? 3000.0f : 0.0f;
    // is_charging mirrors near_dock — opennav_docking, dock_yaw_to_set_pose,
    // BoundaryGuard, calibrate_imu_yaw, and the BT all gate on this.
    status.is_charging = near_dock;
    status_pub_->publish(status);

    // Dock heading: synthetic Imu with quaternion encoding dock_pose_yaw.
    // The real hardware_bridge publishes this at 1 Hz while charging from
    // the GPS COG snapshot at the moment of docking. We can publish it
    // continuously while near_dock — consumers (calibrate_imu_yaw,
    // dock_yaw_to_set_pose) only read while charging anyway.
    if (near_dock)
    {
      sensor_msgs::msg::Imu heading;
      heading.header.stamp = now;
      heading.header.frame_id = "base_link";
      heading.orientation.w = std::cos(dock_yaw_ / 2.0);
      heading.orientation.x = 0.0;
      heading.orientation.y = 0.0;
      heading.orientation.z = std::sin(dock_yaw_ / 2.0);
      heading.orientation_covariance[0] = -1.0;  // signal: only orientation valid
      heading.orientation_covariance[8] = 0.001;
      dock_heading_pub_->publish(heading);
    }

    mowgli_interfaces::msg::Power power;
    power.stamp = now;
    power.v_battery = 28.0;
    power.v_charge = near_dock ? 28.5 : 0.0;
    power.charge_current = near_dock ? 1.5 : 0.0;
    power.charger_enabled = near_dock;
    power_pub_->publish(power);

    // BatteryState for opennav_docking charge detection
    sensor_msgs::msg::BatteryState battery;
    battery.header.stamp = now;
    battery.header.frame_id = "base_link";
    battery.voltage = 28.0f;
    battery.current = near_dock ? 1.5f : 0.0f;
    battery.percentage = 1.0f;
    battery.power_supply_status =
        near_dock ? sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING
                  : sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING;
    battery.present = true;
    battery_state_pub_->publish(battery);

    mowgli_interfaces::msg::Emergency emergency;
    emergency.stamp = now;
    emergency.active_emergency = sim_emergency_active_;
    emergency.latched_emergency = sim_emergency_active_;
    emergency_pub_->publish(emergency);
  }

  bool mow_enabled_{false};
  bool sim_emergency_active_{false};
  rclcpp::Service<mowgli_interfaces::srv::MowerControl>::SharedPtr mower_control_srv_;
  rclcpp::Service<mowgli_interfaces::srv::EmergencyStop>::SharedPtr emergency_stop_srv_;
  rclcpp::Publisher<mowgli_interfaces::msg::Status>::SharedPtr status_pub_;
  rclcpp::Publisher<mowgli_interfaces::msg::Power>::SharedPtr power_pub_;
  rclcpp::Publisher<mowgli_interfaces::msg::Emergency>::SharedPtr emergency_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr dock_heading_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Robot position from odometry
  std::mutex odom_mutex_;
  double robot_x_ = 0.0;
  double robot_y_ = 0.0;
  bool odom_received_ = false;

  // Dock position and proximity threshold
  double dock_x_ = 0.0;
  double dock_y_ = 0.0;
  double dock_yaw_ = 0.0;
  double dock_proximity_ = 0.3;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeHardwareBridgeNode>());
  rclcpp::shutdown();
  return 0;
}

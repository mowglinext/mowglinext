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
    declare_parameter<double>("dock_proximity", 0.3);
    dock_x_ = get_parameter("dock_x").as_double();
    dock_y_ = get_parameter("dock_y").as_double();
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

    // Publish at 1 Hz
    timer_ = create_wall_timer(std::chrono::seconds(1),
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
    status.mow_enabled = mow_enabled_;
    // is_charging mirrors near_dock — opennav_docking, dock_yaw_to_set_pose,
    // BoundaryGuard, calibrate_imu_yaw, and the BT all gate on this.
    // Was unset (default false) before, which silently broke the EKF
    // map-yaw seed: dock_yaw_to_set_pose only fires on rising edge of
    // is_charging, but with is_charging never going true the seed never
    // fired, EKF stayed at yaw=0 instead of dock_pose_yaw=4.17, the GPS
    // lever-arm correction got the wrong rotation, and /odometry/filtered_map
    // reported base_link 0.45 m off Gazebo truth. BoundaryGuard then
    // false-positive-tripped before any coverage iteration could start.
    status.is_charging = near_dock;
    status_pub_->publish(status);

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
  double dock_proximity_ = 0.3;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeHardwareBridgeNode>());
  rclcpp::shutdown();
  return 0;
}

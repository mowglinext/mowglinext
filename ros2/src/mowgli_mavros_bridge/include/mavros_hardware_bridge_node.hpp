#pragma once

#include <mutex>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <mavros_msgs/msg/manual_control.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mowgli_interfaces/msg/emergency.hpp>
#include <mowgli_interfaces/msg/high_level_status.hpp>
#include <mowgli_interfaces/msg/power.hpp>
#include <mowgli_interfaces/msg/status.hpp>
#include <mowgli_interfaces/srv/emergency_stop.hpp>
#include <mowgli_interfaces/srv/mower_control.hpp>

namespace mowgli_mavros_bridge
{

class MavrosHardwareBridgeNode : public rclcpp::Node
{
public:
  explicit MavrosHardwareBridgeNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  void create_publishers();
  void create_subscriptions();
  void create_services();
  void create_timers();
  void create_clients();

  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg);
  void on_high_level_status(const mowgli_interfaces::msg::HighLevelStatus::SharedPtr msg);

  void on_mavros_state(const mavros_msgs::msg::State::SharedPtr msg);
  void on_mavros_imu(const sensor_msgs::msg::Imu::SharedPtr msg);
  void on_mavros_battery(const sensor_msgs::msg::BatteryState::SharedPtr msg);
  void on_mavros_odom(const nav_msgs::msg::Odometry::SharedPtr msg);

  void on_mower_control(
      const std::shared_ptr<mowgli_interfaces::srv::MowerControl::Request> request,
      std::shared_ptr<mowgli_interfaces::srv::MowerControl::Response> response);

  void on_emergency_stop(
      const std::shared_ptr<mowgli_interfaces::srv::EmergencyStop::Request> request,
      std::shared_ptr<mowgli_interfaces::srv::EmergencyStop::Response> response);

  void publish_status();
  void publish_emergency();
  void publish_power();

  bool send_arm_command(bool arm);
  bool send_mode_command(const std::string& mode);

private:
  std::mutex mutex_;

  double status_publish_rate_hz_{10.0};
  bool rain_detected_{false};
  bool esc_power_{true};
  bool raspberry_pi_power_{true};
  bool sound_module_available_{false};
  bool sound_module_busy_{false};
  bool ui_board_available_{false};

  bool mow_enabled_{false};
  uint8_t mow_direction_{0};

  bool emergency_active_{false};
  bool emergency_latched_{false};
  std::string emergency_reason_{"NONE"};

  bool is_charging_{false};
  bool charger_enabled_{false};
  std::string charger_status_{"unknown"};

  double battery_voltage_{0.0};
  double charge_voltage_{0.0};
  double charge_current_{0.0};

  uint8_t mower_esc_status_{0};
  float mower_esc_temperature_{0.0F};
  float mower_esc_current_{0.0F};
  float mower_motor_temperature_{0.0F};
  float mower_motor_rpm_{0.0F};

  mavros_msgs::msg::State mavros_state_{};
  sensor_msgs::msg::Imu last_imu_{};
  nav_msgs::msg::Odometry last_odom_{};
  sensor_msgs::msg::BatteryState last_battery_{};
  mowgli_interfaces::msg::HighLevelStatus last_high_level_status_{};

  rclcpp::Publisher<mowgli_interfaces::msg::Status>::SharedPtr pub_status_;
  rclcpp::Publisher<mowgli_interfaces::msg::Emergency>::SharedPtr pub_emergency_;
  rclcpp::Publisher<mowgli_interfaces::msg::Power>::SharedPtr pub_power_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_wheel_odom_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr pub_battery_state_;
  rclcpp::Publisher<mavros_msgs::msg::ManualControl>::SharedPtr pub_manual_control_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  rclcpp::Subscription<mowgli_interfaces::msg::HighLevelStatus>::SharedPtr sub_hl_status_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr sub_mavros_state_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_mavros_imu_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr sub_mavros_battery_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_mavros_odom_;

  rclcpp::Service<mowgli_interfaces::srv::MowerControl>::SharedPtr srv_mower_control_;
  rclcpp::Service<mowgli_interfaces::srv::EmergencyStop>::SharedPtr srv_emergency_stop_;

  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr cli_arm_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr cli_set_mode_;

  rclcpp::TimerBase::SharedPtr timer_status_;
};

}  // namespace mowgli_mavros_bridge
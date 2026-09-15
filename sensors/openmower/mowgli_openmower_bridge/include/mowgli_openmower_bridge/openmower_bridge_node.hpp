// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0
/**
 * @file openmower_bridge_node.hpp
 * @brief ROS2 node bridging stock OpenMower (v1) electronics to the
 *        MowgliNext hardware_bridge contract.
 *
 * Node name is `hardware_bridge` on purpose: every consumer in the stack
 * (behaviour tree, GUI, MQTT, fusion_graph, Nav2 docking) resolves
 * `/hardware_bridge/...` topics and services, and the launch file applies the
 * same remaps as mowgli.launch.py does for the STM32 bridge.
 *
 * Three serial links, all polled from timers (no threads):
 *   - LowLevel (Pico) board: COBS frames — status, IMU, UI buttons, config;
 *     we send heartbeat, high-level state and the config packet.
 *   - left / right drive xESC: duty out, tachometer + telemetry in.
 *   - mow xESC (optional): blade duty out, rpm / current / temperature in.
 */

#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <std_msgs/msg/bool.hpp>

#include "mowgli_hardware/clock_fit.hpp"
#include "mowgli_hardware/ll_datatypes.hpp"
#include "mowgli_hardware/packet_handler.hpp"
#include "mowgli_hardware/serial_port.hpp"
#include "mowgli_interfaces/msg/emergency.hpp"
#include "mowgli_interfaces/msg/gnss_status.hpp"
#include "mowgli_interfaces/msg/high_level_status.hpp"
#include "mowgli_interfaces/msg/power.hpp"
#include "mowgli_interfaces/msg/status.hpp"
#include "mowgli_interfaces/msg/wheel_tick.hpp"
#include "mowgli_interfaces/srv/emergency_stop.hpp"
#include "mowgli_interfaces/srv/high_level_control.hpp"
#include "mowgli_interfaces/srv/mower_control.hpp"
#include "mowgli_openmower_bridge/blade_policy.hpp"
#include "mowgli_openmower_bridge/emergency_tracker.hpp"
#include "mowgli_openmower_bridge/imu_bias_estimator.hpp"
#include "mowgli_openmower_bridge/lowlevel_config.hpp"
#include "mowgli_openmower_bridge/motor_link.hpp"
#include "mowgli_openmower_bridge/wheel_velocity_controller.hpp"
#include "mowgli_openmower_bridge/xesc_odometry.hpp"
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace mowgli_openmower_bridge
{

class OpenMowerBridgeNode : public rclcpp::Node
{
public:
  OpenMowerBridgeNode();
  ~OpenMowerBridgeNode() override;

private:
  // ---- setup (openmower_bridge_node.cpp) ----
  void declare_parameters();
  void create_interfaces();
  void create_motor_links();
  void create_timers();
  void on_cmd_vel(geometry_msgs::msg::TwistStamped::ConstSharedPtr msg);
  void on_high_level_status(mowgli_interfaces::msg::HighLevelStatus::ConstSharedPtr msg);
  void on_gnss_status(mowgli_interfaces::msg::GnssStatus::ConstSharedPtr msg);
  void on_mower_control(const std::shared_ptr<mowgli_interfaces::srv::MowerControl::Request> req,
                        std::shared_ptr<mowgli_interfaces::srv::MowerControl::Response> res);
  void on_emergency_stop(const std::shared_ptr<mowgli_interfaces::srv::EmergencyStop::Request> req,
                         std::shared_ptr<mowgli_interfaces::srv::EmergencyStop::Response> res);

  // ---- LowLevel board link (openmower_bridge_lowlevel.cpp) ----
  void lowlevel_read_tick();
  bool lowlevel_send(const uint8_t* payload, std::size_t len);
  void on_lowlevel_packet(const uint8_t* data, std::size_t len);
  void handle_ll_status(const uint8_t* data, std::size_t len);
  void handle_ll_imu(const uint8_t* data, std::size_t len);
  void handle_ll_ui_event(const uint8_t* data, std::size_t len);
  void handle_ll_config(const uint8_t* data, std::size_t len);
  void send_heartbeat();
  void send_high_level_state();
  void service_config_handshake();
  void publish_status_bundle(const mowgli_hardware::LlStatus& pkt);
  [[nodiscard]] bool lowlevel_alive() const;

  // ---- actuation + odometry (openmower_bridge_actuation.cpp) ----
  void control_tick();
  void update_odometry(double dt_s);
  void drive_wheels(double dt_s, bool wheels_allowed);
  void drive_blade(bool blade_allowed);
  void publish_cmd_vel_applied(double vx, double wz);
  [[nodiscard]] double motor_sign(std::size_t motor) const;
  [[nodiscard]] bool drive_links_connected() const;
  [[nodiscard]] std::string motor_summary() const;

  // ---- parameters ----
  std::string ll_serial_port_;
  int ll_baud_rate_{115200};
  double ll_rx_timeout_s_{2.0};
  double ll_read_rate_hz_{100.0};
  double control_rate_hz_{50.0};
  double heartbeat_rate_hz_{25.0};
  double high_level_rate_hz_{2.0};
  std::string xesc_type_;
  std::array<std::string, 3> xesc_ports_{};
  bool mow_xesc_enabled_{true};
  bool left_motor_inverted_{false};
  bool right_motor_inverted_{true};
  int motor_pole_pairs_{4};
  double ticks_per_meter_{1600.0};
  double wheel_track_{0.325};
  double max_mps_{0.5};
  double max_radps_{2.0};
  double min_linear_vel_{0.05};
  double cmd_vel_timeout_s_{1.0};
  double high_level_status_timeout_s_{5.0};
  double lin_accel_limit_{0.30};
  double lin_decel_limit_{0.60};
  double ang_accel_limit_{1.0};
  double ang_decel_limit_{2.0};
  WheelLoopGains wheel_gains_{};
  double blade_duty_{1.0};
  bool mowing_enabled_{true};
  std::string imu_frame_id_{"imu_link"};
  int imu_cal_samples_{200};
  double imu_cal_auto_rest_s_{15.0};
  lowlevel::HighLevelConfig ll_config_{};

  // ---- LowLevel link state ----
  std::unique_ptr<mowgli_hardware::SerialPort> ll_serial_;
  mowgli_hardware::PacketHandler ll_packets_;
  rclcpp::Time ll_last_rx_{0, 0, RCL_ROS_TIME};
  rclcpp::Time ll_last_status_{0, 0, RCL_ROS_TIME};
  rclcpp::Time ll_last_open_attempt_{0, 0, RCL_ROS_TIME};
  bool ll_initialized_{false};
  bool ll_config_acked_{false};
  int ll_config_tries_left_{0};
  rclcpp::Time ll_config_last_req_{0, 0, RCL_ROS_TIME};
  mowgli_hardware::HostFirmwareClockFit imu_clock_fit_;
  ImuBiasEstimator imu_bias_;
  rclcpp::Time imu_at_rest_since_{0, 0, RCL_ROS_TIME};
  bool is_charging_{false};
  uint8_t last_ll_status_bitmask_{0u};
  uint8_t last_ll_emergency_bitmask_{0u};
  EmergencyTracker emergency_;
  bool emergency_active_now_{false};

  // ---- motor links ----
  enum Motor : std::size_t
  {
    kLeft = 0u,
    kRight = 1u,
    kMow = 2u
  };
  std::array<std::unique_ptr<MotorLink>, 3> motors_{};
  std::array<WheelVelocityController, 2> wheel_loops_{};
  std::array<WheelSpeedFilter, 2> wheel_speed_{};
  std::array<int64_t, 2> prev_wheel_ticks_{};
  std::array<bool, 2> wheel_ticks_primed_{};
  std::array<uint32_t, 2> wheel_tick_magnitude_{};
  std::array<uint8_t, 2> wheel_tick_direction_{1u, 1u};
  XescOdometry odometry_{1600.0, 0.325};
  mowgli_hardware::HostFirmwareClockFit odom_clock_fit_;
  SteadyClock::time_point last_control_tick_{};
  std::size_t control_ticks_{0u};
  bool motors_were_connected_{false};

  // ---- command state ----
  double cmd_vx_{0.0};
  double cmd_wz_{0.0};
  rclcpp::Time cmd_vel_stamp_{0, 0, RCL_ROS_TIME};
  double applied_vx_{0.0};
  double applied_wz_{0.0};
  double published_vx_{-1.0};
  double published_wz_{-1.0};
  bool last_wheels_allowed_{true};
  bool mow_requested_{false};
  uint8_t mow_direction_{0u};
  bool blade_running_{false};
  rclcpp::Time blade_status_stamp_{0, 0, RCL_ROS_TIME};
  uint8_t current_mode_{HL_MODE_NULL};
  rclcpp::Time hl_status_stamp_{0, 0, RCL_ROS_TIME};
  uint8_t last_sent_mode_{0xFFu};
  uint8_t gps_quality_percent_{0u};
  rclcpp::Time gps_status_stamp_{0, 0, RCL_ROS_TIME};

  // ---- ROS interfaces ----
  rclcpp::Publisher<mowgli_interfaces::msg::Status>::SharedPtr pub_status_;
  rclcpp::Publisher<mowgli_interfaces::msg::Emergency>::SharedPtr pub_emergency_;
  rclcpp::Publisher<mowgli_interfaces::msg::Power>::SharedPtr pub_power_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr pub_battery_state_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr pub_mag_raw_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_wheel_odom_;
  rclcpp::Publisher<mowgli_interfaces::msg::WheelTick>::SharedPtr pub_wheel_ticks_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_cmd_vel_applied_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_dig_escalated_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_cmd_vel_;
  rclcpp::Subscription<mowgli_interfaces::msg::HighLevelStatus>::SharedPtr sub_high_level_status_;
  rclcpp::Subscription<mowgli_interfaces::msg::GnssStatus>::SharedPtr sub_gnss_status_;
  rclcpp::Service<mowgli_interfaces::srv::MowerControl>::SharedPtr srv_mower_control_;
  rclcpp::Service<mowgli_interfaces::srv::EmergencyStop>::SharedPtr srv_emergency_stop_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_reboot_board_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_set_firmware_debug_;
  rclcpp::Client<mowgli_interfaces::srv::HighLevelControl>::SharedPtr client_high_level_control_;
  rclcpp::TimerBase::SharedPtr timer_ll_read_;
  rclcpp::TimerBase::SharedPtr timer_control_;
  rclcpp::TimerBase::SharedPtr timer_high_level_;
};

}  // namespace mowgli_openmower_bridge

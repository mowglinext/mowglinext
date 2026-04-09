#include "mavros_hardware_bridge_node.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

namespace mowgli_mavros_bridge
{

using namespace std::chrono_literals;

MavrosHardwareBridgeNode::MavrosHardwareBridgeNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("hardware_bridge", options)
{
  status_publish_rate_hz_ = declare_parameter<double>("status_publish_rate_hz", 10.0);
  rain_detected_ = declare_parameter<bool>("rain_detected_default", false);
  esc_power_ = declare_parameter<bool>("esc_power_default", true);
  raspberry_pi_power_ = declare_parameter<bool>("raspberry_pi_power_default", true);

  create_publishers();
  create_subscriptions();
  create_services();
  create_clients();
  create_timers();

  RCLCPP_INFO(get_logger(), "MAVROS hardware bridge started.");
}

void MavrosHardwareBridgeNode::create_publishers()
{
  pub_status_ = create_publisher<mowgli_interfaces::msg::Status>("~/status", 10);
  pub_emergency_ = create_publisher<mowgli_interfaces::msg::Emergency>("~/emergency", 10);
  pub_power_ = create_publisher<mowgli_interfaces::msg::Power>("~/power", 10);
  pub_imu_ = create_publisher<sensor_msgs::msg::Imu>("~/imu/data_raw", 10);
  pub_wheel_odom_ = create_publisher<nav_msgs::msg::Odometry>("~/wheel_odom", 10);
  pub_battery_state_ = create_publisher<sensor_msgs::msg::BatteryState>("/battery_state", 10);

  pub_manual_control_ =
      create_publisher<mavros_msgs::msg::ManualControl>("/mavros/manual_control/send", 10);
}

void MavrosHardwareBridgeNode::create_subscriptions()
{
  auto default_qos = rclcpp::QoS(rclcpp::KeepLast(10));
  auto sensor_qos = rclcpp::SensorDataQoS();

  sub_cmd_vel_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel",
      default_qos,
      std::bind(&MavrosHardwareBridgeNode::on_cmd_vel, this, std::placeholders::_1));

  sub_hl_status_ = create_subscription<mowgli_interfaces::msg::HighLevelStatus>(
      "/high_level_status",
      default_qos,
      std::bind(&MavrosHardwareBridgeNode::on_high_level_status, this, std::placeholders::_1));

  sub_mavros_state_ = create_subscription<mavros_msgs::msg::State>(
      "/mavros/state",
      default_qos,
      std::bind(&MavrosHardwareBridgeNode::on_mavros_state, this, std::placeholders::_1));

  sub_mavros_imu_ = create_subscription<sensor_msgs::msg::Imu>(
      "/mavros/imu/data",
      sensor_qos,
      std::bind(&MavrosHardwareBridgeNode::on_mavros_imu, this, std::placeholders::_1));

  sub_mavros_battery_ = create_subscription<sensor_msgs::msg::BatteryState>(
      "/mavros/battery",
      sensor_qos,
      std::bind(&MavrosHardwareBridgeNode::on_mavros_battery, this, std::placeholders::_1));

  sub_mavros_odom_ = create_subscription<nav_msgs::msg::Odometry>(
      "/mavros/local_position/odom",
      sensor_qos,
      std::bind(&MavrosHardwareBridgeNode::on_mavros_odom, this, std::placeholders::_1));
}

void MavrosHardwareBridgeNode::create_services()
{
  srv_mower_control_ = create_service<mowgli_interfaces::srv::MowerControl>(
      "~/mower_control",
      std::bind(&MavrosHardwareBridgeNode::on_mower_control,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

  srv_emergency_stop_ = create_service<mowgli_interfaces::srv::EmergencyStop>(
      "~/emergency_stop",
      std::bind(&MavrosHardwareBridgeNode::on_emergency_stop,
                this,
                std::placeholders::_1,
                std::placeholders::_2));
}

void MavrosHardwareBridgeNode::create_clients()
{
  cli_arm_ = create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
  cli_set_mode_ = create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
}

void MavrosHardwareBridgeNode::create_timers()
{
  const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, status_publish_rate_hz_));

  timer_status_ = create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(period),
                                    [this]()
                                    {
                                      publish_status();
                                      publish_emergency();
                                      publish_power();

                                      std::lock_guard<std::mutex> lock(mutex_);
                                      pub_imu_->publish(last_imu_);
                                      pub_wheel_odom_->publish(last_odom_);
                                      pub_battery_state_->publish(last_battery_);
                                    });
}

void MavrosHardwareBridgeNode::on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  mavros_msgs::msg::ManualControl cmd{};
  cmd.header.stamp = now();

  // TODO: adapt scaling to ArduPilot expectations.
  cmd.x = static_cast<int16_t>(msg->linear.x * 1000.0);
  cmd.r = static_cast<int16_t>(msg->angular.z * 1000.0);

  pub_manual_control_->publish(cmd);
}

void MavrosHardwareBridgeNode::on_high_level_status(
    const mowgli_interfaces::msg::HighLevelStatus::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  last_high_level_status_ = *msg;
}

void MavrosHardwareBridgeNode::on_mavros_state(const mavros_msgs::msg::State::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  mavros_state_ = *msg;
}

void MavrosHardwareBridgeNode::on_mavros_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  last_imu_ = *msg;
}

void MavrosHardwareBridgeNode::on_mavros_battery(
    const sensor_msgs::msg::BatteryState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  last_battery_ = *msg;
  battery_voltage_ = msg->voltage;
}

void MavrosHardwareBridgeNode::on_mavros_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  last_odom_ = *msg;
}

void MavrosHardwareBridgeNode::on_mower_control(
    const std::shared_ptr<mowgli_interfaces::srv::MowerControl::Request> request,
    std::shared_ptr<mowgli_interfaces::srv::MowerControl::Response> response)
{
  std::lock_guard<std::mutex> lock(mutex_);
  mow_enabled_ = request->mow_enabled;
  mow_direction_ = request->mow_direction;

  // TODO: replace with actual Pixhawk output / MAVLink command for blade control.
  response->success = true;
}

void MavrosHardwareBridgeNode::on_emergency_stop(
    const std::shared_ptr<mowgli_interfaces::srv::EmergencyStop::Request> request,
    std::shared_ptr<mowgli_interfaces::srv::EmergencyStop::Response> response)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (request->emergency != 0U)
  {
    emergency_active_ = true;
    emergency_latched_ = true;
    emergency_reason_ = "SERVICE_EMERGENCY_STOP";
    mow_enabled_ = false;
  }
  else
  {
    emergency_active_ = false;
    emergency_reason_ = "NONE";
  }

  response->success = true;
}

void MavrosHardwareBridgeNode::publish_status()
{
  mowgli_interfaces::msg::Status msg;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    msg.stamp = now();
    msg.raspberry_pi_power = raspberry_pi_power_;
    msg.is_charging = is_charging_;
    msg.esc_power = esc_power_;
    msg.rain_detected = rain_detected_;
    msg.sound_module_available = sound_module_available_;
    msg.sound_module_busy = sound_module_busy_;
    msg.ui_board_available = ui_board_available_;
    msg.mow_enabled = mow_enabled_;

    // TODO: define proper mower status enum mapping.
    msg.mower_status = mavros_state_.armed ? 1U : 0U;

    msg.mower_esc_status = mower_esc_status_;
    msg.mower_esc_temperature = mower_esc_temperature_;
    msg.mower_esc_current = mower_esc_current_;
    msg.mower_motor_temperature = mower_motor_temperature_;
    msg.mower_motor_rpm = mower_motor_rpm_;
  }

  pub_status_->publish(msg);
}

void MavrosHardwareBridgeNode::publish_emergency()
{
  mowgli_interfaces::msg::Emergency msg;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    msg.stamp = now();
    msg.active_emergency = emergency_active_;
    msg.latched_emergency = emergency_latched_;
    msg.reason = emergency_reason_;
  }

  pub_emergency_->publish(msg);
}

void MavrosHardwareBridgeNode::publish_power()
{
  mowgli_interfaces::msg::Power msg;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    msg.stamp = now();
    msg.v_charge = static_cast<float>(charge_voltage_);
    msg.v_battery = static_cast<float>(battery_voltage_);
    msg.charge_current = static_cast<float>(charge_current_);
    msg.charger_enabled = charger_enabled_;
    msg.charger_status = charger_status_;
  }

  pub_power_->publish(msg);
}

bool MavrosHardwareBridgeNode::send_arm_command(bool arm)
{
  if (!cli_arm_->wait_for_service(1s))
  {
    RCLCPP_WARN(get_logger(), "Arming service not available.");
    return false;
  }

  auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
  req->value = arm;

  auto future = cli_arm_->async_send_request(req);
  const auto result = rclcpp::spin_until_future_complete(get_node_base_interface(), future, 2s);

  return result == rclcpp::FutureReturnCode::SUCCESS && future.get()->success;
}

bool MavrosHardwareBridgeNode::send_mode_command(const std::string& mode)
{
  if (!cli_set_mode_->wait_for_service(1s))
  {
    RCLCPP_WARN(get_logger(), "Set mode service not available.");
    return false;
  }

  auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
  req->custom_mode = mode;

  auto future = cli_set_mode_->async_send_request(req);
  const auto result = rclcpp::spin_until_future_complete(get_node_base_interface(), future, 2s);

  return result == rclcpp::FutureReturnCode::SUCCESS && future.get()->mode_sent;
}

}  // namespace mowgli_mavros_bridge

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mowgli_mavros_bridge::MavrosHardwareBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
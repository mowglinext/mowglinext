#include "mavros_hardware_bridge_node.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <cmath>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace
{

geometry_msgs::msg::Vector3 ned_to_enu_vector(const geometry_msgs::msg::Vector3& v)
{
  geometry_msgs::msg::Vector3 out;
  out.x = v.y;
  out.y = v.x;
  out.z = -v.z;
  return out;
}

geometry_msgs::msg::Point ned_to_enu_point(const geometry_msgs::msg::Point& p)
{
  geometry_msgs::msg::Point out;
  out.x = p.y;
  out.y = p.x;
  out.z = -p.z;
  return out;
}

geometry_msgs::msg::Vector3 frd_to_flu_vector(const geometry_msgs::msg::Vector3& v)
{
  geometry_msgs::msg::Vector3 out;
  out.x = v.x;
  out.y = -v.y;
  out.z = -v.z;
  return out;
}

geometry_msgs::msg::Quaternion ned_to_enu_frd_to_flu_quaternion(
    const geometry_msgs::msg::Quaternion& q_in)
{
  tf2::Quaternion q_body_to_world;
  tf2::fromMsg(q_in, q_body_to_world);

  // NED -> ENU
  tf2::Quaternion q_ned_to_enu;
  q_ned_to_enu.setRPY(M_PI, 0.0, M_PI_2);

  // FRD -> FLU
  tf2::Quaternion q_frd_to_flu;
  q_frd_to_flu.setRPY(M_PI, 0.0, 0.0);

  tf2::Quaternion q_out = q_ned_to_enu * q_body_to_world * q_frd_to_flu;
  q_out.normalize();
  return tf2::toMsg(q_out);
}

}  // namespace


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
  auto sensor_qos = rclcpp::SensorDataQoS();

  pub_status_ = create_publisher<mowgli_interfaces::msg::Status>("~/status", 10);
  pub_emergency_ = create_publisher<mowgli_interfaces::msg::Emergency>("~/emergency", 10);
  pub_power_ = create_publisher<mowgli_interfaces::msg::Power>("~/power", 10);
  pub_imu_ = create_publisher<sensor_msgs::msg::Imu>("~/imu/data_raw", sensor_qos);
  pub_wheel_odom_ = create_publisher<nav_msgs::msg::Odometry>("~/wheel_odom", sensor_qos);
  pub_battery_state_ =
      create_publisher<sensor_msgs::msg::BatteryState>("/battery_state", sensor_qos);

  pub_manual_control_ =
      create_publisher<mavros_msgs::msg::ManualControl>("/mavros/manual_control/send", 10);
}

void MavrosHardwareBridgeNode::create_subscriptions()
{
  auto default_qos = rclcpp::SystemDefaultsQoS();
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
  sensor_msgs::msg::Imu imu = *msg;

  imu.header.frame_id = "imu_link";
  imu.orientation = ned_to_enu_frd_to_flu_quaternion(msg->orientation);
  imu.angular_velocity = frd_to_flu_vector(msg->angular_velocity);
  imu.linear_acceleration = frd_to_flu_vector(msg->linear_acceleration);

  std::lock_guard<std::mutex> lock(mutex_);
  last_imu_ = imu;
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
  nav_msgs::msg::Odometry odom = *msg;

  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";

  odom.pose.pose.position = ned_to_enu_point(msg->pose.pose.position);
  odom.pose.pose.orientation = ned_to_enu_frd_to_flu_quaternion(msg->pose.pose.orientation);

  odom.twist.twist.linear = ned_to_enu_vector(msg->twist.twist.linear);
  odom.twist.twist.angular = frd_to_flu_vector(msg->twist.twist.angular);

  std::lock_guard<std::mutex> lock(mutex_);
  last_odom_ = odom;
}

void MavrosHardwareBridgeNode::on_mower_control(
    const std::shared_ptr<mowgli_interfaces::srv::MowerControl::Request> request,
    std::shared_ptr<mowgli_interfaces::srv::MowerControl::Response> response)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    mow_enabled_ = request->mow_enabled;
    mow_direction_ = request->mow_direction;
  }

  RCLCPP_WARN(
      get_logger(),
      "Mower control requested (enabled=%d, direction=%d) but cutting motor mapping on the Pixhawk/ArduPilot is not implemented yet.",
      request->mow_enabled,
      request->mow_direction);

  response->success = false;
}

void MavrosHardwareBridgeNode::on_emergency_stop(
    const std::shared_ptr<mowgli_interfaces::srv::EmergencyStop::Request> request,
    std::shared_ptr<mowgli_interfaces::srv::EmergencyStop::Response> response)
{
  const bool emergency_requested = (request->emergency != 0U);

  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (emergency_requested)
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
  }

  if (emergency_requested)
  {
    // Request a safe autopilot state.
    // HOLD should stop autonomous motion immediately.
    // Disarm is also requested, but the exact effect on blade/traction outputs
    // depends on the Pixhawk/ArduPilot output configuration and must be validated on hardware.
    send_mode_command("HOLD");
    send_arm_command(false);
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

void MavrosHardwareBridgeNode::send_arm_command(bool arm)
{
  if (!cli_arm_->wait_for_service(1s))
  {
    RCLCPP_WARN(get_logger(), "Arming service not available.");
    return;
  }

  auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
  req->value = arm;

  cli_arm_->async_send_request(
      req,
      [this, arm](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future)
      {
        try
        {
          const auto result = future.get();
          if (result->success)
          {
            RCLCPP_INFO(get_logger(), "%s command accepted.", arm ? "Arm" : "Disarm");
          }
          else
          {
            RCLCPP_WARN(get_logger(), "%s command rejected by MAVROS/Pixhawk.",
                        arm ? "Arm" : "Disarm");
          }
        }
        catch (const std::exception& e)
        {
          RCLCPP_ERROR(get_logger(), "Arm/disarm request failed: %s", e.what());
        }
      });
}

void MavrosHardwareBridgeNode::send_mode_command(const std::string& mode)
{
  if (!cli_set_mode_->wait_for_service(1s))
  {
    RCLCPP_WARN(get_logger(), "Set mode service not available.");
    return;
  }

  auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
  req->custom_mode = mode;

  cli_set_mode_->async_send_request(
      req,
      [this, mode](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future)
      {
        try
        {
          const auto result = future.get();
          if (result->mode_sent)
          {
            RCLCPP_INFO(get_logger(), "Mode change to '%s' accepted.", mode.c_str());
          }
          else
          {
            RCLCPP_WARN(get_logger(), "Mode change to '%s' rejected by MAVROS/Pixhawk.",
                        mode.c_str());
          }
        }
        catch (const std::exception& e)
        {
          RCLCPP_ERROR(get_logger(), "Set mode request failed for '%s': %s",
                       mode.c_str(), e.what());
        }
      });
}

}  // namespace mowgli_mavros_bridge

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mowgli_mavros_bridge::MavrosHardwareBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
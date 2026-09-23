// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Owns the LD19 spin-motor's PWM control line. Subscribes to the robot's
// idle/active status and to /scan, drives mowgli_lidar_pwm::PwmControllerStep
// (pwm_controller.hpp) against a sysfs PWM device (sysfs_pwm.hpp), and
// publishes mowgli_interfaces/LidarMotorStatus so mowgli_behavior can gate
// leaving IDLE on the motor actually being healthy. See pwm_controller.hpp
// for the full design rationale (issue #569).

#include <chrono>
#include <memory>
#include <mutex>

#include "mowgli_interfaces/msg/high_level_status.hpp"
#include "mowgli_interfaces/msg/lidar_motor_status.hpp"
#include "mowgli_lidar_pwm/pwm_controller.hpp"
#include "mowgli_lidar_pwm/sysfs_pwm.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace mowgli_lidar_pwm
{

class LidarPwmNode : public rclcpp::Node
{
public:
  LidarPwmNode()
  : rclcpp::Node("lidar_pwm_node")
  {
    // lidar_pwm_enabled is a host/install-level GUI setting (schema-only
    // default, Invariant 15 style — see gui/pkg/api/schema_template_parity_test.go),
    // not a mowgli_bringup-injected ROS parameter. It still flows here as a
    // declared parameter so this node can be launched unconditionally and
    // simply do nothing when it is false — the launch file does not need to
    // know about it.
    enabled_ = declare_parameter<bool>("lidar_pwm_enabled", false);
    if (!enabled_) {
      RCLCPP_INFO(get_logger(), "lidar_pwm_enabled=false — PWM motor control disabled, node idle.");
      return;
    }

    const int gpio_pin = declare_parameter<int>("lidar_pwm_gpio_pin", 12);
    // NOT derived from gpio_pin — the pwmchip/channel mapping for a given
    // GPIO depends on the board's device tree and is not fixed across
    // overlay versions. Confirm with `ls /sys/class/pwm/` on real hardware
    // (see the lidar_pwm_gpio_pin settings description) and set these to
    // match; the defaults here are unverified placeholders.
    const std::string base_path =
      declare_parameter<std::string>("pwm_sysfs_base_path", "/sys/class/pwm");
    const int pwm_chip = declare_parameter<int>("pwm_chip_index", 0);
    const int pwm_channel = declare_parameter<int>("pwm_channel_index", 0);
    const double carrier_hz = declare_parameter<double>("pwm_carrier_hz", 30000.0);

    cfg_.handshake_duty = declare_parameter<double>("pwm_handshake_duty", cfg_.handshake_duty);
    cfg_.handshake_hold_s =
      declare_parameter<double>("pwm_handshake_hold_s", cfg_.handshake_hold_s);
    cfg_.run_duty = declare_parameter<double>("pwm_run_duty", cfg_.run_duty);
    cfg_.stop_duty = declare_parameter<double>("pwm_stop_duty", cfg_.stop_duty);
    cfg_.spinup_timeout_s =
      declare_parameter<double>("pwm_spinup_timeout_s", cfg_.spinup_timeout_s);
    cfg_.spindown_settle_s =
      declare_parameter<double>("pwm_spindown_settle_s", cfg_.spindown_settle_s);
    scan_fresh_max_age_s_ = declare_parameter<double>("scan_fresh_max_age_s", 0.5);
    const double control_rate_hz = declare_parameter<double>("control_rate_hz", 20.0);

    const auto period_ns = static_cast<std::int64_t>(1e9 / carrier_hz);
    pwm_ = std::make_unique<SysfsPwm>(base_path, pwm_chip, pwm_channel, period_ns);
    try {
      pwm_->Open();
      pwm_->SetEnabled(true);
    } catch (const std::exception & e) {
      // Startup fault: log loudly and keep publishing FAULT forever rather
      // than crash-looping the container. The pre-motion BT gate this
      // feeds (mowgli_behavior) treats a stuck FAULT the same as a motor
      // that never spins up.
      RCLCPP_ERROR(
        get_logger(),
        "Failed to open PWM device (chip=%d channel=%d base=%s): %s — "
        "GPIO %d likely needs the dtoverlay=pwm boot config step; "
        "see the LiDAR PWM settings description.",
        pwm_chip, pwm_channel, base_path.c_str(), e.what(), gpio_pin);
      pwm_.reset();
    }

    status_pub_ = create_publisher<mowgli_interfaces::msg::LidarMotorStatus>(
      "~/status", rclcpp::QoS(10));

    status_sub_ = create_subscription<mowgli_interfaces::msg::HighLevelStatus>(
      "/behavior_tree_node/high_level_status", rclcpp::QoS(10),
      [this](mowgli_interfaces::msg::HighLevelStatus::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        active_ = msg->state != mowgli_interfaces::msg::HighLevelStatus::HIGH_LEVEL_STATE_IDLE;
      });

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_scan_time_ = now();
      });

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / control_rate_hz),
      std::bind(&LidarPwmNode::OnTimer, this));

    last_tick_ = now();
    RCLCPP_INFO(
      get_logger(), "lidar_pwm_node active: GPIO %d, pwmchip%d/pwm%d", gpio_pin, pwm_chip,
      pwm_channel);
  }

private:
  void OnTimer()
  {
    std::lock_guard<std::mutex> lock(mutex_);

    const auto n = now();
    const double dt = (n - last_tick_).seconds();
    last_tick_ = n;

    const bool scan_fresh = last_scan_time_.nanoseconds() != 0 &&
      (n - last_scan_time_).seconds() <= scan_fresh_max_age_s_;

    const PwmCommand cmd = PwmControllerStep(cfg_, controller_state_, active_, scan_fresh, dt);

    if (pwm_) {
      try {
        if (cmd.duty_cycle != last_commanded_duty_) {
          pwm_->SetDutyCycle(cmd.duty_cycle);
          last_commanded_duty_ = cmd.duty_cycle;
        }
      } catch (const std::exception & e) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "PWM write failed: %s", e.what());
      }
    }

    mowgli_interfaces::msg::LidarMotorStatus msg;
    msg.header.stamp = n;
    // pwm_ == nullptr (device never opened) always reports FAULT, regardless
    // of what the pure state machine computed — a status message claiming
    // RUNNING while nothing is actually driving the motor would be worse
    // than useless to the BT gate that trusts it.
    msg.state = pwm_ ? static_cast<uint8_t>(cmd.state) :
      mowgli_interfaces::msg::LidarMotorStatus::FAULT;
    msg.commanded_duty_cycle = static_cast<float>(cmd.duty_cycle);
    msg.time_in_state_sec = static_cast<float>(controller_state_.time_in_state);
    status_pub_->publish(msg);
  }

  bool enabled_ = false;
  PwmControllerCfg cfg_;
  PwmControllerState controller_state_;
  std::unique_ptr<SysfsPwm> pwm_;
  double scan_fresh_max_age_s_ = 0.5;
  double last_commanded_duty_ = -1.0;  // force an initial write

  std::mutex mutex_;
  bool active_ = false;
  rclcpp::Time last_scan_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_tick_;

  rclcpp::Publisher<mowgli_interfaces::msg::LidarMotorStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<mowgli_interfaces::msg::HighLevelStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace mowgli_lidar_pwm

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mowgli_lidar_pwm::LidarPwmNode>());
  rclcpp::shutdown();
  return 0;
}

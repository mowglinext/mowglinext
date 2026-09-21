// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0
//
// Node setup: parameters, ROS interfaces, motor links, timers and the
// callbacks that only touch host-side state. Serial handling lives in
// openmower_bridge_lowlevel.cpp and openmower_bridge_actuation.cpp.

#include "mowgli_openmower_bridge/openmower_bridge_node.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <utility>

#include "mowgli_hardware/blade_gate.hpp"
#include "mowgli_hardware/cmd_vel_validation.hpp"
#include "mowgli_hardware/timer_period.hpp"
#include "mowgli_interfaces/gnss_status_utils.hpp"
#include "mowgli_openmower_bridge/xesc_2040_link.hpp"
#include "mowgli_openmower_bridge/xesc_mini_link.hpp"

namespace mowgli_openmower_bridge
{

namespace
{

constexpr double kMinTicksPerMeter = 50.0;
constexpr double kMaxTicksPerMeter = 20000.0;
constexpr double kMaxWheelTrack = 1.0;
constexpr double kMinWheelTrack = 0.1;

Xesc2040Settings settings_from_params(rclcpp::Node& node, const std::string& prefix)
{
  Xesc2040Settings s{};
  const auto table = node.get_parameter(prefix + "_hall_table").as_integer_array();
  if (table.size() != xesc2040::kHallTableSize)
  {
    throw std::invalid_argument(prefix + "_hall_table must have exactly 8 entries");
  }
  for (std::size_t i = 0; i < xesc2040::kHallTableSize; ++i)
  {
    if (table[i] < 0 || table[i] > 255)
    {
      throw std::invalid_argument(prefix + "_hall_table entries must be 0..255");
    }
    s.hall_table[i] = static_cast<uint8_t>(table[i]);
  }
  s.motor_current_limit =
      static_cast<float>(node.get_parameter(prefix + "_motor_current_limit").as_double());
  s.acceleration = static_cast<float>(node.get_parameter(prefix + "_acceleration").as_double());
  s.has_motor_temp = node.get_parameter(prefix + "_has_motor_temp").as_bool();
  s.min_motor_temp = static_cast<float>(node.get_parameter(prefix + "_min_motor_temp").as_double());
  s.max_motor_temp = static_cast<float>(node.get_parameter(prefix + "_max_motor_temp").as_double());
  s.min_pcb_temp = static_cast<float>(node.get_parameter(prefix + "_min_pcb_temp").as_double());
  s.max_pcb_temp = static_cast<float>(node.get_parameter(prefix + "_max_pcb_temp").as_double());
  return s;
}

}  // namespace

OpenMowerBridgeNode::OpenMowerBridgeNode() : rclcpp::Node("hardware_bridge")
{
  declare_parameters();
  create_interfaces();
  create_motor_links();
  create_timers();

  RCLCPP_INFO(get_logger(),
              "OpenMower bridge up: LowLevel on %s, %s xESC left=%s right=%s mow=%s, "
              "ticks_per_meter=%.1f wheel_track=%.3f wheel loop %s",
              ll_serial_port_.c_str(),
              xesc_type_.c_str(),
              xesc_ports_[kLeft].c_str(),
              xesc_ports_[kRight].c_str(),
              mow_xesc_enabled_ ? xesc_ports_[kMow].c_str() : "(disabled)",
              ticks_per_meter_,
              wheel_track_,
              wheel_gains_.closed_loop ? "closed" : "open (feed-forward only)");
}

OpenMowerBridgeNode::~OpenMowerBridgeNode()
{
  // Leave every controller stopped; the xESC watchdogs would do it a moment
  // later, but an explicit zero is the polite exit.
  for (auto& motor : motors_)
  {
    if (motor)
    {
      motor->SendDuty(0.0);
    }
  }
}

void OpenMowerBridgeNode::declare_parameters()
{
  using mowgli_hardware::require_finite_nonnegative_timeout;
  using mowgli_hardware::require_finite_positive;

  // -- serial links --
  ll_serial_port_ = declare_parameter<std::string>("ll_serial_port", "/dev/ttyAMA0");
  ll_baud_rate_ = static_cast<int>(declare_parameter<int64_t>("ll_baud_rate", 115200));
  ll_rx_timeout_s_ = declare_parameter<double>("ll_rx_timeout_s", 2.0);
  require_finite_nonnegative_timeout(ll_rx_timeout_s_, "ll_rx_timeout_s");
  xesc_type_ = declare_parameter<std::string>("xesc_type", "xesc_mini");
  if (xesc_type_ != "xesc_mini" && xesc_type_ != "xesc_2040")
  {
    throw std::invalid_argument("xesc_type must be xesc_mini or xesc_2040, got '" + xesc_type_ +
                                "'");
  }
  xesc_ports_[kLeft] = declare_parameter<std::string>("left_xesc_port", "/dev/ttyAMA5");
  xesc_ports_[kRight] = declare_parameter<std::string>("right_xesc_port", "/dev/ttyAMA3");
  xesc_ports_[kMow] = declare_parameter<std::string>("mow_xesc_port", "/dev/ttyAMA4");
  mow_xesc_enabled_ = declare_parameter<bool>("mow_xesc_enabled", true);
  left_motor_inverted_ = declare_parameter<bool>("left_motor_inverted", false);
  // OpenMower wires the right ESC mirrored to the left one, so its duty (and
  // its tachometer direction) are inverted by default — same as mower_comms.
  right_motor_inverted_ = declare_parameter<bool>("right_motor_inverted", true);
  motor_pole_pairs_ = static_cast<int>(declare_parameter<int64_t>("motor_pole_pairs", 4));

  // xESC 2040 settings — pushed to the controller after every (re)connect.
  for (const std::string prefix : {"drive_xesc", "mow_xesc"})
  {
    const bool mow = prefix == "mow_xesc";
    declare_parameter<std::vector<int64_t>>(prefix + "_hall_table",
                                            mow ? std::vector<int64_t>{255, 3, 1, 2, 5, 4, 6, 255}
                                                : std::vector<int64_t>{255, 5, 3, 4, 1, 6, 2, 255});
    declare_parameter<double>(prefix + "_motor_current_limit", mow ? 1.0 : 0.5);
    declare_parameter<double>(prefix + "_acceleration", mow ? 5.0 : 20.0);
    declare_parameter<bool>(prefix + "_has_motor_temp", mow);
    declare_parameter<double>(prefix + "_min_motor_temp", 0.0);
    declare_parameter<double>(prefix + "_max_motor_temp", mow ? 60.0 : 0.0);
    declare_parameter<double>(prefix + "_min_pcb_temp", 0.0);
    declare_parameter<double>(prefix + "_max_pcb_temp", 60.0);
  }

  // -- rates --
  ll_read_rate_hz_ = declare_parameter<double>("ll_read_rate", 100.0);
  control_rate_hz_ = declare_parameter<double>("control_rate", 50.0);
  heartbeat_rate_hz_ = declare_parameter<double>("heartbeat_rate", 25.0);
  high_level_rate_hz_ = declare_parameter<double>("high_level_rate", 2.0);
  require_finite_positive(ll_read_rate_hz_, "ll_read_rate");
  require_finite_positive(control_rate_hz_, "control_rate");
  require_finite_positive(heartbeat_rate_hz_, "heartbeat_rate");
  require_finite_positive(high_level_rate_hz_, "high_level_rate");
  if (heartbeat_rate_hz_ > control_rate_hz_)
  {
    throw std::invalid_argument("heartbeat_rate must not exceed control_rate");
  }

  // -- kinematics --
  ticks_per_meter_ = declare_parameter<double>("ticks_per_meter", 1600.0);
  wheel_track_ = declare_parameter<double>("wheel_track", 0.325);
  max_mps_ = declare_parameter<double>("max_mps", 0.5);
  max_radps_ = declare_parameter<double>("max_radps", 2.0);
  min_linear_vel_ = declare_parameter<double>("min_linear_vel", 0.05);
  if (!std::isfinite(ticks_per_meter_) || ticks_per_meter_ < kMinTicksPerMeter ||
      ticks_per_meter_ > kMaxTicksPerMeter)
  {
    throw std::invalid_argument("ticks_per_meter out of range (50..20000)");
  }
  if (!std::isfinite(wheel_track_) || wheel_track_ < kMinWheelTrack ||
      wheel_track_ > kMaxWheelTrack)
  {
    throw std::invalid_argument("wheel_track out of range (0.1..1.0 m)");
  }
  require_finite_positive(max_mps_, "max_mps");
  require_finite_positive(max_radps_, "max_radps");
  odometry_.set_kinematics(ticks_per_meter_, wheel_track_);

  // -- command shaping (same defaults as hardware_bridge.yaml) --
  cmd_vel_timeout_s_ = declare_parameter<double>("cmd_vel_timeout_s", 1.0);
  high_level_status_timeout_s_ = declare_parameter<double>("high_level_status_timeout_s", 5.0);
  require_finite_positive(high_level_status_timeout_s_, "high_level_status_timeout_s");
  lin_accel_limit_ = declare_parameter<double>("cmd_vel_linear_accel_limit", 0.30);
  lin_decel_limit_ = declare_parameter<double>("cmd_vel_linear_decel_limit", 0.60);
  ang_accel_limit_ = declare_parameter<double>("cmd_vel_angular_accel_limit", 1.0);
  ang_decel_limit_ = declare_parameter<double>("cmd_vel_angular_decel_limit", 2.0);
  require_finite_positive(cmd_vel_timeout_s_, "cmd_vel_timeout_s");
  for (const double limit :
       {lin_accel_limit_, lin_decel_limit_, ang_accel_limit_, ang_decel_limit_})
  {
    require_finite_positive(limit, "cmd_vel_*_limit");
  }

  // -- wheel velocity loop --
  wheel_gains_.closed_loop = declare_parameter<bool>("wheel_loop_enabled", true);
  wheel_gains_.duty_per_mps = declare_parameter<double>("wheel_duty_per_mps", 1.0);
  wheel_gains_.kp = declare_parameter<double>("wheel_kp", 0.5);
  wheel_gains_.ki = declare_parameter<double>("wheel_ki", 2.0);
  wheel_gains_.integral_limit = declare_parameter<double>("wheel_integral_limit", 0.3);
  wheel_gains_.max_duty = declare_parameter<double>("wheel_max_duty", 1.0);
  for (const double g : {wheel_gains_.duty_per_mps,
                         wheel_gains_.kp,
                         wheel_gains_.ki,
                         wheel_gains_.integral_limit,
                         wheel_gains_.max_duty})
  {
    if (!std::isfinite(g) || g < 0.0)
    {
      throw std::invalid_argument("wheel_* gains must be finite and >= 0");
    }
  }
  for (auto& loop : wheel_loops_)
  {
    loop.set_gains(wheel_gains_);
  }

  // -- blade --
  blade_duty_ = declare_parameter<double>("blade_duty", 1.0);
  mowing_enabled_ = declare_parameter<bool>("mowing_enabled", true);
  if (!std::isfinite(blade_duty_) || blade_duty_ <= 0.0 || blade_duty_ > 1.0)
  {
    throw std::invalid_argument("blade_duty must be in (0, 1]");
  }

  // -- IMU --
  imu_frame_id_ = declare_parameter<std::string>("imu_frame_id", "imu_link");
  imu_cal_samples_ = static_cast<int>(declare_parameter<int64_t>("imu_cal_samples", 200));
  imu_cal_auto_rest_s_ = declare_parameter<double>("imu_cal_auto_rest_sec", 15.0);
  imu_bias_ = ImuBiasEstimator(static_cast<std::size_t>(std::max(1, imu_cal_samples_)));

  // -- LowLevel config packet (charge / battery protection, hall inputs) --
  ll_config_ = lowlevel::DefaultConfig();
  ll_config_.v_charge_cutoff =
      static_cast<float>(declare_parameter<double>("max_charge_voltage", 29.4));
  ll_config_.i_charge_cutoff =
      static_cast<float>(declare_parameter<double>("max_charge_current", 1.2));
  ll_config_.v_battery_cutoff =
      static_cast<float>(declare_parameter<double>("battery_cutoff_voltage", 29.0));
  ll_config_.v_battery_empty =
      static_cast<float>(declare_parameter<double>("battery_empty_voltage", 24.0));
  ll_config_.v_battery_full =
      static_cast<float>(declare_parameter<double>("battery_full_voltage", 28.0));
  const auto lift_ms = declare_parameter<int64_t>("both_wheels_lift_emergency_ms", 1000);
  const auto tilt_ms = declare_parameter<int64_t>("one_wheel_lift_emergency_ms", 2000);
  ll_config_.lift_period = static_cast<uint16_t>(std::clamp<int64_t>(lift_ms, 0, 0xFFFE));
  ll_config_.tilt_period = static_cast<uint16_t>(std::clamp<int64_t>(tilt_ms, 0, 0xFFFE));
  ll_config_.options.ignore_charging_current =
      declare_parameter<bool>("ignore_charging_current", false) ? lowlevel::OptionState::ON
                                                                : lowlevel::OptionState::OFF;
  const auto language = declare_parameter<std::string>("ll_language", "en");
  if (language.size() == 2u)
  {
    ll_config_.language[0] = language[0];
    ll_config_.language[1] = language[1];
  }
  lowlevel::ApplyHallConfigString(ll_config_,
                                  declare_parameter<std::string>("emergency_input_config", ""));
}

void OpenMowerBridgeNode::create_interfaces()
{
  pub_status_ = create_publisher<mowgli_interfaces::msg::Status>("~/status", rclcpp::QoS(10));
  pub_emergency_ =
      create_publisher<mowgli_interfaces::msg::Emergency>("~/emergency", rclcpp::QoS(10));
  pub_power_ = create_publisher<mowgli_interfaces::msg::Power>("~/power", rclcpp::QoS(10));
  pub_battery_state_ =
      create_publisher<sensor_msgs::msg::BatteryState>("/battery_state", rclcpp::QoS(10));
  // RELIABLE on purpose, as in mowgli_hardware: fusion_graph subscribes reliable.
  pub_imu_ = create_publisher<sensor_msgs::msg::Imu>("~/imu/data_raw", rclcpp::QoS(10));
  pub_mag_raw_ =
      create_publisher<sensor_msgs::msg::MagneticField>("~/imu/mag_raw", rclcpp::QoS(10));
  pub_wheel_odom_ = create_publisher<nav_msgs::msg::Odometry>("~/wheel_odom", rclcpp::QoS(10));
  pub_wheel_ticks_ =
      create_publisher<mowgli_interfaces::msg::WheelTick>("~/wheel_ticks", rclcpp::QoS(10));
  pub_cmd_vel_applied_ =
      create_publisher<geometry_msgs::msg::TwistStamped>("~/cmd_vel_applied", rclcpp::QoS(10));
  // The behaviour tree's DigObstructionGuard reads this latched flag. The
  // wheel-slip dig detector is not part of this backend (it needs the fused
  // map pose and the firmware anti-dig backstop), so the flag is always false.
  pub_dig_escalated_ =
      create_publisher<std_msgs::msg::Bool>("~/dig_escalated", rclcpp::QoS(1).transient_local());
  std_msgs::msg::Bool not_escalated;
  not_escalated.data = false;
  pub_dig_escalated_->publish(not_escalated);

  sub_cmd_vel_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "~/cmd_vel",
      rclcpp::SystemDefaultsQoS(),
      std::bind(&OpenMowerBridgeNode::on_cmd_vel, this, std::placeholders::_1));
  sub_high_level_status_ = create_subscription<mowgli_interfaces::msg::HighLevelStatus>(
      "/behavior_tree_node/high_level_status",
      rclcpp::QoS(10),
      std::bind(&OpenMowerBridgeNode::on_high_level_status, this, std::placeholders::_1));
  sub_gnss_status_ = create_subscription<mowgli_interfaces::msg::GnssStatus>(
      "/gps/status",
      rclcpp::QoS(10),
      std::bind(&OpenMowerBridgeNode::on_gnss_status, this, std::placeholders::_1));

  srv_mower_control_ = create_service<mowgli_interfaces::srv::MowerControl>(
      "~/mower_control",
      std::bind(&OpenMowerBridgeNode::on_mower_control,
                this,
                std::placeholders::_1,
                std::placeholders::_2));
  srv_emergency_stop_ = create_service<mowgli_interfaces::srv::EmergencyStop>(
      "~/emergency_stop",
      std::bind(&OpenMowerBridgeNode::on_emergency_stop,
                this,
                std::placeholders::_1,
                std::placeholders::_2));
  // The STM32-only services stay reachable so the GUI's calls fail cleanly
  // with a reason instead of timing out on a missing service.
  srv_reboot_board_ = create_service<std_srvs::srv::Trigger>(
      "~/reboot_board",
      [](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
         std::shared_ptr<std_srvs::srv::Trigger::Response> res)
      {
        res->success = false;
        res->message = "reboot_board is not supported by the OpenMower LowLevel board";
      });
  srv_set_firmware_debug_ = create_service<std_srvs::srv::SetBool>(
      "~/set_firmware_debug",
      [](const std::shared_ptr<std_srvs::srv::SetBool::Request>,
         std::shared_ptr<std_srvs::srv::SetBool::Response> res)
      {
        res->success = false;
        res->message = "firmware debug flags do not exist on the OpenMower LowLevel board";
      });

  // The GUI offers this action whenever a bridge is present and surfaces the
  // refusal message, so answer with a reason instead of timing out.
  srv_clear_dig_escalation_ = create_service<std_srvs::srv::Trigger>(
      "~/clear_dig_escalation",
      [](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
         std::shared_ptr<std_srvs::srv::Trigger::Response> res)
      {
        res->success = false;
        res->message =
            "no dig escalation to clear: the wheel-slip dig detector does not run on the "
            "OpenMower backend";
      });

  client_high_level_control_ = create_client<mowgli_interfaces::srv::HighLevelControl>(
      "/behavior_tree_node/high_level_control");
}

void OpenMowerBridgeNode::create_motor_links()
{
  auto warn = [this](const std::string& text)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", text.c_str());
  };
  const std::size_t motor_count = mow_xesc_enabled_ ? 3u : 2u;
  for (std::size_t m = 0; m < motor_count; ++m)
  {
    if (xesc_type_ == "xesc_2040")
    {
      const auto settings = settings_from_params(*this, m == kMow ? "mow_xesc" : "drive_xesc");
      motors_[m] = std::make_unique<Xesc2040Link>(xesc_ports_[m], settings, warn);
    }
    else
    {
      motors_[m] = std::make_unique<XescMiniLink>(xesc_ports_[m], motor_pole_pairs_, warn);
    }
  }
}

void OpenMowerBridgeNode::create_timers()
{
  using mowgli_hardware::timer_period_from_rate_hz;
  timer_ll_read_ = create_wall_timer(timer_period_from_rate_hz(ll_read_rate_hz_),
                                     [this]()
                                     {
                                       lowlevel_read_tick();
                                     });
  timer_control_ = create_wall_timer(timer_period_from_rate_hz(control_rate_hz_),
                                     [this]()
                                     {
                                       control_tick();
                                     });
  timer_high_level_ = create_wall_timer(timer_period_from_rate_hz(high_level_rate_hz_),
                                        [this]()
                                        {
                                          send_high_level_state();
                                        });
}

void OpenMowerBridgeNode::on_cmd_vel(geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
{
  double vx = msg->twist.linear.x;
  double wz = msg->twist.angular.z;
  if (!mowgli_hardware::is_finite_velocity_command(vx, wz))
  {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Rejecting non-finite cmd_vel (%f, %f)", vx, wz);
    return;
  }
  // Same fallback as the STM32 bridge: motion before the BT has published a
  // mode means someone is driving, treat it as AUTONOMOUS so the wheels are
  // not held in the IDLE stop.
  if (current_mode_ == HL_MODE_NULL && !emergency_.is_emergency() && (vx != 0.0 || wz != 0.0))
  {
    current_mode_ = HL_MODE_AUTONOMOUS;
    RCLCPP_WARN(get_logger(),
                "cmd_vel before any high-level state; assuming AUTONOMOUS for the LowLevel board.");
    send_high_level_state();
  }
  constexpr double kDust = 1.0e-3;
  if (std::abs(vx) > kDust && std::abs(vx) < min_linear_vel_)
  {
    vx = 0.0;
  }
  cmd_vx_ = std::clamp(vx, -max_mps_, max_mps_);
  cmd_wz_ = std::clamp(wz, -max_radps_, max_radps_);
  cmd_vel_stamp_ = now();
}

void OpenMowerBridgeNode::on_high_level_status(
    mowgli_interfaces::msg::HighLevelStatus::ConstSharedPtr msg)
{
  current_mode_ = msg->state;
  hl_status_stamp_ = now();
}

void OpenMowerBridgeNode::on_gnss_status(mowgli_interfaces::msg::GnssStatus::ConstSharedPtr msg)
{
  gps_quality_percent_ = mowgli_interfaces::gnss_status_utils::HardwareQualityPercent(*msg);
  gps_status_stamp_ = now();
}

void OpenMowerBridgeNode::on_mower_control(
    const std::shared_ptr<mowgli_interfaces::srv::MowerControl::Request> req,
    std::shared_ptr<mowgli_interfaces::srv::MowerControl::Response> res)
{
  const bool requested = req->mow_enabled != 0u;
  // blade_gate.hpp's comment describes the STM32 backend, where the firmware is
  // the blade authority. Here the host IS the authority: this request only
  // becomes duty once BladeMayRun() (blade_policy.hpp) agrees, every tick.
  mow_requested_ = mowgli_hardware::blade_enable_allowed(requested, mowing_enabled_);
  mow_direction_ = req->mow_direction;
  if (requested && !mow_requested_)
  {
    RCLCPP_WARN(get_logger(),
                "MowerControl: blade enable SUPPRESSED — mowing_enabled=false (dry run).");
  }
  RCLCPP_INFO(get_logger(),
              "MowerControl: mow_enabled=%s direction=%u",
              mow_requested_ ? "true" : "false",
              mow_direction_);
  res->success = true;
}

void OpenMowerBridgeNode::on_emergency_stop(
    const std::shared_ptr<mowgli_interfaces::srv::EmergencyStop::Request> req,
    std::shared_ptr<mowgli_interfaces::srv::EmergencyStop::Response> res)
{
  if (req->emergency != 0u)
  {
    RCLCPP_WARN(get_logger(), "Emergency stop requested via service.");
    emergency_.RequestEmergency();
    // Stop the motors NOW, not on the next control tick.
    for (auto& motor : motors_)
    {
      if (motor)
      {
        motor->SendDuty(0.0);
      }
    }
  }
  else
  {
    RCLCPP_INFO(get_logger(), "Emergency release requested via service.");
    emergency_.RequestRelease();
  }
  send_heartbeat();
  res->success = true;
}

}  // namespace mowgli_openmower_bridge

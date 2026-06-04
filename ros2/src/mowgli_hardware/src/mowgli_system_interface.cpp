// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later

#include "mowgli_hardware/mowgli_system_interface.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_map>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "mowgli_hardware/velocity_pwm.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

namespace mowgli_hardware
{
namespace
{
constexpr char kLogger[] = "MowgliSystemInterface";

bool has_param(const std::unordered_map<std::string, std::string>& p, const std::string& k)
{
  return p.find(k) != p.end();
}
double param_double(const std::unordered_map<std::string, std::string>& p,
                    const std::string& k,
                    double def)
{
  auto it = p.find(k);
  return it == p.end() ? def : std::stod(it->second);
}
std::string param_string(const std::unordered_map<std::string, std::string>& p,
                         const std::string& k,
                         const std::string& def)
{
  auto it = p.find(k);
  return it == p.end() ? def : it->second;
}
}  // namespace

hardware_interface::CallbackReturn MowgliSystemInterface::on_init(
    const hardware_interface::HardwareInfo& info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != kNumWheels)
  {
    RCLCPP_FATAL(rclcpp::get_logger(kLogger),
                 "Expected %zu wheel joints, got %zu.",
                 kNumWheels,
                 info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Map joints to left(0)/right(1) by name so the URDF joint ORDER does not
  // matter; fall back to positional if neither name carries left/right.
  int left_idx = -1, right_idx = -1;
  for (std::size_t i = 0; i < info_.joints.size(); ++i)
  {
    const std::string& n = info_.joints[i].name;
    if (n.find("left") != std::string::npos)
    {
      left_idx = static_cast<int>(i);
    }
    else if (n.find("right") != std::string::npos)
    {
      right_idx = static_cast<int>(i);
    }
  }
  if (left_idx < 0 || right_idx < 0)
  {
    left_idx = 0;
    right_idx = 1;
    RCLCPP_WARN(rclcpp::get_logger(kLogger),
                "Could not infer left/right from joint names — using URDF order "
                "(%s=left, %s=right).",
                info_.joints[0].name.c_str(),
                info_.joints[1].name.c_str());
  }
  joint_names_[0] = info_.joints[static_cast<std::size_t>(left_idx)].name;
  joint_names_[1] = info_.joints[static_cast<std::size_t>(right_idx)].name;

  // Feedforward + scale parameters from the <ros2_control><hardware> block.
  const auto& hp = info_.hardware_parameters;
  wheel_radius_ = param_double(hp, "wheel_radius", wheel_radius_);
  pwm_per_mps_ = param_double(hp, "pwm_per_mps", pwm_per_mps_);
  deadband_pwm_ = param_double(hp, "deadband_pwm", deadband_pwm_);
  max_pwm_ = param_double(hp, "max_pwm", max_pwm_);
  v_eps_ = param_double(hp, "min_velocity_mps", v_eps_);

  RCLCPP_INFO(rclcpp::get_logger(kLogger),
              "Initialised: joints [%s, %s], wheel_radius=%.4f, "
              "feedforward pwm_per_mps=%.1f deadband=%.1f max=%.0f.",
              joint_names_[0].c_str(),
              joint_names_[1].c_str(),
              wheel_radius_,
              pwm_per_mps_,
              deadband_pwm_,
              max_pwm_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MowgliSystemInterface::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
  const auto& hp = info_.hardware_parameters;

  // Build the embedded comms node's options: name, the topic remaps the
  // standalone launch used to apply, and parameter overrides from the URDF.
  std::vector<std::string> args = {
      "--ros-args",
      "-r",
      "__node:=hardware_bridge",
      "-r",
      "~/imu/data_raw:=/imu/data",
      "-r",
      "~/mag_raw:=/imu/mag_raw",
      "-r",
      "~/wheel_odom:=/wheel_odom",
      "-r",
      "~/wheel_ticks:=/wheel_ticks",
      "-r",
      "~/dock_heading:=/gnss/heading",
  };

  std::vector<rclcpp::Parameter> overrides;
  auto push_d = [&](const char* k)
  {
    if (has_param(hp, k))
      overrides.emplace_back(k, param_double(hp, k, 0.0));
  };
  auto push_s = [&](const char* k)
  {
    if (has_param(hp, k))
      overrides.emplace_back(k, param_string(hp, k, ""));
  };
  push_s("serial_port");
  if (has_param(hp, "baud_rate"))
    overrides.emplace_back("baud_rate", static_cast<int>(param_double(hp, "baud_rate", 115200)));
  push_d("serial_rx_timeout_s");
  push_d("heartbeat_rate");
  push_d("publish_rate");
  push_d("high_level_rate");
  push_d("wheel_track");
  push_d("ticks_per_meter");
  push_d("wheel_radius");
  push_d("wheel_meas_filter_tau");
  push_d("dock_pose_x");
  push_d("dock_pose_y");
  push_d("dock_pose_yaw");
  if (has_param(hp, "imu_cal_samples"))
    overrides.emplace_back("imu_cal_samples",
                           static_cast<int>(param_double(hp, "imu_cal_samples", 200)));
  push_s("imu_cal_persist_path");
  push_d("imu_cal_auto_rest_sec");
  push_d("imu_cal_periodic_recal_sec");
  if (has_param(hp, "lift_recovery_mode"))
    overrides.emplace_back("lift_recovery_mode",
                           param_string(hp, "lift_recovery_mode", "false") == "true");
  if (has_param(hp, "use_motor_speed_velocity"))
    overrides.emplace_back("use_motor_speed_velocity",
                           param_string(hp, "use_motor_speed_velocity", "false") == "true");
  push_d("motor_speed_scale_alpha");

  rclcpp::NodeOptions options;
  options.arguments(args);
  options.parameter_overrides(overrides);

  comms_ = create_bridge_comms(options);
  if (!comms_)
  {
    RCLCPP_FATAL(rclcpp::get_logger(kLogger), "Failed to create the STM32 comms node.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(comms_->node_base());

  // Keep the plugin's wheel_radius in sync with the comms node (they must match
  // — the radian value cancels in the odom path, so consistency is all that
  // matters). If the URDF set wheel_radius the node got the same override.
  wheel_radius_ = comms_->wheel_radius();

  RCLCPP_INFO(rclcpp::get_logger(kLogger),
              "Configured: STM32 comms node created and added to executor.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MowgliSystemInterface::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
  for (std::size_t i = 0; i < kNumWheels; ++i)
  {
    vel_cmd_[i] = 0.0;
  }
  if (comms_)
  {
    comms_->set_wheel_pwm(0.0, 0.0);
  }
  executor_running_ = true;
  executor_thread_ = std::thread(
      [this]()
      {
        executor_->spin();
      });
  RCLCPP_INFO(rclcpp::get_logger(kLogger), "Activated: comms executor spinning.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MowgliSystemInterface::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
  if (comms_)
  {
    comms_->set_wheel_pwm(0.0, 0.0);
  }
  stop_executor();
  RCLCPP_INFO(rclcpp::get_logger(kLogger), "Deactivated: motors commanded to zero.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MowgliSystemInterface::on_cleanup(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
  stop_executor();
  if (executor_ && comms_)
  {
    executor_->remove_node(comms_->node_base());
  }
  comms_.reset();
  executor_.reset();
  return hardware_interface::CallbackReturn::SUCCESS;
}

void MowgliSystemInterface::stop_executor()
{
  if (executor_running_.exchange(false))
  {
    if (executor_)
    {
      executor_->cancel();
    }
    if (executor_thread_.joinable())
    {
      executor_thread_.join();
    }
  }
}

std::vector<hardware_interface::StateInterface> MowgliSystemInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> ifaces;
  for (std::size_t i = 0; i < kNumWheels; ++i)
  {
    ifaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &pos_state_[i]);
    ifaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &vel_state_[i]);
  }
  return ifaces;
}

std::vector<hardware_interface::CommandInterface> MowgliSystemInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> ifaces;
  for (std::size_t i = 0; i < kNumWheels; ++i)
  {
    ifaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &vel_cmd_[i]);
  }
  return ifaces;
}

hardware_interface::return_type MowgliSystemInterface::read(const rclcpp::Time& /*time*/,
                                                            const rclcpp::Duration& /*period*/)
{
  if (comms_)
  {
    const WheelJointState s = comms_->wheel_state();
    for (std::size_t i = 0; i < kNumWheels; ++i)
    {
      pos_state_[i] = s.position_rad[i];
      vel_state_[i] = s.velocity_rads[i];
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MowgliSystemInterface::write(const rclcpp::Time& /*time*/,
                                                             const rclcpp::Duration& /*period*/)
{
  if (!comms_)
  {
    return hardware_interface::return_type::OK;
  }
  // Joint velocity command (rad/s) → linear wheel speed (m/s) → signed PWM via
  // the static feedforward. The chained pid_controller has already trimmed the
  // command to drive the measured velocity to its reference; the firmware
  // deadband is inverted here so the integrator carries almost nothing.
  const double pwm_l =
      velocity_to_pwm(vel_cmd_[0] * wheel_radius_, pwm_per_mps_, deadband_pwm_, max_pwm_, v_eps_);
  const double pwm_r =
      velocity_to_pwm(vel_cmd_[1] * wheel_radius_, pwm_per_mps_, deadband_pwm_, max_pwm_, v_eps_);
  comms_->set_wheel_pwm(pwm_l, pwm_r);
  return hardware_interface::return_type::OK;
}

}  // namespace mowgli_hardware

PLUGINLIB_EXPORT_CLASS(mowgli_hardware::MowgliSystemInterface, hardware_interface::SystemInterface)

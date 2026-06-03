// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// mowgli_system_interface — ros2_control SystemInterface for the STM32 drive
// base. It is the single owner of the STM32 serial link: it embeds the
// HardwareBridgeNode comms core (BridgeComms) and spins it on its own executor
// thread, exposing the two drive wheels as ros2_control joints.
//
//   read()  : copy the latest per-wheel joint angle + velocity from the comms
//             core into the position/velocity STATE interfaces.
//   write() : convert the per-wheel VELOCITY command (rad/s, from the chained
//             pid_controller) into a raw signed PWM via a static feedforward
//             (slope + break-free deadband) and hand it to the comms core,
//             which relays it to the firmware on its next odometry packet.
//
// The controller stack on top (configured in mowgli_controllers.yaml) is the
// standard chained pattern: diff_drive_controller → pid_controller ×2 → here.
// The old custom host wheel-PI + gyro angular-rate loop are gone — the
// pid_controller closes the per-wheel velocity loop and the feedforward here
// inverts the firmware's static deadband so the integrator has little to do.

#ifndef MOWGLI_HARDWARE__MOWGLI_SYSTEM_INTERFACE_HPP_
#define MOWGLI_HARDWARE__MOWGLI_SYSTEM_INTERFACE_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "mowgli_hardware/bridge_comms.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace mowgli_hardware
{

class MowgliSystemInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MowgliSystemInterface)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  hardware_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
      const rclcpp_lifecycle::State& previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;

  hardware_interface::return_type write(const rclcpp::Time& time,
                                        const rclcpp::Duration& period) override;

private:
  void stop_executor();

  // Joint order: index 0 = left, 1 = right (matches BridgeComms / WheelJointState).
  static constexpr std::size_t kNumWheels = 2;
  std::string joint_names_[kNumWheels];

  // State + command storage the controllers bind to.
  double pos_state_[kNumWheels] = {0.0, 0.0};
  double vel_state_[kNumWheels] = {0.0, 0.0};
  double vel_cmd_[kNumWheels] = {0.0, 0.0};

  // Feedforward + scale parameters (from the URDF <hardware> block).
  double wheel_radius_ = 0.0925;
  double pwm_per_mps_ = 300.0;
  double deadband_pwm_ = 40.0;
  double max_pwm_ = 255.0;
  double v_eps_ = 5.0e-3;

  // Embedded comms core + its executor thread.
  std::shared_ptr<BridgeComms> comms_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread executor_thread_;
  std::atomic<bool> executor_running_{false};
};

}  // namespace mowgli_hardware

#endif  // MOWGLI_HARDWARE__MOWGLI_SYSTEM_INTERFACE_HPP_

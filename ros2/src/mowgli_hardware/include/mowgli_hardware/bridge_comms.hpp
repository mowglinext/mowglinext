// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// bridge_comms — minimal in-process interface between the STM32 serial comms
// node (HardwareBridgeNode) and the ros2_control hardware plugin
// (MowgliSystemInterface) that embeds it.
//
// WHY this exists
//   Migrating the motor-control path to ros2_control (diff_drive_controller +
//   chained pid_controller) means the controller_manager process must own the
//   STM32 serial link — a serial port can have exactly one owner, and that
//   link carries odom, IMU, status, blade, battery, emergency. So the proven
//   comms node is embedded INSIDE the hardware plugin rather than run
//   standalone. The plugin needs only three things from the node:
//     1. the latest per-wheel joint state (angle + velocity) for read(),
//     2. a sink for the per-wheel PWM command from write(),
//     3. the node base interface so it can be spun on the plugin's executor.
//   Everything else the node does (IMU+cal, status/power/battery, blade,
//   emergency+heartbeat, HL-state mirror, charging, dock-heading, wheel_ticks,
//   reboot, /wheel_odom) stays exactly as validated and is invisible to the
//   plugin. This tiny abstract interface keeps the ~2k-line node in its .cpp
//   (no giant header) while letting the plugin drive it.
//
// Threading: the node runs on the plugin's executor thread (owns the serial
// port and all timers/callbacks); the plugin's read()/write() run on the
// controller_manager thread. Only the shared joint-state / PWM-command words
// cross threads — the implementation guards them with a mutex. The serial port
// itself is touched only from the executor thread (the node sends the relayed
// PWM from its own odometry callback), so there is no serial write race.

#ifndef MOWGLI_HARDWARE__BRIDGE_COMMS_HPP_
#define MOWGLI_HARDWARE__BRIDGE_COMMS_HPP_

#include <memory>

#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_options.hpp"

namespace mowgli_hardware
{

/// Snapshot of the two drive-wheel joints, in joint (radian) units.
struct WheelJointState
{
  double position_rad[2] = {0.0, 0.0};  ///< cumulative wheel angle (index 0=left, 1=right)
  double velocity_rads[2] = {0.0, 0.0};  ///< de-quantised wheel angular velocity
};

/// Abstract handle the ros2_control plugin holds onto the embedded comms node.
class BridgeComms
{
public:
  virtual ~BridgeComms() = default;

  /// Latest per-wheel joint state (thread-safe). Called from read().
  virtual WheelJointState wheel_state() const = 0;

  /// Set the per-wheel signed PWM command (±255) to relay to the firmware on
  /// the next odometry packet (thread-safe). Called from write(). The node
  /// applies its own emergency / command-watchdog safety overrides before the
  /// value reaches the motors; the firmware's 200 ms watchdog is the backstop.
  virtual void set_wheel_pwm(double left_pwm, double right_pwm) = 0;

  /// Wheel radius (m) the node uses for tick→radian conversion, so the plugin
  /// can convert commanded joint velocity (rad/s) back to linear (m/s) for its
  /// feedforward with the SAME constant (the value cancels in the odom path —
  /// only ticks_per_meter sets the true ground scale — so consistency is what
  /// matters, not the absolute number).
  virtual double wheel_radius() const = 0;

  /// Node base interface, so the plugin can add the node to its executor.
  virtual rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base() = 0;
};

/// Construct the embedded comms node. `options` carries the node name, topic
/// remaps and parameter overrides the plugin builds from the URDF <hardware>
/// block. The returned handle owns the node; spin it via node_base().
std::shared_ptr<BridgeComms> create_bridge_comms(const rclcpp::NodeOptions& options);

}  // namespace mowgli_hardware

#endif  // MOWGLI_HARDWARE__BRIDGE_COMMS_HPP_

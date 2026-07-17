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

// SPDX-License-Identifier: GPL-3.0
/**
 * @file hardware_bridge_node.cpp
 * @brief ROS2 node: serial bridge between the STM32 firmware and the rest of
 *        the Mowgli ROS2 stack.
 *
 * The node communicates with the STM32 over USB-serial using the COBS-framed,
 * CRC-16-protected packet protocol defined in ll_datatypes.hpp.
 *
 * Published topics (relative to node namespace):
 *   ~/status        mowgli_interfaces/msg/Status
 *   ~/emergency     mowgli_interfaces/msg/Emergency
 *   ~/power         mowgli_interfaces/msg/Power
 *   ~/imu/data_raw  sensor_msgs/msg/Imu
 *   ~/wheel_odom    nav_msgs/msg/Odometry
 *   ~/dock_heading  sensor_msgs/msg/Imu  (dock yaw while charging, remapped → /gnss/heading)
 *   /battery_state  sensor_msgs/msg/BatteryState  (for opennav_docking)
 *
 * Subscribed topics:
 *   ~/cmd_vel      geometry_msgs/msg/Twist  → LlCmdVel packet to STM32
 *
 * Services:
 *   ~/mower_control  mowgli_interfaces/srv/MowerControl
 *   ~/emergency_stop mowgli_interfaces/srv/EmergencyStop
 *
 * Parameters:
 *   serial_port      (string,  default "/dev/mowgli")
 *   baud_rate        (int,     default 115200)
 *   heartbeat_rate   (double,  default 4.0 Hz  → 250 ms period)
 *   publish_rate     (double,  default 100.0 Hz → 10 ms period)
 *   high_level_rate  (double,  default 2.0 Hz   → 500 ms period)
 */

#include <chrono>
#include <cmath>
#include <cstring>
#include <ctime>
#include <fstream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "mowgli_hardware/clock_fit.hpp"
#include "mowgli_hardware/ll_datatypes.hpp"
#include "mowgli_hardware/odometry_publisher.hpp"
#include "mowgli_hardware/packet_handler.hpp"
#include "mowgli_hardware/serial_port.hpp"

// High-level mode constants — must match HighLevelStatus.msg and the
// HL_MODE_* defines in firmware/mowgli_protocol.h. Declared locally to
// avoid a brittle relative include of the firmware-shared header.
static constexpr uint8_t HL_MODE_NULL = 0u;  ///< Emergency / transitional
static constexpr uint8_t HL_MODE_IDLE = 1u;  ///< Docked or between missions
static constexpr uint8_t HL_MODE_AUTONOMOUS = 2u;  ///< Autonomous mowing
static constexpr uint8_t HL_MODE_RECORDING = 3u;  ///< Area recording
static constexpr uint8_t HL_MODE_MANUAL_MOWING = 4u;  ///< Manual teleop with blade
static constexpr double kMinRuntimeTicksPerMeter = 50.0;
static constexpr double kMaxRuntimeTicksPerMeter = 5000.0;
static constexpr double kMinRuntimePwmPerMps = 50.0;
static constexpr double kMaxRuntimePwmPerMps = 600.0;
static constexpr double kMinRuntimeWheelKp = 0.0;
static constexpr double kMaxRuntimeWheelKp = 200.0;
static constexpr double kMinRuntimeWheelKi = 0.0;
static constexpr double kMaxRuntimeWheelKi = 20000.0;
static constexpr double kMinRuntimeWheelKd = 0.0;
static constexpr double kMaxRuntimeWheelKd = 500.0;
static constexpr double kMinRuntimeWheelIntegralLimit = 0.0;
static constexpr double kMaxRuntimeWheelIntegralLimit = 255.0;
// Firmware gyro yaw-rate loop (Option C) — mirrors the firmware's own
// pid_constrain() ranges (Firmware-2, task #33 report) so an out-of-range
// host value is rejected here instead of being silently re-clamped on the
// wire to something the operator didn't ask for.
static constexpr double kMinRuntimeYawKp = 0.0;
static constexpr double kMaxRuntimeYawKp = 5.0;
static constexpr double kMinRuntimeYawKi = 0.0;
static constexpr double kMaxRuntimeYawKi = 20.0;
static constexpr double kMinRuntimeYawTrimLimitMps = 0.0;
static constexpr double kMaxRuntimeYawTrimLimitMps = 0.5;  // firmware MAX_MPS

static const char* high_level_mode_name(const uint8_t mode)
{
  switch (mode)
  {
    case HL_MODE_NULL:
      return "NULL";
    case HL_MODE_IDLE:
      return "IDLE";
    case HL_MODE_AUTONOMOUS:
      return "AUTONOMOUS";
    case HL_MODE_RECORDING:
      return "RECORDING";
    case HL_MODE_MANUAL_MOWING:
      return "MANUAL_MOWING";
    default:
      return "UNKNOWN";
  }
}

#include "mowgli_interfaces/gnss_status_utils.hpp"
#include "mowgli_interfaces/msg/emergency.hpp"
#include "mowgli_interfaces/msg/gnss_status.hpp"
#include "mowgli_interfaces/msg/high_level_status.hpp"
#include "mowgli_interfaces/msg/power.hpp"
#include "mowgli_interfaces/msg/status.hpp"
#include "mowgli_interfaces/msg/wheel_tick.hpp"
#include "mowgli_interfaces/srv/emergency_stop.hpp"
#include "mowgli_interfaces/srv/high_level_control.hpp"
#include "mowgli_interfaces/srv/mower_control.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace mowgli_hardware
{

using namespace std::chrono_literals;

static const char* reset_cause_name(const uint8_t cause)
{
  switch (cause)
  {
    case RESET_CAUSE_PIN:
      return "PINRST";
    case RESET_CAUSE_POR_PDR:
      return "POR/PDR";
    case RESET_CAUSE_BOR:
      return "BOR";
    case RESET_CAUSE_SFTRST:
      return "SFTRST";
    case RESET_CAUSE_IWDG:
      return "IWDG";
    case RESET_CAUSE_WWDG:
      return "WWDG";
    case RESET_CAUSE_LPWR:
      return "LPWR";
    case RESET_CAUSE_UNKNOWN:
    default:
      return "UNKNOWN";
  }
}

static const char* reset_cause_description(const uint8_t cause)
{
  switch (cause)
  {
    case RESET_CAUSE_PIN:
      return "External reset pin asserted: likely manual reset or hardware disturbance";
    case RESET_CAUSE_POR_PDR:
      return "Power-on / power-down reset: board cold-booted or supply was removed";
    case RESET_CAUSE_BOR:
      return "Brownout reset: supply voltage dipped below threshold";
    case RESET_CAUSE_SFTRST:
      return "Software reset: reboot requested by firmware or host";
    case RESET_CAUSE_IWDG:
      return "Independent watchdog reset: firmware stopped servicing watchdog";
    case RESET_CAUSE_WWDG:
      return "Window watchdog reset: main loop missed watchdog timing window; likely "
             "timing/blocking issue";
    case RESET_CAUSE_LPWR:
      return "Low-power reset: MCU resumed through a low-power reset path";
    case RESET_CAUSE_UNKNOWN:
    default:
      return "Unknown reset source: RCC reset flags were empty or unsupported";
  }
}

static const char* watchdog_stage_name(const uint8_t stage)
{
  switch (stage)
  {
    case WATCHDOG_STAGE_NONE:
      return "NONE";
    case WATCHDOG_STAGE_CHATTER:
      return "CHATTER";
    case WATCHDOG_STAGE_MOTORS:
      return "MOTORS";
    case WATCHDOG_STAGE_PANEL:
      return "PANEL";
    case WATCHDOG_STAGE_ROS_SPIN:
      return "ROS_SPIN";
    case WATCHDOG_STAGE_BROADCAST:
      return "BROADCAST";
    case WATCHDOG_STAGE_DRIVEMOTOR_RX:
      return "DRIVEMOTOR_RX";
    case WATCHDOG_STAGE_PERIMETER:
      return "PERIMETER";
    case WATCHDOG_STAGE_ADC:
      return "ADC";
    case WATCHDOG_STAGE_CHARGER:
      return "CHARGER";
    case WATCHDOG_STAGE_STATUS_LED:
      return "STATUS_LED";
    case WATCHDOG_STAGE_ULTRASONIC_HANDLER:
      return "ULTRASONIC_HANDLER";
    case WATCHDOG_STAGE_ULTRASONIC_APP:
      return "ULTRASONIC_APP";
    case WATCHDOG_STAGE_WATCHDOG_REFRESH:
      return "WATCHDOG_REFRESH";
    case WATCHDOG_STAGE_DRIVEMOTOR_10MS:
      return "DRIVEMOTOR_10MS";
    case WATCHDOG_STAGE_BLADEMOTOR:
      return "BLADEMOTOR";
    case WATCHDOG_STAGE_BUZZER:
      return "BUZZER";
    case WATCHDOG_STAGE_EMERGENCY:
      return "EMERGENCY";
    case WATCHDOG_STAGE_BROADCAST_ENTER:
      return "BROADCAST_ENTER";
    case WATCHDOG_STAGE_BROADCAST_IMU_BUILD:
      return "BROADCAST_IMU_BUILD";
    case WATCHDOG_STAGE_BROADCAST_IMU_SEND:
      return "BROADCAST_IMU_SEND";
    case WATCHDOG_STAGE_BROADCAST_RESET_SEND:
      return "BROADCAST_RESET_SEND";
    case WATCHDOG_STAGE_BROADCAST_STATUS_SEND:
      return "BROADCAST_STATUS_SEND";
    case WATCHDOG_STAGE_BROADCAST_BLADE_SEND:
      return "BROADCAST_BLADE_SEND";
    case WATCHDOG_STAGE_BROADCAST_EXIT:
      return "BROADCAST_EXIT";
    case WATCHDOG_STAGE_CDC_TX_ENTER:
      return "CDC_TX_ENTER";
    case WATCHDOG_STAGE_CDC_TX_QUEUE:
      return "CDC_TX_QUEUE";
    case WATCHDOG_STAGE_CDC_TX_RESUME:
      return "CDC_TX_RESUME";
    case WATCHDOG_STAGE_CDC_TX_EXIT:
      return "CDC_TX_EXIT";
    case WATCHDOG_STAGE_IMU_ACCEL:
      return "IMU_ACCEL";
    case WATCHDOG_STAGE_IMU_GYRO:
      return "IMU_GYRO";
    case WATCHDOG_STAGE_IMU_MAG:
      return "IMU_MAG";
    case WATCHDOG_STAGE_IMU_PACKET_FILL:
      return "IMU_PACKET_FILL";
    case WATCHDOG_STAGE_USB_IRQ_ENTER:
      return "USB_IRQ_ENTER";
    case WATCHDOG_STAGE_USB_IRQ_EXIT:
      return "USB_IRQ_EXIT";
    case WATCHDOG_STAGE_CDC_RX_ENTER:
      return "CDC_RX_ENTER";
    case WATCHDOG_STAGE_CDC_RX_PROCESS:
      return "CDC_RX_PROCESS";
    case WATCHDOG_STAGE_CDC_RX_EXIT:
      return "CDC_RX_EXIT";
    case WATCHDOG_STAGE_CDC_TX_PACKET:
      return "CDC_TX_PACKET";
    case WATCHDOG_STAGE_CDC_TX_COMPLETE:
      return "CDC_TX_COMPLETE";
    case WATCHDOG_STAGE_USB_RESET:
      return "USB_RESET";
    case WATCHDOG_STAGE_USB_SUSPEND:
      return "USB_SUSPEND";
    case WATCHDOG_STAGE_USB_RESUME:
      return "USB_RESUME";
    case WATCHDOG_STAGE_CDC_TX_PACKET_FAIL:
      return "CDC_TX_PACKET_FAIL";
    case WATCHDOG_STAGE_CDC_TX_BUSY_STUCK:
      return "CDC_TX_BUSY_STUCK";
    case WATCHDOG_STAGE_CDC_TX_QUEUE_FULL:
      return "CDC_TX_QUEUE_FULL";
    case WATCHDOG_STAGE_CDC_HOST_CLOSED:
      return "CDC_HOST_CLOSED";
    default:
      return "UNKNOWN";
  }
}

class HardwareBridgeNode : public rclcpp::Node
{
public:
  explicit HardwareBridgeNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : Node("hardware_bridge", options), odometry_publisher_(*this)
  {
    declare_parameters();
    create_publishers();
    create_subscribers();
    create_services();
    create_service_clients();
    open_serial_port();
    // Arm the firmware version handshake for the initial connect (the reconnect
    // path in read_serial_tick re-arms on every subsequent re-enumeration).
    rearm_firmware_handshake();
    create_timers();
    // Must run after declare_parameters — reads imu_cal_persist_path_. Runs
    // before any IMU packet arrives because the serial port callbacks are
    // dispatched by the executor from main(), not from the constructor.
    load_persisted_imu_calibration();
  }

  ~HardwareBridgeNode() override = default;

private:
  // ---------------------------------------------------------------------------
  // Initialisation helpers
  // ---------------------------------------------------------------------------

  void declare_parameters()
  {
    serial_port_path_ = declare_parameter<std::string>("serial_port", "/dev/mowgli");
    baud_rate_ = declare_parameter<int>("baud_rate", 115200);
    heartbeat_rate_ = declare_parameter<double>("heartbeat_rate", 4.0);
    publish_rate_ = declare_parameter<double>("publish_rate", 100.0);
    // Serial-link watchdog: if no bytes arrive for this long while the port is
    // open, treat the link as dead and reopen it. The STM32 streams status
    // (~4 Hz) + odom (~20 Hz) + IMU (~50-100 Hz) continuously whenever it is
    // running, so a multi-second RX gap means the USB endpoint went away —
    // e.g. a firmware flash or board reboot re-enumerated the CDC device. The
    // OS leaves the stale fd "open" (reads just return nothing), so without
    // this the bridge would sit forever writing into a dead fd. Reopening
    // re-resolves /dev/mowgli to the new ttyACM.
    serial_rx_timeout_s_ = declare_parameter<double>("serial_rx_timeout_s", 2.0);
    high_level_rate_ = declare_parameter<double>("high_level_rate", 2.0);
    dock_x_ = declare_parameter<double>("dock_pose_x", 0.0);
    dock_y_ = declare_parameter<double>("dock_pose_y", 0.0);
    dock_yaw_ = declare_parameter<double>("dock_pose_yaw", 0.0);

    // Wheel kinematics — single source of truth lives in mowgli_robot.yaml.
    // Previously hardcoded as kWheelBase=0.325 / kTicksPerMeter=300.0; that
    // duplicated the URDF args and the firmware TICKS_PER_M, so any
    // re-calibration touched three places. wheel_track is the centre-to-
    // centre drive-wheel distance; ticks_per_meter is the runtime encoder
    // scale used by this bridge for host-side odometry and re-sent to the
    // STM32 so the firmware wheel PI / odom share the same tuned value.
    wheel_track_ = declare_parameter<double>("wheel_track", 0.325);
    ticks_per_meter_ = declare_parameter<double>("ticks_per_meter", 300.0);
    // Drive-motor wheel-velocity PID gains + feedforward. Pushed to the STM32
    // firmware (PACKET_ID_LL_SET_DRIVE_PID) so the GUI can retune the per-wheel
    // loop without reflashing. Defaults mirror the firmware compile-time
    // fallback (cpp_main.cpp WHEEL_PI_* / board.h PWM_PER_MPS). Live-tunable via
    // the set-parameters callback below; the firmware re-clamps every value.
    wheel_pid_kp_ = declare_parameter<double>("wheel_pid_kp", 30.0);
    wheel_pid_ki_ = declare_parameter<double>("wheel_pid_ki", 5000.0);
    wheel_pid_kd_ = declare_parameter<double>("wheel_pid_kd", 0.0);
    wheel_pid_integral_limit_ = declare_parameter<double>("wheel_pid_integral_limit", 100.0);
    wheel_pid_pwm_per_mps_ = declare_parameter<double>("wheel_pid_pwm_per_mps", 300.0);
    // Firmware gyro yaw-rate loop (Option C, task #33/#34 — replaces the
    // removed host-side angular_rate_controller.hpp). Pushed to the STM32 via
    // PACKET_ID_LL_SET_YAW_PID (see send_yaw_pid()). Defaults are the
    // firmware's own power-on fallback (Firmware-2, task #33 report) so a
    // host that never connects still gets the same behaviour firmware boots
    // with. kp/ki/trim_limit_mps are live-tunable via the set-parameters
    // callback below (same firmware clamps: kp∈[0,5], ki∈[0,20],
    // trim_limit∈[0,0.5]); loop_enabled/gyro_sign are read once at startup —
    // gyro_sign is the field sign-check remedy for the physical IMU mounting
    // (UNVALIDATED default +1) and is not expected to change at runtime.
    yaw_kp_ = declare_parameter<double>("yaw_kp", 0.30);
    yaw_ki_ = declare_parameter<double>("yaw_ki", 0.40);
    yaw_trim_limit_mps_ = declare_parameter<double>("yaw_trim_limit_mps", 0.15);
    yaw_loop_enabled_ = declare_parameter<bool>("yaw_loop_enabled", true);
    yaw_gyro_sign_ = declare_parameter<int>("yaw_gyro_sign", 1);
    // Sub-deadband forward-velocity clamp threshold (see min_linear_vel_).
    // Default 0.05 (was a hardcoded 0.15) — the PX4 PID firmware can track
    // slow setpoints now. Live-tunable via the callback below.
    min_linear_vel_ = declare_parameter<double>("min_linear_vel", 0.05);
    min_lin_vel_cb_handle_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& params)
        {
          rcl_interfaces::msg::SetParametersResult result;
          result.successful = true;
          bool drive_pid_changed = false;
          bool yaw_pid_changed = false;
          double next_min_linear_vel = min_linear_vel_;
          double next_ticks_per_meter = ticks_per_meter_;
          double next_wheel_pid_kp = wheel_pid_kp_;
          double next_wheel_pid_ki = wheel_pid_ki_;
          double next_wheel_pid_kd = wheel_pid_kd_;
          double next_wheel_pid_integral_limit = wheel_pid_integral_limit_;
          double next_wheel_pid_pwm_per_mps = wheel_pid_pwm_per_mps_;
          double next_yaw_kp = yaw_kp_;
          double next_yaw_ki = yaw_ki_;
          double next_yaw_trim_limit_mps = yaw_trim_limit_mps_;
          auto reject_invalid_double = [&result](const std::string& name,
                                                 const double value,
                                                 const double lower,
                                                 const double upper)
          {
            if (!std::isfinite(value) || value < lower || value > upper)
            {
              result.successful = false;
              result.reason = name + " must be finite and within [" + std::to_string(lower) + ", " +
                              std::to_string(upper) + "]";
              return true;
            }
            return false;
          };
          for (const auto& p : params)
          {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
              continue;
            }
            const std::string& name = p.get_name();
            if (name == "min_linear_vel")
            {
              if (reject_invalid_double(name, p.as_double(), 0.0, 1.0))
              {
                break;
              }
              next_min_linear_vel = p.as_double();
            }
            else if (name == "ticks_per_meter")
            {
              if (reject_invalid_double(
                      name, p.as_double(), kMinRuntimeTicksPerMeter, kMaxRuntimeTicksPerMeter))
              {
                break;
              }
              next_ticks_per_meter = p.as_double();
              drive_pid_changed = true;
            }
            else if (name == "wheel_pid_kp")
            {
              if (reject_invalid_double(
                      name, p.as_double(), kMinRuntimeWheelKp, kMaxRuntimeWheelKp))
              {
                break;
              }
              next_wheel_pid_kp = p.as_double();
              drive_pid_changed = true;
            }
            else if (name == "wheel_pid_ki")
            {
              if (reject_invalid_double(
                      name, p.as_double(), kMinRuntimeWheelKi, kMaxRuntimeWheelKi))
              {
                break;
              }
              next_wheel_pid_ki = p.as_double();
              drive_pid_changed = true;
            }
            else if (name == "wheel_pid_kd")
            {
              if (reject_invalid_double(
                      name, p.as_double(), kMinRuntimeWheelKd, kMaxRuntimeWheelKd))
              {
                break;
              }
              next_wheel_pid_kd = p.as_double();
              drive_pid_changed = true;
            }
            else if (name == "wheel_pid_integral_limit")
            {
              if (reject_invalid_double(name,
                                        p.as_double(),
                                        kMinRuntimeWheelIntegralLimit,
                                        kMaxRuntimeWheelIntegralLimit))
              {
                break;
              }
              next_wheel_pid_integral_limit = p.as_double();
              drive_pid_changed = true;
            }
            else if (name == "wheel_pid_pwm_per_mps")
            {
              if (reject_invalid_double(
                      name, p.as_double(), kMinRuntimePwmPerMps, kMaxRuntimePwmPerMps))
              {
                break;
              }
              next_wheel_pid_pwm_per_mps = p.as_double();
              drive_pid_changed = true;
            }
            else if (name == "yaw_kp")
            {
              if (reject_invalid_double(name, p.as_double(), kMinRuntimeYawKp, kMaxRuntimeYawKp))
              {
                break;
              }
              next_yaw_kp = p.as_double();
              yaw_pid_changed = true;
            }
            else if (name == "yaw_ki")
            {
              if (reject_invalid_double(name, p.as_double(), kMinRuntimeYawKi, kMaxRuntimeYawKi))
              {
                break;
              }
              next_yaw_ki = p.as_double();
              yaw_pid_changed = true;
            }
            else if (name == "yaw_trim_limit_mps")
            {
              if (reject_invalid_double(
                      name, p.as_double(), kMinRuntimeYawTrimLimitMps, kMaxRuntimeYawTrimLimitMps))
              {
                break;
              }
              next_yaw_trim_limit_mps = p.as_double();
              yaw_pid_changed = true;
            }
          }
          if (!result.successful)
          {
            return result;
          }
          min_linear_vel_ = next_min_linear_vel;
          ticks_per_meter_ = next_ticks_per_meter;
          wheel_pid_kp_ = next_wheel_pid_kp;
          wheel_pid_ki_ = next_wheel_pid_ki;
          wheel_pid_kd_ = next_wheel_pid_kd;
          wheel_pid_integral_limit_ = next_wheel_pid_integral_limit;
          wheel_pid_pwm_per_mps_ = next_wheel_pid_pwm_per_mps;
          yaw_kp_ = next_yaw_kp;
          yaw_ki_ = next_yaw_ki;
          yaw_trim_limit_mps_ = next_yaw_trim_limit_mps;
          // Push the new gains to the firmware immediately (live apply, no
          // restart), and arm a couple of heartbeat resends in case this packet
          // is lost. The firmware re-clamps every value, so this callback does
          // not reject out-of-range inputs here.
          if (drive_pid_changed)
          {
            send_drive_pid();
            pid_resend_count_ = std::max(pid_resend_count_, 2);
          }
          if (yaw_pid_changed)
          {
            send_yaw_pid();
            pid_resend_count_ = std::max(pid_resend_count_, 2);
          }
          return result;
        });
    // 2026-07-17 Option C (task #34, supersedes the Option B host ω-PI that
    // used to live here — task #24, now removed): the yaw-rate loop runs in
    // FIRMWARE instead (task #33), closing on the same gyro but with no USB
    // round-trip latency. on_cmd_vel() now sends the commanded wz straight
    // through with no host-side shaping. Firmware's yaw gains are tunable via
    // PACKET_ID_LL_SET_YAW_PID (see send_yaw_pid()) — a separate packet from
    // SET_DRIVE_PID, per Firmware-2's #33 interface report.

    // Dock pose comes solely from mowgli_robot.yaml (declared as ROS
    // parameters above). Calibration and manual GUI adjustments persist
    // back to that file via map_server_node and calibrate_imu_yaw_node,
    // so a redeploy reads the latest values from the same source.
    lift_recovery_mode_ = declare_parameter<bool>("lift_recovery_mode", false);
    lift_blade_resume_delay_sec_ = declare_parameter<double>("lift_blade_resume_delay_sec", 1.0);
    // imu_yaw parameter is used by URDF for mounting rotation, not needed here
    imu_cal_samples_ = declare_parameter<int>("imu_cal_samples", 200);
    // Persist the last successful calibration so container restarts don't
    // leave the filter running on an uncalibrated IMU until the next dock
    // (Voie C Test 1 on 2026-04-24 caught this: container restart after
    // image update + no dock since → gyro_z bias 0.05 rad/s, fusion yaw
    // drifting 2.9°/s, σ_xy inflated to 50 cm because GPS innovations kept
    // getting rejected by the outlier gate).
    imu_cal_persist_path_ =
        declare_parameter<std::string>("imu_cal_persist_path", "/ros2_ws/maps/imu_calibration.txt");
    // Auto-calibrate at rest: if the robot is stationary and NOT charging
    // for this many seconds AND we don't have a calibration yet, trigger
    // the same 20 s sample collection used on dock. Lets the robot recover
    // from boot-without-previous-calibration without requiring a dock.
    imu_cal_auto_rest_sec_ = declare_parameter<double>("imu_cal_auto_rest_sec", 15.0);
    // Periodic recalibration while docked: temperature drifts between a
    // morning dock session and afternoon mowing, so bias drift accumulates
    // (Voie C Test 2026-04-24 measured ~0.003 rad/s residual after dock cal,
    // → 7°/min of yaw drift during mowing). Re-running the cal every N
    // seconds while the robot is stationary on dock keeps the offsets
    // fresh for temperature. Set to 0 to disable.
    //
    // Default lowered 600 → 60 s (2026-05-03): on-robot measurement showed
    // the WT901 raw gyro_z bias drifted from -3.05°/s (calibration mean)
    // to -4.36°/s (live) within seconds of completing a calibration —
    // a 1.3°/s shift well above the gyro's noise floor. The thermal
    // settling time of the chassis (mower ESC heat soaking the IMU
    // board) means the calibration captured during a short dock-stop
    // becomes stale within minutes. 60 s keeps the offset within the
    // bias's first-order time constant on this robot. Cost is trivial:
    // the docked-stationary gate (line 460) makes it a no-op when the
    // robot is moving, and a single recal is ~2.2 s of sample collection
    // at 91 Hz × 200 samples.
    imu_cal_periodic_recal_sec_ = declare_parameter<double>("imu_cal_periodic_recal_sec", 60.0);

    RCLCPP_INFO(get_logger(),
                "Parameters: serial_port=%s baud_rate=%d heartbeat_rate=%.1f Hz "
                "publish_rate=%.1f Hz high_level_rate=%.1f Hz",
                serial_port_path_.c_str(),
                baud_rate_,
                heartbeat_rate_,
                publish_rate_,
                high_level_rate_);
  }

  void create_publishers()
  {
    pub_status_ = create_publisher<mowgli_interfaces::msg::Status>("~/status", rclcpp::QoS(10));
    pub_emergency_ =
        create_publisher<mowgli_interfaces::msg::Emergency>("~/emergency", rclcpp::QoS(10));
    pub_power_ = create_publisher<mowgli_interfaces::msg::Power>("~/power", rclcpp::QoS(10));
    // RELIABLE, not SensorDataQoS — robot_localization's EKF nodes
    // subscribe RELIABLE and refuse BEST_EFFORT publishers with
    // "incompatible QoS policy", which starves the filter of IMU/wheel data.
    pub_imu_ = create_publisher<sensor_msgs::msg::Imu>("~/imu/data_raw", rclcpp::QoS(10));
    // Raw magnetometer µT → Tesla for mag_yaw_publisher (calibration
    // gated on /ros2_ws/maps/mag_calibration.yaml). Also used
    // diagnostically to inspect the chip and see chassis distortion.
    pub_mag_raw_ =
        create_publisher<sensor_msgs::msg::MagneticField>("~/imu/mag_raw", rclcpp::QoS(10));
    // ~/wheel_odom + ~/wheel_ticks are created by odometry_publisher_'s own
    // constructor (odometry_publisher_(*this) in the init-list) — see
    // odometry_publisher.hpp.
    pub_battery_state_ =
        create_publisher<sensor_msgs::msg::BatteryState>("/battery_state", rclcpp::QoS(10));
    // Dock heading: publish dock_yaw at 1 Hz while charging so
    // dock_yaw_to_set_pose.py (robot_localization helper) can bridge it
    // into ekf_map/ekf_odom set_pose. Remapped to /gnss/heading in
    // mowgli.launch.py. Stops automatically when the robot undocks.
    pub_dock_heading_ = create_publisher<sensor_msgs::msg::Imu>("~/dock_heading", rclcpp::QoS(10));
    timer_dock_heading_ = create_wall_timer(std::chrono::seconds(1),
                                            [this]()
                                            {
                                              publish_dock_heading();
                                            });
  }

  void create_subscribers()
  {
    sub_cmd_vel_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        "~/cmd_vel",
        rclcpp::SystemDefaultsQoS(),
        [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
        {
          on_cmd_vel(msg);
        });

    // /gps/status is the authoritative fix-state source for the GPS-LOCK LED.
    // Use the same typed status helper as the behavior tree so a missing or
    // rejected NavSatFix covariance cannot disagree with the BT/GUI RTK state.
    sub_gnss_status_ = create_subscription<mowgli_interfaces::msg::GnssStatus>(
        "/gps/status",
        rclcpp::QoS(10),
        [this](mowgli_interfaces::msg::GnssStatus::ConstSharedPtr msg)
        {
          gps_quality_ = mowgli_interfaces::gnss_status_utils::HardwareQualityPercent(*msg);
        });

    // Mirror the behavior tree's high-level state to the firmware so it
    // knows when to accept cmd_vel (mode != IDLE).
    sub_hl_status_ = create_subscription<mowgli_interfaces::msg::HighLevelStatus>(
        "/behavior_tree_node/high_level_status",
        rclcpp::QoS(10),
        [this](mowgli_interfaces::msg::HighLevelStatus::ConstSharedPtr msg)
        {
          const uint8_t previous_mode = current_mode_;
          const std::string previous_state_name = current_mode_state_name_;
          current_mode_ = msg->state;
          current_mode_state_name_ = msg->state_name;
          if (previous_mode != current_mode_ || previous_state_name != current_mode_state_name_)
          {
            RCLCPP_INFO(get_logger(),
                        "hardware_bridge received HighLevelStatus: state=%u (%s), state_name='%s'",
                        current_mode_,
                        high_level_mode_name(current_mode_),
                        current_mode_state_name_.c_str());
            send_high_level_state();
          }
          RCLCPP_DEBUG(get_logger(),
                       "High-level mode updated to %u (%s)",
                       msg->state,
                       msg->state_name.c_str());
        });
  }

  void create_services()
  {
    srv_mower_control_ = create_service<mowgli_interfaces::srv::MowerControl>(
        "~/mower_control",
        [this](const std::shared_ptr<mowgli_interfaces::srv::MowerControl::Request> req,
               std::shared_ptr<mowgli_interfaces::srv::MowerControl::Response> res)
        {
          on_mower_control(req, res);
        });

    srv_emergency_stop_ = create_service<mowgli_interfaces::srv::EmergencyStop>(
        "~/emergency_stop",
        [this](const std::shared_ptr<mowgli_interfaces::srv::EmergencyStop::Request> req,
               std::shared_ptr<mowgli_interfaces::srv::EmergencyStop::Response> res)
        {
          on_emergency_stop(req, res);
        });

    // Reboot the STM32 board (NVIC_SystemReset). Recovers a wedged firmware
    // state — e.g. the IMU emitting NaN — without a manual power-cycle.
    srv_reboot_board_ = create_service<std_srvs::srv::Trigger>(
        "~/reboot_board",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res)
        {
          on_reboot_board(req, res);
        });

    srv_set_firmware_debug_ = create_service<std_srvs::srv::SetBool>(
        "~/set_firmware_debug",
        [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
               std::shared_ptr<std_srvs::srv::SetBool::Response> res)
        {
          on_set_firmware_debug(req, res);
        });
  }

  // Services this node CALLS (as opposed to create_services() above, which
  // creates services this node HOSTS). Kept separate for that distinction.
  void create_service_clients()
  {
    // START/HOME panel buttons → the same high_level_control service the GUI
    // calls (task #31 — see handle_ui_event). Client-side only; the service
    // itself is served by behavior_tree_node as "~/high_level_control", which
    // (launch files set name="behavior_tree_node", no remapping) resolves to
    // the ABSOLUTE name below — confirmed against the GUI Go backend, which
    // calls this exact string from 7+ sites (gui/pkg/providers/homekit.go,
    // scheduler.go, gui/pkg/api/mowglinext.go, and their tests).
    client_high_level_control_ = create_client<mowgli_interfaces::srv::HighLevelControl>(
        "/behavior_tree_node/high_level_control");
  }

  void open_serial_port()
  {
    serial_ = std::make_unique<SerialPort>(serial_port_path_, baud_rate_);
    reset_cause_log_pending_ = true;

    packet_handler_.set_callback(
        [this](const uint8_t* data, std::size_t len)
        {
          on_packet_received(data, len);
        });

    if (!serial_->open())
    {
      RCLCPP_ERROR(get_logger(),
                   "Failed to open serial port '%s' at %d baud. "
                   "The node will retry on each read tick.",
                   serial_port_path_.c_str(),
                   baud_rate_);
    }
    else
    {
      RCLCPP_INFO(get_logger(),
                  "Opened serial port '%s' at %d baud.",
                  serial_port_path_.c_str(),
                  baud_rate_);
    }
    // Seed the RX watchdog so a freshly-(re)opened port gets a full grace
    // window before the no-data check can fire.
    last_serial_rx_time_ = now();
  }

  void create_timers()
  {
    // Serial read / packet dispatch.
    const auto read_period_ms = std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_));
    timer_read_ = create_wall_timer(read_period_ms,
                                    [this]()
                                    {
                                      read_serial_tick();
                                    });

    // Heartbeat.
    const auto hb_period_ms = std::chrono::milliseconds(static_cast<int>(1000.0 / heartbeat_rate_));
    timer_heartbeat_ =
        create_wall_timer(hb_period_ms,
                          [this]()
                          {
                            // On startup, send emergency release for the first few
                            // heartbeats to clear any watchdog-latched emergency
                            // from the container restart gap.
                            if (startup_release_count_ > 0)
                            {
                              emergency_release_pending_ = true;
                              --startup_release_count_;
                            }
                            send_heartbeat();
                            // Re-push the drive PID (and the yaw-loop PID,
                            // same burst — task #34) on the first few
                            // heartbeats after each (re)connect so the
                            // firmware (which has no config persistence)
                            // gets the host's gains even if one packet is
                            // lost during USB re-enumeration.
                            if (pid_resend_count_ > 0 && serial_->is_open())
                            {
                              send_drive_pid();
                              send_yaw_pid();
                              --pid_resend_count_;
                            }
                            // Firmware version handshake: ask the
                            // board for its protocol/firmware
                            // version on the first few heartbeats
                            // after each (re)connect, then enforce
                            // a timeout so firmware too old to
                            // answer is flagged incompatible.
                            service_firmware_handshake();
                            if (config_control_resend_count_ > 0 && serial_->is_open())
                            {
                              send_config_request();
                              --config_control_resend_count_;
                            }
                          });

    // High-level state.
    const auto hl_period_ms =
        std::chrono::milliseconds(static_cast<int>(1000.0 / high_level_rate_));
    timer_high_level_ = create_wall_timer(hl_period_ms,
                                          [this]()
                                          {
                                            send_high_level_state();
                                          });
  }

  // ---------------------------------------------------------------------------
  // Serial I/O
  // ---------------------------------------------------------------------------

  void reset_serial_dependent_state()
  {
    packet_handler_.reset_receive_state();
    odometry_publisher_.reset();
  }

  void close_serial_for_reconnect()
  {
    reset_serial_dependent_state();
    serial_->close();
  }

  void read_serial_tick()
  {
    // If the port was never opened or was closed due to an error / dead-link
    // watchdog, attempt to (re)open it. open() re-resolves the device path, so
    // this picks up a new ttyACM after a USB re-enumeration.
    if (!serial_->is_open())
    {
      if (!serial_->open())
      {
        return;  // Still not open; will retry next tick.
      }
      last_serial_rx_time_ = now();
      reset_serial_dependent_state();
      reset_cause_log_pending_ = true;
      clear_firmware_debug_state_for_reconnect();
      // The firmware just re-enumerated (flash / reboot / replug) and is back on
      // its compile-time default gains — re-arm the drive-PID push so the host's
      // values are restored over the next few heartbeats.
      pid_resend_count_ = 5;
      // Re-run the firmware version handshake: a re-enumeration may be a
      // reflash to a different firmware, so re-ask and re-evaluate compatibility.
      rearm_firmware_handshake();
      RCLCPP_INFO(get_logger(), "Serial port re-opened successfully.");
    }

    constexpr std::size_t kReadBufSize = 512u;
    uint8_t buf[kReadBufSize];

    // Drain all available bytes in one tick.
    while (true)
    {
      const ssize_t n = serial_->read(buf, kReadBufSize);
      if (n < 0)
      {
        // Hard read error (e.g. the USB CDC device was removed/re-enumerated by
        // a firmware flash or board reboot): the fd is dead. Close now so the
        // next tick reopens and re-resolves /dev/mowgli to the live device.
        RCLCPP_WARN_THROTTLE(get_logger(),
                             *get_clock(),
                             2000,
                             "Serial read error — closing port to reconnect.");
        close_serial_for_reconnect();
        return;
      }
      if (n == 0)
      {
        break;  // No more data available this tick.
      }
      packet_handler_.feed(buf, static_cast<std::size_t>(n));
      last_serial_rx_time_ = now();
    }

    // Dead-link watchdog: the STM32 streams continuously whenever it is up, so
    // a multi-second RX gap on an "open" port means the endpoint vanished (a
    // re-enumeration that left the fd nominally open but mute). Close it so the
    // next tick reopens and reconnects — this is what makes a firmware flash or
    // board reboot self-heal instead of wedging the bridge.
    if (serial_rx_timeout_s_ > 0.0 &&
        (now() - last_serial_rx_time_).seconds() > serial_rx_timeout_s_)
    {
      RCLCPP_WARN(get_logger(),
                  "No serial data for %.1f s — reopening %s to reconnect to the STM32.",
                  serial_rx_timeout_s_,
                  serial_port_path_.c_str());
      if (firmware_debug_requested_ || firmware_debug_enabled_)
      {
        RCLCPP_WARN(get_logger(),
                    "[FW_DIAG] No serial data for %.1f s — reopening %s to reconnect to the STM32.",
                    serial_rx_timeout_s_,
                    serial_port_path_.c_str());
      }
      close_serial_for_reconnect();
    }
  }

  bool send_raw_packet(const uint8_t* data, std::size_t len)
  {
    if (!serial_->is_open())
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Cannot send: serial port not open.");
      if (firmware_debug_requested_ || firmware_debug_enabled_)
      {
        RCLCPP_WARN_THROTTLE(get_logger(),
                             *get_clock(),
                             5000,
                             "[FW_DIAG] Cannot send: serial port not open.");
      }
      return false;
    }

    const std::vector<uint8_t> frame = packet_handler_.encode_packet(data, len);
    const ssize_t written = serial_->write_all(frame.data(), frame.size());

    if (written < 0 || static_cast<std::size_t>(written) != frame.size())
    {
      RCLCPP_WARN(
          get_logger(),
          "Short write or error sending packet (%zd/%zu bytes) — closing port to reconnect.",
          written,
          frame.size());
      if (firmware_debug_requested_ || firmware_debug_enabled_)
      {
        RCLCPP_WARN(get_logger(),
                    "[FW_DIAG] Short write or error sending packet (%zd/%zu bytes) — closing "
                    "port to reconnect.",
                    written,
                    frame.size());
      }
      close_serial_for_reconnect();
      return false;
    }
    return true;
  }

  // ---------------------------------------------------------------------------
  // Packet dispatch (STM32 → ROS2)
  // ---------------------------------------------------------------------------

  void on_packet_received(const uint8_t* data, std::size_t len)
  {
    if (len == 0)
    {
      return;
    }

    const auto type = static_cast<PacketId>(data[0]);

    switch (type)
    {
      case PACKET_ID_LL_STATUS:
        handle_status(data, len);
        break;
      case PACKET_ID_LL_RESET_CAUSE:
        handle_reset_cause(data, len);
        break;
      case PACKET_ID_LL_IMU:
        handle_imu(data, len);
        break;
      case PACKET_ID_LL_UI_EVENT:
        handle_ui_event(data, len);
        break;
      case PACKET_ID_LL_ODOMETRY:
        handle_odometry(data, len);
        break;
      case PACKET_ID_LL_BLADE_STATUS:
        handle_blade_status(data, len);
        break;
      case PACKET_ID_LL_HIGH_LEVEL_CONFIG_RSP:
        handle_config_rsp(data, len);
        break;
      default:
        RCLCPP_DEBUG(get_logger(), "Unhandled packet type 0x%02X (len=%zu)", data[0], len);
        break;
    }
  }

  void handle_reset_cause(const uint8_t* data, std::size_t len)
  {
    if (len < 4u)
    {
      RCLCPP_WARN(get_logger(),
                  "Reset-cause packet too short: %zu < %zu",
                  len,
                  static_cast<std::size_t>(4u));
      return;
    }

    const bool has_watchdog_stage = len >= sizeof(LlResetCause);
    const uint8_t reset_cause = data[1];
    const uint8_t last_stage_before_reset = has_watchdog_stage ? data[2] : WATCHDOG_STAGE_NONE;

    const bool cause_changed = !reset_cause_seen_ || reset_cause != last_reset_cause_;
    const bool stage_changed = !reset_stage_seen_ ||
                               has_watchdog_stage != last_reset_has_watchdog_stage_ ||
                               last_stage_before_reset != last_reset_stage_before_reset_;

    last_reset_cause_ = reset_cause;
    last_reset_cause_name_ = reset_cause_name(reset_cause);
    reset_cause_seen_ = true;
    last_reset_stage_before_reset_ = last_stage_before_reset;
    last_reset_stage_name_ = watchdog_stage_name(last_stage_before_reset);
    last_reset_has_watchdog_stage_ = has_watchdog_stage;
    reset_stage_seen_ = true;

    if (reset_cause_log_pending_ || cause_changed || stage_changed)
    {
      if (last_reset_cause_ == RESET_CAUSE_WWDG && last_reset_has_watchdog_stage_)
      {
        RCLCPP_INFO(get_logger(),
                    "STM32 boot reset cause: %s — %s (last stage before reset: %s)",
                    last_reset_cause_name_.c_str(),
                    reset_cause_description(last_reset_cause_),
                    last_reset_stage_name_.c_str());
        if (firmware_debug_requested_ || firmware_debug_enabled_)
        {
          RCLCPP_INFO(get_logger(),
                      "[FW_DIAG] STM32 boot reset cause: %s — %s (last stage before reset: %s)",
                      last_reset_cause_name_.c_str(),
                      reset_cause_description(last_reset_cause_),
                      last_reset_stage_name_.c_str());
        }
      }
      else
      {
        RCLCPP_INFO(get_logger(),
                    "STM32 boot reset cause: %s — %s",
                    last_reset_cause_name_.c_str(),
                    reset_cause_description(last_reset_cause_));
        if (firmware_debug_requested_ || firmware_debug_enabled_)
        {
          RCLCPP_INFO(get_logger(),
                      "[FW_DIAG] STM32 boot reset cause: %s — %s",
                      last_reset_cause_name_.c_str(),
                      reset_cause_description(last_reset_cause_));
        }
      }
      reset_cause_log_pending_ = false;
    }
  }

  void handle_status(const uint8_t* data, std::size_t len)
  {
    if (len < sizeof(LlStatus))
    {
      RCLCPP_WARN(get_logger(), "Status packet too short: %zu < %zu", len, sizeof(LlStatus));
      return;
    }

    LlStatus pkt{};
    std::memcpy(&pkt, data, sizeof(LlStatus));

    const auto stamp = now();

    // ---- Status message ----
    {
      auto msg = mowgli_interfaces::msg::Status{};
      msg.stamp = stamp;
      msg.mower_status = (pkt.status_bitmask & STATUS_BIT_INITIALIZED) != 0u
                             ? mowgli_interfaces::msg::Status::MOWER_STATUS_OK
                             : mowgli_interfaces::msg::Status::MOWER_STATUS_INITIALIZING;
      msg.reset_cause = last_reset_cause_;
      msg.reset_cause_name = last_reset_cause_name_;
      msg.raspberry_pi_power = (pkt.status_bitmask & STATUS_BIT_RASPI_POWER) != 0u;
      const bool was_charging = is_charging_;
      is_charging_ = (pkt.status_bitmask & STATUS_BIT_CHARGING) != 0u;
      msg.is_charging = is_charging_;

      // Dock heading anchor trigger: on charging transition, start the
      // wide-σ dock_heading window so dock_yaw_to_set_pose picks up the
      // current heading. The robot_localization stack does not require a
      // filter reset — set_pose on both EKFs is issued by
      // dock_yaw_to_set_pose when it sees the rising edge.
      if (is_charging_ && !was_charging)
      {
        charging_anchor_start_ = now();
        charging_anchor_active_ = true;
        RCLCPP_INFO(get_logger(),
                    "Charging transition: dock_heading anchor window "
                    "(%.1fs) opened.",
                    kChargingAnchorWindowSec);
      }

      // Start IMU calibration when charging and not already calibrating.
      // Triggers on: (1) dock transition, (2) first status packet if
      // already on dock at boot. A freshly-docked cal overrides a loaded-
      // from-file cal because the dock is the most trustworthy at-rest
      // environment (bias may have drifted since the file was written).
      if (is_charging_ && !imu_cal_collecting_ &&
          (!imu_cal_ready_ || (!was_charging && imu_cal_ready_)))
      {
        start_imu_calibration(was_charging ? "on dock (boot)" : "dock transition");
      }

      // Periodic recalibration while docked. The motor-controller chip's
      // temperature shifts between a morning charge and afternoon mowing;
      // bias drifts accordingly (measured ~0.003 rad/s residual → 7°/min
      // yaw-integration error). While the robot sits stationary on dock,
      // refresh the cal every imu_cal_periodic_recal_sec_ seconds so the
      // offsets match current-temperature.
      if (is_charging_ && imu_cal_ready_ && !imu_cal_collecting_ &&
          odometry_publisher_.wheels_stationary() && imu_cal_periodic_recal_sec_ > 0.0 &&
          imu_cal_last_completed_.nanoseconds() > 0)
      {
        const double age_sec = (now() - imu_cal_last_completed_).seconds();
        if (age_sec >= imu_cal_periodic_recal_sec_)
        {
          start_imu_calibration("periodic recal while docked");
        }
      }
      msg.rain_detected = (pkt.status_bitmask & STATUS_BIT_RAIN) != 0u;
      msg.sound_module_available = (pkt.status_bitmask & STATUS_BIT_SOUND_AVAIL) != 0u;
      msg.sound_module_busy = (pkt.status_bitmask & STATUS_BIT_SOUND_BUSY) != 0u;
      msg.ui_board_available = (pkt.status_bitmask & STATUS_BIT_UI_AVAIL) != 0u;
      // Legacy field: kept for backward compatibility with existing GUI /
      // diagnostics expectations. It reflects blade-controller activity, not a
      // traction PAC5210 power/arm state (the STM32 status packet does not
      // currently report that signal).
      msg.esc_power = mow_enabled_ || blade_active_;
      // Blade motor fields from live telemetry
      msg.mow_enabled = mow_enabled_;
      msg.firmware_debug_enabled = firmware_debug_enabled_;
      msg.mower_esc_status = blade_active_ ? 1u : 0u;
      msg.mower_motor_rpm = blade_rpm_;
      msg.mower_motor_temperature = blade_temperature_;
      msg.mower_esc_current = blade_esc_current_;
      // Firmware version handshake result (image <-> firmware compatibility).
      msg.firmware_version = fw_version_str_;
      msg.firmware_protocol_version = fw_protocol_version_;
      msg.firmware_compatible = fw_compatible_;
      pub_status_->publish(msg);
    }

    // ---- Dock heading ----
    // Dock heading is published at 1 Hz on ~/dock_heading while charging
    // (see publish_dock_heading()). dock_pose_yaw is also used for SLAM
    // map_start_pose (on saved maps) and by the BT for heading reference.

    // ---- Emergency message ----
    {
      auto msg = mowgli_interfaces::msg::Emergency{};
      msg.stamp = stamp;
      const bool stop_active = (pkt.emergency_bitmask & EMERGENCY_BIT_STOP) != 0u;
      const bool lift_active = (pkt.emergency_bitmask & EMERGENCY_BIT_LIFT) != 0u;
      const bool latch_active = (pkt.emergency_bitmask & EMERGENCY_BIT_LATCH) != 0u;

      if (lift_recovery_mode_ && lift_active && !stop_active)
      {
        // Lift recovery mode: blade off, wheels keep running, no emergency.
        // Firmware may set its own emergency latch — auto-release it.
        msg.active_emergency = false;
        msg.latched_emergency = false;
        fw_latched_emergency_ = false;
        msg.lift_warning = true;

        // Track lift duration
        if (!lift_detected_)
        {
          lift_detected_ = true;
          lift_start_time_ = now();
          blade_was_enabled_before_lift_ = mow_enabled_;
          if (mow_enabled_)
          {
            send_blade_command(0, 0);
            RCLCPP_WARN(get_logger(), "LIFT detected — blade disabled (recovery mode)");
          }
        }
        msg.lift_duration_sec = static_cast<float>((now() - lift_start_time_).seconds());
        msg.reason = "Lift (blade off, recovery mode)";

        // Auto-release firmware latch caused by lift
        if (latch_active)
        {
          emergency_release_pending_ = true;
        }
      }
      else
      {
        // Normal mode or stop button: full emergency
        msg.active_emergency = stop_active || lift_active;
        msg.latched_emergency = latch_active;
        fw_latched_emergency_ = latch_active;
        msg.lift_warning = false;
        msg.lift_duration_sec = 0.0f;

        if (stop_active)
          msg.reason = "STOP button";
        else if (lift_active)
          msg.reason = "Lift detected";
        else if (latch_active)
          msg.reason = "Latched (press play button to release)";
      }

      // Lift cleared — resume blade after delay
      if (lift_detected_ && !lift_active)
      {
        lift_detected_ = false;
        if (blade_was_enabled_before_lift_)
        {
          lift_cleared_time_ = now();
          waiting_blade_resume_ = true;
          RCLCPP_INFO(get_logger(),
                      "LIFT cleared — blade will resume after %.1f s",
                      lift_blade_resume_delay_sec_);
        }
      }

      if (waiting_blade_resume_)
      {
        const double since_clear = (now() - lift_cleared_time_).seconds();
        if (since_clear >= lift_blade_resume_delay_sec_)
        {
          send_blade_command(1, 0);
          blade_was_enabled_before_lift_ = false;
          waiting_blade_resume_ = false;
          RCLCPP_INFO(get_logger(), "LIFT recovery — blade re-enabled");
        }
      }

      pub_emergency_->publish(msg);
    }

    // ---- Power message ----
    {
      auto msg = mowgli_interfaces::msg::Power{};
      msg.stamp = stamp;
      msg.v_charge = pkt.v_charge;
      msg.v_battery = pkt.v_system;
      msg.charge_current = pkt.charging_current;
      msg.charger_enabled = (pkt.status_bitmask & STATUS_BIT_CHARGING) != 0u;
      msg.charger_status = msg.charger_enabled ? "charging" : "idle";
      pub_power_->publish(msg);
    }

    // ---- BatteryState message (for opennav_docking charge detection) ----
    {
      auto msg = sensor_msgs::msg::BatteryState{};
      msg.header.stamp = stamp;
      msg.header.frame_id = "base_link";
      msg.voltage = pkt.v_system;
      // SimpleChargingDock checks current > charging_threshold for both
      // isDocked() and hasStoppedCharging().  Firmware reports negative
      // current when charging and positive when discharging.  Publish
      // abs(current) when charging so the threshold is exceeded, and
      // 0.0 when not charging so hasStoppedCharging() detects the
      // transition after undocking.
      msg.current = is_charging_ ? std::abs(pkt.charging_current) : 0.0f;
      msg.percentage = static_cast<float>(pkt.batt_percentage) / 100.0f;
      msg.power_supply_status =
          is_charging_ ? sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING
                       : sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
      msg.present = true;
      pub_battery_state_->publish(msg);
    }
  }

  // ---------------------------------------------------------------------------
  // IMU calibration persistence
  //
  // The at-dock calibration lives only in RAM — a ros2 container restart
  // (e.g. pulling a new image) throws it away, and the filter then runs on
  // the raw gyro until the robot next docks. The raw WT901 gyro bias is
  // around 0.05 rad/s → 2.9°/s phantom yaw drift → GPS-innovation rejection
  // spiral → σ_xy inflates to 50 cm while the robot is literally still.
  //
  // We persist offsets + variances to a small text file on the maps volume
  // (survives container/host restarts) and load them at boot. Line format:
  //   # mowgli_imu_calibration_v1
  //   <timestamp_unix> <n_samples>
  //   <off_ax> <off_ay> <off_gx> <off_gy> <off_gz>
  //   <cov_ax> <cov_ay> <cov_gx> <cov_gy> <cov_gz>
  //   <implied_pitch_deg> <implied_roll_deg>
  // Human-inspectable, no YAML/JSON dependency, easy to delete if bad.
  // ---------------------------------------------------------------------------

  void persist_imu_calibration(double implied_pitch_deg, double implied_roll_deg)
  {
    std::ofstream f(imu_cal_persist_path_, std::ios::trunc);
    if (!f.is_open())
    {
      RCLCPP_WARN(get_logger(),
                  "Could not open %s for write — IMU cal NOT persisted.",
                  imu_cal_persist_path_.c_str());
      return;
    }
    f << "# mowgli_imu_calibration_v1\n";
    f << std::fixed;
    f.precision(6);
    f << static_cast<int64_t>(std::time(nullptr)) << " " << imu_cal_count_ << "\n";
    f << imu_cal_offset_ax_ << " " << imu_cal_offset_ay_ << " " << imu_cal_offset_gx_ << " "
      << imu_cal_offset_gy_ << " " << imu_cal_offset_gz_ << "\n";
    f << imu_cal_cov_ax_ << " " << imu_cal_cov_ay_ << " " << imu_cal_cov_gx_ << " "
      << imu_cal_cov_gy_ << " " << imu_cal_cov_gz_ << "\n";
    f << implied_pitch_deg << " " << implied_roll_deg << "\n";
    f.close();
    RCLCPP_INFO(get_logger(), "Persisted IMU calibration to %s", imu_cal_persist_path_.c_str());
  }

  void load_persisted_imu_calibration()
  {
    std::ifstream f(imu_cal_persist_path_);
    if (!f.is_open())
    {
      RCLCPP_INFO(get_logger(),
                  "No persisted IMU calibration at %s — will auto-cal on dock "
                  "or after %.0f s stationary off-dock.",
                  imu_cal_persist_path_.c_str(),
                  imu_cal_auto_rest_sec_);
      return;
    }
    std::string header;
    std::getline(f, header);
    if (header != "# mowgli_imu_calibration_v1")
    {
      RCLCPP_WARN(get_logger(),
                  "Persisted IMU cal header mismatch (got '%s') — ignoring, "
                  "will re-calibrate.",
                  header.c_str());
      return;
    }
    int64_t ts = 0;
    int n = 0;
    if (!(f >> ts >> n) ||
        !(f >> imu_cal_offset_ax_ >> imu_cal_offset_ay_ >> imu_cal_offset_gx_ >>
          imu_cal_offset_gy_ >> imu_cal_offset_gz_) ||
        !(f >> imu_cal_cov_ax_ >> imu_cal_cov_ay_ >> imu_cal_cov_gx_ >> imu_cal_cov_gy_ >>
          imu_cal_cov_gz_))
    {
      RCLCPP_WARN(get_logger(), "Persisted IMU cal parse failed — ignoring, will re-calibrate.");
      return;
    }
    // Sanity: if a previous cal ran while the robot was actually rotating
    // (false at-rest detection, dock glitch, whatever), the saved gyro
    // offset will be huge. Reject to force a clean re-cal rather than
    // systematically miscorrecting every /imu/data sample. 0.2 rad/s ~=
    // 11.5°/s; real chip bias on WT901 is empirically < 0.1 rad/s.
    const double max_plausible = 0.2;
    if (std::abs(imu_cal_offset_gx_) > max_plausible ||
        std::abs(imu_cal_offset_gy_) > max_plausible ||
        std::abs(imu_cal_offset_gz_) > max_plausible)
    {
      RCLCPP_WARN(get_logger(),
                  "Persisted IMU cal rejected — gyro offset implausible "
                  "[%.4f, %.4f, %.4f] rad/s > %.2f. Will re-calibrate.",
                  imu_cal_offset_gx_,
                  imu_cal_offset_gy_,
                  imu_cal_offset_gz_,
                  max_plausible);
      imu_cal_offset_gx_ = imu_cal_offset_gy_ = imu_cal_offset_gz_ = 0.0;
      imu_cal_offset_ax_ = imu_cal_offset_ay_ = 0.0;
      return;
    }
    imu_cal_count_ = n;
    imu_cal_ready_ = true;
    imu_cal_loaded_from_file_ = true;
    imu_cal_last_completed_ = now();  // grace period before periodic recal fires
    const double age_hours =
        (static_cast<double>(std::time(nullptr)) - static_cast<double>(ts)) / 3600.0;
    RCLCPP_INFO(get_logger(),
                "Loaded IMU calibration from %s (%.1f h old, %d samples) — "
                "gyro offset [%.5f, %.5f, %.5f] rad/s, "
                "accel offset [%.4f, %.4f] m/s². "
                "Will re-calibrate at next dock.",
                imu_cal_persist_path_.c_str(),
                age_hours,
                n,
                imu_cal_offset_gx_,
                imu_cal_offset_gy_,
                imu_cal_offset_gz_,
                imu_cal_offset_ax_,
                imu_cal_offset_ay_);
  }

  void start_imu_calibration(const char* reason)
  {
    imu_cal_ready_ = false;
    imu_cal_collecting_ = true;
    imu_cal_count_ = 0;
    imu_cal_sum_ax_ = imu_cal_sum_ay_ = imu_cal_sum_az_ = 0.0;
    imu_cal_sum_gx_ = imu_cal_sum_gy_ = imu_cal_sum_gz_ = 0.0;
    imu_cal_samples_ax_.clear();
    imu_cal_samples_ay_.clear();
    imu_cal_samples_gx_.clear();
    imu_cal_samples_gy_.clear();
    imu_cal_samples_gz_.clear();
    RCLCPP_INFO(get_logger(),
                "Starting IMU calibration (%d samples) — %s",
                imu_cal_samples_,
                reason);
  }

  void handle_imu(const uint8_t* data, std::size_t len)
  {
    if (len < sizeof(LlImu))
    {
      RCLCPP_WARN(get_logger(), "IMU packet too short: %zu < %zu", len, sizeof(LlImu));
      return;
    }

    LlImu pkt{};
    std::memcpy(&pkt, data, sizeof(LlImu));

    auto msg = sensor_msgs::msg::Imu{};
    // Smoothed timestamp from the firmware-host clock fitter.
    // pkt.dt_millis is the firmware-reported interval since the
    // previous IMU packet (free of USB jitter); the fitter
    // accumulates it into a virtual firmware clock and regresses
    // a linear map to the host clock over the last ~2 s of
    // packets. Result: published stamps are jitter-free even when
    // host-side scheduling delays the actual decode by 5-20 ms.
    msg.header.stamp = imu_clock_fit_.Ingest(pkt.dt_millis, now());
    msg.header.frame_id = "imu_link";

    double ax = static_cast<double>(pkt.acceleration_mss[0]);
    double ay = static_cast<double>(pkt.acceleration_mss[1]);
    double az = static_cast<double>(pkt.acceleration_mss[2]);
    double gx = static_cast<double>(pkt.gyro_rads[0]);
    double gy = static_cast<double>(pkt.gyro_rads[1]);
    double gz = static_cast<double>(pkt.gyro_rads[2]);

    // Auto-calibrate off-dock when stationary. Covers the "image pulled,
    // container restarted, robot has not docked since" case that leaves
    // the filter running on raw gyro (2-3°/s bias → yaw diverges in seconds).
    // Gate: no cal yet + not charging + wheels stationary for auto_rest_sec.
    if (!imu_cal_ready_ && !imu_cal_collecting_ && !is_charging_)
    {
      if (odometry_publisher_.wheels_stationary())
      {
        if (imu_cal_at_rest_since_.nanoseconds() == 0)
        {
          imu_cal_at_rest_since_ = now();
        }
        else
        {
          const double at_rest_sec = (now() - imu_cal_at_rest_since_).seconds();
          if (at_rest_sec >= imu_cal_auto_rest_sec_)
          {
            start_imu_calibration("off-dock auto-cal after stationary window");
            imu_cal_at_rest_since_ = rclcpp::Time{};
          }
        }
      }
      else
      {
        imu_cal_at_rest_since_ = rclcpp::Time{};
      }
    }

    // IMU calibration: collect samples while docked and idle, then compute
    // offsets (mean) and covariances (variance). Same algorithm as firmware
    // IMU_CalibrateExternal() but run on the ROS2 side each time the robot
    // docks — catches residual drift the boot calibration missed.
    // Accel Z is not calibrated (preserves gravity for sensor fusion), but
    // we still sum it so we can report implied mounting pitch/roll.
    //
    // SAFETY/QUALITY: only accumulate while the robot is genuinely AT REST on
    // the dock. There is no stationarity/charging re-check once collection
    // starts, so if COMMAND_START undocks mid-window the real motion is averaged
    // into the offset, subtracted from every future /imu/data_raw sample, and
    // persisted — a constant gyro bias feeding fusion_graph's gyro factor → yaw
    // drift (the load gate only rejects |gyro|>0.2 rad/s, so moderate corruption
    // passes). Abort the in-progress cal the moment the robot moves or leaves
    // the charger, and keep the last completed calibration instead.
    if (imu_cal_collecting_ && (!odometry_publisher_.wheels_stationary() || !is_charging_))
    {
      imu_cal_collecting_ = false;
      imu_cal_count_ = 0;
      // Restore the previously-completed cal if there was one
      // (start_imu_calibration cleared imu_cal_ready_); otherwise stay
      // uncalibrated (raw passthrough) rather than apply a corrupt partial.
      if (imu_cal_last_completed_.nanoseconds() > 0)
      {
        imu_cal_ready_ = true;
      }
      RCLCPP_WARN(get_logger(),
                  "IMU calibration aborted mid-collection (%s) — robot not at rest; "
                  "keeping previous calibration",
                  !is_charging_ ? "left charger" : "wheels moving");
    }
    else if (imu_cal_collecting_)
    {
      imu_cal_sum_ax_ += ax;
      imu_cal_sum_ay_ += ay;
      imu_cal_sum_az_ += az;
      imu_cal_sum_gx_ += gx;
      imu_cal_sum_gy_ += gy;
      imu_cal_sum_gz_ += gz;
      imu_cal_samples_ax_.push_back(ax);
      imu_cal_samples_ay_.push_back(ay);
      imu_cal_samples_gx_.push_back(gx);
      imu_cal_samples_gy_.push_back(gy);
      imu_cal_samples_gz_.push_back(gz);
      ++imu_cal_count_;

      if (imu_cal_count_ >= imu_cal_samples_)
      {
        const double n = static_cast<double>(imu_cal_count_);
        imu_cal_offset_ax_ = imu_cal_sum_ax_ / n;
        imu_cal_offset_ay_ = imu_cal_sum_ay_ / n;
        imu_cal_offset_gx_ = imu_cal_sum_gx_ / n;
        imu_cal_offset_gy_ = imu_cal_sum_gy_ / n;
        imu_cal_offset_gz_ = imu_cal_sum_gz_ / n;

        // Compute variance for covariance diagonal
        imu_cal_cov_ax_ = imu_cal_cov_ay_ = 0.0;
        imu_cal_cov_gx_ = imu_cal_cov_gy_ = imu_cal_cov_gz_ = 0.0;
        for (int i = 0; i < imu_cal_count_; ++i)
        {
          imu_cal_cov_ax_ += std::pow(imu_cal_samples_ax_[i] - imu_cal_offset_ax_, 2);
          imu_cal_cov_ay_ += std::pow(imu_cal_samples_ay_[i] - imu_cal_offset_ay_, 2);
          imu_cal_cov_gx_ += std::pow(imu_cal_samples_gx_[i] - imu_cal_offset_gx_, 2);
          imu_cal_cov_gy_ += std::pow(imu_cal_samples_gy_[i] - imu_cal_offset_gy_, 2);
          imu_cal_cov_gz_ += std::pow(imu_cal_samples_gz_[i] - imu_cal_offset_gz_, 2);
        }
        imu_cal_cov_ax_ /= n;
        imu_cal_cov_ay_ /= n;
        imu_cal_cov_gx_ /= n;
        imu_cal_cov_gy_ /= n;
        imu_cal_cov_gz_ /= n;

        imu_cal_collecting_ = false;
        imu_cal_ready_ = true;
        imu_cal_last_completed_ = now();
        RCLCPP_INFO(get_logger(),
                    "IMU calibration complete (%d samples) — "
                    "accel offset [%.4f, %.4f] m/s², "
                    "gyro offset [%.6f, %.6f, %.6f] rad/s, "
                    "accel cov [%.6f, %.6f], gyro cov [%.6f, %.6f, %.6f]",
                    imu_cal_count_,
                    imu_cal_offset_ax_,
                    imu_cal_offset_ay_,
                    imu_cal_offset_gx_,
                    imu_cal_offset_gy_,
                    imu_cal_offset_gz_,
                    imu_cal_cov_ax_,
                    imu_cal_cov_ay_,
                    imu_cal_cov_gx_,
                    imu_cal_cov_gy_,
                    imu_cal_cov_gz_);

        // ---- Implied mounting pitch/roll from at-rest gravity vector ----
        // At rest on a level dock, the chip accel reads [0, 0, g] in the
        // *IMU* frame. If it reads non-zero on X/Y, either (a) the IMU is
        // physically tilted relative to base_link (mounting error), or
        // (b) the chip has a factory accel bias. Assuming the dock is
        // level, the angular offsets below capture the combined effect,
        // which you can feed into mowgli_robot.yaml as imu_pitch / imu_roll
        // so the URDF base_link->imu_link rotation matches reality.
        //   pitch (nose-down = +) = atan2(-ax_raw, az_raw)
        //   roll  (right-down = +) = atan2( ay_raw, az_raw)
        // Magnitudes ≫ ~1° warrant YAML correction; smaller values are
        // likely chip bias and are already removed by this calibration
        // for ax/ay on every future sample.
        const double az_mean = imu_cal_sum_az_ / n;
        const double a_mag = std::sqrt(imu_cal_offset_ax_ * imu_cal_offset_ax_ +
                                       imu_cal_offset_ay_ * imu_cal_offset_ay_ + az_mean * az_mean);
        const double implied_pitch_deg = std::atan2(-imu_cal_offset_ax_, az_mean) * 180.0 / M_PI;
        const double implied_roll_deg = std::atan2(imu_cal_offset_ay_, az_mean) * 180.0 / M_PI;
        RCLCPP_INFO(get_logger(),
                    "Implied mounting tilt: pitch=%.3f°, roll=%.3f° "
                    "(|accel|=%.3f m/s², az_mean=%.3f). "
                    "If magnitudes exceed ~1° set imu_pitch / imu_roll in "
                    "mowgli_robot.yaml and redeploy.",
                    implied_pitch_deg,
                    implied_roll_deg,
                    a_mag,
                    az_mean);

        // Persist to disk so container restarts don't lose the calibration
        // (this is the fix for the "stale gyro bias after pull" class of bugs).
        persist_imu_calibration(implied_pitch_deg, implied_roll_deg);

        // Free sample buffers
        imu_cal_samples_ax_.clear();
        imu_cal_samples_ay_.clear();
        imu_cal_samples_gx_.clear();
        imu_cal_samples_gy_.clear();
        imu_cal_samples_gz_.clear();
      }
    }

    // Apply calibration offsets
    if (imu_cal_ready_)
    {
      ax -= imu_cal_offset_ax_;
      ay -= imu_cal_offset_ay_;
      gx -= imu_cal_offset_gx_;
      gy -= imu_cal_offset_gy_;
      gz -= imu_cal_offset_gz_;
    }

    msg.linear_acceleration.x = ax;
    msg.linear_acceleration.y = ay;
    msg.linear_acceleration.z =
        static_cast<double>(pkt.acceleration_mss[2]);  // Z uncalibrated (gravity)

    msg.angular_velocity.x = gx;
    msg.angular_velocity.y = gy;
    msg.angular_velocity.z = gz;

    // Magnetometer data is ignored — uncalibrated on metal robot chassis,
    // gives ~229° error vs real heading. dock_pose_yaw is a map-frame ENU
    // yaw, set by the GUI "Set Docking Point" calibration or the undock
    // GPS-trajectory fit (NOT a phone-compass heading).

    // Write resolved dock pose to file for SLAM initialization.
    // On fresh map start, navigation.launch.py reads this file to set
    // SLAM's map_start_pose so the map frame aligns with GPS/datum.
    if (is_charging_ && !dock_pose_written_)
    {
      const double dx = dock_x_;
      const double dy = dock_y_;
      // dock_yaw_ is from user config only (magnetometer no longer used)
      std::ofstream f("/tmp/dock_start_pose.txt");
      if (f.is_open())
      {
        f << dx << " " << dy << " " << dock_yaw_ << std::endl;
        dock_pose_written_ = true;
        RCLCPP_INFO(get_logger(),
                    "Wrote dock start pose to /tmp/dock_start_pose.txt: [%.2f, %.2f, %.3f]",
                    dx,
                    dy,
                    dock_yaw_);
      }
    }

    // Flat-ground constraint: the robot is always on a level surface, so
    // roll=0 and pitch=0. Yaw comes from gyro_z integration in the
    // local EKF plus GPS-COG absolute yaw in the global EKF.
    // Set orientation to identity with tight roll/pitch covariance and
    // loose yaw covariance so robot_localization constrains roll/pitch
    // to zero without fighting its own yaw estimate.
    msg.orientation.w = 1.0;
    msg.orientation_covariance[0] = 0.001;  // roll  variance (tight)
    msg.orientation_covariance[4] = 0.001;  // pitch variance (tight)
    msg.orientation_covariance[8] = 99.0;  // yaw   variance (don't constrain)

    // Accel covariance: use calibrated values if available, else defaults.
    if (imu_cal_ready_)
    {
      // Floor at 0.001 so covariance is never zero (EKF singularity).
      msg.linear_acceleration_covariance[0] = std::max(imu_cal_cov_ax_, 0.001);
      msg.linear_acceleration_covariance[4] = std::max(imu_cal_cov_ay_, 0.001);
    }
    else
    {
      msg.linear_acceleration_covariance[0] = 0.01;
      msg.linear_acceleration_covariance[4] = 0.01;
    }
    msg.linear_acceleration_covariance[8] = 0.01;  // Z — uncalibrated default

    // Gyro covariance: WT901 gyro_z is accurate to ~7% over-report on the
    // ground-truth 90° CCW rotation test (2026-04-19). The legacy "17%
    // under-report" claim predates the firmware scaling fixes
    // (WT901_G_FACTOR float-correct, RAD_PER_DEG rename). We still keep a
    // loose yaw floor because WT901 has ~0.01 rad/s bias drift that
    // couples into heading if trusted too tightly. Calibrated values for
    // roll/pitch rate are used directly (robot is planar, so those stay
    // near zero and the calibration sum is a clean noise estimate).
    if (imu_cal_ready_)
    {
      msg.angular_velocity_covariance[0] = std::max(imu_cal_cov_gx_, 0.001);
      msg.angular_velocity_covariance[4] = std::max(imu_cal_cov_gy_, 0.001);
      msg.angular_velocity_covariance[8] =
          std::max(imu_cal_cov_gz_, 0.01);  // keep high floor for yaw
    }
    else
    {
      msg.angular_velocity_covariance[0] = 0.1;  // roll rate
      msg.angular_velocity_covariance[4] = 0.1;  // pitch rate
      msg.angular_velocity_covariance[8] = 1.0;  // yaw rate — low confidence
    }

    pub_imu_->publish(msg);

    // Raw magnetometer on a diagnostic-only topic. Not fused anywhere —
    // the chip sits inside a metal chassis so uncalibrated heading is
    // ~229° off true north; this topic only lets an operator inspect the
    // live field vector to check the sensor is alive or measure local
    // distortion. µT on the wire → Tesla for sensor_msgs/MagneticField.
    if (pub_mag_raw_->get_subscription_count() > 0)
    {
      sensor_msgs::msg::MagneticField mag_msg;
      mag_msg.header.stamp = msg.header.stamp;
      mag_msg.header.frame_id = "imu_link";
      mag_msg.magnetic_field.x = pkt.mag_uT[0] * 1.0e-6;
      mag_msg.magnetic_field.y = pkt.mag_uT[1] * 1.0e-6;
      mag_msg.magnetic_field.z = pkt.mag_uT[2] * 1.0e-6;
      // Covariance unknown (chip distortion is large and unmeasured).
      // ROS convention: set the first element to -1.0 to signal "no data".
      mag_msg.magnetic_field_covariance[0] = -1.0;
      pub_mag_raw_->publish(mag_msg);
    }
  }

  void publish_dock_heading()
  {
    if (!is_charging_)
      return;

    // Publish dock heading as sensor_msgs/Imu on ~/dock_heading
    // (remapped to /gnss/heading in launch). dock_yaw_to_set_pose
    // consumes it and seeds both EKFs via their set_pose services.
    // The orientation quaternion is heading in ENU.
    // dock_yaw_ (= dock_pose_yaw) is ALREADY a map-frame ENU yaw: it is
    // written that way by the GUI set_docking_point service
    // (area_manager.cpp) and the undock GPS-trajectory calibration
    // (calibration_nodes.cpp), and consumed as ENU directly by
    // opennav_docking's home_dock pose. Use it verbatim — the old
    // `pi/2 - dock_yaw_` compass→ENU conversion was wrong (it double-
    // rotated this seed relative to every other consumer/writer).
    const double enu_yaw = dock_yaw_;

    // During the anchor window after a charging transition, publish with
    // σ=π so dock_yaw_to_set_pose accepts the first heading update as a
    // wide-σ seed no matter how far the filter's initial yaw is from the
    // dock.
    double yaw_cov = 0.01;  // steady-state: σ ≈ 0.1 rad (~6°)
    if (charging_anchor_active_)
    {
      const double elapsed = (now() - charging_anchor_start_).seconds();
      if (elapsed < kChargingAnchorWindowSec)
      {
        yaw_cov = M_PI * M_PI;  // σ = π: any innovation passes
      }
      else
      {
        charging_anchor_active_ = false;
        RCLCPP_INFO(get_logger(), "Dock heading anchor window closed; tightening σ to 0.1 rad.");
      }
    }

    auto msg = sensor_msgs::msg::Imu{};
    msg.header.stamp = now();
    msg.header.frame_id = "base_footprint";
    msg.orientation.z = std::sin(enu_yaw / 2.0);
    msg.orientation.w = std::cos(enu_yaw / 2.0);
    msg.orientation_covariance[0] = 0.01;  // roll
    msg.orientation_covariance[4] = 0.01;  // pitch
    msg.orientation_covariance[8] = yaw_cov;
    pub_dock_heading_->publish(msg);
  }

  // Panel button → mowing action (task #31). button_id 4 (START/PLAY) and 5
  // (HOME) call the SAME ~/high_level_control service the GUI uses — DRY,
  // and no new authority: this issues a high-level command the GUI can
  // already send at any time, it does not bypass any firmware safety check.
  // button_id 1/2/3 (S1/S2/LOCK) are UART panel buttons with no mapped
  // action yet (per user decision) — log-only, same as before.
  //
  // SAFETY: COMMAND_START drives the BT into MOWING, which enables the
  // blades — pressing the physical PLAY button now has the same physical
  // effect as pressing Start in the GUI. Firmware remains the sole blade-
  // safety authority (E-stop, lift, latch) regardless of which client asked
  // for AUTONOMOUS mode; this only adds a second caller of an already-
  // existing, already-GUI-reachable high-level command.
  void handle_ui_event(const uint8_t* data, std::size_t len)
  {
    if (len < sizeof(LlUiEvent))
    {
      RCLCPP_WARN(get_logger(), "UI event packet too short: %zu < %zu", len, sizeof(LlUiEvent));
      return;
    }

    LlUiEvent pkt{};
    std::memcpy(&pkt, data, sizeof(LlUiEvent));

    RCLCPP_INFO(get_logger(),
                "UI button event: button_id=%u duration=%u",
                pkt.button_id,
                pkt.press_duration);

    uint8_t command;
    switch (pkt.button_id)
    {
      case 4:  // START/PLAY
        command = mowgli_interfaces::srv::HighLevelControl::Request::COMMAND_START;
        break;
      case 5:  // HOME
        command = mowgli_interfaces::srv::HighLevelControl::Request::COMMAND_HOME;
        break;
      default:
        // S1/S2/LOCK (1/2/3) or an unrecognized id: log-only, no action.
        return;
    }

    // async_send_request must not block the packet-handler/serial thread —
    // the response is only used for logging, so a fire-and-forget callback
    // (mirrors mqtt_bridge_node's on_mqtt_command, the existing precedent
    // for a ROS2 client of this exact service) is enough.
    if (!client_high_level_control_->service_is_ready())
    {
      RCLCPP_WARN(get_logger(),
                  "handle_ui_event: high_level_control service not available; "
                  "panel button_id=%u command dropped.",
                  pkt.button_id);
      return;
    }

    auto request = std::make_shared<mowgli_interfaces::srv::HighLevelControl::Request>();
    request->command = command;
    client_high_level_control_->async_send_request(
        request,
        [this, button_id = pkt.button_id, command](
            rclcpp::Client<mowgli_interfaces::srv::HighLevelControl>::SharedFuture future)
        {
          const auto response = future.get();
          if (response->success)
          {
            RCLCPP_INFO(get_logger(),
                        "handle_ui_event: panel button_id=%u -> high_level_control command=%u "
                        "succeeded.",
                        button_id,
                        command);
          }
          else
          {
            RCLCPP_WARN(get_logger(),
                        "handle_ui_event: panel button_id=%u -> high_level_control command=%u "
                        "reported failure.",
                        button_id,
                        command);
          }
        });
  }

  // Decode the wire packet and delegate to odometry_publisher_, which owns
  // ~/wheel_odom + ~/wheel_ticks and all the tick-delta/aggregation logic
  // (16-bit wrap recovery, spike rejection, 50 ms aggregation, dock-charging
  // force-zero) — see odometry_publisher.hpp for the full rationale (task
  // #11 god-node breakup; verbatim port, this method only decodes now).
  void handle_odometry(const uint8_t* data, std::size_t len)
  {
    if (len < sizeof(LlOdometry))
    {
      RCLCPP_WARN(get_logger(), "Odometry packet too short: %zu < %zu", len, sizeof(LlOdometry));
      return;
    }

    LlOdometry pkt{};
    std::memcpy(&pkt, data, sizeof(LlOdometry));
    odometry_publisher_.handle_packet(pkt, ticks_per_meter_, wheel_track_, is_charging_);
  }

  // ---------------------------------------------------------------------------
  // Periodic transmit (Pi → STM32)
  // ---------------------------------------------------------------------------

  void send_heartbeat()
  {
    LlHeartbeat pkt{};
    pkt.type = PACKET_ID_LL_HEARTBEAT;
    pkt.emergency_requested = emergency_active_ ? 1u : 0u;
    pkt.emergency_release_requested = emergency_release_pending_ ? 1u : 0u;

    // Consume the one-shot release flag.
    emergency_release_pending_ = false;

    send_raw_packet(reinterpret_cast<const uint8_t*>(&pkt),
                    sizeof(LlHeartbeat) - sizeof(uint16_t));  // CRC appended by encode_packet.
  }

  void send_high_level_state()
  {
    LlHighLevelState pkt{};
    pkt.type = PACKET_ID_LL_HIGH_LEVEL_STATE;
    pkt.current_mode = current_mode_;
    pkt.gps_quality = gps_quality_;

    if (last_sent_mode_ != current_mode_ || last_sent_mode_state_name_ != current_mode_state_name_)
    {
      RCLCPP_INFO(get_logger(),
                  "hardware_bridge forwarding HL state to STM32: mode=%u (%s), state_name='%s', "
                  "gps_quality=%u",
                  current_mode_,
                  high_level_mode_name(current_mode_),
                  current_mode_state_name_.c_str(),
                  gps_quality_);
      last_sent_mode_ = current_mode_;
      last_sent_mode_state_name_ = current_mode_state_name_;
    }

    send_raw_packet(reinterpret_cast<const uint8_t*>(&pkt),
                    sizeof(LlHighLevelState) - sizeof(uint16_t));
  }

  void send_blade_command(uint8_t on, uint8_t dir)
  {
    LlCmdBlade pkt{};
    pkt.type = PACKET_ID_LL_CMD_BLADE;
    pkt.blade_on = on;
    pkt.blade_dir = dir;

    send_raw_packet(reinterpret_cast<const uint8_t*>(&pkt), sizeof(LlCmdBlade) - sizeof(uint16_t));
  }

  void send_reboot_command()
  {
    LlReboot pkt{};
    pkt.type = PACKET_ID_LL_REBOOT;
    pkt.magic = kLlRebootMagic;
    send_raw_packet(reinterpret_cast<const uint8_t*>(&pkt), sizeof(LlReboot) - sizeof(uint16_t));
  }

  // Push the drive-motor runtime tuning to the firmware. The board has no
  // config persistence, so the bridge is the source of truth and re-sends
  // these on every (re)connect (pid_resend_count_) and whenever a parameter
  // changes. The firmware validates/clamps every field on receipt.
  void send_drive_pid()
  {
    // Defensive: the set-parameters callback can fire during declare_parameters
    // (before open_serial_port() constructs serial_), and send_raw_packet would
    // dereference a null serial_. The current declaration order avoids it, but
    // guard so a future reorder / added wheel_pid_* param can't crash the node.
    if (!serial_)
    {
      return;
    }
    LlSetDrivePid pkt{};
    pkt.type = PACKET_ID_LL_SET_DRIVE_PID;
    pkt.ticks_per_meter = static_cast<float>(ticks_per_meter_);
    pkt.kp = static_cast<float>(wheel_pid_kp_);
    pkt.ki = static_cast<float>(wheel_pid_ki_);
    pkt.kd = static_cast<float>(wheel_pid_kd_);
    pkt.integral_limit = static_cast<float>(wheel_pid_integral_limit_);
    pkt.pwm_per_mps = static_cast<float>(wheel_pid_pwm_per_mps_);
    if (send_raw_packet(reinterpret_cast<const uint8_t*>(&pkt),
                        sizeof(LlSetDrivePid) - sizeof(uint16_t)))
    {
      RCLCPP_WARN_ONCE(
          get_logger(),
          "Drive runtime tuning now uses protocol v%u packet 0x%02X (27-byte payload with "
          "ticks_per_meter). Older STM32 firmware that only understands the legacy 0x53 packet "
          "will ignore live drive tuning until you flash the matching firmware.",
          static_cast<unsigned>(kMowgliProtocolVersion),
          static_cast<unsigned>(PACKET_ID_LL_SET_DRIVE_PID));
      RCLCPP_INFO(get_logger(),
                  "Sent drive params: ticks_per_meter=%.3f kp=%.3f ki=%.3f kd=%.3f "
                  "integral_limit=%.3f pwm_per_mps=%.3f",
                  ticks_per_meter_,
                  wheel_pid_kp_,
                  wheel_pid_ki_,
                  wheel_pid_kd_,
                  wheel_pid_integral_limit_,
                  wheel_pid_pwm_per_mps_);
    }
  }

  // Firmware gyro yaw-rate loop tuning (Option C, task #33/#34). Mirrors
  // send_drive_pid() exactly — separate packet (PACKET_ID_LL_SET_YAW_PID),
  // same "re-send on (re)connect via pid_resend_count_" burst (Firmware-2's
  // #33 report: firmware has no config persistence, so a lost packet must be
  // retried). encode_packet appends the CRC, so it is left zero-initialized
  // here, matching send_drive_pid()'s convention.
  void send_yaw_pid()
  {
    if (!serial_)
    {
      return;
    }
    LlSetYawPid pkt{};
    pkt.type = PACKET_ID_LL_SET_YAW_PID;
    pkt.yaw_kp = static_cast<float>(yaw_kp_);
    pkt.yaw_ki = static_cast<float>(yaw_ki_);
    pkt.trim_limit_mps = static_cast<float>(yaw_trim_limit_mps_);
    pkt.enabled = yaw_loop_enabled_ ? 1u : 0u;
    pkt.gyro_sign = static_cast<int8_t>(yaw_gyro_sign_);
    if (send_raw_packet(reinterpret_cast<const uint8_t*>(&pkt),
                        sizeof(LlSetYawPid) - sizeof(uint16_t)))
    {
      RCLCPP_INFO(get_logger(),
                  "Sent yaw-loop params: kp=%.3f ki=%.3f trim_limit_mps=%.3f enabled=%d "
                  "gyro_sign=%d",
                  yaw_kp_,
                  yaw_ki_,
                  yaw_trim_limit_mps_,
                  static_cast<int>(pkt.enabled),
                  static_cast<int>(pkt.gyro_sign));
    }
  }

  void on_reboot_board(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    if (!serial_ || !serial_->is_open())
    {
      res->success = false;
      res->message = "serial port not open";
      return;
    }
    RCLCPP_WARN(get_logger(), "reboot_board: sending NVIC_SystemReset request to STM32.");
    // Fire twice — a single packet lost to USB jitter shouldn't silently
    // no-op a deliberate recovery action.
    send_reboot_command();
    send_reboot_command();
    res->success = true;
    res->message = "reboot request sent; board will reset within ~1 s";
  }

  void on_set_firmware_debug(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                             std::shared_ptr<std_srvs::srv::SetBool::Response> res)
  {
    if (!serial_ || !serial_->is_open())
    {
      res->success = false;
      res->message = "serial port not open";
      return;
    }

    const bool previous_requested = firmware_debug_requested_;
    firmware_debug_requested_ = req->data;

    if (!send_config_request())
    {
      firmware_debug_requested_ = previous_requested;
      res->success = false;
      res->message = "failed to send firmware debug config request";
      return;
    }

    if (fw_handshake_done_)
    {
      config_control_resend_count_ = std::max(config_control_resend_count_, 2);
    }
    else
    {
      config_req_resend_count_ = std::max(config_req_resend_count_, 2);
    }

    if (req->data)
    {
      RCLCPP_INFO(get_logger(), "[FW_DIAG] Firmware debug enable requested.");
      log_fw_diag_snapshot();
    }
    else if (firmware_debug_enabled_ || previous_requested)
    {
      RCLCPP_INFO(get_logger(), "[FW_DIAG] Firmware debug disable requested.");
    }

    res->success = true;
    res->message =
        req->data ? "firmware debug enable request sent" : "firmware debug disable request sent";
  }

  void handle_blade_status(const uint8_t* data, std::size_t len)
  {
    if (len < sizeof(LlBladeStatus))
    {
      return;
    }

    LlBladeStatus pkt{};
    std::memcpy(&pkt, data, sizeof(LlBladeStatus));

    // Update the Status message fields with live blade data
    blade_active_ = pkt.is_active != 0u;
    blade_rpm_ = static_cast<float>(pkt.rpm);
    blade_temperature_ = pkt.temperature;
    blade_esc_current_ = static_cast<float>(pkt.power_watts);
  }

  // ---------------------------------------------------------------------------
  // Firmware version handshake (image <-> firmware compatibility)
  // ---------------------------------------------------------------------------

  void handle_config_rsp(const uint8_t* data, std::size_t len)
  {
    if (len < sizeof(LlConfigRsp))
    {
      return;
    }

    const bool had_handshake = fw_handshake_done_;
    const uint8_t previous_protocol = fw_protocol_version_;
    const std::string previous_version = fw_version_str_;
    const bool previous_fw_diag_enabled = firmware_debug_enabled_;

    LlConfigRsp pkt{};
    std::memcpy(&pkt, data, sizeof(LlConfigRsp));

    fw_protocol_version_ = pkt.protocol_version;
    firmware_debug_enabled_ = (pkt.active_flags & CONFIG_FLAG_FIRMWARE_DEBUG) != 0u;
    firmware_debug_requested_ = firmware_debug_enabled_;
    config_control_resend_count_ = 0;
    fw_version_major_ = pkt.fw_version_major;
    fw_version_minor_ = pkt.fw_version_minor;
    fw_version_patch_ = pkt.fw_version_patch;
    fw_handshake_done_ = true;
    // The wire-protocol version is the compatibility key: an image built for
    // protocol vN can only correctly parse/command firmware of the same vN.
    fw_compatible_ = (fw_protocol_version_ == kMowgliProtocolVersion);

    char ver[16];
    snprintf(ver,
             sizeof(ver),
             "%u.%u.%u",
             static_cast<unsigned>(fw_version_major_),
             static_cast<unsigned>(fw_version_minor_),
             static_cast<unsigned>(fw_version_patch_));
    fw_version_str_ = ver;

    const bool version_changed = !had_handshake || previous_protocol != fw_protocol_version_ ||
                                 previous_version != fw_version_str_;
    if (fw_compatible_ && version_changed)
    {
      RCLCPP_INFO(get_logger(),
                  "Firmware handshake OK: firmware v%s, protocol v%u (image expects v%u).",
                  fw_version_str_.c_str(),
                  static_cast<unsigned>(fw_protocol_version_),
                  static_cast<unsigned>(kMowgliProtocolVersion));
    }
    else if (!fw_compatible_ && version_changed)
    {
      RCLCPP_ERROR(get_logger(),
                   "INCOMPATIBLE FIRMWARE: firmware v%s speaks protocol v%u but this image "
                   "expects protocol v%u. Reflash the STM32 firmware (see docs/FIRST_BOOT.md). "
                   "Mowing is blocked until the versions match.",
                   fw_version_str_.c_str(),
                   static_cast<unsigned>(fw_protocol_version_),
                   static_cast<unsigned>(kMowgliProtocolVersion));
    }

    if (firmware_debug_enabled_)
    {
      if (!previous_fw_diag_enabled)
      {
        RCLCPP_INFO(get_logger(), "[FW_DIAG] Firmware debug enabled.");
      }
      if (!had_handshake || !previous_fw_diag_enabled || version_changed)
      {
        log_fw_diag_snapshot();
      }
    }
    else if (previous_fw_diag_enabled)
    {
      RCLCPP_INFO(get_logger(), "[FW_DIAG] Firmware debug disabled.");
    }
  }

  // Reset the handshake state on (re)connect so a reflashed board is re-checked.
  void rearm_firmware_handshake()
  {
    fw_handshake_done_ = false;
    fw_compatible_ = false;
    fw_protocol_version_ = 0u;
    fw_version_str_.clear();
    config_req_resend_count_ = 5;
    config_control_resend_count_ = 0;
    fw_handshake_start_ = now();
  }

  // Sends PACKET_ID_LL_HIGH_LEVEL_CONFIG_REQ on the first few heartbeats after a
  // (re)connect; once the resends are exhausted with no reply, the firmware is
  // too old to implement the handshake → flag incompatible (reflash needed).
  void service_firmware_handshake()
  {
    if (fw_handshake_done_ || !serial_ || !serial_->is_open())
    {
      return;
    }
    if (config_req_resend_count_ > 0)
    {
      send_config_request();
      --config_req_resend_count_;
      return;
    }
    // No CONFIG_RSP after the request budget + a grace window: firmware predates
    // the handshake (or is otherwise unresponsive) → incompatible.
    if ((now() - fw_handshake_start_).seconds() > fw_handshake_timeout_s_)
    {
      fw_handshake_done_ = true;
      fw_compatible_ = false;
      fw_protocol_version_ = 0u;
      fw_version_str_ = "unknown";
      RCLCPP_ERROR(get_logger(),
                   "INCOMPATIBLE FIRMWARE: the STM32 did not answer the version handshake "
                   "(no CONFIG_RSP in %.0f s). This firmware predates the version protocol — "
                   "reflash it (see docs/FIRST_BOOT.md). Mowing is blocked until then.",
                   fw_handshake_timeout_s_);
    }
  }

  bool send_config_request()
  {
    if (!serial_)
    {
      return false;
    }
    LlConfigReq pkt{};
    pkt.type = PACKET_ID_LL_HIGH_LEVEL_CONFIG_REQ;
    pkt.flags = firmware_debug_requested_ ? CONFIG_FLAG_FIRMWARE_DEBUG : 0u;
    return send_raw_packet(reinterpret_cast<const uint8_t*>(&pkt),
                           sizeof(LlConfigReq) - sizeof(uint16_t));
  }

  void clear_firmware_debug_state_for_reconnect()
  {
    const bool was_requested = firmware_debug_requested_;
    const bool was_enabled = firmware_debug_enabled_;
    if (was_requested || was_enabled)
    {
      RCLCPP_INFO(get_logger(), "[FW_DIAG] Firmware debug reset to OFF after serial reconnect.");
    }
    firmware_debug_requested_ = false;
    firmware_debug_enabled_ = false;
  }

  void log_fw_diag_snapshot()
  {
    if (!(firmware_debug_requested_ || firmware_debug_enabled_))
    {
      return;
    }

    if (!fw_version_str_.empty())
    {
      RCLCPP_INFO(get_logger(),
                  "[FW_DIAG] Firmware handshake OK: firmware v%s, protocol v%u.",
                  fw_version_str_.c_str(),
                  static_cast<unsigned>(fw_protocol_version_));
    }

    if (reset_cause_seen_)
    {
      if (last_reset_cause_ == RESET_CAUSE_WWDG && last_reset_has_watchdog_stage_)
      {
        RCLCPP_INFO(get_logger(),
                    "[FW_DIAG] STM32 boot reset cause: %s — %s (last stage before reset: %s)",
                    last_reset_cause_name_.c_str(),
                    reset_cause_description(last_reset_cause_),
                    last_reset_stage_name_.c_str());
      }
      else
      {
        RCLCPP_INFO(get_logger(),
                    "[FW_DIAG] STM32 boot reset cause: %s — %s",
                    last_reset_cause_name_.c_str(),
                    reset_cause_description(last_reset_cause_));
      }
    }
  }

  // ---------------------------------------------------------------------------
  // cmd_vel subscriber
  // ---------------------------------------------------------------------------

  void on_cmd_vel(geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
  {
    double vx = msg->twist.linear.x;
    double wz = msg->twist.angular.z;

    // The firmware ignores cmd_vel when mode is IDLE.  When velocity commands
    // arrive before the BT publishes any high-level state, ensure the firmware
    // is not left in NULL/transition mode.
    //
    // 2026-07-17 (task #18) — KNOWN, BOUNDED AUTHORITY LEAK, documented rather
    // than fully closed: this ~/cmd_vel is twist_mux's MERGED output (5
    // sources: navigation prio 10, docking 15, teleop 20, tuning 30,
    // emergency 100 — see twist_mux.yaml). hardware_bridge has no
    // source-attribution on the merged topic today, so ANY non-zero traffic
    // from ANY lane — including a teleop joystick nudge or a drive-PID tuning
    // command sent before the BT has booted — trips this fallback and forces
    // HL_MODE_AUTONOMOUS, not just genuine nav-lane traffic. Narrowing this to
    // "navigation lane only" needs either a new subscription to twist_mux's
    // active-source diagnostics, or an explicit BT-boot handshake (BT
    // publishes an initial state before any node is allowed to command
    // velocity) — both are bigger changes than this task's scope and the mode
    // semantics deserve a Firmware-side conversation first: HL_MODE_AUTONOMOUS
    // is the mission-in-progress label, and reusing it for "some non-BT
    // process is bootstrapping velocity" conflates the two on the firmware
    // side (dock-auto-reset, telemetry displays, etc. all read current_mode_
    // as ground truth for "is the robot autonomously mowing"). Coordinate
    // with Firmware before changing what mode this fallback targets.
    //
    // What IS closed here (needs no firmware coordination): the fallback can
    // no longer fire while the firmware has an ACTUAL latch asserted
    // (fw_latched_emergency_, computed from pkt.emergency_bitmask in the
    // telemetry poll — the firmware-reported state, not just the ROS2-side
    // ~/emergency_stop service flag, so a physical STOP-button press that
    // never went through that service still blocks this). Without this guard,
    // stray velocity traffic on any mux lane while e-stopped would silently
    // promote the firmware to AUTONOMOUS — exactly the kind of side effect
    // Invariant 9 (firmware is sole safety authority) means to prevent from
    // ever being masked by ROS2-side state.
    if (current_mode_ == HL_MODE_NULL && !fw_latched_emergency_ && (vx != 0.0 || wz != 0.0))
    {
      current_mode_ = HL_MODE_AUTONOMOUS;
      current_mode_state_name_ = "AUTONOMOUS_CMD_VEL_FALLBACK";
      RCLCPP_WARN(get_logger(),
                  "Received non-zero cmd_vel before BT high-level state propagation; "
                  "forcing STM32 mode to %u (%s).",
                  current_mode_,
                  high_level_mode_name(current_mode_));
      send_high_level_state();
    }

    // Sub-deadband forward-velocity guard.
    //
    // A previous closed-loop deadband compensator BOOSTED sub-deadband
    // commands to ~0.85 rad/s (or 0.13 m/s) to break through the
    // firmware's PWM static friction. The boost produced motor pulses
    // strong enough to register on the gyro but too short to generate
    // measurable wheel encoder ticks. The wheel between-factor in
    // fusion_graph then saw "stationary" while the IMU integrated real
    // rotation, and the wheel/IMU disagreement piled up as
    // stationary_hand_push spikes and σ_yaw drift during PRE_ROTATE
    // and headland pivots.
    //
    // Policy: clamp any |vx| below min_linear_vel_ to zero. This was a
    // hardcoded 0.15 m/s written for the old hand-rolled wheel-PI, whose
    // PWM static friction couldn't move the chassis below ~0.15 — so
    // commanding a sub-deadband forward velocity only produced motor buzz
    // and a wheel/IMU mismatch (the boost approach pulsed the motors enough
    // for the gyro to see rotation but too little for encoder ticks).
    // Now the vendored PX4 PID firmware tracks slow setpoints, so the
    // threshold is a runtime PARAM defaulting to 0.05: MPPI's regulated
    // slow-creep (frequently < 0.15 near goals / on alignment) reaches the
    // wheels and drives smoothly instead of a 0↔0.15 stop-go. Set
    // min_linear_vel:=0.0 to disable the guard entirely, or raise it back
    // toward 0.15 if a given chassis still can't execute slow forward.
    //
    // wz handling — Option C (task #34): the closed-loop yaw-rate shaping
    // that used to live here (Option B, task #24 — a host-side PI closing
    // on the gyro to absorb the firmware's nonlinear PWM→rotation response)
    // has moved INTO FIRMWARE (task #33), which now runs the same closed
    // loop without the ~50-90 ms USB round-trip latency that limited the
    // host-side gains. wz is sent straight through, unshaped; see the
    // firmware's own yaw_kp/yaw_ki (tuned via SET_DRIVE_PID, send_drive_pid())
    // for the loop that used to be angular_rate_controller.hpp here.
    //
    // The sub-deadband |vx| → 0 guard is unchanged (linear has no clean
    // host-side rate feedback — encoders slip; leave it to Nav2's loops).
    constexpr double kMinCmdToConsider = 1.0e-3;  // ignore floating-point dust
    if (std::abs(vx) > kMinCmdToConsider && std::abs(vx) < min_linear_vel_)
    {
      vx = 0.0;
    }

    LlCmdVel pkt{};
    pkt.type = PACKET_ID_LL_CMD_VEL;
    pkt.linear_x = static_cast<float>(vx);
    pkt.angular_z = static_cast<float>(wz);

    send_raw_packet(reinterpret_cast<const uint8_t*>(&pkt), sizeof(LlCmdVel) - sizeof(uint16_t));
  }

  // ---------------------------------------------------------------------------
  // Service handlers
  // ---------------------------------------------------------------------------

  void on_mower_control(const std::shared_ptr<mowgli_interfaces::srv::MowerControl::Request> req,
                        std::shared_ptr<mowgli_interfaces::srv::MowerControl::Response> res)
  {
    mow_enabled_ = (req->mow_enabled != 0u);

    RCLCPP_INFO(get_logger(),
                "MowerControl: mow_enabled=%s mow_direction=%u",
                mow_enabled_ ? "true" : "false",
                req->mow_direction);

    // Send blade command to STM32
    send_blade_command(mow_enabled_ ? 1u : 0u, req->mow_direction);

    res->success = true;
  }

  void on_emergency_stop(const std::shared_ptr<mowgli_interfaces::srv::EmergencyStop::Request> req,
                         std::shared_ptr<mowgli_interfaces::srv::EmergencyStop::Response> res)
  {
    if (req->emergency != 0u)
    {
      RCLCPP_WARN(get_logger(), "Emergency stop requested via service.");
      emergency_active_ = true;
    }
    else
    {
      RCLCPP_INFO(get_logger(), "Emergency release requested via service.");
      emergency_active_ = false;
      emergency_release_pending_ = true;
    }

    send_heartbeat();

    res->success = true;
  }

  // ---------------------------------------------------------------------------
  // Members: ROS2 interfaces
  // ---------------------------------------------------------------------------

  rclcpp::Publisher<mowgli_interfaces::msg::Status>::SharedPtr pub_status_;
  rclcpp::Publisher<mowgli_interfaces::msg::Emergency>::SharedPtr pub_emergency_;
  rclcpp::Publisher<mowgli_interfaces::msg::Power>::SharedPtr pub_power_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr pub_mag_raw_;
  // Owns ~/wheel_odom + ~/wheel_ticks and all wheel-tick decode/aggregation
  // state (see odometry_publisher.hpp — extracted from the former inline
  // handle_odometry() as part of the god-node breakup, task #11).
  // Constructed in the init-list above (odometry_publisher_(*this)).
  OdometryPublisher odometry_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr pub_battery_state_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_dock_heading_;

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_cmd_vel_;
  rclcpp::Subscription<mowgli_interfaces::msg::GnssStatus>::SharedPtr sub_gnss_status_;
  rclcpp::Subscription<mowgli_interfaces::msg::HighLevelStatus>::SharedPtr sub_hl_status_;

  rclcpp::Service<mowgli_interfaces::srv::MowerControl>::SharedPtr srv_mower_control_;
  rclcpp::Service<mowgli_interfaces::srv::EmergencyStop>::SharedPtr srv_emergency_stop_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_reboot_board_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_set_firmware_debug_;

  // Client (not server, unlike the srv_* members above): calls
  // behavior_tree_node's high_level_control service on panel button presses.
  // See create_service_clients() and handle_ui_event() (task #31).
  rclcpp::Client<mowgli_interfaces::srv::HighLevelControl>::SharedPtr client_high_level_control_;

  rclcpp::TimerBase::SharedPtr timer_read_;
  rclcpp::TimerBase::SharedPtr timer_heartbeat_;
  rclcpp::TimerBase::SharedPtr timer_high_level_;
  rclcpp::TimerBase::SharedPtr timer_dock_heading_;

  // ---------------------------------------------------------------------------
  // Members: serial and protocol
  // ---------------------------------------------------------------------------

  std::string serial_port_path_;
  int baud_rate_{115200};
  double heartbeat_rate_{4.0};
  double publish_rate_{100.0};
  // Serial-link RX watchdog (auto-reconnect on flash / board reboot / unplug).
  double serial_rx_timeout_s_{2.0};
  rclcpp::Time last_serial_rx_time_{0, 0, RCL_ROS_TIME};
  double high_level_rate_{2.0};

  std::unique_ptr<SerialPort> serial_;
  PacketHandler packet_handler_;

  // ---------------------------------------------------------------------------
  // Members: stateful state communicated to the STM32
  // ---------------------------------------------------------------------------

  bool emergency_active_{false};
  bool emergency_release_pending_{false};
  int startup_release_count_{5};  // Send release for first 5 heartbeats

  // Firmware-REPORTED latch state (mirrors msg.latched_emergency, computed
  // from pkt.emergency_bitmask in the telemetry poll — see the ~/emergency
  // publish block). Unlike emergency_active_ above (which only reflects a
  // ROS2-side ~/emergency_stop service request), this reflects what the
  // firmware itself is telling us, including a physical STOP-button press
  // that never went through the ROS2 service. Gates the cmd_vel→AUTONOMOUS
  // mode-inference fallback in on_cmd_vel (task #18): stray velocity traffic
  // must never auto-promote HL_MODE while the firmware has a latch asserted.
  bool fw_latched_emergency_{false};

  // Lift recovery mode: blade off on lift, no emergency, auto-resume
  bool lift_recovery_mode_{false};
  double lift_blade_resume_delay_sec_{1.0};
  bool lift_detected_{false};
  rclcpp::Time lift_start_time_;
  bool blade_was_enabled_before_lift_{false};
  rclcpp::Time lift_cleared_time_;
  bool waiting_blade_resume_{false};
  double dock_x_{0.0};
  double dock_y_{0.0};
  double dock_yaw_{0.0};
  double wheel_track_{0.325};
  double ticks_per_meter_{300.0};
  // Drive-motor wheel-velocity PID gains + feedforward, pushed to the STM32
  // (PACKET_ID_LL_SET_DRIVE_PID). Defaults mirror the firmware compile-time
  // fallback. The board has no persistence, so the bridge re-sends on every
  // (re)connect; pid_resend_count_ > 0 makes send_drive_pid() fire on the next
  // N heartbeat ticks (seeded so the first packet survives USB re-enumeration /
  // firmware boot even if one is dropped).
  double wheel_pid_kp_{30.0};
  double wheel_pid_ki_{5000.0};
  double wheel_pid_kd_{0.0};
  double wheel_pid_integral_limit_{100.0};
  double wheel_pid_pwm_per_mps_{300.0};
  int pid_resend_count_{5};

  // Firmware gyro yaw-rate loop (Option C, task #33/#34). Defaults are the
  // firmware's own power-on fallback. Shares pid_resend_count_ above with
  // the wheel PID for the reconnect resend burst — see send_yaw_pid().
  double yaw_kp_{0.30};
  double yaw_ki_{0.40};
  double yaw_trim_limit_mps_{0.15};
  bool yaw_loop_enabled_{true};
  int yaw_gyro_sign_{1};

  // Firmware version handshake state (image <-> firmware compatibility). The
  // bridge requests the firmware's protocol/semantic version on (re)connect and
  // compares the wire-protocol version against kMowgliProtocolVersion. Until a
  // definitive answer, fw_compatible_ is false so PreFlightCheck blocks mowing.
  bool fw_handshake_done_{false};
  bool fw_compatible_{false};
  uint8_t fw_protocol_version_{0u};
  uint8_t fw_version_major_{0u};
  uint8_t fw_version_minor_{0u};
  uint8_t fw_version_patch_{0u};
  std::string fw_version_str_{};
  int config_req_resend_count_{5};
  int config_control_resend_count_{0};
  rclcpp::Time fw_handshake_start_{0, 0, RCL_ROS_TIME};
  // Grace window after the request budget before declaring an unanswered
  // handshake incompatible (firmware too old to reply).
  double fw_handshake_timeout_s_{5.0};
  bool firmware_debug_requested_{false};
  bool firmware_debug_enabled_{false};

  // Host-side sub-deadband forward-velocity clamp (on_cmd_vel): any |vx| below
  // this is zeroed before reaching the firmware. Lowered from the legacy 0.15
  // (hand-rolled wheel-PI breakaway) to 0.05 now the vendored PX4 PID can track
  // slow setpoints — lets MPPI's regulated slow-creep actually drive instead of
  // a 0↔0.15 stop-go. Runtime-tunable (add_on_set_parameters_callback) for live
  // field iteration without a rebuild.
  double min_linear_vel_{0.05};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr min_lin_vel_cb_handle_;
  bool mow_enabled_{false};
  bool is_charging_{false};
  uint8_t current_mode_{0};
  std::string current_mode_state_name_{"UNSET"};
  uint8_t last_sent_mode_{255};
  std::string last_sent_mode_state_name_{"UNSET"};
  uint8_t gps_quality_{0};

  // Dock heading anchor: on is_charging false→true transition, publish
  // dock_heading with wide σ=π for a short window so dock_yaw_to_set_pose
  // has time to grab a sample before it narrows to the steady-state σ.
  rclcpp::Time charging_anchor_start_;
  bool charging_anchor_active_{false};
  static constexpr double kChargingAnchorWindowSec = 5.0;

  // Blade motor state (updated from LlBladeStatus packets)
  bool blade_active_{false};
  float blade_rpm_{0.0f};
  float blade_temperature_{0.0f};
  float blade_esc_current_{0.0f};
  uint8_t last_reset_cause_{RESET_CAUSE_UNKNOWN};
  std::string last_reset_cause_name_{"UNKNOWN"};
  bool reset_cause_seen_{false};
  uint8_t last_reset_stage_before_reset_{WATCHDOG_STAGE_NONE};
  std::string last_reset_stage_name_{"NONE"};
  bool last_reset_has_watchdog_stage_{false};
  bool reset_stage_seen_{false};
  bool reset_cause_log_pending_{true};

  // Odometry state (wheel-tick decode/aggregation, wheels_stationary_) now
  // lives in odometry_publisher_ (see the ROS2-interfaces member section
  // above) — task #11 god-node breakup.

  // IMU clock fitter (50 Hz). The odometry clock fitter now lives inside
  // odometry_publisher_ — both still run independently because the firmware
  // emits IMU and odometry on separate cadences and a stall on one channel
  // shouldn't perturb the other's stamp smoothing.
  HostFirmwareClockFit imu_clock_fit_;

  // IMU calibration state (computed while docked and idle, OR when stationary
  // off-dock via auto-cal, OR loaded from the persisted file at boot)
  int imu_cal_samples_{200};
  std::string imu_cal_persist_path_{"/ros2_ws/maps/imu_calibration.txt"};
  double imu_cal_auto_rest_sec_{15.0};
  double imu_cal_periodic_recal_sec_{60.0};  // 0 disables; default 60 s (was 600 — see ctor)
  rclcpp::Time imu_cal_at_rest_since_{};  // default-constructed (nanoseconds=0) = "not at rest yet"
  rclcpp::Time imu_cal_last_completed_{};  // when the last successful cal finished
  bool imu_cal_loaded_from_file_{false};
  bool imu_cal_collecting_{false};
  bool imu_cal_ready_{false};
  int imu_cal_count_{0};
  double imu_cal_sum_ax_{0.0}, imu_cal_sum_ay_{0.0}, imu_cal_sum_az_{0.0};
  double imu_cal_sum_gx_{0.0}, imu_cal_sum_gy_{0.0}, imu_cal_sum_gz_{0.0};
  std::vector<double> imu_cal_samples_ax_, imu_cal_samples_ay_;
  std::vector<double> imu_cal_samples_gx_, imu_cal_samples_gy_, imu_cal_samples_gz_;
  double imu_cal_offset_ax_{0.0}, imu_cal_offset_ay_{0.0};
  double imu_cal_offset_gx_{0.0}, imu_cal_offset_gy_{0.0}, imu_cal_offset_gz_{0.0};
  double imu_cal_cov_ax_{0.01}, imu_cal_cov_ay_{0.01};
  double imu_cal_cov_gx_{0.1}, imu_cal_cov_gy_{0.1}, imu_cal_cov_gz_{0.1};

  bool dock_pose_written_{false};
};

}  // namespace mowgli_hardware

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mowgli_hardware::HardwareBridgeNode>());
  rclcpp::shutdown();
  return 0;
}

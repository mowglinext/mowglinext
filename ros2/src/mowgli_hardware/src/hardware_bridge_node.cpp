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
#include "mowgli_hardware/ll_datatypes.hpp"
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
#include "mowgli_interfaces/msg/emergency.hpp"
#include "mowgli_interfaces/msg/high_level_status.hpp"
#include "mowgli_interfaces/msg/power.hpp"
#include "mowgli_interfaces/msg/status.hpp"
#include "mowgli_interfaces/srv/emergency_stop.hpp"
#include "mowgli_interfaces/srv/mower_control.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace mowgli_hardware
{

using namespace std::chrono_literals;

class HardwareBridgeNode : public rclcpp::Node
{
public:
  explicit HardwareBridgeNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : Node("hardware_bridge", options)
  {
    declare_parameters();
    create_publishers();
    create_subscribers();
    create_services();
    open_serial_port();
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
    high_level_rate_ = declare_parameter<double>("high_level_rate", 2.0);
    dock_x_ = declare_parameter<double>("dock_pose_x", 0.0);
    dock_y_ = declare_parameter<double>("dock_pose_y", 0.0);
    dock_yaw_ = declare_parameter<double>("dock_pose_yaw", 0.0);

    // Wheel kinematics — single source of truth lives in mowgli_robot.yaml.
    // Previously hardcoded as kWheelBase=0.325 / kTicksPerMeter=300.0; that
    // duplicated the URDF args and the firmware TICKS_PER_M, so any
    // re-calibration touched three places. wheel_track is the centre-to-
    // centre drive-wheel distance; ticks_per_meter is what the STM32
    // firmware advertises in board.h and uses when reporting cumulative
    // tick deltas in the odom packet (so the conversion m = ticks /
    // ticks_per_meter matches the firmware-side scaling).
    wheel_track_ = declare_parameter<double>("wheel_track", 0.325);
    ticks_per_meter_ = declare_parameter<double>("ticks_per_meter", 300.0);

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
    imu_cal_periodic_recal_sec_ = declare_parameter<double>("imu_cal_periodic_recal_sec", 600.0);

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
    pub_wheel_odom_ = create_publisher<nav_msgs::msg::Odometry>("~/wheel_odom", rclcpp::QoS(10));
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

    // Mirror the behavior tree's high-level state to the firmware so it
    // knows when to accept cmd_vel (mode != IDLE).
    sub_hl_status_ = create_subscription<mowgli_interfaces::msg::HighLevelStatus>(
        "/behavior_tree_node/high_level_status",
        rclcpp::QoS(10),
        [this](mowgli_interfaces::msg::HighLevelStatus::ConstSharedPtr msg)
        {
          current_mode_ = msg->state;
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
  }

  void open_serial_port()
  {
    serial_ = std::make_unique<SerialPort>(serial_port_path_, baud_rate_);

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
    timer_heartbeat_ = create_wall_timer(hb_period_ms,
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

  void read_serial_tick()
  {
    // If the port was never opened or was closed due to an error, attempt to
    // (re)open it.
    if (!serial_->is_open())
    {
      if (!serial_->open())
      {
        return;  // Still not open; will retry next tick.
      }
      RCLCPP_INFO(get_logger(), "Serial port re-opened successfully.");
    }

    constexpr std::size_t kReadBufSize = 512u;
    uint8_t buf[kReadBufSize];

    // Drain all available bytes in one tick.
    while (true)
    {
      const ssize_t n = serial_->read(buf, kReadBufSize);
      if (n <= 0)
      {
        break;
      }
      packet_handler_.feed(buf, static_cast<std::size_t>(n));
    }
  }

  bool send_raw_packet(const uint8_t* data, std::size_t len)
  {
    if (!serial_->is_open())
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Cannot send: serial port not open.");
      return false;
    }

    const std::vector<uint8_t> frame = packet_handler_.encode_packet(data, len);
    const ssize_t written = serial_->write(frame.data(), frame.size());

    if (written < 0 || static_cast<std::size_t>(written) != frame.size())
    {
      RCLCPP_WARN(get_logger(), "Short write or error sending packet.");
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
      default:
        RCLCPP_DEBUG(get_logger(), "Unhandled packet type 0x%02X (len=%zu)", data[0], len);
        break;
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
      if (is_charging_ && imu_cal_ready_ && !imu_cal_collecting_ && wheels_stationary_ &&
          imu_cal_periodic_recal_sec_ > 0.0 && imu_cal_last_completed_.nanoseconds() > 0)
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
      // Blade motor fields from live telemetry
      msg.mow_enabled = mow_enabled_;
      msg.esc_power = mow_enabled_ || blade_active_;
      msg.mower_esc_status = blade_active_ ? 1u : 0u;
      msg.mower_motor_rpm = blade_rpm_;
      msg.mower_motor_temperature = blade_temperature_;
      msg.mower_esc_current = blade_esc_current_;
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
    msg.header.stamp = now();
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
      if (wheels_stationary_)
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
    if (imu_cal_collecting_)
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
    // gives ~229° error vs real heading. dock_pose_yaw is set from config
    // (user measures with phone compass).

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
    // dock_yaw_ is compass heading; convert to ENU: yaw_enu = pi/2 - compass
    const double enu_yaw = M_PI / 2.0 - dock_yaw_;

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
  }

  void handle_odometry(const uint8_t* data, std::size_t len)
  {
    if (len < sizeof(LlOdometry))
    {
      RCLCPP_WARN(get_logger(), "Odometry packet too short: %zu < %zu", len, sizeof(LlOdometry));
      return;
    }

    LlOdometry pkt{};
    std::memcpy(&pkt, data, sizeof(LlOdometry));

    // Signed tick deltas since last firmware packet (polarity = direction).
    int32_t d_left = pkt.left_ticks - prev_left_ticks_;
    int32_t d_right = pkt.right_ticks - prev_right_ticks_;
    prev_left_ticks_ = pkt.left_ticks;
    prev_right_ticks_ = pkt.right_ticks;

    if (!odom_initialized_)
    {
      odom_initialized_ = true;
      return;
    }

    // 16-bit unsigned-counter wraparound recovery. Firmware packets carry
    // int32_t left_ticks / right_ticks but the underlying motor-controller
    // encoder counter is 16-bit and wraps 0xFFFF↔0x0000. After a wrap a
    // raw subtraction produces a delta of ±65535 (with small ±N noise from
    // the actual motion that occurred during the wrap), which is the
    // signature we observe ("dL=-65528", "dR=65535" etc.). Unwrap any
    // delta whose magnitude is closer to 65536 than to 0 by adding/
    // subtracting 65536 — this recovers the true small physical delta
    // instead of dropping the packet and losing position information.
    auto unwrap_16bit = [](int32_t d)
    {
      if (d > 32768)
        return d - 65536;
      if (d < -32768)
        return d + 65536;
      return d;
    };
    d_left = unwrap_16bit(d_left);
    d_right = unwrap_16bit(d_right);

    // Sanity-clamp residual implausible deltas. After wrap-recovery the
    // remaining oversize deltas are firmware glitches (e.g. motor-controller
    // encoder reset on direction change not in lockstep with the firmware's
    // own prev tracking). Drop those — at 21 ms packet period and a hard
    // 2 m/s upper bound the physical max is ~13 ticks; 100 leaves margin.
    constexpr int32_t kTickSpikeLimit = 100;
    if (std::abs(d_left) > kTickSpikeLimit || std::abs(d_right) > kTickSpikeLimit)
    {
      RCLCPP_WARN_THROTTLE(get_logger(),
                           *get_clock(),
                           2000,
                           "Dropping residual wheel tick spike: dL=%d dR=%d (limit=%d).",
                           d_left,
                           d_right,
                           kTickSpikeLimit);
      d_left = 0;
      d_right = 0;
    }

    // ----- Aggregate firmware packets into ~10 Hz wheel_odom publishes -----
    // Firmware packets arrive at ~47 Hz (every ~21 ms). At slow speeds this
    // gives only 0-3 ticks per window, so single-tick encoder noise (1 tick
    // = ~167 mm/s over 21 ms!) gets amplified into phantom velocity spikes
    // that robot_localization trusts thanks to the tight wheel covariance. Sum 5
    // packets (~100 ms, ~15 ticks at 0.5 m/s) so the velocity denominator
    // grows and single-tick noise collapses to ~7 % relative error.
    odom_acc_delta_left_ += d_left;
    odom_acc_delta_right_ += d_right;
    odom_acc_dt_ms_ += pkt.dt_millis;

    // 50 ms aggregation → ~20 Hz /wheel_odom. Tested: 33 ms (30 Hz)
    // saturated the EKF on this ARM CPU and produced "Failed to meet
    // update rate" errors on every cycle. 50 ms is twice the GPS rate
    // and twice the controller rate — sufficient for closed-loop
    // velocity control without choking the filter.
    static constexpr uint32_t kAggregateMs = 50;
    if (odom_acc_dt_ms_ < kAggregateMs)
    {
      return;
    }

    const auto stamp = now();
    const int32_t acc_d_left = odom_acc_delta_left_;
    const int32_t acc_d_right = odom_acc_delta_right_;
    const uint32_t acc_dt_ms = odom_acc_dt_ms_;
    odom_acc_delta_left_ = 0;
    odom_acc_delta_right_ = 0;
    odom_acc_dt_ms_ = 0;

    wheels_stationary_ = (acc_d_left == 0 && acc_d_right == 0);

    // Debug: log the aggregated window periodically.
    static int odom_debug_count = 0;
    if (++odom_debug_count % 10 == 0)
    {
      RCLCPP_INFO(get_logger(),
                  "Odom: acc_dL=%d acc_dR=%d acc_dt=%u ms  (cum L=%d R=%d)",
                  acc_d_left,
                  acc_d_right,
                  acc_dt_ms,
                  pkt.left_ticks,
                  pkt.right_ticks);
    }

    const double dt_sec = static_cast<double>(acc_dt_ms) / 1000.0;
    const double d_left_m = static_cast<double>(acc_d_left) / ticks_per_meter_;
    const double d_right_m = static_cast<double>(acc_d_right) / ticks_per_meter_;
    double vx = (d_left_m + d_right_m) * 0.5 / dt_sec;
    double vyaw = (d_right_m - d_left_m) / wheel_track_ / dt_sec;

    auto msg = nav_msgs::msg::Odometry{};
    msg.header.stamp = stamp;
    msg.header.frame_id = "odom";
    msg.child_frame_id = "base_link";

    // Force zero whenever the dock contacts are live. Charging current
    // proves the robot is mechanically anchored to the dock — the motors
    // cannot have moved it regardless of BT state. Previous narrower
    // condition (charging AND mode ∈ {NULL, IDLE}) missed the transient
    // RETURNING_HOME / end-of-mission states, during which the BT is
    // still mode=AUTONOMOUS(2) but the robot has already re-docked and
    // is physically stationary. Without the zero constraint, gyro_z
    // bias (~0.01 rad/s on the WT901) integrates into fusion yaw at
    // ~30°/min, which then corrupts the fused heading estimate and
    // manifests as a slowly-rotating robot icon while on the dock.
    //
    // Edge case: the charger bit can briefly stay high during a BackUp
    // undock before the contacts separate. That moment is handled by
    // the UndockRobot action, not by the wheel-odom path, so zeroing
    // for those ~100 ms is harmless.
    const bool force_zero = is_charging_;
    if (force_zero)
    {
      vx = 0.0;
      vyaw = 0.0;
    }

    msg.twist.twist.linear.x = vx;
    msg.twist.twist.angular.z = vyaw;

    // Covariance: force_zero → very tight (we're certain we're not moving).
    // Otherwise: linear vel σ = 0.1 m/s, yaw rate σ = 0.03 rad/s (tight
    // wheel trust — calibrated drivetrain, dominated by grass slip ~3%).
    const double vel_var = force_zero ? 1e-6 : 0.01;
    msg.twist.covariance[0] = vel_var;  // vx variance
    // Non-holonomic constraint: diff-drive can't slide sideways. Tight
    // variance on VY=0 tells robot_localization to treat this as a hard constraint;
    // leaving at 1e6 ("unknown") lets GPS+IMU noise accumulate as apparent
    // lateral drift during outdoor runs.
    msg.twist.covariance[7] = 1e-4;  // vy (enforce VY = 0)
    msg.twist.covariance[14] = 1e6;  // vz - unknown
    msg.twist.covariance[21] = 1e6;  // wx - unknown
    msg.twist.covariance[28] = 1e6;  // wy - unknown
    msg.twist.covariance[35] = force_zero ? 1e-6 : 9e-4;  // wz variance

    pub_wheel_odom_->publish(msg);
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
  // cmd_vel subscriber
  // ---------------------------------------------------------------------------

  void on_cmd_vel(geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
  {
    double vx = msg->twist.linear.x;
    double wz = msg->twist.angular.z;

    // The firmware ignores cmd_vel when mode is IDLE.  When velocity commands
    // arrive (from Nav2 or teleop), ensure the firmware is in AUTONOMOUS mode.
    if (current_mode_ == 0u && (vx != 0.0 || wz != 0.0))
    {
      current_mode_ = 1u;  // AUTONOMOUS
      send_high_level_state();
    }

    // Motor deadband boost: at low |ω| with no linear motion, each wheel
    // gets PWM ≈ |ω|·L/2·PWM_PER_MPS. With L=0.325 and PWM_PER_MPS=300,
    // |ω|=0.5 rad/s → PWM 24, well below the firmware deadband (~PWM 40)
    // → motors buzz and the robot doesn't rotate. Boost sub-threshold pure
    // rotations to MIN_ROT_VEL so the wheels actually engage. Forward
    // motion supplies its own PWM via vx, so we only boost when the
    // robot is essentially stationary in linear.
    constexpr double kMinRotVel = 0.85;  // rad/s, ≈ wheel 0.14 m/s
    constexpr double kVxStationaryThreshold = 0.05;
    if (std::abs(vx) < kVxStationaryThreshold && wz != 0.0 && std::abs(wz) < kMinRotVel)
    {
      wz = std::copysign(kMinRotVel, wz);
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
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_wheel_odom_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr pub_battery_state_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_dock_heading_;

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_cmd_vel_;
  rclcpp::Subscription<mowgli_interfaces::msg::HighLevelStatus>::SharedPtr sub_hl_status_;

  rclcpp::Service<mowgli_interfaces::srv::MowerControl>::SharedPtr srv_mower_control_;
  rclcpp::Service<mowgli_interfaces::srv::EmergencyStop>::SharedPtr srv_emergency_stop_;

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
  double high_level_rate_{2.0};

  std::unique_ptr<SerialPort> serial_;
  PacketHandler packet_handler_;

  // ---------------------------------------------------------------------------
  // Members: stateful state communicated to the STM32
  // ---------------------------------------------------------------------------

  bool emergency_active_{false};
  bool emergency_release_pending_{false};
  int startup_release_count_{5};  // Send release for first 5 heartbeats

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
  bool mow_enabled_{false};
  bool is_charging_{false};
  uint8_t current_mode_{0};
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

  // Odometry state
  int32_t prev_left_ticks_{0};
  int32_t prev_right_ticks_{0};
  bool odom_initialized_{false};
  bool wheels_stationary_{true};
  // Aggregation window: sum firmware packets (~21 ms) until we reach
  // kAggregateMs (~100 ms) and publish one /wheel_odom at ~10 Hz. Widens
  // the velocity denominator so single-tick encoder noise doesn't blow up.
  int32_t odom_acc_delta_left_{0};
  int32_t odom_acc_delta_right_{0};
  uint32_t odom_acc_dt_ms_{0};

  // IMU calibration state (computed while docked and idle, OR when stationary
  // off-dock via auto-cal, OR loaded from the persisted file at boot)
  int imu_cal_samples_{200};
  std::string imu_cal_persist_path_{"/ros2_ws/maps/imu_calibration.txt"};
  double imu_cal_auto_rest_sec_{15.0};
  double imu_cal_periodic_recal_sec_{600.0};  // 0 disables; default 10 min
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

// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0
//
// LowLevel (Pico) board link: serial reconnect + dead-link watchdog, packet
// dispatch, status/emergency/power/battery fan-out, IMU, panel buttons,
// heartbeat, high-level state and the config handshake.

#include <algorithm>
#include <cmath>
#include <cstring>

#include "mowgli_hardware/battery_state_semantics.hpp"
#include "mowgli_hardware/gnss_hardware_status.hpp"
#include "mowgli_interfaces/update_maintenance.hpp"
#include "mowgli_openmower_bridge/blade_policy.hpp"
#include "mowgli_openmower_bridge/openmower_bridge_node.hpp"

namespace mowgli_openmower_bridge
{

namespace
{
constexpr double kLlReconnectPeriodS = 1.0;
constexpr int kConfigTries = 5;
constexpr double kConfigRetryPeriodS = 0.5;
constexpr double kGnssStatusTimeoutS = 2.0;
constexpr std::size_t kReadChunk = 512u;
// OpenMower's LowLevel firmware streams status every 100 ms; a 1 s gap means
// it rebooted or was reflashed, and our config must be pushed again.
constexpr double kLlRestartGapS = 1.0;
}  // namespace

bool OpenMowerBridgeNode::lowlevel_alive() const
{
  if (!ll_serial_ || !ll_serial_->is_open() || ll_last_status_.nanoseconds() == 0)
  {
    return false;
  }
  return (now() - ll_last_status_).seconds() <= std::max(ll_rx_timeout_s_, kLlRestartGapS);
}

void OpenMowerBridgeNode::lowlevel_read_tick()
{
  if (!ll_serial_)
  {
    ll_serial_ = std::make_unique<mowgli_hardware::SerialPort>(ll_serial_port_, ll_baud_rate_);
    ll_packets_.set_callback(
        [this](const uint8_t* data, std::size_t len)
        {
          on_lowlevel_packet(data, len);
        });
  }

  if (!ll_serial_->is_open())
  {
    if ((now() - ll_last_open_attempt_).seconds() < kLlReconnectPeriodS)
    {
      return;
    }
    ll_last_open_attempt_ = now();
    if (!ll_serial_->open())
    {
      RCLCPP_WARN_THROTTLE(get_logger(),
                           *get_clock(),
                           10000,
                           "LowLevel board: cannot open %s — retrying",
                           ll_serial_port_.c_str());
      return;
    }
    ll_packets_.reset_receive_state();
    ll_last_rx_ = now();
    ll_initialized_ = false;
    ll_config_acked_ = false;
    ll_config_tries_left_ = kConfigTries;
    imu_clock_fit_.Reset();
    RCLCPP_INFO(get_logger(), "LowLevel board: opened %s", ll_serial_port_.c_str());
  }

  uint8_t buf[kReadChunk];
  while (true)
  {
    const ssize_t n = ll_serial_->read(buf, kReadChunk);
    if (n < 0)
    {
      RCLCPP_WARN_THROTTLE(get_logger(),
                           *get_clock(),
                           2000,
                           "LowLevel board: read error — reopening %s",
                           ll_serial_port_.c_str());
      ll_serial_->close();
      return;
    }
    if (n == 0)
    {
      break;
    }
    ll_packets_.feed(buf, static_cast<std::size_t>(n));
    ll_last_rx_ = now();
  }

  if (ll_rx_timeout_s_ > 0.0 && (now() - ll_last_rx_).seconds() > ll_rx_timeout_s_)
  {
    RCLCPP_WARN(get_logger(),
                "LowLevel board: no data for %.1f s — reopening %s",
                ll_rx_timeout_s_,
                ll_serial_port_.c_str());
    ll_serial_->close();
  }
}

bool OpenMowerBridgeNode::lowlevel_send(const uint8_t* payload, std::size_t len)
{
  if (!ll_serial_ || !ll_serial_->is_open())
  {
    return false;
  }
  const std::vector<uint8_t> frame = ll_packets_.encode_packet(payload, len);
  const ssize_t written = ll_serial_->write_all(frame.data(), frame.size());
  if (written < 0 || static_cast<std::size_t>(written) != frame.size())
  {
    RCLCPP_WARN(get_logger(),
                "LowLevel board: short write — reopening %s",
                ll_serial_port_.c_str());
    ll_serial_->close();
    return false;
  }
  return true;
}

void OpenMowerBridgeNode::on_lowlevel_packet(const uint8_t* data, std::size_t len)
{
  if (len == 0u)
  {
    return;
  }
  switch (data[0])
  {
    case mowgli_hardware::PACKET_ID_LL_STATUS:
      handle_ll_status(data, len);
      break;
    case mowgli_hardware::PACKET_ID_LL_IMU:
      handle_ll_imu(data, len);
      break;
    case mowgli_hardware::PACKET_ID_LL_UI_EVENT:
      handle_ll_ui_event(data, len);
      break;
    case mowgli_hardware::PACKET_ID_LL_HIGH_LEVEL_CONFIG_REQ:
    case mowgli_hardware::PACKET_ID_LL_HIGH_LEVEL_CONFIG_RSP:
      handle_ll_config(data, len);
      break;
    default:
      RCLCPP_DEBUG(get_logger(), "LowLevel board: unhandled packet 0x%02X (%zu B)", data[0], len);
      break;
  }
}

void OpenMowerBridgeNode::handle_ll_status(const uint8_t* data, std::size_t len)
{
  using mowgli_hardware::LlStatus;
  if (len != sizeof(LlStatus))
  {
    RCLCPP_WARN_THROTTLE(get_logger(),
                         *get_clock(),
                         5000,
                         "LowLevel status packet has %zu bytes, expected %zu",
                         len,
                         sizeof(LlStatus));
    return;
  }
  LlStatus pkt{};
  std::memcpy(&pkt, data, sizeof(LlStatus));

  const rclcpp::Time stamp = now();
  // A status gap longer than the stream period means the board restarted
  // (reflash, brown-out): re-run the config handshake.
  if (ll_last_status_.nanoseconds() != 0 && (stamp - ll_last_status_).seconds() > kLlRestartGapS)
  {
    RCLCPP_WARN(get_logger(), "LowLevel board came back after a gap — re-sending config");
    ll_config_acked_ = false;
    ll_config_tries_left_ = kConfigTries;
  }
  ll_last_status_ = stamp;
  ll_initialized_ = (pkt.status_bitmask & mowgli_hardware::STATUS_BIT_INITIALIZED) != 0u;
  last_ll_status_bitmask_ = pkt.status_bitmask;
  last_ll_emergency_bitmask_ = pkt.emergency_bitmask;

  const bool was_charging = is_charging_;
  is_charging_ = (pkt.status_bitmask & mowgli_hardware::STATUS_BIT_CHARGING) != 0u;
  if (is_charging_ && !was_charging)
  {
    RCLCPP_INFO(get_logger(), "Charging contact detected");
  }
  // Fresh at-rest IMU bias every time the robot lands on the charger.
  if (is_charging_ && !was_charging && !imu_bias_.collecting())
  {
    imu_bias_.Start();
    RCLCPP_INFO(get_logger(), "IMU bias calibration started (docked)");
  }

  publish_status_bundle(pkt);
}

void OpenMowerBridgeNode::publish_status_bundle(const mowgli_hardware::LlStatus& pkt)
{
  const rclcpp::Time stamp = now();
  const MotorTelemetry* mow = motors_[kMow] ? &motors_[kMow]->telemetry() : nullptr;

  // ---- Status ----
  {
    mowgli_interfaces::msg::Status msg{};
    msg.stamp = stamp;
    msg.mower_status = ll_initialized_ ? mowgli_interfaces::msg::Status::MOWER_STATUS_OK
                                       : mowgli_interfaces::msg::Status::MOWER_STATUS_INITIALIZING;
    msg.reset_cause = mowgli_interfaces::msg::Status::RESET_CAUSE_UNKNOWN;
    msg.reset_cause_name = "unknown";
    msg.raspberry_pi_power = (pkt.status_bitmask & mowgli_hardware::STATUS_BIT_RASPI_POWER) != 0u;
    msg.is_charging = is_charging_;
    msg.rain_detected = (pkt.status_bitmask & mowgli_hardware::STATUS_BIT_RAIN) != 0u;
    msg.sound_module_available =
        (pkt.status_bitmask & mowgli_hardware::STATUS_BIT_SOUND_AVAIL) != 0u;
    msg.sound_module_busy = (pkt.status_bitmask & mowgli_hardware::STATUS_BIT_SOUND_BUSY) != 0u;
    msg.ui_board_available = (pkt.status_bitmask & mowgli_hardware::STATUS_BIT_UI_AVAIL) != 0u;
    // Legacy field, blade-activity semantics as in the STM32 bridge.
    msg.esc_power = mow_requested_ || blade_running_;
    msg.mow_enabled = mow_requested_;
    msg.firmware_debug_enabled = false;
    msg.mower_esc_status = blade_running_ ? 1u : 0u;
    if (mow != nullptr && mow->has_status)
    {
      msg.mower_esc_temperature = static_cast<float>(mow->temp_pcb);
      msg.mower_esc_current = static_cast<float>(mow->current_in);
      msg.mower_motor_temperature = static_cast<float>(mow->temp_motor);
      msg.mower_motor_rpm = static_cast<float>(mow->rpm);
    }
    msg.blade_status_stamp = blade_status_stamp_;
    // Firmware handshake: there is no protocol version to compare on the
    // OpenMower LowLevel board. "Compatible" here means the whole hardware set
    // the stack needs is talking: LowLevel status stream + both drive
    // controllers. PreFlightCheck refuses to mow otherwise.
    const bool drives_ok = drive_links_connected();
    msg.firmware_version = "openmower-ll" +
                           std::string(ll_config_acked_ ? "" : " (no config ack)") + " / " +
                           motor_summary();
    msg.firmware_protocol_version = 0u;
    msg.firmware_compatible = ll_initialized_ && lowlevel_alive() && drives_ok;
    pub_status_->publish(msg);
  }

  // ---- Emergency ----
  {
    const EmergencyEvaluation ev = emergency_.Evaluate(pkt.emergency_bitmask);
    emergency_active_now_ = emergency_.is_emergency();
    mowgli_interfaces::msg::Emergency msg{};
    msg.stamp = stamp;
    msg.active_emergency = ev.active;
    msg.latched_emergency = ev.latched;
    msg.lift_warning = false;
    msg.lift_duration_sec = 0.0f;
    msg.reason = ev.reason;
    pub_emergency_->publish(msg);
    if (ev.latched)
    {
      RCLCPP_WARN_THROTTLE(get_logger(),
                           *get_clock(),
                           2000,
                           "Emergency: %s (LowLevel bitmask 0x%02X)",
                           ev.reason.c_str(),
                           pkt.emergency_bitmask);
    }
  }

  // ---- Power ----
  {
    mowgli_interfaces::msg::Power msg{};
    msg.stamp = stamp;
    msg.v_charge = pkt.v_charge;
    msg.v_battery = pkt.v_system;
    msg.charge_current = pkt.charging_current;
    msg.charger_enabled = is_charging_;
    msg.charger_status = is_charging_ ? "charging" : "idle";
    pub_power_->publish(msg);
  }

  // ---- BatteryState (Nav2 docking charge detection, Invariant 12) ----
  {
    sensor_msgs::msg::BatteryState msg{};
    msg.header.stamp = stamp;
    msg.header.frame_id = "base_link";
    msg.voltage = pkt.v_system;
    msg.current = is_charging_ ? std::abs(pkt.charging_current) : 0.0f;
    msg.percentage = mowgli_hardware::battery_percentage_from_firmware(pkt.batt_percentage);
    msg.power_supply_status = is_charging_
                                  ? sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING
                                  : sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
    msg.present = true;
    pub_battery_state_->publish(msg);
  }
}

void OpenMowerBridgeNode::handle_ll_imu(const uint8_t* data, std::size_t len)
{
  using mowgli_hardware::LlImu;
  if (len != sizeof(LlImu))
  {
    RCLCPP_WARN_THROTTLE(get_logger(),
                         *get_clock(),
                         5000,
                         "LowLevel IMU packet has %zu bytes, expected %zu",
                         len,
                         sizeof(LlImu));
    return;
  }
  LlImu pkt{};
  std::memcpy(&pkt, data, sizeof(LlImu));

  const ImuSample raw{pkt.acceleration_mss[0],
                      pkt.acceleration_mss[1],
                      pkt.acceleration_mss[2],
                      pkt.gyro_rads[0],
                      pkt.gyro_rads[1],
                      pkt.gyro_rads[2]};

  // Off-dock auto-calibration after a quiet window (container restarted
  // mid-lawn, robot has not docked since).
  const bool at_rest = odometry_.stationary();
  if (!imu_bias_.ready() && !imu_bias_.collecting() && !is_charging_)
  {
    if (at_rest)
    {
      if (imu_at_rest_since_.nanoseconds() == 0)
      {
        imu_at_rest_since_ = now();
      }
      else if ((now() - imu_at_rest_since_).seconds() >= imu_cal_auto_rest_s_)
      {
        imu_bias_.Start();
        imu_at_rest_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        RCLCPP_INFO(get_logger(), "IMU bias calibration started (stationary off-dock)");
      }
    }
    else
    {
      imu_at_rest_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    }
  }
  if (imu_bias_.collecting())
  {
    if (!at_rest)
    {
      imu_bias_.Abort();
      RCLCPP_WARN(get_logger(), "IMU bias calibration aborted — wheels moved");
    }
    else if (imu_bias_.Feed(raw))
    {
      const ImuBias& b = imu_bias_.bias();
      RCLCPP_INFO(get_logger(),
                  "IMU bias ready: gyro (%.4f %.4f %.4f) rad/s accel (%.3f %.3f) m/s²",
                  b.gx,
                  b.gy,
                  b.gz,
                  b.ax,
                  b.ay);
    }
  }

  const ImuSample s = imu_bias_.Apply(raw);
  sensor_msgs::msg::Imu msg{};
  msg.header.stamp = imu_clock_fit_.Ingest(pkt.dt_millis, now());
  msg.header.frame_id = imu_frame_id_;
  msg.linear_acceleration.x = s.ax;
  msg.linear_acceleration.y = s.ay;
  msg.linear_acceleration.z = s.az;
  msg.angular_velocity.x = s.gx;
  msg.angular_velocity.y = s.gy;
  msg.angular_velocity.z = s.gz;
  // No absolute orientation from this board: identity with a tiny roll/pitch
  // variance and an unknown yaw, the same convention as the STM32 bridge.
  msg.orientation.w = 1.0;
  msg.orientation_covariance[0] = 0.001;
  msg.orientation_covariance[4] = 0.001;
  msg.orientation_covariance[8] = 99.0;
  msg.linear_acceleration_covariance[0] = 0.01;
  msg.linear_acceleration_covariance[4] = 0.01;
  msg.linear_acceleration_covariance[8] = 0.01;
  if (imu_bias_.ready())
  {
    msg.angular_velocity_covariance[0] = 0.001;
    msg.angular_velocity_covariance[4] = 0.001;
    msg.angular_velocity_covariance[8] = std::max(imu_bias_.bias().gz_variance, 0.01);
  }
  else
  {
    msg.angular_velocity_covariance[0] = 0.1;
    msg.angular_velocity_covariance[4] = 0.1;
    msg.angular_velocity_covariance[8] = 1.0;
  }
  pub_imu_->publish(msg);

  if (pub_mag_raw_->get_subscription_count() > 0)
  {
    sensor_msgs::msg::MagneticField mag{};
    mag.header = msg.header;
    mag.magnetic_field.x = pkt.mag_uT[0] * 1.0e-6;
    mag.magnetic_field.y = pkt.mag_uT[1] * 1.0e-6;
    mag.magnetic_field.z = pkt.mag_uT[2] * 1.0e-6;
    mag.magnetic_field_covariance[0] = -1.0;
    pub_mag_raw_->publish(mag);
  }
}

void OpenMowerBridgeNode::handle_ll_ui_event(const uint8_t* data, std::size_t len)
{
  using mowgli_hardware::LlUiEvent;
  if (len != sizeof(LlUiEvent))
  {
    return;
  }
  LlUiEvent pkt{};
  std::memcpy(&pkt, data, sizeof(LlUiEvent));
  RCLCPP_INFO(get_logger(), "UI button: id=%u duration=%u", pkt.button_id, pkt.press_duration);

  // OpenMower CoverUI ids: 2 = HOME, 3 = PLAY, 6 = LOCK (very long press
  // clears the emergency, as in mower_comms_v1).
  using Request = mowgli_interfaces::srv::HighLevelControl::Request;
  uint8_t command = 0u;
  switch (pkt.button_id)
  {
    case 2u:
      command = Request::COMMAND_HOME;
      break;
    case 3u:
      command = Request::COMMAND_START;
      break;
    case 6u:
      if (pkt.press_duration != 2u)
      {
        return;
      }
      command = Request::COMMAND_RESET_EMERGENCY;
      break;
    default:
      return;
  }
  if (!client_high_level_control_->service_is_ready())
  {
    RCLCPP_WARN(get_logger(), "high_level_control not available; button %u dropped", pkt.button_id);
    return;
  }
  auto request = std::make_shared<Request>();
  request->command = command;
  client_high_level_control_->async_send_request(
      request,
      [this, command](rclcpp::Client<mowgli_interfaces::srv::HighLevelControl>::SharedFuture f)
      {
        RCLCPP_INFO(get_logger(),
                    "high_level_control command=%u -> %s",
                    command,
                    f.get()->success ? "ok" : "refused");
      });
}

void OpenMowerBridgeNode::handle_ll_config(const uint8_t* data, std::size_t len)
{
  const lowlevel::HighLevelConfig received = lowlevel::ParseConfigPayload(data, len, ll_config_);
  ll_config_acked_ = true;
  ll_config_tries_left_ = 0;
  RCLCPP_INFO(get_logger(),
              "LowLevel config %s: v_charge_cutoff=%.1f i_charge_cutoff=%.2f v_batt_cutoff=%.1f "
              "v_batt_empty=%.1f v_batt_full=%.1f lift=%u ms tilt=%u ms lang=%.2s",
              data[0] == mowgli_hardware::PACKET_ID_LL_HIGH_LEVEL_CONFIG_RSP ? "response"
                                                                             : "request",
              received.v_charge_cutoff,
              received.i_charge_cutoff,
              received.v_battery_cutoff,
              received.v_battery_empty,
              received.v_battery_full,
              received.lift_period,
              received.tilt_period,
              received.language);
}

void OpenMowerBridgeNode::service_config_handshake()
{
  if (ll_config_acked_ || ll_config_tries_left_ <= 0 || !ll_serial_ || !ll_serial_->is_open())
  {
    return;
  }
  if (ll_config_last_req_.nanoseconds() != 0 &&
      (now() - ll_config_last_req_).seconds() < kConfigRetryPeriodS)
  {
    return;
  }
  ll_config_last_req_ = now();
  --ll_config_tries_left_;
  const auto payload =
      lowlevel::BuildConfigPayload(ll_config_, mowgli_hardware::PACKET_ID_LL_HIGH_LEVEL_CONFIG_REQ);
  lowlevel_send(payload.data(), payload.size());
  if (ll_config_tries_left_ == 0)
  {
    RCLCPP_WARN(get_logger(),
                "LowLevel board never answered the config packet — old firmware? Its compiled "
                "charge/battery limits stay in force.");
  }
}

void OpenMowerBridgeNode::send_heartbeat()
{
  const EmergencyEvaluation ev = emergency_.Evaluate(last_ll_emergency_bitmask_);
  mowgli_hardware::LlHeartbeat pkt{};
  pkt.type = mowgli_hardware::PACKET_ID_LL_HEARTBEAT;
  pkt.emergency_requested = ev.heartbeat_request ? 1u : 0u;
  pkt.emergency_release_requested = ev.heartbeat_release ? 1u : 0u;
  lowlevel_send(reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt) - sizeof(uint16_t));
}

void OpenMowerBridgeNode::send_high_level_state()
{
  const bool gnss_fresh = gps_status_stamp_.nanoseconds() != 0 &&
                          (now() - gps_status_stamp_).seconds() <= kGnssStatusTimeoutS;
  mowgli_hardware::LlHighLevelState pkt{};
  pkt.type = mowgli_hardware::PACKET_ID_LL_HIGH_LEVEL_STATE;
  pkt.current_mode = current_mode_;
  pkt.gps_quality = mowgli_hardware::GnssQualityForFirmware(gnss_fresh, gps_quality_percent_);
  if (last_sent_mode_ != current_mode_)
  {
    RCLCPP_INFO(get_logger(),
                "LowLevel high-level state: mode=%u gps=%u %%",
                current_mode_,
                pkt.gps_quality);
    last_sent_mode_ = current_mode_;
  }
  lowlevel_send(reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt) - sizeof(uint16_t));
}

}  // namespace mowgli_openmower_bridge

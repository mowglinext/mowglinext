// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later

#include "mowgli_leds/led_ring_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

#include "mowgli_interfaces/gnss_status_utils.hpp"

namespace mowgli_leds
{

namespace
{

/// The pure pattern header mirrors HighLevelStatus's HIGH_LEVEL_STATE_*
/// constants so it can stay ROS-free. Pin the two together at COMPILE time --
/// a renumbering on the message side must break the build, not the ring.
using HighLevelStatusMsg = mowgli_interfaces::msg::HighLevelStatus;
static_assert(static_cast<std::uint8_t>(HighLevelState::kNull) ==
                  HighLevelStatusMsg::HIGH_LEVEL_STATE_NULL,
              "HighLevelState::kNull diverged from HighLevelStatus");
static_assert(static_cast<std::uint8_t>(HighLevelState::kIdle) ==
                  HighLevelStatusMsg::HIGH_LEVEL_STATE_IDLE,
              "HighLevelState::kIdle diverged from HighLevelStatus");
static_assert(static_cast<std::uint8_t>(HighLevelState::kAutonomous) ==
                  HighLevelStatusMsg::HIGH_LEVEL_STATE_AUTONOMOUS,
              "HighLevelState::kAutonomous diverged from HighLevelStatus");
static_assert(static_cast<std::uint8_t>(HighLevelState::kRecording) ==
                  HighLevelStatusMsg::HIGH_LEVEL_STATE_RECORDING,
              "HighLevelState::kRecording diverged from HighLevelStatus");
static_assert(static_cast<std::uint8_t>(HighLevelState::kManualMowing) ==
                  HighLevelStatusMsg::HIGH_LEVEL_STATE_MANUAL_MOWING,
              "HighLevelState::kManualMowing diverged from HighLevelStatus");

/// A ring this long would take 4.6 kB per frame; anything larger is a typo in
/// the config, not a real strip, and we refuse to allocate for it.
constexpr std::size_t kMaxLedCount = 512u;

HighLevelState ToHighLevelState(std::uint8_t raw)
{
  switch (raw)
  {
    case HighLevelStatusMsg::HIGH_LEVEL_STATE_IDLE:
      return HighLevelState::kIdle;
    case HighLevelStatusMsg::HIGH_LEVEL_STATE_AUTONOMOUS:
      return HighLevelState::kAutonomous;
    case HighLevelStatusMsg::HIGH_LEVEL_STATE_RECORDING:
      return HighLevelState::kRecording;
    case HighLevelStatusMsg::HIGH_LEVEL_STATE_MANUAL_MOWING:
      return HighLevelState::kManualMowing;
    default:
      return HighLevelState::kNull;
  }
}

}  // namespace

LedRingNode::LedRingNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("led_ring_node", options), start_time_(std::chrono::steady_clock::now())
{
  // ---- Parameters (all declared here; never get_parameter without declare) --
  enabled_ = declare_parameter<bool>("led_enabled", false);
  spi_device_path_ = declare_parameter<std::string>("led_spi_device", "/dev/spidev4.1");
  const int led_count = declare_parameter<int>("led_count", 16);
  brightness_ = static_cast<float>(declare_parameter<double>("led_brightness", 0.6));
  const int spi_speed =
      declare_parameter<int>("led_spi_speed_hz", static_cast<int>(ws2812::kSpiClockHz));
  refresh_hz_ = declare_parameter<double>("led_refresh_hz", 20.0);
  status_timeout_s_ = declare_parameter<double>("led_status_timeout_s", 5.0);
  keepalive_s_ = declare_parameter<double>("led_keepalive_s", 2.0);
  device_retry_s_ = declare_parameter<double>("led_device_retry_s", 30.0);
  const double low_battery = declare_parameter<double>("led_low_battery_percent", 20.0);
  const double charge_full = declare_parameter<double>("led_charge_full_percent", 99.0);
  const double idle_scale = declare_parameter<double>("led_idle_scale", 0.10);

  led_count_ = static_cast<std::size_t>(std::clamp(led_count, 0, static_cast<int>(kMaxLedCount)));
  if (static_cast<int>(led_count_) != led_count)
  {
    RCLCPP_WARN(get_logger(),
                "led_count %d out of range [0, %zu]; clamped to %zu",
                led_count,
                kMaxLedCount,
                led_count_);
  }

  brightness_ = std::clamp(brightness_, 0.0f, 1.0f);
  spi_speed_hz_ = static_cast<std::uint32_t>(std::max(spi_speed, 1));
  if (spi_speed_hz_ != ws2812::kSpiClockHz)
  {
    // The 0b100 / 0b110 symbol table in ws2812_encoder.hpp is derived from
    // 2.4 MHz. At any other clock the bit period is no longer 1.25 us and the
    // strip will show wrong colours -- so say so loudly rather than let it be
    // debugged as a wiring fault.
    RCLCPP_WARN(get_logger(),
                "led_spi_speed_hz=%u is not the %u Hz the WS2812 symbol table assumes; "
                "bit timing will be wrong (see ws2812_encoder.hpp)",
                spi_speed_hz_,
                ws2812::kSpiClockHz);
  }
  refresh_hz_ = std::clamp(refresh_hz_, 1.0, 60.0);
  status_timeout_s_ = std::max(status_timeout_s_, 0.5);
  keepalive_s_ = std::max(keepalive_s_, 0.2);
  device_retry_s_ = std::max(device_retry_s_, 1.0);

  pattern_cfg_.led_count = led_count_;
  pattern_cfg_.low_battery_percent = static_cast<float>(std::clamp(low_battery, 0.0, 100.0));
  pattern_cfg_.charge_full_percent = static_cast<float>(std::clamp(charge_full, 0.0, 100.0));
  pattern_cfg_.idle_scale = static_cast<float>(std::clamp(idle_scale, 0.0, 1.0));

  if (!enabled_ || led_count_ == 0u)
  {
    // Stand down completely: no subscriptions, no timer, no device. The launch
    // file also gates on led_enabled, so this only fires when the node is run
    // directly -- but it must still cost nothing.
    RCLCPP_INFO(get_logger(),
                "WS2812 status ring disabled (led_enabled=%s, led_count=%zu). Idle.",
                enabled_ ? "true" : "false",
                led_count_);
    return;
  }

  // ---- Subscriptions -------------------------------------------------------
  // Status topics: reliable, depth 10 (see .claude/rules/ros2.md), matching the
  // publishers in mowgli_behavior and mowgli_hardware.
  status_sub_ = create_subscription<HighLevelStatus>("/behavior_tree_node/high_level_status",
                                                     rclcpp::QoS(10),
                                                     [this](HighLevelStatus::ConstSharedPtr msg)
                                                     {
                                                       latest_status_ = *msg;
                                                       have_status_ = true;
                                                       status_time_s_ = monotonicSeconds();
                                                     });

  gnss_sub_ = create_subscription<GnssStatus>(
      "/gps/status",
      rclcpp::QoS(10),
      [this](GnssStatus::ConstSharedPtr msg)
      {
        // Same typed helper the behavior tree and hardware bridge use, so the
        // ring can never disagree with them about what "RTK fixed" means.
        gnss_rtk_fixed_ = mowgli_interfaces::gnss_status_utils::BehaviorTreeRtkFixed(*msg);
        have_gnss_ = true;
        gnss_time_s_ = monotonicSeconds();
      });

  // Charging fallback for when the behavior tree is silent. charger_enabled is
  // the firmware's STATUS_BIT_CHARGING, the same bit that drives
  // HighLevelStatus.is_charging, so the two can never contradict each other.
  power_sub_ = create_subscription<Power>("/hardware_bridge/power",
                                          rclcpp::QoS(10),
                                          [this](Power::ConstSharedPtr msg)
                                          {
                                            power_charging_ = msg->charger_enabled;
                                            have_power_ = true;
                                            power_time_s_ = monotonicSeconds();
                                          });

  // Integer-millisecond period, matching the rest of the stack. refresh_hz_ is
  // clamped to [1, 60] above, so this always lands in [17, 1000] ms.
  const auto period =
      std::chrono::milliseconds(static_cast<int>(std::lround(1000.0 / refresh_hz_)));
  timer_ = create_wall_timer(period,
                             [this]()
                             {
                               onTimer();
                             });

  RCLCPP_INFO(get_logger(),
              "WS2812 status ring: %zu LEDs on %s at %u Hz SPI, brightness %.2f, %.1f Hz refresh",
              led_count_,
              spi_device_path_.c_str(),
              spi_speed_hz_,
              static_cast<double>(brightness_),
              refresh_hz_);
}

LedRingNode::~LedRingNode()
{
  blank();
}

double LedRingNode::monotonicSeconds() const
{
  const std::chrono::duration<double> elapsed = std::chrono::steady_clock::now() - start_time_;
  return elapsed.count();
}

LedInputs LedRingNode::collectInputs() const
{
  const double now_s = monotonicSeconds();

  LedInputs in;
  in.now_s = now_s;
  in.status_fresh = have_status_ && (now_s - status_time_s_) <= status_timeout_s_;

  if (in.status_fresh)
  {
    in.state = ToHighLevelState(latest_status_.state);
    in.coverage_percent = latest_status_.coverage_percent;
    in.battery_percent = latest_status_.battery_percent;
    in.battery_valid = std::isfinite(latest_status_.battery_percent);
    in.is_charging = latest_status_.is_charging;
    in.emergency = latest_status_.emergency;
  }
  else if (have_power_ && (now_s - power_time_s_) <= status_timeout_s_)
  {
    // Behavior tree silent, hardware bridge still talking. Charging is the one
    // thing Power can answer on its own; the percent is left invalid on
    // purpose (see the CHARGING note in led_pattern.hpp).
    in.is_charging = power_charging_;
  }

  in.rtk_fixed = have_gnss_ && (now_s - gnss_time_s_) <= status_timeout_s_ && gnss_rtk_fixed_;
  return in;
}

void LedRingNode::ensureDevice()
{
  if (spi_.IsOpen())
  {
    return;
  }

  const double now_s = monotonicSeconds();
  if (now_s < next_retry_s_)
  {
    return;
  }
  next_retry_s_ = now_s + device_retry_s_;

  const SpiResult result = spi_.Open(spi_device_path_, spi_speed_hz_);
  if (result.ok)
  {
    // Force the next render to write, since the strip's contents are unknown
    // after a device outage.
    have_last_pixels_ = false;
    if (warned_device_)
    {
      RCLCPP_INFO(get_logger(), "SPI device %s is available again", spi_device_path_.c_str());
      warned_device_ = false;
    }
    return;
  }

  if (!warned_device_)
  {
    // ONE warning per outage. A missing overlay (no /dev/spidev at all) is the
    // expected state on a robot where the operator has not enabled it yet, and
    // it must not produce a log line every refresh tick.
    RCLCPP_WARN(get_logger(),
                "WS2812 ring unavailable: %s. Standing down; retrying every %.0f s. "
                "Enable the SPI4-M0 overlay (see mowgli_leds/README.md) to use the ring.",
                result.error.c_str(),
                device_retry_s_);
    warned_device_ = true;
  }
}

void LedRingNode::writeFrame(const std::vector<Rgb>& pixels)
{
  ensureDevice();
  if (!spi_.IsOpen())
  {
    return;
  }

  const double now_s = monotonicSeconds();
  const bool changed = !have_last_pixels_ || pixels != last_pixels_;
  const bool keepalive_due = (now_s - last_write_s_) >= keepalive_s_;
  if (!changed && !keepalive_due)
  {
    return;
  }

  const SpiResult result = spi_.Write(ws2812::Encode(pixels, brightness_));
  if (!result.ok)
  {
    if (!warned_device_)
    {
      RCLCPP_WARN(get_logger(),
                  "WS2812 ring write failed: %s. Standing down; retrying every %.0f s.",
                  result.error.c_str(),
                  device_retry_s_);
      warned_device_ = true;
    }
    // Drop the descriptor so ensureDevice() re-opens it -- covers a device
    // that disappeared (USB/overlay reload) rather than one that is merely
    // busy.
    spi_.Close();
    have_last_pixels_ = false;
    next_retry_s_ = now_s + device_retry_s_;
    return;
  }

  last_pixels_ = pixels;
  have_last_pixels_ = true;
  last_write_s_ = now_s;
}

void LedRingNode::blank()
{
  if (!spi_.IsOpen())
  {
    return;
  }
  const std::vector<Rgb> off(led_count_, colors::kOff);
  const SpiResult result = spi_.Write(ws2812::Encode(off, brightness_));
  (void)result;  // Best effort on shutdown; nothing useful left to do on error.
  spi_.Close();
}

void LedRingNode::onTimer()
{
  writeFrame(RenderFrame(collectInputs(), pattern_cfg_));
}

}  // namespace mowgli_leds

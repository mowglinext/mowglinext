// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// ROS2 glue for the WS2812 status ring: subscribe to robot status, render the
// pure pattern (led_pattern.hpp), encode it (ws2812_encoder.hpp) and push the
// bytes at the SPI device (spi_device.hpp).
//
// This node is READ-ONLY with respect to the robot. It subscribes and it
// writes to /dev/spidev. It publishes nothing, commands nothing, and shares no
// state with the motion, dig-detection, coverage or localization stacks.

#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "mowgli_interfaces/msg/gnss_status.hpp"
#include "mowgli_interfaces/msg/high_level_status.hpp"
#include "mowgli_interfaces/msg/power.hpp"
#include "mowgli_leds/led_pattern.hpp"
#include "mowgli_leds/spi_device.hpp"
#include "mowgli_leds/ws2812_encoder.hpp"

namespace mowgli_leds
{

class LedRingNode : public rclcpp::Node
{
public:
  explicit LedRingNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~LedRingNode() override;

private:
  using GnssStatus = mowgli_interfaces::msg::GnssStatus;
  using HighLevelStatus = mowgli_interfaces::msg::HighLevelStatus;
  using Power = mowgli_interfaces::msg::Power;

  /// Seconds since node construction, from a STEADY clock.
  ///
  /// Deliberately not rclcpp::Node::now(): the ring is a physical device on
  /// the operator's wall clock, so its animations must not freeze under
  /// use_sim_time and must not jump when NTP steps the system clock.
  double monotonicSeconds() const;

  void onTimer();
  LedInputs collectInputs() const;

  /// (Re)open the SPI device if it is closed and the retry backoff has
  /// elapsed. Warns at most once per outage; silent while standing down.
  void ensureDevice();

  /// Write `pixels` if they differ from the last frame written, or if the
  /// keepalive interval has elapsed. Cheap by construction: a static pattern
  /// costs one SPI write every led_keepalive_s.
  void writeFrame(const std::vector<Rgb>& pixels);

  /// Best-effort all-off frame, used on shutdown so the ring does not stay
  /// lit after the stack stops.
  void blank();

  // -- Parameters ------------------------------------------------------------
  bool enabled_ = false;
  std::string spi_device_path_;
  std::size_t led_count_ = 16u;
  float brightness_ = 0.6f;
  std::uint32_t spi_speed_hz_ = ws2812::kSpiClockHz;
  double refresh_hz_ = 20.0;
  double status_timeout_s_ = 5.0;
  double keepalive_s_ = 2.0;
  double device_retry_s_ = 30.0;
  LedPatternCfg pattern_cfg_{};

  // -- Latest status ---------------------------------------------------------
  HighLevelStatus latest_status_{};
  bool have_status_ = false;
  double status_time_s_ = 0.0;

  bool gnss_rtk_fixed_ = false;
  bool have_gnss_ = false;
  double gnss_time_s_ = 0.0;

  bool power_charging_ = false;
  bool have_power_ = false;
  double power_time_s_ = 0.0;

  // -- Output state ----------------------------------------------------------
  SpiDevice spi_;
  std::vector<Rgb> last_pixels_;
  bool have_last_pixels_ = false;
  double last_write_s_ = 0.0;
  /// One WARN per outage, cleared when the device comes back.
  bool warned_device_ = false;
  double next_retry_s_ = 0.0;

  std::chrono::steady_clock::time_point start_time_;

  rclcpp::Subscription<HighLevelStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<GnssStatus>::SharedPtr gnss_sub_;
  rclcpp::Subscription<Power>::SharedPtr power_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace mowgli_leds

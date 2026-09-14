// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: GPL-3.0

#include "mowgli_gnss_bridge/universal_gnss_topic_bridge.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>

namespace mowgli_gnss_bridge
{
namespace
{

// --- Enum translation tables (Universal -> Public) --------------------------
// Mapped by symbolic constant, NOT by numeric value: the CAP_* bit values and
// the FIX_TYPE_* enum values differ between the two message packages.

std::uint8_t toPublicFixType(std::uint8_t universal_fix_type)
{
  switch (universal_fix_type) {
    case UniversalGnssStatus::FIX_TYPE_UNKNOWN:
    case UniversalGnssStatus::FIX_TYPE_NO_FIX:
      return PublicGnssStatus::FIX_TYPE_NO_FIX;
    case UniversalGnssStatus::FIX_TYPE_FIX:
      return PublicGnssStatus::FIX_TYPE_GPS_FIX;
    case UniversalGnssStatus::FIX_TYPE_RTK_FLOAT:
      return PublicGnssStatus::FIX_TYPE_RTK_FLOAT;
    case UniversalGnssStatus::FIX_TYPE_RTK_FIXED:
      return PublicGnssStatus::FIX_TYPE_RTK_FIXED;
    case UniversalGnssStatus::FIX_TYPE_DEAD_RECKONING:
      return PublicGnssStatus::FIX_TYPE_DEAD_RECKONING;
    default:
      return PublicGnssStatus::FIX_TYPE_NO_FIX;
  }
}

std::uint8_t toPublicRtkMode(std::uint8_t universal_rtk_mode)
{
  switch (universal_rtk_mode) {
    case UniversalGnssStatus::RTK_MODE_NONE:
      return PublicGnssStatus::RTK_MODE_NONE;
    case UniversalGnssStatus::RTK_MODE_FLOAT:
      return PublicGnssStatus::RTK_MODE_FLOAT;
    case UniversalGnssStatus::RTK_MODE_FIXED:
      return PublicGnssStatus::RTK_MODE_FIXED;
    case UniversalGnssStatus::RTK_MODE_UNKNOWN:
    default:
      return PublicGnssStatus::RTK_MODE_UNKNOWN;
  }
}

std::uint8_t toPublicBaselineStatus(std::uint8_t universal_status)
{
  switch (universal_status) {
    case UniversalGnssStatus::BASELINE_STATUS_COMPUTED:
      return PublicGnssStatus::BASELINE_STATUS_COMPUTED;
    case UniversalGnssStatus::BASELINE_STATUS_NOT_SOLVED:
      return PublicGnssStatus::BASELINE_STATUS_NOT_SOLVED;
    case UniversalGnssStatus::BASELINE_STATUS_INSUFFICIENT_OBSERVATIONS:
      return PublicGnssStatus::BASELINE_STATUS_INSUFFICIENT_OBSERVATIONS;
    case UniversalGnssStatus::BASELINE_STATUS_NO_CONVERGENCE:
      return PublicGnssStatus::BASELINE_STATUS_NO_CONVERGENCE;
    case UniversalGnssStatus::BASELINE_STATUS_OUT_OF_TOLERANCE:
      return PublicGnssStatus::BASELINE_STATUS_OUT_OF_TOLERANCE;
    case UniversalGnssStatus::BASELINE_STATUS_COVARIANCE_TRACE_EXCEEDED:
      return PublicGnssStatus::BASELINE_STATUS_COVARIANCE_TRACE_EXCEEDED;
    case UniversalGnssStatus::BASELINE_STATUS_NOT_CONFIGURED:
      return PublicGnssStatus::BASELINE_STATUS_NOT_CONFIGURED;
    case UniversalGnssStatus::BASELINE_STATUS_UNKNOWN:
    default:
      return PublicGnssStatus::BASELINE_STATUS_UNKNOWN;
  }
}

// Ordered {universal_bit, public_bit} pairs, mirroring
// UNIVERSAL_TO_PUBLIC_CAPABILITY in the reference Python.
constexpr std::array<std::pair<std::uint32_t, std::uint32_t>, 24> kCapabilityMap{{
  {UniversalGnssStatus::CAP_RTK_MODE, PublicGnssStatus::CAP_RTK_MODE},
  {UniversalGnssStatus::CAP_HORIZONTAL_ACCURACY, PublicGnssStatus::CAP_HORIZONTAL_ACCURACY},
  {UniversalGnssStatus::CAP_VERTICAL_ACCURACY, PublicGnssStatus::CAP_VERTICAL_ACCURACY},
  {UniversalGnssStatus::CAP_HDOP, PublicGnssStatus::CAP_HDOP},
  {UniversalGnssStatus::CAP_VDOP, PublicGnssStatus::CAP_VDOP},
  {UniversalGnssStatus::CAP_SATELLITES_USED, PublicGnssStatus::CAP_SATELLITES_USED},
  {UniversalGnssStatus::CAP_SATELLITES_VISIBLE, PublicGnssStatus::CAP_SATELLITES_VISIBLE},
  {UniversalGnssStatus::CAP_SATELLITES_TRACKED, PublicGnssStatus::CAP_SATELLITES_TRACKED},
  {UniversalGnssStatus::CAP_MEAN_CN0, PublicGnssStatus::CAP_MEAN_CN0},
  {UniversalGnssStatus::CAP_MAX_CN0, PublicGnssStatus::CAP_MAX_CN0},
  {UniversalGnssStatus::CAP_CORRECTION_AGE, PublicGnssStatus::CAP_CORRECTION_AGE},
  {UniversalGnssStatus::CAP_HEADING, PublicGnssStatus::CAP_HEADING},
  {UniversalGnssStatus::CAP_HEADING_ACCURACY, PublicGnssStatus::CAP_HEADING_ACCURACY},
  {UniversalGnssStatus::CAP_DIFFERENTIAL_CORRECTIONS,
    PublicGnssStatus::CAP_DIFFERENTIAL_CORRECTIONS},
  {UniversalGnssStatus::CAP_CORRECTIONS_ACTIVE, PublicGnssStatus::CAP_CORRECTIONS_ACTIVE},
  {UniversalGnssStatus::CAP_DUAL_ANTENNA_HEADING, PublicGnssStatus::CAP_DUAL_ANTENNA_STATUS},
  {UniversalGnssStatus::CAP_INTERFERENCE_STATE, PublicGnssStatus::CAP_INTERFERENCE_STATUS},
  {UniversalGnssStatus::CAP_JAMMING_STATE, PublicGnssStatus::CAP_JAMMING_STATUS},
  {UniversalGnssStatus::CAP_DUAL_ANTENNA_BASELINE, PublicGnssStatus::CAP_DUAL_ANTENNA_BASELINE},
  {UniversalGnssStatus::CAP_BASELINE_AZIMUTH, PublicGnssStatus::CAP_BASELINE_AZIMUTH},
  {UniversalGnssStatus::CAP_BASELINE_PITCH, PublicGnssStatus::CAP_BASELINE_PITCH},
  {UniversalGnssStatus::CAP_BASELINE_LENGTH, PublicGnssStatus::CAP_BASELINE_LENGTH},
  {UniversalGnssStatus::CAP_BASELINE_SOLUTION_STATUS,
    PublicGnssStatus::CAP_BASELINE_SOLUTION_STATUS},
}};

std::uint32_t mapCapabilityFlags(std::uint32_t flags)
{
  std::uint32_t mapped = 0;
  for (const auto & [source_flag, target_flag] : kCapabilityMap) {
    if (flags & source_flag) {
      mapped |= target_flag;
    }
  }
  return mapped;
}

float fixTypeQuality(std::uint8_t public_fix_type)
{
  switch (public_fix_type) {
    case PublicGnssStatus::FIX_TYPE_GPS_FIX:
      return 25.0F;
    case PublicGnssStatus::FIX_TYPE_RTK_FLOAT:
      return 50.0F;
    case PublicGnssStatus::FIX_TYPE_RTK_FIXED:
      return 100.0F;
    case PublicGnssStatus::FIX_TYPE_DEAD_RECKONING:
      return 10.0F;
    case PublicGnssStatus::FIX_TYPE_NO_FIX:
    default:
      return 0.0F;
  }
}

// --- String helpers (mirror the Python module-level functions) --------------

std::string toLower(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  return value;
}

std::string strip(const std::string & value)
{
  const auto is_space = [](unsigned char c) {return std::isspace(c) != 0;};
  auto begin = value.begin();
  while (begin != value.end() && is_space(*begin)) {
    ++begin;
  }
  auto end = value.end();
  while (end != begin && is_space(*(end - 1))) {
    --end;
  }
  return std::string(begin, end);
}

std::string normalizeReceiverVendor(const std::string & receiver_family)
{
  const std::string family = toLower(strip(receiver_family));
  if (family == "ublox") {
    return "u-blox";
  }
  if (family == "unicore") {
    return "Unicore";
  }
  if (family == "nmea") {
    return "NMEA";
  }
  return "";
}

}  // namespace

UniversalGnssTopicBridge::UniversalGnssTopicBridge(const rclcpp::NodeOptions & options)
: rclcpp::Node("universal_gnss_topic_bridge", options)
{
  backend_ = declare_parameter<std::string>("backend", "universal");
  const std::string receiver_family = declare_parameter<std::string>("receiver_family", "auto");
  frame_id_ = declare_parameter<std::string>("frame_id", "gps_link");

  const std::string input_status_topic =
    declare_parameter<std::string>("input_status_topic", "/_gps_internal/universal/status");
  const std::string output_status_topic =
    declare_parameter<std::string>("output_status_topic", "/gps/status");
  const std::string input_diagnostics_topic =
    declare_parameter<std::string>("input_diagnostics_topic", "/diagnostics");
  const std::string input_rtcm_topic =
    declare_parameter<std::string>("input_rtcm_topic", "/_gps_internal/universal/rtcm");
  const std::string output_rtcm_topic =
    declare_parameter<std::string>("output_rtcm_topic", "/rtcm");
  const double correction_diagnostic_timeout_s =
    declare_parameter<double>("correction_diagnostic_timeout_s", 2.0);

  receiver_vendor_ = normalizeReceiverVendor(receiver_family);
  correction_diagnostics_ = std::make_unique<CorrectionDiagnosticTracker>(
    std::chrono::duration<double>(correction_diagnostic_timeout_s));

  // Reliable / volatile, matching the reference Python bridge exactly so this
  // node stays connected to the C++ receiver_node and ntrip_node publishers.
  // Status + diagnostics: depth 10. RTCM correction stream: depth 50.
  const rclcpp::QoS reliable_qos = rclcpp::QoS(10).reliable().durability_volatile();
  const rclcpp::QoS rtcm_qos = rclcpp::QoS(50).reliable().durability_volatile();

  status_pub_ = create_publisher<PublicGnssStatus>(output_status_topic, reliable_qos);
  rtcm_pub_ = create_publisher<PublicRtcmMessage>(output_rtcm_topic, rtcm_qos);

  status_sub_ = create_subscription<UniversalGnssStatus>(
    input_status_topic, reliable_qos,
    [this](const UniversalGnssStatus & msg) {onStatus(msg);});
  diagnostics_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    input_diagnostics_topic, reliable_qos,
    [this](const diagnostic_msgs::msg::DiagnosticArray & msg) {onDiagnostics(msg);});
  rtcm_sub_ = create_subscription<RtcmFrame>(
    input_rtcm_topic, rtcm_qos,
    [this](const RtcmFrame & msg) {onRtcm(msg);});

  RCLCPP_INFO(
    get_logger(),
    "Bridging Universal GNSS topics: %s -> %s, %s -> %s correction_stream/msm_summary, %s -> %s",
    input_status_topic.c_str(), output_status_topic.c_str(),
    input_diagnostics_topic.c_str(), output_status_topic.c_str(),
    input_rtcm_topic.c_str(), output_rtcm_topic.c_str());
}

void UniversalGnssTopicBridge::onStatus(const UniversalGnssStatus & msg)
{
  PublicGnssStatus public_msg;
  public_msg.header.stamp = msg.stamp;
  public_msg.header.frame_id = frame_id_;
  // Receiver observation identity, copied verbatim. It advances only for a
  // genuinely accepted position observation and stays put for the timer-driven
  // cached republication, which is what lets consumers tell a frozen receiver
  // from a live one. See mowgli_interfaces/gnss_observation_freshness.hpp.
  public_msg.position_observation_sequence = msg.position_observation_sequence;
  public_msg.backend = backend_;
  public_msg.receiver_vendor = receiver_vendor_;

  const std::uint8_t fix_type = toPublicFixType(msg.fix_type);
  public_msg.fix_type = fix_type;
  public_msg.fix_valid = msg.fix_valid;
  public_msg.dead_reckoning = (fix_type == PublicGnssStatus::FIX_TYPE_DEAD_RECKONING);
  public_msg.rtk_mode = toPublicRtkMode(msg.rtk_mode);
  public_msg.quality_percent = fixTypeQuality(fix_type);
  public_msg.capability_flags = mapCapabilityFlags(msg.capability_flags);
  public_msg.value_flags = mapCapabilityFlags(msg.value_flags);

  public_msg.hdop = msg.hdop;
  public_msg.vdop = msg.vdop;
  public_msg.horizontal_accuracy_m = msg.horizontal_accuracy_m;
  public_msg.vertical_accuracy_m = msg.vertical_accuracy_m;
  public_msg.heading_deg = msg.heading_deg;
  public_msg.heading_accuracy_deg = msg.heading_accuracy_deg;
  public_msg.differential_corrections = msg.differential_corrections;
  public_msg.corrections_active = msg.corrections_active;
  public_msg.satellites_used = msg.satellites_used;
  public_msg.satellites_visible = msg.satellites_visible;
  public_msg.satellites_tracked = msg.satellites_tracked;
  public_msg.correction_age_s = msg.correction_age_s;
  public_msg.mean_cn0_db_hz = msg.mean_cn0_db_hz;
  public_msg.max_cn0_db_hz = msg.max_cn0_db_hz;
  public_msg.dual_antenna_heading = msg.dual_antenna_heading;
  public_msg.dual_antenna_baseline = msg.dual_antenna_baseline;
  public_msg.interference_detected = msg.interference_detected;
  public_msg.jamming_detected = msg.jamming_detected;
  public_msg.baseline_azimuth_deg = msg.baseline_azimuth_deg;
  public_msg.baseline_pitch_deg = msg.baseline_pitch_deg;
  public_msg.baseline_length_m = msg.baseline_length_m;
  public_msg.baseline_solution_status = toPublicBaselineStatus(msg.baseline_solution_status);

  applyDiagnosticProjection(public_msg);

  status_pub_->publish(public_msg);
}

void UniversalGnssTopicBridge::onDiagnostics(const diagnostic_msgs::msg::DiagnosticArray & msg)
{
  correction_diagnostics_->update(msg);
}

void UniversalGnssTopicBridge::applyDiagnosticProjection(PublicGnssStatus & public_msg) const
{
  correction_diagnostics_->apply(public_msg);
}

void UniversalGnssTopicBridge::onRtcm(const RtcmFrame & msg)
{
  PublicRtcmMessage public_msg;
  // Direct byte-vector copy (both are std::vector<uint8_t>) — this is the hot
  // path that the Python port made expensive via per-byte list() conversion.
  public_msg.message = msg.data;
  rtcm_pub_->publish(public_msg);
}

}  // namespace mowgli_gnss_bridge

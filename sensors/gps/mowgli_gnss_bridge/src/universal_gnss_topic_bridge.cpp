// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: GPL-3.0

#include "mowgli_gnss_bridge/universal_gnss_topic_bridge.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdint>
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
    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return value;
}

std::string strip(const std::string & value)
{
  const auto is_space = [](unsigned char c) { return std::isspace(c) != 0; };
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

bool contains(const std::string & haystack, const std::string & needle)
{
  return haystack.find(needle) != std::string::npos;
}

bool endsWith(const std::string & value, const std::string & suffix)
{
  if (suffix.size() > value.size()) {
    return false;
  }
  return std::equal(suffix.rbegin(), suffix.rend(), value.rbegin());
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

std::uint8_t correctionStreamStatusFromMessage(const std::string & message)
{
  const std::string normalized = toLower(strip(message));
  if (contains(normalized, "write error") || endsWith(normalized, "error")) {
    return PublicGnssStatus::CORRECTION_STREAM_STATUS_ERROR;
  }
  if (contains(normalized, "unavailable")) {
    return PublicGnssStatus::CORRECTION_STREAM_STATUS_UNAVAILABLE;
  }
  if (contains(normalized, "waiting")) {
    return PublicGnssStatus::CORRECTION_STREAM_STATUS_WAITING;
  }
  if (contains(normalized, "active")) {
    return PublicGnssStatus::CORRECTION_STREAM_STATUS_ACTIVE;
  }
  if (contains(normalized, "idle")) {
    return PublicGnssStatus::CORRECTION_STREAM_STATUS_IDLE;
  }
  return PublicGnssStatus::CORRECTION_STREAM_STATUS_UNKNOWN;
}

// Optional-parse helpers: mirror _parse_diagnostic_bool/uint/float. A missing or
// unparseable value leaves the caller's target untouched (matching Python None).

bool parseBool(const std::string & value, bool & out)
{
  const std::string normalized = toLower(strip(value));
  if (normalized == "true") {
    out = true;
    return true;
  }
  if (normalized == "false") {
    out = false;
    return true;
  }
  return false;
}

bool parseUint(const std::string & value, std::uint32_t & out)
{
  const std::string trimmed = strip(value);
  if (trimmed.empty()) {
    return false;
  }
  try {
    std::size_t consumed = 0;
    const long long parsed = std::stoll(trimmed, &consumed, 10);  // NOLINT(runtime/int)
    if (consumed != trimmed.size() || parsed < 0) {
      return false;
    }
    out = static_cast<std::uint32_t>(parsed);
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

bool parseFloat(const std::string & value, float & out)
{
  const std::string trimmed = strip(value);
  if (trimmed.empty()) {
    return false;
  }
  try {
    std::size_t consumed = 0;
    const double parsed = std::stod(trimmed, &consumed);
    if (consumed != trimmed.size()) {
      return false;
    }
    out = static_cast<float>(parsed);
    return true;
  } catch (const std::exception &) {
    return false;
  }
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

  receiver_vendor_ = normalizeReceiverVendor(receiver_family);

  // Reliable / volatile, matching the reference Python bridge exactly so this
  // node stays connected to the C++ receiver_node and ntrip_node publishers.
  // Status + diagnostics: depth 10. RTCM correction stream: depth 50.
  const rclcpp::QoS reliable_qos = rclcpp::QoS(10).reliable().durability_volatile();
  const rclcpp::QoS rtcm_qos = rclcpp::QoS(50).reliable().durability_volatile();

  status_pub_ = create_publisher<PublicGnssStatus>(output_status_topic, reliable_qos);
  rtcm_pub_ = create_publisher<PublicRtcmMessage>(output_rtcm_topic, rtcm_qos);

  status_sub_ = create_subscription<UniversalGnssStatus>(
    input_status_topic, reliable_qos,
    [this](const UniversalGnssStatus & msg) { onStatus(msg); });
  diagnostics_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    input_diagnostics_topic, reliable_qos,
    [this](const diagnostic_msgs::msg::DiagnosticArray & msg) { onDiagnostics(msg); });
  rtcm_sub_ = create_subscription<RtcmFrame>(
    input_rtcm_topic, rtcm_qos,
    [this](const RtcmFrame & msg) { onRtcm(msg); });

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
  for (const auto & status : msg.status) {
    DiagnosticEntry entry;
    entry.message = strip(status.message);
    for (const auto & item : status.values) {
      const std::string key = strip(item.key);
      if (key.empty()) {
        continue;
      }
      entry.values[key] = strip(item.value);
    }
    diagnostic_entries_[status.name] = std::move(entry);
  }
}

const UniversalGnssTopicBridge::DiagnosticEntry * UniversalGnssTopicBridge::pickDiagnosticEntry(
  std::initializer_list<const char *> names) const
{
  for (const char * name : names) {
    const auto it = diagnostic_entries_.find(name);
    if (it != diagnostic_entries_.end()) {
      return &it->second;
    }
  }
  return nullptr;
}

void UniversalGnssTopicBridge::applyDiagnosticProjection(PublicGnssStatus & public_msg) const
{
  // Correction-stream summary.
  const DiagnosticEntry * correction_entry = pickDiagnosticEntry(
    {"universal_gnss_ntrip/rtcm_forwarding", "universal_gnss/rtcm_forwarding"});
  if (correction_entry != nullptr) {
    const std::uint8_t correction_stream_status =
      correctionStreamStatusFromMessage(correction_entry->message);
    public_msg.capability_flags |= PublicGnssStatus::CAP_CORRECTION_STREAM;
    public_msg.correction_stream_status = correction_stream_status;
    if (correction_stream_status != PublicGnssStatus::CORRECTION_STREAM_STATUS_UNKNOWN) {
      public_msg.value_flags |= PublicGnssStatus::CAP_CORRECTION_STREAM;
    }
  }

  // MSM summary.
  const DiagnosticEntry * msm_entry = pickDiagnosticEntry(
    {"universal_gnss_ntrip/rtcm_semantic/msm_summary",
      "universal_gnss/rtcm_semantic/msm_summary"});
  if (msm_entry == nullptr) {
    return;
  }

  const auto & values = msm_entry->values;
  const auto lookup = [&values](const char * key) -> const std::string * {
    const auto it = values.find(key);
    return it != values.end() ? &it->second : nullptr;
  };

  bool has_value = false;

  bool seen = false;
  if (const std::string * v = lookup("seen"); v && parseBool(*v, seen)) {
    has_value = true;
  }
  bool decoded = false;
  if (const std::string * v = lookup("decoded"); v && parseBool(*v, decoded)) {
    has_value = true;
  }
  bool valid = false;
  if (const std::string * v = lookup("valid"); v && parseBool(*v, valid)) {
    has_value = true;
  }

  std::uint32_t message_type = 0;
  if (const std::string * v = lookup("message_type"); v && parseUint(*v, message_type)) {
    has_value = true;
  }
  std::uint32_t station_id = 0;
  if (const std::string * v = lookup("station_id"); v && parseUint(*v, station_id)) {
    has_value = true;
  }
  std::uint32_t satellite_count = 0;
  if (const std::string * v = lookup("satellite_count"); v && parseUint(*v, satellite_count)) {
    has_value = true;
  }
  std::uint32_t signal_count = 0;
  if (const std::string * v = lookup("signal_count"); v && parseUint(*v, signal_count)) {
    has_value = true;
  }
  std::uint32_t cell_count = 0;
  if (const std::string * v = lookup("cell_count"); v && parseUint(*v, cell_count)) {
    has_value = true;
  }
  float age_s = 0.0F;
  if (const std::string * v = lookup("age_s"); v && parseFloat(*v, age_s)) {
    has_value = true;
  }
  std::string constellations_seen;
  if (const std::string * v = lookup("constellations_seen"); v != nullptr) {
    constellations_seen = *v;
    if (!constellations_seen.empty()) {
      has_value = true;
    }
  }

  public_msg.capability_flags |= PublicGnssStatus::CAP_MSM_SUMMARY;
  public_msg.msm_summary_seen = seen;
  public_msg.msm_summary_decoded = decoded;
  public_msg.msm_summary_valid = valid;
  public_msg.msm_summary_message_type = static_cast<std::uint16_t>(message_type);
  public_msg.msm_summary_station_id = static_cast<std::uint16_t>(station_id);
  public_msg.msm_summary_constellations_seen = constellations_seen;
  public_msg.msm_summary_satellite_count = static_cast<std::uint16_t>(satellite_count);
  public_msg.msm_summary_signal_count = static_cast<std::uint16_t>(signal_count);
  public_msg.msm_summary_cell_count = static_cast<std::uint16_t>(cell_count);
  public_msg.msm_summary_age_s = age_s;
  if (has_value) {
    public_msg.value_flags |= PublicGnssStatus::CAP_MSM_SUMMARY;
  }
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

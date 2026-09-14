// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: GPL-3.0
//
// Correction-diagnostics projection, extracted from the bridge's inline
// helpers. Mirrors CorrectionDiagnosticTracker in the reference Python bridge
// (sensors/gps/universal_gnss_topic_bridge.py) exactly.

#include "mowgli_gnss_bridge/correction_diagnostic_tracker.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>

namespace mowgli_gnss_bridge
{
namespace
{

using PublicGnssStatus = mowgli_interfaces::msg::GnssStatus;

constexpr char kNtripPrefix[] = "universal_gnss_ntrip/";
constexpr char kReceiverPrefix[] = "universal_gnss/";
constexpr double kSemanticFreshnessSeconds = 5.0;

/// Cached (message, key/value) pair for one DiagnosticStatus entry.
struct DiagnosticEntry
{
  std::string message;
  std::unordered_map<std::string, std::string> values;
};

/// One producer's complete, source-identified contribution.
struct Snapshot
{
  bool valid{false};
  std::string source;
  std::map<std::string, DiagnosticEntry> entries;
  CorrectionDiagnosticTracker::TimePoint received_at{};
};

struct OwnerState
{
  std::optional<Snapshot> snapshot;
  std::optional<std::int64_t> stamp_watermark_ns;
};

struct Observation
{
  bool present{false};
  bool has_value{false};
  bool seen{false};
  bool decoded{false};
  bool valid{false};
  std::uint32_t message_type{0};
  std::uint32_t station_id{0};
  std::uint32_t satellite_count{0};
  std::uint32_t signal_count{0};
  std::uint32_t cell_count{0};
  std::uint32_t decode_failure_count{0};
  std::uint32_t malformed_count{0};
  float age_s{0.0F};
  std::string constellations_seen;
};

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

std::string toLower(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  return value;
}

bool startsWith(const std::string & value, const std::string & prefix)
{
  return value.compare(0, prefix.size(), prefix) == 0;
}

bool contains(const std::string & value, const std::string & needle)
{
  return value.find(needle) != std::string::npos;
}

bool endsWith(const std::string & value, const std::string & suffix)
{
  if (suffix.size() > value.size()) {
    return false;
  }
  return std::equal(suffix.rbegin(), suffix.rend(), value.rbegin());
}

// Optional-parse helpers: mirror _parse_diagnostic_bool/uint/float. A missing or
// unparseable value leaves the caller's target untouched (matching Python None).

bool parseBool(const std::string * value, bool & out)
{
  if (value == nullptr) {
    return false;
  }
  const std::string normalized = toLower(strip(*value));
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

bool parseUint(const std::string * value, std::uint32_t & out)
{
  if (value == nullptr) {
    return false;
  }
  const std::string trimmed = strip(*value);
  if (trimmed.empty() || trimmed.front() == '-') {
    return false;
  }
  try {
    std::size_t consumed = 0;
    const unsigned long long parsed = std::stoull(trimmed, &consumed, 10);  // NOLINT(runtime/int)
    if (consumed != trimmed.size()) {
      return false;
    }
    out = static_cast<std::uint32_t>(parsed);
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

bool parseFloat(const std::string * value, float & out)
{
  if (value == nullptr) {
    return false;
  }
  const std::string trimmed = strip(*value);
  if (trimmed.empty()) {
    return false;
  }
  try {
    std::size_t consumed = 0;
    const float parsed = std::stof(trimmed, &consumed);
    if (consumed != trimmed.size() || !std::isfinite(parsed)) {
      return false;
    }
    out = parsed;
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

const std::string * lookup(const DiagnosticEntry & entry, const char * key)
{
  const auto it = entry.values.find(key);
  return it == entry.values.end() ? nullptr : &it->second;
}

const DiagnosticEntry * findEntry(const Snapshot & snapshot, const char * name)
{
  const auto it = snapshot.entries.find(name);
  return it == snapshot.entries.end() ? nullptr : &it->second;
}

bool hasEntry(const Snapshot & snapshot, const char * name)
{
  return findEntry(snapshot, name) != nullptr;
}

std::optional<std::int64_t> diagnosticStampNs(
  const diagnostic_msgs::msg::DiagnosticArray & diagnostics)
{
  if (diagnostics.header.stamp.sec == 0 && diagnostics.header.stamp.nanosec == 0U) {
    return std::nullopt;
  }
  return static_cast<std::int64_t>(diagnostics.header.stamp.sec) * 1000000000LL +
         static_cast<std::int64_t>(diagnostics.header.stamp.nanosec);
}

void updateOwner(
  OwnerState & owner, const diagnostic_msgs::msg::DiagnosticArray & diagnostics,
  const std::string & prefix, CorrectionDiagnosticTracker::TimePoint received_at)
{
  Snapshot candidate;
  candidate.valid = true;
  candidate.received_at = received_at;
  bool represented = false;
  std::set<std::string> source_ids;

  for (const auto & status : diagnostics.status) {
    if (!startsWith(status.name, prefix)) {
      continue;
    }
    represented = true;
    const std::string hardware_id = strip(status.hardware_id);
    if (!hardware_id.empty()) {
      source_ids.insert(hardware_id);
    }

    DiagnosticEntry entry;
    entry.message = strip(status.message);
    for (const auto & item : status.values) {
      const std::string key = strip(item.key);
      if (!key.empty()) {
        entry.values[key] = strip(item.value);
      }
    }
    candidate.entries[status.name] = std::move(entry);
  }

  // An array about another component is OMIT for this owner.
  if (!represented) {
    return;
  }

  // Pinned Universal GNSS supplies one non-empty hardware_id for every status.
  // Missing or conflicting identity is therefore an explicit fail-closed CLEAR.
  if (source_ids.size() != 1U) {
    candidate.valid = false;
    candidate.entries.clear();
  } else {
    candidate.source = *source_ids.begin();
  }

  const auto stamp_ns = diagnosticStampNs(diagnostics);
  if (stamp_ns.has_value() && owner.stamp_watermark_ns.has_value() &&
    *stamp_ns < *owner.stamp_watermark_ns)
  {
    // ROS time is ordering evidence only. An older snapshot cannot refresh the
    // steady-clock lifetime or resurrect a source-owned value after CLEAR.
    return;
  }
  if (stamp_ns.has_value()) {
    if (!owner.stamp_watermark_ns.has_value() || *stamp_ns > *owner.stamp_watermark_ns) {
      owner.stamp_watermark_ns = stamp_ns;
    }
  }
  owner.snapshot = std::move(candidate);
}

bool isFresh(
  const std::optional<Snapshot> & snapshot, CorrectionDiagnosticTracker::TimePoint now,
  CorrectionDiagnosticTracker::Clock::duration timeout)
{
  if (!snapshot.has_value() || !snapshot->valid || now < snapshot->received_at) {
    return false;
  }
  return now - snapshot->received_at < timeout;
}

std::uint8_t correctionStreamStatusFromMessage(const std::string & message)
{
  const std::string normalized = toLower(strip(message));
  if (contains(normalized, "write error") || contains(normalized, "write errors") ||
    endsWith(normalized, "error"))
  {
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
  if (contains(normalized, "stale") || contains(normalized, "idle")) {
    return PublicGnssStatus::CORRECTION_STREAM_STATUS_IDLE;
  }
  return PublicGnssStatus::CORRECTION_STREAM_STATUS_UNKNOWN;
}

std::uint8_t transportStatus(const Snapshot & ntrip)
{
  if (hasEntry(ntrip, "universal_gnss_ntrip/ntrip_failed")) {
    return PublicGnssStatus::CORRECTION_TRANSPORT_STATUS_FAILED;
  }
  if (hasEntry(ntrip, "universal_gnss_ntrip/ntrip_reconnecting")) {
    return PublicGnssStatus::CORRECTION_TRANSPORT_STATUS_RECONNECTING;
  }
  if (hasEntry(ntrip, "universal_gnss_ntrip/ntrip_streaming")) {
    return PublicGnssStatus::CORRECTION_TRANSPORT_STATUS_STREAMING;
  }
  if (hasEntry(ntrip, "universal_gnss_ntrip/ntrip_connected")) {
    return PublicGnssStatus::CORRECTION_TRANSPORT_STATUS_CONNECTED;
  }
  if (hasEntry(ntrip, "universal_gnss_ntrip/ntrip_connecting")) {
    return PublicGnssStatus::CORRECTION_TRANSPORT_STATUS_CONNECTING;
  }
  if (hasEntry(ntrip, "universal_gnss_ntrip/ntrip_disconnected")) {
    return PublicGnssStatus::CORRECTION_TRANSPORT_STATUS_DISCONNECTED;
  }
  return PublicGnssStatus::CORRECTION_TRANSPORT_STATUS_UNKNOWN;
}

Observation parseObservation(const DiagnosticEntry * entry)
{
  Observation observation;
  if (entry == nullptr) {
    return observation;
  }
  observation.present = true;
  observation.has_value |= parseBool(lookup(*entry, "seen"), observation.seen);
  observation.has_value |= parseBool(lookup(*entry, "decoded"), observation.decoded);
  observation.has_value |= parseBool(lookup(*entry, "valid"), observation.valid);
  observation.has_value |= parseUint(lookup(*entry, "message_type"), observation.message_type);
  observation.has_value |= parseUint(lookup(*entry, "station_id"), observation.station_id);
  observation.has_value |=
    parseUint(lookup(*entry, "satellite_count"), observation.satellite_count);
  observation.has_value |= parseUint(lookup(*entry, "signal_count"), observation.signal_count);
  observation.has_value |= parseUint(lookup(*entry, "cell_count"), observation.cell_count);
  observation.has_value |=
    parseUint(lookup(*entry, "decode_failure_count"), observation.decode_failure_count);
  observation.has_value |=
    parseUint(lookup(*entry, "malformed_count"), observation.malformed_count);
  observation.has_value |= parseFloat(lookup(*entry, "age_s"), observation.age_s);
  if (const std::string * value = lookup(*entry, "constellations_seen"); value != nullptr) {
    observation.constellations_seen = *value;
    observation.has_value |= !value->empty();
  }
  return observation;
}

std::uint8_t flowStatus(const Snapshot & ntrip, std::uint8_t transport)
{
  if (transport != PublicGnssStatus::CORRECTION_TRANSPORT_STATUS_STREAMING) {
    return PublicGnssStatus::CORRECTION_FLOW_STATUS_IDLE;
  }

  const DiagnosticEntry * summary = findEntry(ntrip, "universal_gnss_ntrip/summary");
  bool parser_healthy = true;
  if (summary != nullptr && parseBool(lookup(*summary, "parser_healthy"), parser_healthy) &&
    !parser_healthy)
  {
    return PublicGnssStatus::CORRECTION_FLOW_STATUS_INVALID;
  }
  bool stale_data = false;
  if ((summary != nullptr && parseBool(lookup(*summary, "stale_data"), stale_data) && stale_data) ||
    hasEntry(ntrip, "universal_gnss_ntrip/rtcm.stream_stale"))
  {
    return PublicGnssStatus::CORRECTION_FLOW_STATUS_STALE;
  }
  if (hasEntry(ntrip, "universal_gnss_ntrip/correction_flowing")) {
    return PublicGnssStatus::CORRECTION_FLOW_STATUS_ACTIVE;
  }
  return PublicGnssStatus::CORRECTION_FLOW_STATUS_WAITING;
}

std::uint8_t semanticStatus(const Snapshot & ntrip, std::uint8_t transport)
{
  if (transport != PublicGnssStatus::CORRECTION_TRANSPORT_STATUS_STREAMING) {
    return PublicGnssStatus::CORRECTION_SEMANTIC_STATUS_UNAVAILABLE;
  }

  const DiagnosticEntry * summary = findEntry(ntrip, "universal_gnss_ntrip/summary");
  if (summary == nullptr) {
    return PublicGnssStatus::CORRECTION_SEMANTIC_STATUS_WAITING;
  }

  bool parser_healthy = false;
  const bool has_parser_health = parseBool(lookup(*summary, "parser_healthy"), parser_healthy);
  if (has_parser_health && !parser_healthy) {
    return PublicGnssStatus::CORRECTION_SEMANTIC_STATUS_INVALID;
  }

  bool stale_data = false;
  if (parseBool(lookup(*summary, "stale_data"), stale_data) && stale_data) {
    return PublicGnssStatus::CORRECTION_SEMANTIC_STATUS_STALE;
  }

  bool correction_available = false;
  const bool has_correction_available =
    parseBool(lookup(*summary, "correction_available"), correction_available);
  const Observation base =
    parseObservation(findEntry(ntrip, "universal_gnss_ntrip/rtcm_semantic/base_station_arp"));
  const Observation msm =
    parseObservation(findEntry(ntrip, "universal_gnss_ntrip/rtcm_semantic/msm_summary"));

  const bool base_usable = base.present && base.seen && base.decoded && base.valid;
  const bool msm_fresh = msm.age_s >= 0.0F && msm.age_s <= kSemanticFreshnessSeconds;
  const bool msm_usable = msm.present && msm.seen && msm.decoded && msm.valid &&
    msm.cell_count > 0U && msm_fresh && msm.malformed_count == 0U;
  const bool station_matches =
    base.station_id != 0U && msm.station_id != 0U && base.station_id == msm.station_id;

  if (has_correction_available && correction_available && has_parser_health && parser_healthy &&
    base_usable && msm_usable && station_matches)
  {
    return PublicGnssStatus::CORRECTION_SEMANTIC_STATUS_HEALTHY;
  }
  if ((msm.present && msm.seen &&
    (!msm.decoded || !msm.valid || msm.cell_count == 0U || msm.malformed_count > 0U)) ||
    (base.present && base.seen && (!base.decoded || !base.valid)) ||
    (base_usable && msm_usable && !station_matches) ||
    (has_correction_available && correction_available))
  {
    return PublicGnssStatus::CORRECTION_SEMANTIC_STATUS_INVALID;
  }
  if (msm.present && msm.seen && !msm_fresh) {
    return PublicGnssStatus::CORRECTION_SEMANTIC_STATUS_STALE;
  }
  return PublicGnssStatus::CORRECTION_SEMANTIC_STATUS_WAITING;
}

void applyMsm(const Snapshot & snapshot, const char * name, PublicGnssStatus & status)
{
  const Observation msm = parseObservation(findEntry(snapshot, name));
  if (!msm.present) {
    return;
  }
  status.msm_summary_seen = msm.seen;
  status.msm_summary_decoded = msm.decoded;
  status.msm_summary_valid = msm.valid;
  status.msm_summary_message_type = static_cast<std::uint16_t>(msm.message_type);
  status.msm_summary_station_id = static_cast<std::uint16_t>(msm.station_id);
  status.msm_summary_constellations_seen = msm.constellations_seen;
  status.msm_summary_satellite_count = static_cast<std::uint16_t>(msm.satellite_count);
  status.msm_summary_signal_count = static_cast<std::uint16_t>(msm.signal_count);
  status.msm_summary_cell_count = static_cast<std::uint16_t>(msm.cell_count);
  status.msm_summary_age_s = msm.age_s;
  status.msm_summary_source = snapshot.source;
  if (msm.has_value) {
    status.value_flags |= PublicGnssStatus::CAP_MSM_SUMMARY;
  }
}

}  // namespace

class CorrectionDiagnosticTracker::Impl
{
public:
  explicit Impl(std::chrono::duration<double> timeout)
  : timeout_(std::chrono::duration_cast<Clock::duration>(timeout))
  {
    if (!std::isfinite(timeout.count()) || timeout.count() <= 0.0) {
      throw std::invalid_argument("correction diagnostic timeout must be finite and positive");
    }
  }

  void update(const diagnostic_msgs::msg::DiagnosticArray & diagnostics, TimePoint received_at)
  {
    updateOwner(ntrip_, diagnostics, kNtripPrefix, received_at);
    updateOwner(receiver_, diagnostics, kReceiverPrefix, received_at);
  }

  void apply(PublicGnssStatus & status, TimePoint now) const
  {
    constexpr std::uint32_t kCorrectionCapabilities =
      PublicGnssStatus::CAP_CORRECTION_STREAM | PublicGnssStatus::CAP_MSM_SUMMARY |
      PublicGnssStatus::CAP_CORRECTION_TRANSPORT | PublicGnssStatus::CAP_CORRECTION_FLOW |
      PublicGnssStatus::CAP_CORRECTION_SEMANTIC;
    status.capability_flags |= kCorrectionCapabilities;
    status.value_flags &= ~kCorrectionCapabilities;

    status.correction_stream_status = PublicGnssStatus::CORRECTION_STREAM_STATUS_UNKNOWN;
    status.correction_transport_status = PublicGnssStatus::CORRECTION_TRANSPORT_STATUS_UNKNOWN;
    status.correction_response_accepted = false;
    status.correction_flow_status = PublicGnssStatus::CORRECTION_FLOW_STATUS_UNKNOWN;
    status.correction_semantic_status = PublicGnssStatus::CORRECTION_SEMANTIC_STATUS_UNKNOWN;
    status.correction_source.clear();
    status.correction_forwarding_source.clear();
    status.msm_summary_source.clear();

    const Snapshot * current_ntrip =
      isFresh(ntrip_.snapshot, now, timeout_) ? &*ntrip_.snapshot : nullptr;
    const Snapshot * current_receiver =
      isFresh(receiver_.snapshot, now, timeout_) ? &*receiver_.snapshot : nullptr;

    // Receiver forwarding is confirmed device-write state. NTRIP forwarding
    // only reports publication and is a legitimate compatibility fallback.
    const DiagnosticEntry * forwarding = nullptr;
    const Snapshot * forwarding_owner = nullptr;
    if (current_receiver != nullptr) {
      forwarding = findEntry(*current_receiver, "universal_gnss/rtcm_forwarding");
      forwarding_owner = current_receiver;
    } else if (current_ntrip != nullptr) {
      forwarding = findEntry(*current_ntrip, "universal_gnss_ntrip/rtcm_forwarding");
      forwarding_owner = current_ntrip;
    }
    if (forwarding != nullptr && forwarding_owner != nullptr) {
      status.correction_stream_status = correctionStreamStatusFromMessage(forwarding->message);
      status.correction_forwarding_source = forwarding_owner->source;
      if (status.correction_stream_status != PublicGnssStatus::CORRECTION_STREAM_STATUS_UNKNOWN) {
        status.value_flags |= PublicGnssStatus::CAP_CORRECTION_STREAM;
      }
    }

    if (current_ntrip != nullptr) {
      const std::uint8_t transport = transportStatus(*current_ntrip);
      status.correction_transport_status = transport;
      status.correction_response_accepted =
        transport == PublicGnssStatus::CORRECTION_TRANSPORT_STATUS_STREAMING;
      status.correction_flow_status = flowStatus(*current_ntrip, transport);
      status.correction_semantic_status = semanticStatus(*current_ntrip, transport);
      status.correction_source = current_ntrip->source;
      if (transport != PublicGnssStatus::CORRECTION_TRANSPORT_STATUS_UNKNOWN) {
        status.value_flags |= PublicGnssStatus::CAP_CORRECTION_TRANSPORT;
      }
      status.value_flags |= PublicGnssStatus::CAP_CORRECTION_FLOW;
      status.value_flags |= PublicGnssStatus::CAP_CORRECTION_SEMANTIC;
    }

    // MSM details follow the live NTRIP source only while it is streaming.
    // Otherwise a current receiver observation is shown explicitly, without
    // fabricating NTRIP transport or semantic health from it.
    if (current_ntrip != nullptr &&
      status.correction_transport_status ==
      PublicGnssStatus::CORRECTION_TRANSPORT_STATUS_STREAMING)
    {
      applyMsm(*current_ntrip, "universal_gnss_ntrip/rtcm_semantic/msm_summary", status);
    } else if (current_receiver != nullptr) {
      applyMsm(*current_receiver, "universal_gnss/rtcm_semantic/msm_summary", status);
    }
  }

private:
  Clock::duration timeout_;
  OwnerState ntrip_;
  OwnerState receiver_;
};

CorrectionDiagnosticTracker::CorrectionDiagnosticTracker(std::chrono::duration<double> timeout)
: impl_(std::make_unique<Impl>(timeout)) {}

CorrectionDiagnosticTracker::~CorrectionDiagnosticTracker() = default;

CorrectionDiagnosticTracker::CorrectionDiagnosticTracker(CorrectionDiagnosticTracker &&) noexcept =
  default;

CorrectionDiagnosticTracker & CorrectionDiagnosticTracker::operator=(
  CorrectionDiagnosticTracker &&) noexcept = default;

void CorrectionDiagnosticTracker::update(
  const diagnostic_msgs::msg::DiagnosticArray & diagnostics, TimePoint received_at)
{
  impl_->update(diagnostics, received_at);
}

void CorrectionDiagnosticTracker::apply(PublicGnssStatus & status, TimePoint now) const
{
  impl_->apply(status, now);
}

}  // namespace mowgli_gnss_bridge

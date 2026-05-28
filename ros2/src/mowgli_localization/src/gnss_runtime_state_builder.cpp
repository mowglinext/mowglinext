// Copyright 2026 Mowgli Project
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

#include "mowgli_localization/gnss_runtime_state_builder.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <exception>
#include <limits>
#include <optional>
#include <string>
#include <utility>

namespace mowgli_localization
{

namespace
{

constexpr float kCorrectionsFreshThresholdSec = 5.0f;

std::string Trim(std::string value)
{
  const auto begin = std::find_if_not(value.begin(),
                                      value.end(),
                                      [](unsigned char c)
                                      {
                                        return std::isspace(c) != 0;
                                      });
  const auto end = std::find_if_not(value.rbegin(),
                                    value.rend(),
                                    [](unsigned char c)
                                    {
                                      return std::isspace(c) != 0;
                                    })
                       .base();
  if (begin >= end)
  {
    return {};
  }
  return std::string(begin, end);
}

std::string Lowercase(std::string value)
{
  value = Trim(std::move(value));
  std::transform(value.begin(),
                 value.end(),
                 value.begin(),
                 [](unsigned char c)
                 {
                   return static_cast<char>(std::tolower(c));
                 });
  return value;
}

std::optional<bool> ParseBool(const std::string& value)
{
  const std::string lowered = Lowercase(value);
  if (lowered == "true" || lowered == "1" || lowered == "yes" || lowered == "on")
  {
    return true;
  }
  if (lowered == "false" || lowered == "0" || lowered == "no" || lowered == "off")
  {
    return false;
  }
  return std::nullopt;
}

std::optional<float> ParseFloat(const std::string& value)
{
  const std::string lowered = Lowercase(value);
  if (lowered.empty() || lowered == "n/a" || lowered == "na" || lowered == "inf" ||
      lowered == "infinity" || lowered == "∞" || lowered == "--")
  {
    return std::nullopt;
  }

  try
  {
    const float parsed = std::stof(value);
    if (!std::isfinite(parsed))
    {
      return std::nullopt;
    }
    return parsed;
  }
  catch (const std::exception&)
  {
    return std::nullopt;
  }
}

std::optional<int> ParseInt(const std::string& value)
{
  const std::string lowered = Lowercase(value);
  if (lowered.empty() || lowered == "n/a" || lowered == "na" || lowered == "--")
  {
    return std::nullopt;
  }

  try
  {
    return std::stoi(value);
  }
  catch (const std::exception&)
  {
    return std::nullopt;
  }
}

std::optional<uint16_t> ParseU16(const std::string& value)
{
  const auto parsed = ParseInt(value);
  if (!parsed.has_value() || *parsed < 0 || *parsed > std::numeric_limits<uint16_t>::max())
  {
    return std::nullopt;
  }
  return static_cast<uint16_t>(*parsed);
}

std::optional<std::string> LookupString(const GnssDiagnosticEntry* entry, const std::string& key)
{
  if (entry == nullptr)
  {
    return std::nullopt;
  }
  const auto it = entry->values.find(key);
  if (it == entry->values.end() || it->second.empty())
  {
    return std::nullopt;
  }
  const std::string trimmed = Trim(it->second);
  if (trimmed.empty())
  {
    return std::nullopt;
  }
  return trimmed;
}

std::optional<bool> LookupBool(const GnssDiagnosticEntry* entry, const std::string& key)
{
  const auto value = LookupString(entry, key);
  if (!value.has_value())
  {
    return std::nullopt;
  }
  return ParseBool(*value);
}

std::optional<float> LookupFloat(const GnssDiagnosticEntry* entry, const std::string& key)
{
  const auto value = LookupString(entry, key);
  if (!value.has_value())
  {
    return std::nullopt;
  }
  return ParseFloat(*value);
}

std::optional<uint16_t> LookupU16(const GnssDiagnosticEntry* entry, const std::string& key)
{
  const auto value = LookupString(entry, key);
  if (!value.has_value())
  {
    return std::nullopt;
  }
  return ParseU16(*value);
}

const GnssDiagnosticEntry* FindEntry(const GnssDiagnosticSnapshot& snapshot, const std::string& name)
{
  const auto it = snapshot.entries.find(name);
  if (it == snapshot.entries.end())
  {
    return nullptr;
  }
  return &it->second;
}

void SetBackendMetadata(GnssRuntimeState& state, GnssBackendKind backend)
{
  switch (backend)
  {
    case GnssBackendKind::kNmea:
      state.backend = "nmea";
      state.receiver_vendor = "generic";
      MarkCapability(state, GnssRuntimeCapability::kRtkMode);
      MarkCapability(state, GnssRuntimeCapability::kDifferentialCorrections);
      break;
    case GnssBackendKind::kUblox:
      state.backend = "ublox";
      state.receiver_vendor = "u-blox";
      MarkCapability(state, GnssRuntimeCapability::kRtkMode);
      MarkCapability(state, GnssRuntimeCapability::kHdop);
      MarkCapability(state, GnssRuntimeCapability::kVdop);
      MarkCapability(state, GnssRuntimeCapability::kHorizontalAccuracy);
      MarkCapability(state, GnssRuntimeCapability::kVerticalAccuracy);
      MarkCapability(state, GnssRuntimeCapability::kSatellitesUsed);
      MarkCapability(state, GnssRuntimeCapability::kSatellitesVisible);
      MarkCapability(state, GnssRuntimeCapability::kDifferentialCorrections);
      MarkCapability(state, GnssRuntimeCapability::kCorrectionsActive);
      MarkCapability(state, GnssRuntimeCapability::kCorrectionAge);
      MarkCapability(state, GnssRuntimeCapability::kMeanCn0);
      MarkCapability(state, GnssRuntimeCapability::kMaxCn0);
      break;
    case GnssBackendKind::kUnicore:
      state.backend = "unicore";
      state.receiver_vendor = "Unicore";
      MarkCapability(state, GnssRuntimeCapability::kRtkMode);
      MarkCapability(state, GnssRuntimeCapability::kHorizontalAccuracy);
      MarkCapability(state, GnssRuntimeCapability::kVerticalAccuracy);
      MarkCapability(state, GnssRuntimeCapability::kSatellitesUsed);
      MarkCapability(state, GnssRuntimeCapability::kSatellitesVisible);
      MarkCapability(state, GnssRuntimeCapability::kSatellitesTracked);
      MarkCapability(state, GnssRuntimeCapability::kDifferentialCorrections);
      MarkCapability(state, GnssRuntimeCapability::kCorrectionsActive);
      MarkCapability(state, GnssRuntimeCapability::kCorrectionAge);
      MarkCapability(state, GnssRuntimeCapability::kMeanCn0);
      MarkCapability(state, GnssRuntimeCapability::kMaxCn0);
      MarkCapability(state, GnssRuntimeCapability::kDualAntennaStatus);
      MarkCapability(state, GnssRuntimeCapability::kInterferenceStatus);
      MarkCapability(state, GnssRuntimeCapability::kJammingStatus);
      break;
    case GnssBackendKind::kUnknown:
    default:
      break;
  }
}

void SetFixTypeFromQuality(GnssRuntimeState& state, int quality)
{
  switch (quality)
  {
    case 4:
      state.fix_type = GnssFixType::kRtkFixed;
      state.fix_valid = true;
      state.differential_corrections = true;
      state.rtk_mode = GnssRtkMode::kFixed;
      break;
    case 5:
      state.fix_type = GnssFixType::kRtkFloat;
      state.fix_valid = true;
      state.differential_corrections = true;
      state.rtk_mode = GnssRtkMode::kFloat;
      break;
    default:
      if (quality > 0)
      {
        state.fix_type = GnssFixType::kGpsFix;
        state.fix_valid = true;
        state.differential_corrections = false;
        state.rtk_mode = GnssRtkMode::kNone;
      }
      else
      {
        state.fix_type = GnssFixType::kNoFix;
        state.fix_valid = false;
      }
      break;
  }
}

void SetFixTypeFromFixTypeLabel(GnssRuntimeState& state, const std::string& value)
{
  const std::string lowered = Lowercase(value);
  if (lowered.find("dead") != std::string::npos || lowered.find("+dr") != std::string::npos)
  {
    state.fix_type = GnssFixType::kDeadReckoning;
    state.dead_reckoning = true;
    if (!state.rtk_mode.has_value())
    {
      state.rtk_mode = GnssRtkMode::kNone;
    }
    return;
  }
  if (lowered == "no-fix" || lowered == "none")
  {
    state.fix_type = GnssFixType::kNoFix;
    state.fix_valid = false;
  }
}

std::optional<bool> CorrectionsActiveFromReceiverAge(std::optional<float> age_s)
{
  if (!age_s.has_value())
  {
    return std::nullopt;
  }
  return *age_s <= kCorrectionsFreshThresholdSec;
}

std::optional<bool> CorrectionsActiveFromTransport(std::optional<float> age_s,
                                                   std::optional<float> msgs_per_sec)
{
  if (age_s.has_value())
  {
    return *age_s <= kCorrectionsFreshThresholdSec &&
           (!msgs_per_sec.has_value() || *msgs_per_sec > 0.0f);
  }
  if (msgs_per_sec.has_value())
  {
    return *msgs_per_sec > 0.0f;
  }
  return std::nullopt;
}

std::optional<bool> ParseUnicoreDualAntennaStatus(const std::string& value)
{
  // Recognized Unicore RTK diagnostic strings from RTKSTATUSA/B:
  //   "within limit"                 -> heading baseline solved/usable
  //   "baseline unsolved"            -> dual-antenna heading unavailable
  //   "out of limit"                 -> baseline invalid
  //   "baseline length not configured" -> baseline feature present but disabled
  //   "unknown"                      -> explicit negative state from the driver
  const std::string lowered = Lowercase(value);
  if (lowered == "within limit")
  {
    return true;
  }
  if (lowered == "baseline unsolved" || lowered == "out of limit" ||
      lowered == "baseline length not configured" || lowered == "unknown")
  {
    return false;
  }
  return std::nullopt;
}

void ApplyUbloxDiagnostics(GnssRuntimeState& state, const GnssDiagnosticSnapshot& snapshot)
{
  const auto* fix = FindEntry(snapshot, "GPS: fix");
  const auto* sats = FindEntry(snapshot, "GPS: satellites");
  const auto* rtcm = FindEntry(snapshot, "GPS: NTRIP/RTCM");

  if (const auto gps_fix_ok = LookupBool(fix, "gps_fix_ok"); gps_fix_ok.has_value())
  {
    state.fix_valid = *gps_fix_ok;
  }
  if (const auto diff_corr = LookupBool(fix, "diff_corr"); diff_corr.has_value())
  {
    state.differential_corrections = *diff_corr;
  }
  if (const auto carr_soln = LookupString(fix, "carr_soln"); carr_soln.has_value())
  {
    const std::string lowered = Lowercase(*carr_soln);
    if (lowered == "fixed")
    {
      state.fix_type = GnssFixType::kRtkFixed;
      state.fix_valid = true;
      state.rtk_mode = GnssRtkMode::kFixed;
      state.differential_corrections = true;
    }
    else if (lowered == "float")
    {
      state.fix_type = GnssFixType::kRtkFloat;
      state.fix_valid = true;
      state.rtk_mode = GnssRtkMode::kFloat;
      state.differential_corrections = true;
    }
    else if (lowered == "none")
    {
      if (state.fix_valid)
      {
        state.fix_type = GnssFixType::kGpsFix;
        state.rtk_mode = GnssRtkMode::kNone;
      }
      else
      {
        state.fix_type = GnssFixType::kNoFix;
      }
    }
  }
  if (const auto fix_type = LookupString(fix, "fix_type"); fix_type.has_value())
  {
    SetFixTypeFromFixTypeLabel(state, *fix_type);
    if (state.fix_type == GnssFixType::kNoFix &&
        Lowercase(*fix_type).find("dead") == std::string::npos && state.fix_valid)
    {
      state.fix_type = GnssFixType::kGpsFix;
    }
  }

  if (const auto hdop = LookupFloat(fix, "hdop"); hdop.has_value())
  {
    state.hdop = *hdop;
    MarkCapability(state, GnssRuntimeCapability::kHdop);
  }
  if (const auto vdop = LookupFloat(fix, "vdop"); vdop.has_value())
  {
    state.vdop = *vdop;
    MarkCapability(state, GnssRuntimeCapability::kVdop);
  }
  if (const auto hacc = LookupFloat(fix, "horizontal_accuracy_m"); hacc.has_value())
  {
    state.horizontal_accuracy_m = *hacc;
    MarkCapability(state, GnssRuntimeCapability::kHorizontalAccuracy);
  }
  if (const auto vacc = LookupFloat(fix, "vertical_accuracy_m"); vacc.has_value())
  {
    state.vertical_accuracy_m = *vacc;
    MarkCapability(state, GnssRuntimeCapability::kVerticalAccuracy);
  }

  if (const auto used = LookupU16(sats, "used"); used.has_value())
  {
    state.satellites_used = *used;
  }
  if (const auto visible = LookupU16(sats, "visible"); visible.has_value())
  {
    state.satellites_visible = *visible;
  }
  if (const auto mean_cn0 = LookupFloat(sats, "mean_cno_db_hz"); mean_cn0.has_value())
  {
    state.mean_cn0_db_hz = *mean_cn0;
  }
  if (const auto max_cn0 = LookupFloat(sats, "max_cno_db_hz"); max_cn0.has_value())
  {
    state.max_cn0_db_hz = *max_cn0;
  }

  const auto corr_age = LookupFloat(rtcm, "age_of_last_corr_s");
  const auto msgs_per_sec = LookupFloat(rtcm, "msgs_per_sec");
  if (corr_age.has_value())
  {
    state.correction_age_s = *corr_age;
  }
  if (const auto corrections_active = CorrectionsActiveFromTransport(corr_age, msgs_per_sec);
      corrections_active.has_value())
  {
    state.corrections_active = *corrections_active;
  }
}

void ApplyUnicoreDiagnostics(GnssRuntimeState& state, const GnssDiagnosticSnapshot& snapshot)
{
  const auto* fix = FindEntry(snapshot, "GPS: fix");
  const auto* sats = FindEntry(snapshot, "GPS: satellites");
  const auto* rtk = FindEntry(snapshot, "GPS: RTK");
  const auto* rtcm = FindEntry(snapshot, "GPS: NTRIP/RTCM");
  const auto* rf = FindEntry(snapshot, "GPS: rf");
  const auto* jamming = FindEntry(snapshot, "GPS: jamming");

  if (const auto quality = LookupString(fix, "fix_quality"); quality.has_value())
  {
    if (const auto parsed = ParseInt(*quality); parsed.has_value())
    {
      SetFixTypeFromQuality(state, *parsed);
    }
  }
  if (const auto gps_fix_ok = LookupBool(fix, "gps_fix_ok"); gps_fix_ok.has_value())
  {
    state.fix_valid = *gps_fix_ok;
  }
  if (const auto diff_corr = LookupBool(fix, "diff_corr"); diff_corr.has_value())
  {
    state.differential_corrections = *diff_corr;
  }
  if (const auto solution_status = LookupString(fix, "solution_status"); solution_status.has_value())
  {
    state.solution_status = *solution_status;
  }
  if (const auto position_type = LookupString(fix, "position_type"); position_type.has_value())
  {
    state.position_type = *position_type;
  }
  if (const auto tracked = LookupU16(fix, "tracked_svs"); tracked.has_value())
  {
    state.satellites_tracked = *tracked;
  }
  if (const auto used = LookupU16(fix, "soln_svs"); used.has_value())
  {
    state.satellites_used = *used;
  }

  if (const auto rtk_solution = LookupString(rtk, "solution_status"); rtk_solution.has_value())
  {
    state.solution_status = *rtk_solution;
  }
  if (const auto rtk_position = LookupString(rtk, "position_type"); rtk_position.has_value())
  {
    state.position_type = *rtk_position;
  }
  if (const auto diff_age = LookupFloat(rtk, "diff_age_s"); diff_age.has_value())
  {
    state.correction_age_s = *diff_age;
  }
  else if (const auto diff_age_fix = LookupFloat(fix, "diff_age_s"); diff_age_fix.has_value())
  {
    state.correction_age_s = *diff_age_fix;
  }
  if (const auto tracked = LookupU16(rtk, "tracked_svs"); tracked.has_value())
  {
    state.satellites_tracked = *tracked;
  }
  if (const auto used = LookupU16(rtk, "soln_svs"); used.has_value())
  {
    state.satellites_used = *used;
  }
  if (const auto dual_rtk_flag = LookupString(rtk, "dual_rtk_flag"); dual_rtk_flag.has_value())
  {
    if (const auto dual_antenna = ParseUnicoreDualAntennaStatus(*dual_rtk_flag);
        dual_antenna.has_value())
    {
      state.dual_antenna_heading = *dual_antenna;
    }
  }

  if (const auto visible = LookupU16(sats, "visible"); visible.has_value())
  {
    state.satellites_visible = *visible;
  }
  if (const auto tracked = LookupU16(sats, "tracked"); tracked.has_value())
  {
    state.satellites_tracked = *tracked;
  }
  if (const auto used = LookupU16(sats, "used"); used.has_value())
  {
    state.satellites_used = *used;
  }
  if (const auto mean_cn0 = LookupFloat(sats, "cn0_mean_db_hz"); mean_cn0.has_value())
  {
    state.mean_cn0_db_hz = *mean_cn0;
  }
  if (const auto max_cn0 = LookupFloat(sats, "cn0_max_db_hz"); max_cn0.has_value())
  {
    state.max_cn0_db_hz = *max_cn0;
  }

  const auto injected_age = LookupFloat(rtcm, "age_of_last_injected_corr_s");
  const auto msgs_per_sec = LookupFloat(rtcm, "msgs_per_sec");
  // Unicore's diff_age_s is the receiver-side age of the corrections applied
  // to the current solution, so it is the authoritative source when present.
  // We only fall back to the transport-side RTCM injection age when the RTK
  // solution age is unavailable.
  if (!state.correction_age_s.has_value() && injected_age.has_value())
  {
    state.correction_age_s = *injected_age;
  }
  if (const auto corrections_active = CorrectionsActiveFromReceiverAge(state.correction_age_s);
      corrections_active.has_value())
  {
    state.corrections_active = *corrections_active;
  }
  else if (const auto transport_active = CorrectionsActiveFromTransport(injected_age, msgs_per_sec);
           transport_active.has_value())
  {
    state.corrections_active = *transport_active;
  }

  if (const auto interference = LookupBool(rf, "rf_saturation_suspected"); interference.has_value())
  {
    state.interference_detected = *interference;
  }
  if (const auto jam = LookupBool(jamming, "jamming_detected"); jam.has_value())
  {
    state.jamming_detected = *jam;
  }
}

}  // namespace

GnssBackendKind ResolveGnssBackend(const std::string& gnss_backend, const std::string& gps_protocol)
{
  const std::string backend = Lowercase(gnss_backend);
  const std::string protocol = Lowercase(gps_protocol);

  if (backend == "nmea")
  {
    return GnssBackendKind::kNmea;
  }
  if (backend == "ublox")
  {
    return GnssBackendKind::kUblox;
  }
  if (backend == "unicore")
  {
    return GnssBackendKind::kUnicore;
  }
  if (backend.empty() || backend == "gps")
  {
    if (protocol == "nmea")
    {
      return GnssBackendKind::kNmea;
    }
    if (protocol == "ubx")
    {
      return GnssBackendKind::kUblox;
    }
  }
  return GnssBackendKind::kUnknown;
}

GnssDiagnosticSnapshot BuildGnssDiagnosticSnapshot(const diagnostic_msgs::msg::DiagnosticArray& array)
{
  GnssDiagnosticSnapshot snapshot;
  snapshot.stamp = rclcpp::Time(array.header.stamp);

  for (const auto& status : array.status)
  {
    if (status.name.rfind("GPS: ", 0) != 0)
    {
      continue;
    }

    GnssDiagnosticEntry entry;
    entry.level = status.level;
    entry.message = status.message;
    for (const auto& kv : status.values)
    {
      entry.values[kv.key] = kv.value;
    }
    snapshot.entries.emplace(status.name, std::move(entry));
  }

  return snapshot;
}

GnssRuntimeState BuildGnssRuntimeStateFromFix(const sensor_msgs::msg::NavSatFix& fix,
                                              GnssBackendKind backend)
{
  using NavSat = sensor_msgs::msg::NavSatFix;
  using NavStatus = sensor_msgs::msg::NavSatStatus;

  GnssRuntimeState state;
  state.stamp = rclcpp::Time(fix.header.stamp);
  state.frame_id = fix.header.frame_id.empty() ? "gps_link" : fix.header.frame_id;
  SetBackendMetadata(state, backend);

  switch (fix.status.status)
  {
    case NavStatus::STATUS_GBAS_FIX:
      state.fix_type = GnssFixType::kRtkFixed;
      state.fix_valid = true;
      state.differential_corrections = true;
      state.rtk_mode = GnssRtkMode::kFixed;
      break;
    case NavStatus::STATUS_SBAS_FIX:
      state.fix_type = GnssFixType::kRtkFloat;
      state.fix_valid = true;
      state.differential_corrections = true;
      state.rtk_mode = GnssRtkMode::kFloat;
      break;
    case NavStatus::STATUS_FIX:
      state.fix_type = GnssFixType::kGpsFix;
      state.fix_valid = true;
      state.differential_corrections = false;
      state.rtk_mode = GnssRtkMode::kNone;
      break;
    case NavStatus::STATUS_NO_FIX:
      state.fix_type = GnssFixType::kNoFix;
      state.fix_valid = false;
      break;
    default:
      state.fix_type = GnssFixType::kNoFix;
      state.fix_valid = false;
      break;
  }

  if (fix.position_covariance_type != NavSat::COVARIANCE_TYPE_UNKNOWN)
  {
    const double lat_var = fix.position_covariance[0];
    const double lon_var = fix.position_covariance[4];
    state.horizontal_accuracy_m = static_cast<float>(std::sqrt((lat_var + lon_var) / 2.0));
    state.vertical_accuracy_m =
        static_cast<float>(std::sqrt(std::max(0.0, fix.position_covariance[8])));
    MarkCapability(state, GnssRuntimeCapability::kHorizontalAccuracy);
    MarkCapability(state, GnssRuntimeCapability::kVerticalAccuracy);
  }

  return state;
}

void EnrichGnssRuntimeStateFromDiagnostics(GnssRuntimeState& state,
                                           GnssBackendKind backend,
                                           const GnssDiagnosticSnapshot& snapshot,
                                           double diagnostics_timeout_sec)
{
  if (snapshot.stamp.nanoseconds() == 0)
  {
    return;
  }

  const double age_sec = (state.stamp - snapshot.stamp).seconds();
  // Diagnostics are sampled independently from NavSatFix. We only consume the
  // latest diagnostics snapshot when it is older than or equal to the fix and
  // still within the freshness timeout. Newer-than-fix diagnostics are
  // rejected so we never enrich a fix with information that arrived later.
  if (age_sec < 0.0 || age_sec > diagnostics_timeout_sec)
  {
    return;
  }

  switch (backend)
  {
    case GnssBackendKind::kUblox:
      ApplyUbloxDiagnostics(state, snapshot);
      break;
    case GnssBackendKind::kUnicore:
      ApplyUnicoreDiagnostics(state, snapshot);
      break;
    case GnssBackendKind::kNmea:
    case GnssBackendKind::kUnknown:
    default:
      break;
  }
}

}  // namespace mowgli_localization

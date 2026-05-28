// Copyright 2026 Mowgli Project
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

#include "mowgli_localization/gnss_status_adapter.hpp"

#include <cassert>
#include <type_traits>

namespace mowgli_localization
{

static_assert(std::is_same_v<std::underlying_type_t<GnssRuntimeCapability>, uint32_t>,
              "GnssRuntimeCapability must stay within the uint32 public message contract");

namespace
{

uint8_t ToMsgFixType(GnssFixType fix_type)
{
  using Msg = mowgli_interfaces::msg::GnssStatus;
  switch (fix_type)
  {
    case GnssFixType::kGpsFix:
      return Msg::FIX_TYPE_GPS_FIX;
    case GnssFixType::kRtkFloat:
      return Msg::FIX_TYPE_RTK_FLOAT;
    case GnssFixType::kRtkFixed:
      return Msg::FIX_TYPE_RTK_FIXED;
    case GnssFixType::kDeadReckoning:
      return Msg::FIX_TYPE_DEAD_RECKONING;
    case GnssFixType::kNoFix:
    default:
      return Msg::FIX_TYPE_NO_FIX;
  }
}

uint8_t ToMsgRtkMode(GnssRtkMode rtk_mode)
{
  using Msg = mowgli_interfaces::msg::GnssStatus;
  switch (rtk_mode)
  {
    case GnssRtkMode::kNone:
      return Msg::RTK_MODE_NONE;
    case GnssRtkMode::kFloat:
      return Msg::RTK_MODE_FLOAT;
    case GnssRtkMode::kFixed:
      return Msg::RTK_MODE_FIXED;
    case GnssRtkMode::kUnknown:
    default:
      return Msg::RTK_MODE_UNKNOWN;
  }
}

void SetCapabilityIfNeeded(mowgli_interfaces::msg::GnssStatus& msg,
                           const GnssRuntimeState& state,
                           GnssRuntimeCapability capability,
                           uint32_t bit)
{
  if (HasCapability(state, capability))
  {
    msg.capability_flags |= bit;
  }
}

void SetValueFlagIfPresent(mowgli_interfaces::msg::GnssStatus& msg, uint32_t bit)
{
  assert((msg.capability_flags & bit) != 0u &&
         "value_flags may only be set for fields already declared in capability_flags");
  if ((msg.capability_flags & bit) == 0u)
  {
    return;
  }
  msg.value_flags |= bit;
}

}  // namespace

float QualityPercentForFixType(GnssFixType fix_type)
{
  switch (fix_type)
  {
    case GnssFixType::kRtkFixed:
      return 100.0f;
    case GnssFixType::kRtkFloat:
      return 50.0f;
    case GnssFixType::kGpsFix:
      return 25.0f;
    case GnssFixType::kDeadReckoning:
      return 10.0f;
    case GnssFixType::kNoFix:
    default:
      return 0.0f;
  }
}

mowgli_interfaces::msg::GnssStatus ToGnssStatusMessage(const GnssRuntimeState& state)
{
  using Msg = mowgli_interfaces::msg::GnssStatus;

  Msg msg;
  msg.header.stamp = state.stamp;
  msg.header.frame_id = state.frame_id;
  msg.backend = state.backend;
  msg.receiver_vendor = state.receiver_vendor;
  msg.receiver_model = state.receiver_model;
  msg.receiver_firmware = state.receiver_firmware;
  msg.fix_type = ToMsgFixType(state.fix_type);
  msg.fix_valid = state.fix_valid;
  msg.dead_reckoning = state.dead_reckoning;
  msg.quality_percent = state.quality_percent.value_or(QualityPercentForFixType(state.fix_type));

  SetCapabilityIfNeeded(msg, state, GnssRuntimeCapability::kRtkMode, Msg::CAP_RTK_MODE);
  SetCapabilityIfNeeded(msg, state, GnssRuntimeCapability::kHdop, Msg::CAP_HDOP);
  SetCapabilityIfNeeded(msg, state, GnssRuntimeCapability::kVdop, Msg::CAP_VDOP);
  SetCapabilityIfNeeded(msg,
                        state,
                        GnssRuntimeCapability::kHorizontalAccuracy,
                        Msg::CAP_HORIZONTAL_ACCURACY);
  SetCapabilityIfNeeded(msg,
                        state,
                        GnssRuntimeCapability::kVerticalAccuracy,
                        Msg::CAP_VERTICAL_ACCURACY);
  SetCapabilityIfNeeded(msg, state, GnssRuntimeCapability::kHeading, Msg::CAP_HEADING);
  SetCapabilityIfNeeded(msg,
                        state,
                        GnssRuntimeCapability::kHeadingAccuracy,
                        Msg::CAP_HEADING_ACCURACY);
  SetCapabilityIfNeeded(msg,
                        state,
                        GnssRuntimeCapability::kSatellitesUsed,
                        Msg::CAP_SATELLITES_USED);
  SetCapabilityIfNeeded(msg,
                        state,
                        GnssRuntimeCapability::kSatellitesVisible,
                        Msg::CAP_SATELLITES_VISIBLE);
  SetCapabilityIfNeeded(msg,
                        state,
                        GnssRuntimeCapability::kSatellitesTracked,
                        Msg::CAP_SATELLITES_TRACKED);
  SetCapabilityIfNeeded(msg,
                        state,
                        GnssRuntimeCapability::kDifferentialCorrections,
                        Msg::CAP_DIFFERENTIAL_CORRECTIONS);
  SetCapabilityIfNeeded(msg,
                        state,
                        GnssRuntimeCapability::kCorrectionsActive,
                        Msg::CAP_CORRECTIONS_ACTIVE);
  SetCapabilityIfNeeded(msg,
                        state,
                        GnssRuntimeCapability::kCorrectionAge,
                        Msg::CAP_CORRECTION_AGE);
  SetCapabilityIfNeeded(msg, state, GnssRuntimeCapability::kMeanCn0, Msg::CAP_MEAN_CN0);
  SetCapabilityIfNeeded(msg, state, GnssRuntimeCapability::kMaxCn0, Msg::CAP_MAX_CN0);
  SetCapabilityIfNeeded(msg,
                        state,
                        GnssRuntimeCapability::kDualAntennaStatus,
                        Msg::CAP_DUAL_ANTENNA_STATUS);
  SetCapabilityIfNeeded(msg,
                        state,
                        GnssRuntimeCapability::kInterferenceStatus,
                        Msg::CAP_INTERFERENCE_STATUS);
  SetCapabilityIfNeeded(msg,
                        state,
                        GnssRuntimeCapability::kJammingStatus,
                        Msg::CAP_JAMMING_STATUS);

  if (state.differential_corrections.has_value())
  {
    msg.differential_corrections = *state.differential_corrections;
    SetValueFlagIfPresent(msg, Msg::CAP_DIFFERENTIAL_CORRECTIONS);
  }
  if (state.corrections_active.has_value())
  {
    msg.corrections_active = *state.corrections_active;
    SetValueFlagIfPresent(msg, Msg::CAP_CORRECTIONS_ACTIVE);
  }
  if (state.rtk_mode.has_value())
  {
    msg.rtk_mode = ToMsgRtkMode(*state.rtk_mode);
    SetValueFlagIfPresent(msg, Msg::CAP_RTK_MODE);
  }
  else
  {
    msg.rtk_mode = Msg::RTK_MODE_UNKNOWN;
  }
  if (state.dual_antenna_heading.has_value())
  {
    msg.dual_antenna_heading = *state.dual_antenna_heading;
    SetValueFlagIfPresent(msg, Msg::CAP_DUAL_ANTENNA_STATUS);
  }
  if (state.interference_detected.has_value())
  {
    msg.interference_detected = *state.interference_detected;
    SetValueFlagIfPresent(msg, Msg::CAP_INTERFERENCE_STATUS);
  }
  if (state.jamming_detected.has_value())
  {
    msg.jamming_detected = *state.jamming_detected;
    SetValueFlagIfPresent(msg, Msg::CAP_JAMMING_STATUS);
  }
  if (state.hdop.has_value())
  {
    msg.hdop = *state.hdop;
    SetValueFlagIfPresent(msg, Msg::CAP_HDOP);
  }
  if (state.vdop.has_value())
  {
    msg.vdop = *state.vdop;
    SetValueFlagIfPresent(msg, Msg::CAP_VDOP);
  }
  if (state.horizontal_accuracy_m.has_value())
  {
    msg.horizontal_accuracy_m = *state.horizontal_accuracy_m;
    SetValueFlagIfPresent(msg, Msg::CAP_HORIZONTAL_ACCURACY);
  }
  if (state.vertical_accuracy_m.has_value())
  {
    msg.vertical_accuracy_m = *state.vertical_accuracy_m;
    SetValueFlagIfPresent(msg, Msg::CAP_VERTICAL_ACCURACY);
  }
  if (state.heading_deg.has_value())
  {
    msg.heading_deg = *state.heading_deg;
    SetValueFlagIfPresent(msg, Msg::CAP_HEADING);
  }
  if (state.heading_accuracy_deg.has_value())
  {
    msg.heading_accuracy_deg = *state.heading_accuracy_deg;
    SetValueFlagIfPresent(msg, Msg::CAP_HEADING_ACCURACY);
  }
  if (state.satellites_used.has_value())
  {
    msg.satellites_used = *state.satellites_used;
    SetValueFlagIfPresent(msg, Msg::CAP_SATELLITES_USED);
  }
  if (state.satellites_visible.has_value())
  {
    msg.satellites_visible = *state.satellites_visible;
    SetValueFlagIfPresent(msg, Msg::CAP_SATELLITES_VISIBLE);
  }
  if (state.satellites_tracked.has_value())
  {
    msg.satellites_tracked = *state.satellites_tracked;
    SetValueFlagIfPresent(msg, Msg::CAP_SATELLITES_TRACKED);
  }
  if (state.correction_age_s.has_value())
  {
    msg.correction_age_s = *state.correction_age_s;
    SetValueFlagIfPresent(msg, Msg::CAP_CORRECTION_AGE);
  }
  if (state.mean_cn0_db_hz.has_value())
  {
    msg.mean_cn0_db_hz = *state.mean_cn0_db_hz;
    SetValueFlagIfPresent(msg, Msg::CAP_MEAN_CN0);
  }
  if (state.max_cn0_db_hz.has_value())
  {
    msg.max_cn0_db_hz = *state.max_cn0_db_hz;
    SetValueFlagIfPresent(msg, Msg::CAP_MAX_CN0);
  }

  return msg;
}

}  // namespace mowgli_localization

// Copyright 2026 Mowgli Project
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include "rclcpp/time.hpp"

namespace mowgli_localization
{

enum class GnssFixType : uint8_t
{
  kNoFix = 0,
  kGpsFix = 1,
  kRtkFloat = 2,
  kRtkFixed = 3,
  kDeadReckoning = 4,
};

enum class GnssRtkMode : uint8_t
{
  kUnknown = 0,
  kNone = 1,
  kFloat = 2,
  kFixed = 3,
};

enum class GnssRuntimeCapability : uint32_t
{
  kRtkMode = 1u << 0,
  kHdop = 1u << 1,
  kVdop = 1u << 2,
  kHorizontalAccuracy = 1u << 3,
  kVerticalAccuracy = 1u << 4,
  kHeading = 1u << 5,
  kHeadingAccuracy = 1u << 6,
  kSatellitesUsed = 1u << 7,
  kSatellitesVisible = 1u << 8,
  kSatellitesTracked = 1u << 9,
  kDifferentialCorrections = 1u << 10,
  kCorrectionsActive = 1u << 11,
  kCorrectionAge = 1u << 12,
  kMeanCn0 = 1u << 13,
  kMaxCn0 = 1u << 14,
  kDualAntennaStatus = 1u << 15,
  kInterferenceStatus = 1u << 16,
  kJammingStatus = 1u << 17,
};

struct GnssRuntimeState
{
  rclcpp::Time stamp{};
  std::string frame_id{"gps_link"};
  // Optional metadata populated by backend-specific adapters when they know
  // the concrete source. Leave empty in generic NavSatFix-only paths.
  std::string backend{};
  std::string receiver_vendor{};
  std::string receiver_model{};
  std::string receiver_firmware{};
  // Backend-native solution labels kept internal so producers can carry
  // richer status without leaking vendor-specific names into GnssStatus.
  std::string solution_status{};
  std::string position_type{};

  GnssFixType fix_type{GnssFixType::kNoFix};
  bool fix_valid{false};
  bool dead_reckoning{false};

  std::optional<bool> differential_corrections{};
  std::optional<bool> corrections_active{};
  std::optional<GnssRtkMode> rtk_mode{};
  std::optional<bool> dual_antenna_heading{};
  std::optional<bool> interference_detected{};
  std::optional<bool> jamming_detected{};

  std::optional<float> quality_percent{};
  std::optional<float> hdop{};
  std::optional<float> vdop{};
  std::optional<float> horizontal_accuracy_m{};
  std::optional<float> vertical_accuracy_m{};
  std::optional<float> heading_deg{};
  std::optional<float> heading_accuracy_deg{};
  std::optional<uint16_t> satellites_used{};
  std::optional<uint16_t> satellites_visible{};
  std::optional<uint16_t> satellites_tracked{};
  std::optional<float> correction_age_s{};
  std::optional<float> mean_cn0_db_hz{};
  std::optional<float> max_cn0_db_hz{};

  // Backend/runtime support bits. A producer sets these when a field is part
  // of the backend contract even if the current sample lacks a usable value.
  uint32_t supported_capabilities{0};
};

inline void MarkCapability(GnssRuntimeState& state, GnssRuntimeCapability capability)
{
  state.supported_capabilities |= static_cast<uint32_t>(capability);
}

inline bool HasCapability(const GnssRuntimeState& state, GnssRuntimeCapability capability)
{
  return (state.supported_capabilities & static_cast<uint32_t>(capability)) != 0;
}

}  // namespace mowgli_localization

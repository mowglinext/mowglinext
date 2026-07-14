#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>

#include "mowgli_interfaces/msg/absolute_pose.hpp"
#include "mowgli_interfaces/msg/gnss_status.hpp"

namespace mowgli_interfaces::gnss_status_utils
{

using AbsolutePose = mowgli_interfaces::msg::AbsolutePose;
using GnssStatus = mowgli_interfaces::msg::GnssStatus;

inline bool HasCapability(const GnssStatus& status, const std::uint32_t flag)
{
  return (status.capability_flags & flag) != 0u;
}

inline bool HasValue(const GnssStatus& status, const std::uint32_t flag)
{
  return (status.value_flags & flag) != 0u;
}

inline std::optional<float> HorizontalAccuracyMeters(const GnssStatus& status)
{
  if (!HasValue(status, GnssStatus::CAP_HORIZONTAL_ACCURACY) ||
      !std::isfinite(status.horizontal_accuracy_m))
  {
    return std::nullopt;
  }
  return status.horizontal_accuracy_m;
}

inline bool IsRtkFixed(const GnssStatus& status)
{
  if (!status.fix_valid)
  {
    return false;
  }
  return status.rtk_mode == GnssStatus::RTK_MODE_FIXED ||
         status.fix_type == GnssStatus::FIX_TYPE_RTK_FIXED;
}

inline bool IsRtkFloat(const GnssStatus& status)
{
  if (!status.fix_valid)
  {
    return false;
  }
  return status.rtk_mode == GnssStatus::RTK_MODE_FLOAT ||
         status.fix_type == GnssStatus::FIX_TYPE_RTK_FLOAT;
}

inline std::uint8_t AbsolutePoseFlags(const GnssStatus& status)
{
  if (!status.fix_valid)
  {
    return AbsolutePose::FLAG_GPS_DEAD_RECKONING;
  }

  if (IsRtkFixed(status))
  {
    return AbsolutePose::FLAG_GPS_RTK | AbsolutePose::FLAG_GPS_RTK_FIXED;
  }

  if (IsRtkFloat(status))
  {
    return AbsolutePose::FLAG_GPS_RTK | AbsolutePose::FLAG_GPS_RTK_FLOAT;
  }

  if (status.fix_type == GnssStatus::FIX_TYPE_DEAD_RECKONING)
  {
    return AbsolutePose::FLAG_GPS_DEAD_RECKONING;
  }

  return AbsolutePose::FLAG_GPS_RTK;
}

inline std::uint8_t BehaviorTreeFixType(const GnssStatus& status)
{
  if (!status.fix_valid)
  {
    return 0u;
  }

  if (IsRtkFixed(status))
  {
    return 4u;
  }

  if (IsRtkFloat(status))
  {
    return 3u;
  }

  return status.fix_type == GnssStatus::FIX_TYPE_GPS_FIX ? 2u : 0u;
}

inline float NormalizedQuality(const GnssStatus& status)
{
  if (!status.fix_valid)
  {
    return 0.0f;
  }

  if (const auto horizontal_accuracy_m = HorizontalAccuracyMeters(status); horizontal_accuracy_m)
  {
    if (*horizontal_accuracy_m <= 0.05f)
    {
      return 1.0f;
    }
    if (*horizontal_accuracy_m <= 0.5f)
    {
      return 0.7f;
    }
    if (*horizontal_accuracy_m <= 2.0f)
    {
      return 0.4f;
    }
    return 0.1f;
  }

  if (IsRtkFixed(status))
  {
    return 1.0f;
  }
  if (IsRtkFloat(status))
  {
    return 0.7f;
  }
  if (status.fix_type == GnssStatus::FIX_TYPE_GPS_FIX)
  {
    return 0.4f;
  }

  return 0.0f;
}

inline std::uint8_t HardwareQualityPercent(const GnssStatus& status)
{
  return static_cast<std::uint8_t>(
      std::lround(std::clamp(NormalizedQuality(status), 0.0f, 1.0f) * 100.0f));
}

inline bool BehaviorTreeRtkFixed(const GnssStatus& status)
{
  return BehaviorTreeFixType(status) >= 4u && NormalizedQuality(status) >= 0.9f;
}

}  // namespace mowgli_interfaces::gnss_status_utils

#pragma once

#include <cmath>
#include <cstdint>
#include <optional>

#include "mowgli_interfaces/gnss_status_utils.hpp"
#include "mowgli_interfaces/msg/gnss_status.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"

namespace mowgli_localization
{

inline std::uint8_t ResolveAbsolutePoseFlags(
    const std::optional<mowgli_interfaces::msg::GnssStatus>& authoritative_status,
    const std::uint8_t navsat_status)
{
  using sensor_msgs::msg::NavSatStatus;

  if (authoritative_status.has_value())
  {
    return mowgli_interfaces::gnss_status_utils::AbsolutePoseFlags(*authoritative_status);
  }

  switch (navsat_status)
  {
    case NavSatStatus::STATUS_GBAS_FIX:
      return mowgli_interfaces::msg::AbsolutePose::FLAG_GPS_RTK |
             mowgli_interfaces::msg::AbsolutePose::FLAG_GPS_RTK_FIXED;
    case NavSatStatus::STATUS_SBAS_FIX:
      return mowgli_interfaces::msg::AbsolutePose::FLAG_GPS_RTK |
             mowgli_interfaces::msg::AbsolutePose::FLAG_GPS_RTK_FLOAT;
    case NavSatStatus::STATUS_FIX:
      return mowgli_interfaces::msg::AbsolutePose::FLAG_GPS_RTK;
    default:
      return mowgli_interfaces::msg::AbsolutePose::FLAG_GPS_DEAD_RECKONING;
  }
}

inline bool HasAuthoritativeRtkPoseState(
    const std::optional<mowgli_interfaces::msg::GnssStatus>& authoritative_status,
    const std::uint8_t navsat_status)
{
  using sensor_msgs::msg::NavSatStatus;

  return authoritative_status.has_value()
             ? (mowgli_interfaces::gnss_status_utils::IsRtkFixed(*authoritative_status) ||
                mowgli_interfaces::gnss_status_utils::IsRtkFloat(*authoritative_status))
             : (navsat_status == NavSatStatus::STATUS_GBAS_FIX ||
                navsat_status == NavSatStatus::STATUS_SBAS_FIX);
}

}  // namespace mowgli_localization

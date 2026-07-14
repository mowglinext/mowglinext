// Copyright 2026 Mowgli Project
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include <optional>

#include "mowgli_localization/navsat_projection_utils.hpp"
#include <gtest/gtest.h>

namespace
{

using mowgli_interfaces::msg::AbsolutePose;
using mowgli_interfaces::msg::GnssStatus;
using sensor_msgs::msg::NavSatStatus;

TEST(NavsatProjectionUtilsTest, TypedRtkFixedStateSurvivesUnknownCovarianceFallback)
{
  GnssStatus status;
  status.fix_valid = true;
  status.fix_type = GnssStatus::FIX_TYPE_RTK_FIXED;
  status.rtk_mode = GnssStatus::RTK_MODE_FIXED;

  const auto flags = mowgli_localization::ResolveAbsolutePoseFlags(std::make_optional(status),
                                                                   NavSatStatus::STATUS_FIX);

  EXPECT_EQ(flags,
            static_cast<std::uint8_t>(AbsolutePose::FLAG_GPS_RTK |
                                      AbsolutePose::FLAG_GPS_RTK_FIXED));
  EXPECT_TRUE(mowgli_localization::HasAuthoritativeRtkPoseState(std::make_optional(status),
                                                                NavSatStatus::STATUS_FIX));
}

TEST(NavsatProjectionUtilsTest, TypedRtkFloatStateCanDrivePoseCovEligibilityWithoutSbasFallback)
{
  GnssStatus status;
  status.fix_valid = true;
  status.fix_type = GnssStatus::FIX_TYPE_RTK_FLOAT;
  status.rtk_mode = GnssStatus::RTK_MODE_FLOAT;

  EXPECT_TRUE(mowgli_localization::HasAuthoritativeRtkPoseState(std::make_optional(status),
                                                                NavSatStatus::STATUS_FIX));
}

TEST(NavsatProjectionUtilsTest, StandaloneNavsatFixWithoutTypedStatusDoesNotMasqueradeAsRtk)
{
  EXPECT_FALSE(
      mowgli_localization::HasAuthoritativeRtkPoseState(std::nullopt, NavSatStatus::STATUS_FIX));
}

}  // namespace

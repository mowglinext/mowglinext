// Copyright 2026 Mowgli Project

#include <gtest/gtest.h>

#include "mowgli_localization/gnss_status_adapter.hpp"

namespace mowgli_localization
{

TEST(GnssStatusAdapterTest, CapabilityBitsMatchPublicMessageContract)
{
  using Msg = mowgli_interfaces::msg::GnssStatus;

  EXPECT_EQ(static_cast<uint32_t>(GnssRuntimeCapability::kRtkMode), Msg::CAP_RTK_MODE);
  EXPECT_EQ(static_cast<uint32_t>(GnssRuntimeCapability::kHdop), Msg::CAP_HDOP);
  EXPECT_EQ(static_cast<uint32_t>(GnssRuntimeCapability::kVdop), Msg::CAP_VDOP);
  EXPECT_EQ(static_cast<uint32_t>(GnssRuntimeCapability::kHorizontalAccuracy),
            Msg::CAP_HORIZONTAL_ACCURACY);
  EXPECT_EQ(static_cast<uint32_t>(GnssRuntimeCapability::kVerticalAccuracy),
            Msg::CAP_VERTICAL_ACCURACY);
  EXPECT_EQ(static_cast<uint32_t>(GnssRuntimeCapability::kHeading), Msg::CAP_HEADING);
  EXPECT_EQ(static_cast<uint32_t>(GnssRuntimeCapability::kHeadingAccuracy),
            Msg::CAP_HEADING_ACCURACY);
  EXPECT_EQ(static_cast<uint32_t>(GnssRuntimeCapability::kSatellitesUsed), Msg::CAP_SATELLITES_USED);
  EXPECT_EQ(static_cast<uint32_t>(GnssRuntimeCapability::kSatellitesVisible),
            Msg::CAP_SATELLITES_VISIBLE);
  EXPECT_EQ(static_cast<uint32_t>(GnssRuntimeCapability::kSatellitesTracked),
            Msg::CAP_SATELLITES_TRACKED);
  EXPECT_EQ(static_cast<uint32_t>(GnssRuntimeCapability::kDifferentialCorrections),
            Msg::CAP_DIFFERENTIAL_CORRECTIONS);
  EXPECT_EQ(static_cast<uint32_t>(GnssRuntimeCapability::kCorrectionsActive),
            Msg::CAP_CORRECTIONS_ACTIVE);
  EXPECT_EQ(static_cast<uint32_t>(GnssRuntimeCapability::kCorrectionAge), Msg::CAP_CORRECTION_AGE);
  EXPECT_EQ(static_cast<uint32_t>(GnssRuntimeCapability::kMeanCn0), Msg::CAP_MEAN_CN0);
  EXPECT_EQ(static_cast<uint32_t>(GnssRuntimeCapability::kMaxCn0), Msg::CAP_MAX_CN0);
  EXPECT_EQ(static_cast<uint32_t>(GnssRuntimeCapability::kDualAntennaStatus),
            Msg::CAP_DUAL_ANTENNA_STATUS);
  EXPECT_EQ(static_cast<uint32_t>(GnssRuntimeCapability::kInterferenceStatus),
            Msg::CAP_INTERFERENCE_STATUS);
  EXPECT_EQ(static_cast<uint32_t>(GnssRuntimeCapability::kJammingStatus),
            Msg::CAP_JAMMING_STATUS);
}

TEST(GnssStatusAdapterTest, MapsOptionalFieldsToPublicCapabilities)
{
  GnssRuntimeState state;
  state.stamp = rclcpp::Time(123456789);
  state.frame_id = "map";
  state.backend = "ublox";
  state.receiver_vendor = "u-blox";
  state.receiver_model = "ZED-F9P";
  state.fix_type = GnssFixType::kRtkFixed;
  state.fix_valid = true;
  state.differential_corrections = true;
  state.corrections_active = true;
  state.rtk_mode = GnssRtkMode::kFixed;
  state.hdop = 0.8f;
  state.horizontal_accuracy_m = 0.02f;
  state.satellites_used = 18;
  state.correction_age_s = 0.4f;
  MarkCapability(state, GnssRuntimeCapability::kRtkMode);
  MarkCapability(state, GnssRuntimeCapability::kDifferentialCorrections);
  MarkCapability(state, GnssRuntimeCapability::kCorrectionsActive);
  MarkCapability(state, GnssRuntimeCapability::kHdop);
  MarkCapability(state, GnssRuntimeCapability::kHorizontalAccuracy);
  MarkCapability(state, GnssRuntimeCapability::kSatellitesUsed);
  MarkCapability(state, GnssRuntimeCapability::kCorrectionAge);

  const auto msg = ToGnssStatusMessage(state);
  EXPECT_EQ(msg.backend, "ublox");
  EXPECT_EQ(msg.receiver_vendor, "u-blox");
  EXPECT_EQ(msg.receiver_model, "ZED-F9P");
  EXPECT_EQ(msg.fix_type, mowgli_interfaces::msg::GnssStatus::FIX_TYPE_RTK_FIXED);
  EXPECT_TRUE(msg.fix_valid);
  EXPECT_EQ(msg.rtk_mode, mowgli_interfaces::msg::GnssStatus::RTK_MODE_FIXED);
  EXPECT_TRUE(msg.differential_corrections);
  EXPECT_TRUE(msg.corrections_active);
  EXPECT_FLOAT_EQ(msg.hdop, 0.8f);
  EXPECT_FLOAT_EQ(msg.horizontal_accuracy_m, 0.02f);
  EXPECT_EQ(msg.satellites_used, 18);
  EXPECT_FLOAT_EQ(msg.correction_age_s, 0.4f);
  EXPECT_NE(msg.capability_flags & mowgli_interfaces::msg::GnssStatus::CAP_RTK_MODE, 0u);
  EXPECT_NE(msg.capability_flags &
                mowgli_interfaces::msg::GnssStatus::CAP_DIFFERENTIAL_CORRECTIONS,
            0u);
  EXPECT_NE(msg.capability_flags & mowgli_interfaces::msg::GnssStatus::CAP_CORRECTIONS_ACTIVE, 0u);
  EXPECT_NE(msg.capability_flags & mowgli_interfaces::msg::GnssStatus::CAP_HDOP, 0u);
  EXPECT_NE(msg.capability_flags & mowgli_interfaces::msg::GnssStatus::CAP_HORIZONTAL_ACCURACY, 0u);
  EXPECT_NE(msg.capability_flags & mowgli_interfaces::msg::GnssStatus::CAP_SATELLITES_USED, 0u);
  EXPECT_NE(msg.capability_flags & mowgli_interfaces::msg::GnssStatus::CAP_CORRECTION_AGE, 0u);
  EXPECT_NE(msg.value_flags & mowgli_interfaces::msg::GnssStatus::CAP_RTK_MODE, 0u);
  EXPECT_NE(msg.value_flags & mowgli_interfaces::msg::GnssStatus::CAP_DIFFERENTIAL_CORRECTIONS, 0u);
  EXPECT_NE(msg.value_flags & mowgli_interfaces::msg::GnssStatus::CAP_CORRECTIONS_ACTIVE, 0u);
  EXPECT_NE(msg.value_flags & mowgli_interfaces::msg::GnssStatus::CAP_HDOP, 0u);
}

TEST(GnssStatusAdapterTest, FallsBackToDefaultQualityForFixType)
{
  GnssRuntimeState state;
  state.fix_type = GnssFixType::kGpsFix;
  state.fix_valid = true;

  const auto msg = ToGnssStatusMessage(state);
  EXPECT_FLOAT_EQ(msg.quality_percent, 25.0f);
}

TEST(GnssStatusAdapterTest, RejectsValuesWithoutDeclaredCapabilities)
{
  GnssRuntimeState state;
  state.hdop = 0.6f;

#if !defined(NDEBUG)
  EXPECT_DEATH(
      {
        const auto msg = ToGnssStatusMessage(state);
        static_cast<void>(msg);
      },
      "value_flags may only be set for fields already declared in capability_flags");
#else
  GTEST_SKIP() << "Assertions are disabled in this build, so the invariant cannot be validated.";
#endif
}

TEST(GnssStatusAdapterTest, MapsMinimalNmeaLikeStateWithoutInventingRichFields)
{
  GnssRuntimeState state;
  state.backend = "nmea";
  state.fix_type = GnssFixType::kGpsFix;
  state.fix_valid = true;
  state.hdop = 1.4f;
  state.satellites_used = 9;
  MarkCapability(state, GnssRuntimeCapability::kHdop);
  MarkCapability(state, GnssRuntimeCapability::kSatellitesUsed);
  MarkCapability(state, GnssRuntimeCapability::kRtkMode);

  const auto msg = ToGnssStatusMessage(state);
  EXPECT_EQ(msg.backend, "nmea");
  EXPECT_TRUE(msg.fix_valid);
  EXPECT_EQ(msg.fix_type, mowgli_interfaces::msg::GnssStatus::FIX_TYPE_GPS_FIX);
  EXPECT_EQ(msg.rtk_mode, mowgli_interfaces::msg::GnssStatus::RTK_MODE_UNKNOWN);
  EXPECT_FLOAT_EQ(msg.hdop, 1.4f);
  EXPECT_EQ(msg.satellites_used, 9);
  EXPECT_NE(msg.capability_flags & mowgli_interfaces::msg::GnssStatus::CAP_HDOP, 0u);
  EXPECT_NE(msg.capability_flags & mowgli_interfaces::msg::GnssStatus::CAP_SATELLITES_USED, 0u);
  EXPECT_NE(msg.capability_flags & mowgli_interfaces::msg::GnssStatus::CAP_RTK_MODE, 0u);
  EXPECT_EQ(msg.capability_flags & mowgli_interfaces::msg::GnssStatus::CAP_HORIZONTAL_ACCURACY, 0u);
  EXPECT_EQ(msg.capability_flags & mowgli_interfaces::msg::GnssStatus::CAP_CORRECTION_AGE, 0u);
  EXPECT_EQ(msg.value_flags & mowgli_interfaces::msg::GnssStatus::CAP_RTK_MODE, 0u);
  EXPECT_NE(msg.value_flags & mowgli_interfaces::msg::GnssStatus::CAP_HDOP, 0u);
}

TEST(GnssStatusAdapterTest, MapsRicherHeadingAndVisibilityFieldsWhenAvailable)
{
  GnssRuntimeState state;
  state.backend = "unicore";
  state.receiver_vendor = "Unicore";
  state.fix_type = GnssFixType::kRtkFixed;
  state.fix_valid = true;
  state.rtk_mode = GnssRtkMode::kFixed;
  state.dual_antenna_heading = true;
  state.interference_detected = false;
  state.jamming_detected = true;
  state.quality_percent = 92.0f;
  state.vdop = 0.7f;
  state.satellites_visible = 24;
  state.satellites_tracked = 19;
  state.mean_cn0_db_hz = 41.5f;
  state.max_cn0_db_hz = 51.0f;
  MarkCapability(state, GnssRuntimeCapability::kRtkMode);
  MarkCapability(state, GnssRuntimeCapability::kVdop);
  MarkCapability(state, GnssRuntimeCapability::kSatellitesVisible);
  MarkCapability(state, GnssRuntimeCapability::kSatellitesTracked);
  MarkCapability(state, GnssRuntimeCapability::kMeanCn0);
  MarkCapability(state, GnssRuntimeCapability::kMaxCn0);
  MarkCapability(state, GnssRuntimeCapability::kDualAntennaStatus);
  MarkCapability(state, GnssRuntimeCapability::kInterferenceStatus);
  MarkCapability(state, GnssRuntimeCapability::kJammingStatus);

  const auto msg = ToGnssStatusMessage(state);
  EXPECT_EQ(msg.backend, "unicore");
  EXPECT_EQ(msg.receiver_vendor, "Unicore");
  EXPECT_FLOAT_EQ(msg.quality_percent, 92.0f);
  EXPECT_FLOAT_EQ(msg.vdop, 0.7f);
  EXPECT_TRUE(msg.dual_antenna_heading);
  EXPECT_FALSE(msg.interference_detected);
  EXPECT_TRUE(msg.jamming_detected);
  EXPECT_EQ(msg.satellites_visible, 24);
  EXPECT_EQ(msg.satellites_tracked, 19);
  EXPECT_FLOAT_EQ(msg.mean_cn0_db_hz, 41.5f);
  EXPECT_FLOAT_EQ(msg.max_cn0_db_hz, 51.0f);
  EXPECT_NE(msg.capability_flags & mowgli_interfaces::msg::GnssStatus::CAP_RTK_MODE, 0u);
  EXPECT_NE(msg.capability_flags & mowgli_interfaces::msg::GnssStatus::CAP_VDOP, 0u);
  EXPECT_NE(msg.capability_flags & mowgli_interfaces::msg::GnssStatus::CAP_SATELLITES_VISIBLE, 0u);
  EXPECT_NE(msg.capability_flags & mowgli_interfaces::msg::GnssStatus::CAP_SATELLITES_TRACKED, 0u);
  EXPECT_NE(msg.capability_flags & mowgli_interfaces::msg::GnssStatus::CAP_MEAN_CN0, 0u);
  EXPECT_NE(msg.capability_flags & mowgli_interfaces::msg::GnssStatus::CAP_MAX_CN0, 0u);
  EXPECT_NE(msg.capability_flags &
                mowgli_interfaces::msg::GnssStatus::CAP_DUAL_ANTENNA_STATUS,
            0u);
  EXPECT_NE(msg.capability_flags &
                mowgli_interfaces::msg::GnssStatus::CAP_INTERFERENCE_STATUS,
            0u);
  EXPECT_NE(msg.capability_flags & mowgli_interfaces::msg::GnssStatus::CAP_JAMMING_STATUS, 0u);
  EXPECT_NE(msg.value_flags & mowgli_interfaces::msg::GnssStatus::CAP_DUAL_ANTENNA_STATUS, 0u);
  EXPECT_NE(msg.value_flags & mowgli_interfaces::msg::GnssStatus::CAP_INTERFERENCE_STATUS, 0u);
  EXPECT_NE(msg.value_flags & mowgli_interfaces::msg::GnssStatus::CAP_JAMMING_STATUS, 0u);
}

}  // namespace mowgli_localization

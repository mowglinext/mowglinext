// Copyright 2026 Mowgli Project

#include <gtest/gtest.h>

#include "mowgli_localization/gnss_runtime_state_builder.hpp"

namespace mowgli_localization
{

namespace
{

constexpr rcl_clock_type_t kTestClockType = RCL_ROS_TIME;

builtin_interfaces::msg::Time MakeRosStamp(int32_t sec, uint32_t nanosec = 0u)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = sec;
  stamp.nanosec = nanosec;
  return stamp;
}

rclcpp::Time MakeRosTime(int32_t sec, uint32_t nanosec = 0u)
{
  return rclcpp::Time(sec, nanosec, kTestClockType);
}

diagnostic_msgs::msg::DiagnosticStatus MakeDiag(const std::string& name,
                                                std::initializer_list<std::pair<const char*, const char*>> kvs)
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = name;
  for (const auto& [key, value] : kvs)
  {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = key;
    kv.value = value;
    status.values.push_back(kv);
  }
  return status;
}

diagnostic_msgs::msg::DiagnosticArray MakeArray(
    std::initializer_list<diagnostic_msgs::msg::DiagnosticStatus> statuses)
{
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = MakeRosStamp(12);
  array.status.assign(statuses.begin(), statuses.end());
  return array;
}

}  // namespace

TEST(GnssRuntimeStateBuilderTest, ResolveGnssBackendHandlesExplicitLegacyAndUnknownValues)
{
  EXPECT_EQ(ResolveGnssBackend("unicore", "UBX"), GnssBackendKind::kUnicore);
  EXPECT_EQ(ResolveGnssBackend("ublox", "NMEA"), GnssBackendKind::kUblox);
  EXPECT_EQ(ResolveGnssBackend("nmea", "UBX"), GnssBackendKind::kNmea);
  EXPECT_EQ(ResolveGnssBackend("gps", "UBX"), GnssBackendKind::kUblox);
  EXPECT_EQ(ResolveGnssBackend("gps", "NMEA"), GnssBackendKind::kNmea);
  EXPECT_EQ(ResolveGnssBackend("", "UBX"), GnssBackendKind::kUblox);
  EXPECT_EQ(ResolveGnssBackend("", "nmea"), GnssBackendKind::kNmea);
  EXPECT_EQ(ResolveGnssBackend("  GPS  ", "  ubx  "), GnssBackendKind::kUblox);
  EXPECT_EQ(ResolveGnssBackend("", ""), GnssBackendKind::kUnknown);
  EXPECT_EQ(ResolveGnssBackend("mystery", "UBX"), GnssBackendKind::kUnknown);
  EXPECT_EQ(ResolveGnssBackend("gps", "binary"), GnssBackendKind::kUnknown);
}

TEST(GnssRuntimeStateBuilderTest, BuildsMinimalNmeaStateFromNavSatFixWithoutInventingFields)
{
  sensor_msgs::msg::NavSatFix fix;
  fix.header.stamp = MakeRosStamp(12);
  fix.header.frame_id = "gps_link";
  fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
  fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
  fix.position_covariance[0] = 0.04;
  fix.position_covariance[4] = 0.04;
  fix.position_covariance[8] = 0.25;

  const auto state = BuildGnssRuntimeStateFromFix(fix, GnssBackendKind::kNmea);
  EXPECT_EQ(state.backend, "nmea");
  EXPECT_EQ(state.receiver_vendor, "generic");
  EXPECT_EQ(state.fix_type, GnssFixType::kRtkFloat);
  ASSERT_TRUE(state.rtk_mode.has_value());
  EXPECT_EQ(*state.rtk_mode, GnssRtkMode::kFloat);
  ASSERT_TRUE(state.horizontal_accuracy_m.has_value());
  EXPECT_FLOAT_EQ(*state.horizontal_accuracy_m, 0.2f);
  ASSERT_TRUE(state.vertical_accuracy_m.has_value());
  EXPECT_FLOAT_EQ(*state.vertical_accuracy_m, 0.5f);
  EXPECT_FALSE(state.satellites_used.has_value());
  EXPECT_FALSE(state.correction_age_s.has_value());
  EXPECT_TRUE(HasCapability(state, GnssRuntimeCapability::kRtkMode));
  EXPECT_FALSE(HasCapability(state, GnssRuntimeCapability::kCorrectionAge));
}

TEST(GnssRuntimeStateBuilderTest, EnrichesUbloxStateFromStructuredDiagnostics)
{
  sensor_msgs::msg::NavSatFix fix;
  fix.header.stamp = MakeRosStamp(12);
  fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;

  auto state = BuildGnssRuntimeStateFromFix(fix, GnssBackendKind::kUblox);
  const auto snapshot = BuildGnssDiagnosticSnapshot(MakeArray({
      MakeDiag("GPS: fix",
               {{"gps_fix_ok", "True"},
                {"carr_soln", "fixed"},
                {"diff_corr", "True"},
                {"hdop", "0.7"},
                {"vdop", "1.1"}}),
      MakeDiag("GPS: satellites",
               {{"visible", "24"},
                {"used", "18"},
                {"mean_cno_db_hz", "42.5"},
                {"max_cno_db_hz", "49.0"}}),
      MakeDiag("GPS: NTRIP/RTCM",
               {{"msgs_per_sec", "2.0"},
                {"age_of_last_corr_s", "0.4"}}),
  }));

  EnrichGnssRuntimeStateFromDiagnostics(state, GnssBackendKind::kUblox, snapshot, 5.0);
  EXPECT_EQ(state.fix_type, GnssFixType::kRtkFixed);
  ASSERT_TRUE(state.rtk_mode.has_value());
  EXPECT_EQ(*state.rtk_mode, GnssRtkMode::kFixed);
  ASSERT_TRUE(state.differential_corrections.has_value());
  EXPECT_TRUE(*state.differential_corrections);
  ASSERT_TRUE(state.corrections_active.has_value());
  EXPECT_TRUE(*state.corrections_active);
  ASSERT_TRUE(state.hdop.has_value());
  EXPECT_FLOAT_EQ(*state.hdop, 0.7f);
  ASSERT_TRUE(state.vdop.has_value());
  EXPECT_FLOAT_EQ(*state.vdop, 1.1f);
  ASSERT_TRUE(state.satellites_used.has_value());
  EXPECT_EQ(*state.satellites_used, 18);
  ASSERT_TRUE(state.satellites_visible.has_value());
  EXPECT_EQ(*state.satellites_visible, 24);
  ASSERT_TRUE(state.mean_cn0_db_hz.has_value());
  EXPECT_FLOAT_EQ(*state.mean_cn0_db_hz, 42.5f);
  ASSERT_TRUE(state.max_cn0_db_hz.has_value());
  EXPECT_FLOAT_EQ(*state.max_cn0_db_hz, 49.0f);
  ASSERT_TRUE(state.correction_age_s.has_value());
  EXPECT_FLOAT_EQ(*state.correction_age_s, 0.4f);
}

TEST(GnssRuntimeStateBuilderTest,
     UbloxFixedCarrierSolutionKeepsCorrectionsActiveDespiteStaleTransport)
{
  // Real-robot regression: while the F9P is solidly RTK-Fixed at ~4 mm, the
  // transport-side RTCM metric (msgs_per_sec / age_of_last_corr_s) is bursty
  // per-epoch and frequently reads "no recent byte" even though corrections are
  // clearly being applied (you cannot be Fixed without them). corrections_active
  // must follow the carrier solution, not the bursty transport flag, or the GUI
  // and diagnostics flap Fixed<->no-corrections every second.
  sensor_msgs::msg::NavSatFix fix;
  fix.header.stamp = MakeRosStamp(12);
  fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;

  auto state = BuildGnssRuntimeStateFromFix(fix, GnssBackendKind::kUblox);
  const auto snapshot = BuildGnssDiagnosticSnapshot(MakeArray({
      MakeDiag("GPS: fix", {{"gps_fix_ok", "True"}, {"carr_soln", "fixed"}}),
      MakeDiag("GPS: NTRIP/RTCM", {{"msgs_per_sec", "0.0"}, {"age_of_last_corr_s", "8.0"}}),
  }));

  EnrichGnssRuntimeStateFromDiagnostics(state, GnssBackendKind::kUblox, snapshot, 5.0);
  ASSERT_TRUE(state.rtk_mode.has_value());
  EXPECT_EQ(*state.rtk_mode, GnssRtkMode::kFixed);
  ASSERT_TRUE(state.corrections_active.has_value());
  EXPECT_TRUE(*state.corrections_active);
}

TEST(GnssRuntimeStateBuilderTest, UbloxNonRtkStillUsesTransportFreshnessForCorrectionsActive)
{
  // When the carrier solution is not RTK (carr_soln "none"), corrections_active
  // must fall back to the transport metric — a stale transport then correctly
  // reports corrections inactive.
  sensor_msgs::msg::NavSatFix fix;
  fix.header.stamp = MakeRosStamp(12);
  fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;

  auto state = BuildGnssRuntimeStateFromFix(fix, GnssBackendKind::kUblox);
  const auto snapshot = BuildGnssDiagnosticSnapshot(MakeArray({
      MakeDiag("GPS: fix", {{"gps_fix_ok", "True"}, {"carr_soln", "none"}}),
      MakeDiag("GPS: NTRIP/RTCM", {{"msgs_per_sec", "0.0"}, {"age_of_last_corr_s", "8.0"}}),
  }));

  EnrichGnssRuntimeStateFromDiagnostics(state, GnssBackendKind::kUblox, snapshot, 5.0);
  ASSERT_TRUE(state.corrections_active.has_value());
  EXPECT_FALSE(*state.corrections_active);
}

TEST(GnssRuntimeStateBuilderTest, RejectsDiagnosticsThatAreNewerThanFixOrTooStale)
{
  sensor_msgs::msg::NavSatFix fix;
  fix.header.stamp = MakeRosStamp(12);
  fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;

  auto state_future = BuildGnssRuntimeStateFromFix(fix, GnssBackendKind::kUblox);
  auto future_snapshot = BuildGnssDiagnosticSnapshot(MakeArray({
      MakeDiag("GPS: fix", {{"gps_fix_ok", "True"}, {"carr_soln", "fixed"}}),
  }));
  future_snapshot.stamp = MakeRosTime(13);
  EnrichGnssRuntimeStateFromDiagnostics(state_future, GnssBackendKind::kUblox, future_snapshot, 5.0);
  EXPECT_EQ(state_future.fix_type, GnssFixType::kGpsFix);

  auto state_stale = BuildGnssRuntimeStateFromFix(fix, GnssBackendKind::kUblox);
  auto stale_snapshot = BuildGnssDiagnosticSnapshot(MakeArray({
      MakeDiag("GPS: fix", {{"gps_fix_ok", "True"}, {"carr_soln", "fixed"}}),
  }));
  stale_snapshot.stamp = MakeRosTime(6);
  EnrichGnssRuntimeStateFromDiagnostics(state_stale, GnssBackendKind::kUblox, stale_snapshot, 5.0);
  EXPECT_EQ(state_stale.fix_type, GnssFixType::kGpsFix);
}

TEST(GnssRuntimeStateBuilderTest, EnrichesUnicoreStateFromStructuredDiagnostics)
{
  sensor_msgs::msg::NavSatFix fix;
  fix.header.stamp = MakeRosStamp(12);
  fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;

  auto state = BuildGnssRuntimeStateFromFix(fix, GnssBackendKind::kUnicore);
  const auto snapshot = BuildGnssDiagnosticSnapshot(MakeArray({
      MakeDiag("GPS: fix",
               {{"fix_quality", "4"},
                {"gps_fix_ok", "True"},
                {"diff_corr", "True"},
                {"solution_status", "SOL_COMPUTED"},
                {"position_type", "NARROW_INT"},
                {"tracked_svs", "28"},
                {"soln_svs", "19"},
                {"diff_age_s", "0.3"}}),
      MakeDiag("GPS: satellites",
               {{"visible", "31"},
                {"tracked", "28"},
                {"used", "19"},
                {"cn0_mean_db_hz", "39.8"},
                {"cn0_max_db_hz", "51.2"}}),
      MakeDiag("GPS: RTK", {{"dual_rtk_flag", "within limit"}}),
      MakeDiag("GPS: NTRIP/RTCM",
               {{"msgs_per_sec", "1.5"},
                {"age_of_last_injected_corr_s", "0.2"}}),
      MakeDiag("GPS: rf", {{"rf_saturation_suspected", "False"}}),
      MakeDiag("GPS: jamming", {{"jamming_detected", "True"}}),
  }));

  EnrichGnssRuntimeStateFromDiagnostics(state, GnssBackendKind::kUnicore, snapshot, 5.0);
  EXPECT_EQ(state.fix_type, GnssFixType::kRtkFixed);
  ASSERT_TRUE(state.rtk_mode.has_value());
  EXPECT_EQ(*state.rtk_mode, GnssRtkMode::kFixed);
  ASSERT_TRUE(state.differential_corrections.has_value());
  EXPECT_TRUE(*state.differential_corrections);
  ASSERT_TRUE(state.corrections_active.has_value());
  EXPECT_TRUE(*state.corrections_active);
  EXPECT_EQ(state.solution_status, "SOL_COMPUTED");
  EXPECT_EQ(state.position_type, "NARROW_INT");
  ASSERT_TRUE(state.satellites_visible.has_value());
  EXPECT_EQ(*state.satellites_visible, 31);
  ASSERT_TRUE(state.satellites_tracked.has_value());
  EXPECT_EQ(*state.satellites_tracked, 28);
  ASSERT_TRUE(state.satellites_used.has_value());
  EXPECT_EQ(*state.satellites_used, 19);
  ASSERT_TRUE(state.mean_cn0_db_hz.has_value());
  EXPECT_FLOAT_EQ(*state.mean_cn0_db_hz, 39.8f);
  ASSERT_TRUE(state.max_cn0_db_hz.has_value());
  EXPECT_FLOAT_EQ(*state.max_cn0_db_hz, 51.2f);
  ASSERT_TRUE(state.dual_antenna_heading.has_value());
  EXPECT_TRUE(*state.dual_antenna_heading);
  ASSERT_TRUE(state.interference_detected.has_value());
  EXPECT_FALSE(*state.interference_detected);
  ASSERT_TRUE(state.jamming_detected.has_value());
  EXPECT_TRUE(*state.jamming_detected);
  ASSERT_TRUE(state.correction_age_s.has_value());
  EXPECT_FLOAT_EQ(*state.correction_age_s, 0.3f);
}

TEST(GnssRuntimeStateBuilderTest, ParsesUnicoreDualAntennaDiagnosticStringsCaseInsensitively)
{
  const struct
  {
    const char* raw_value;
    std::optional<bool> expected;
  } cases[] = {
      {" within limit ", true},
      {"BASELINE UNSOLVED", false},
      {"Out Of Limit", false},
      {"baseline length not configured", false},
      {"UNKNOWN", false},
      {"unexpected status", std::nullopt},
  };

  for (const auto& c : cases)
  {
    sensor_msgs::msg::NavSatFix fix;
    fix.header.stamp = MakeRosStamp(12);
    fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    auto state = BuildGnssRuntimeStateFromFix(fix, GnssBackendKind::kUnicore);
    auto snapshot = BuildGnssDiagnosticSnapshot(MakeArray({
        MakeDiag("GPS: RTK", {{"dual_rtk_flag", c.raw_value}}),
    }));

    EnrichGnssRuntimeStateFromDiagnostics(state, GnssBackendKind::kUnicore, snapshot, 5.0);
    EXPECT_EQ(state.dual_antenna_heading, c.expected) << "raw value: " << c.raw_value;
  }
}

TEST(GnssRuntimeStateBuilderTest, UnicoreUsesReceiverCorrectionAgeBeforeTransportAge)
{
  sensor_msgs::msg::NavSatFix fix;
  fix.header.stamp = MakeRosStamp(12);
  fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;

  auto state = BuildGnssRuntimeStateFromFix(fix, GnssBackendKind::kUnicore);
  const auto snapshot = BuildGnssDiagnosticSnapshot(MakeArray({
      MakeDiag("GPS: fix",
               {{"fix_quality", "5"},
                {"gps_fix_ok", "True"},
                {"diff_corr", "True"},
                {"diff_age_s", "0.4"}}),
      MakeDiag("GPS: NTRIP/RTCM",
               {{"msgs_per_sec", "0.0"},
                {"age_of_last_injected_corr_s", "8.0"}}),
  }));

  EnrichGnssRuntimeStateFromDiagnostics(state, GnssBackendKind::kUnicore, snapshot, 5.0);
  ASSERT_TRUE(state.correction_age_s.has_value());
  EXPECT_FLOAT_EQ(*state.correction_age_s, 0.4f);
  ASSERT_TRUE(state.corrections_active.has_value());
  EXPECT_TRUE(*state.corrections_active);
}

}  // namespace mowgli_localization

// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0
//
// Unit tests for FirmwareParamTracker: the bridge's record of what it asked
// the STM32 for versus what the firmware reports it applied (protocol v7).

#include <cmath>

#include "mowgli_hardware/firmware_params.hpp"
#include <gtest/gtest.h>

using mowgli_hardware::FirmwareParamReport;
using mowgli_hardware::FirmwareParamTracker;
using mowgli_hardware::LlParamStoreStatus;
using mowgli_hardware::LlParamValue;

namespace
{

LlParamValue report(uint16_t id,
                    float value,
                    uint8_t status = mowgli_hardware::FW_PARAM_STATUS_OK,
                    uint8_t flags = 0u)
{
  LlParamValue pkt{};
  pkt.type = mowgli_hardware::PACKET_ID_LL_PARAM_VALUE;
  pkt.param_id = id;
  pkt.status = status;
  pkt.flags = flags;
  pkt.value = value;
  pkt.default_value = 500.0f;
  pkt.min_value = 10.0f;
  pkt.max_value = 1000.0f;
  return pkt;
}

constexpr uint16_t kTilt = mowgli_hardware::FW_PARAM_ID_TILT_MS;

}  // namespace

TEST(FirmwareParamTracker, MatchingReportIsAnUpdateThenUnchanged)
{
  FirmwareParamTracker tracker;
  tracker.set_requested(kTilt, 300.0f);
  EXPECT_EQ(tracker.on_value(report(kTilt, 300.0f)), FirmwareParamReport::kUpdated);
  EXPECT_EQ(tracker.on_value(report(kTilt, 300.0f)), FirmwareParamReport::kUnchanged);
  const auto& state = tracker.states().at(kTilt);
  EXPECT_TRUE(state.reported);
  EXPECT_FLOAT_EQ(state.applied, 300.0f);
  EXPECT_FLOAT_EQ(state.max_value, 1000.0f);
}

TEST(FirmwareParamTracker, ClampedReportIsFlagged)
{
  FirmwareParamTracker tracker;
  tracker.set_requested(kTilt, 60000.0f);
  EXPECT_EQ(tracker.on_value(report(kTilt, 1000.0f, mowgli_hardware::FW_PARAM_STATUS_CLAMPED)),
            FirmwareParamReport::kDiffersFromRequest);
  EXPECT_TRUE(FirmwareParamTracker::differs_from_request(tracker.states().at(kTilt)));
}

TEST(FirmwareParamTracker, PersistedFlagChangeIsAnUpdate)
{
  FirmwareParamTracker tracker;
  tracker.set_requested(kTilt, 300.0f);
  tracker.on_value(report(kTilt, 300.0f));
  EXPECT_FALSE(tracker.states().at(kTilt).persisted);
  EXPECT_EQ(tracker.on_value(report(kTilt,
                                    300.0f,
                                    mowgli_hardware::FW_PARAM_STATUS_OK,
                                    mowgli_hardware::PARAM_VALUE_FLAG_PERSISTED)),
            FirmwareParamReport::kUpdated);
  EXPECT_TRUE(tracker.states().at(kTilt).persisted);
}

TEST(FirmwareParamTracker, UnknownIdIsReportedWithoutCreatingState)
{
  FirmwareParamTracker tracker;
  EXPECT_EQ(tracker.on_value(report(999, NAN, mowgli_hardware::FW_PARAM_STATUS_UNKNOWN_ID)),
            FirmwareParamReport::kUnknownId);
  EXPECT_EQ(tracker.states().count(999), 0u);
}

TEST(FirmwareParamTracker, UnreportedListsWhatTheFirmwareHasNotAnswered)
{
  FirmwareParamTracker tracker;
  tracker.set_requested(kTilt, 300.0f);
  tracker.set_requested(mowgli_hardware::FW_PARAM_ID_MAX_MPS, 0.4f);
  tracker.on_value(report(kTilt, 300.0f));
  const auto missing = tracker.unreported();
  ASSERT_EQ(missing.size(), 1u);
  EXPECT_EQ(missing[0], mowgli_hardware::FW_PARAM_ID_MAX_MPS);
}

TEST(FirmwareParamTracker, ResetForgetsReportsButKeepsRequests)
{
  FirmwareParamTracker tracker;
  tracker.set_requested(kTilt, 300.0f);
  tracker.on_value(report(kTilt, 300.0f));
  LlParamStoreStatus store{};
  store.boot_source = mowgli_hardware::PARAM_BOOT_FLASH;
  store.records_left = 12;
  EXPECT_TRUE(tracker.on_store_status(store));
  EXPECT_FALSE(tracker.on_store_status(store));

  tracker.reset_reports();
  EXPECT_EQ(tracker.boot_source(), FirmwareParamTracker::kBootUnknown);
  EXPECT_FALSE(tracker.states().at(kTilt).reported);
  EXPECT_TRUE(tracker.states().at(kTilt).requested_valid);
  EXPECT_EQ(tracker.unreported().size(), 1u);
}

TEST(FirmwareParamTracker, NamesCoverEveryFirmwareParameter)
{
  EXPECT_EQ(mowgli_hardware::firmware_param_name(kTilt), "tilt_emergency_ms");
  EXPECT_EQ(mowgli_hardware::firmware_param_name(mowgli_hardware::FW_PARAM_ID_WHEEL_BASE),
            "wheel_track");
  EXPECT_EQ(mowgli_hardware::firmware_param_name(999), "param_999");
}

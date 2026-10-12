// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0
//
// Pins the host mirror of the firmware runtime-parameter catalog
// (firmware/stm32/ros_usbnode/include/fw_param_catalog.h) and the firmware's
// coercion rules. The catalog is compiled directly from the firmware tree like
// the other pure firmware headers; ll_datatypes.hpp / firmware_params.hpp are
// hand mirrors and must agree with it id for id.

#include <cmath>
#include <limits>
#include <set>

#include "board_defaults.h"
#include "fw_param_catalog.h"
#include "mowgli_hardware/firmware_params.hpp"
#include "mowgli_hardware/ll_datatypes.hpp"
#include <gtest/gtest.h>

namespace
{

const fw_param_spec_t& spec(uint16_t id)
{
  const int index = fw_param_index(id);
  EXPECT_GE(index, 0) << "no firmware spec for id " << id;
  return FW_PARAM_SPECS[index < 0 ? 0 : index];
}

float coerce(uint16_t id, float requested, fw_param_status_t* status = nullptr)
{
  float out = -12345.0f;
  const fw_param_status_t st = fw_param_coerce(&spec(id), requested, &out);
  if (status != nullptr)
  {
    *status = st;
  }
  return out;
}

}  // namespace

TEST(FwParamCatalog, HostSendsExactlyTheFirmwareParameters)
{
  std::set<uint16_t> firmware_ids;
  for (std::size_t i = 0; i < FW_PARAM_COUNT; ++i)
  {
    EXPECT_TRUE(firmware_ids.insert(FW_PARAM_SPECS[i].id).second)
        << "duplicate firmware id " << FW_PARAM_SPECS[i].id;
  }
  std::set<uint16_t> host_ids;
  for (const auto& entry : mowgli_hardware::firmware_param_names())
  {
    EXPECT_TRUE(host_ids.insert(entry.id).second) << "duplicate host id " << entry.id;
  }
  EXPECT_EQ(host_ids, firmware_ids);
  EXPECT_EQ(fw_param_index(FW_PARAM_ID_ALL), -1);
  EXPECT_EQ(static_cast<unsigned>(mowgli_hardware::FW_PARAM_ALL),
            static_cast<unsigned>(FW_PARAM_ID_ALL));
}

TEST(FwParamCatalog, HostEnumMatchesFirmwareIds)
{
  using namespace mowgli_hardware;
  EXPECT_EQ(FW_PARAM_ID_TICKS_PER_METER, FW_PARAM_TICKS_PER_METER);
  EXPECT_EQ(FW_PARAM_ID_WHEEL_KP, FW_PARAM_WHEEL_KP);
  EXPECT_EQ(FW_PARAM_ID_WHEEL_KI, FW_PARAM_WHEEL_KI);
  EXPECT_EQ(FW_PARAM_ID_WHEEL_KD, FW_PARAM_WHEEL_KD);
  EXPECT_EQ(FW_PARAM_ID_WHEEL_INTEGRAL_LIMIT, FW_PARAM_WHEEL_INTEGRAL_LIMIT);
  EXPECT_EQ(FW_PARAM_ID_PWM_PER_MPS, FW_PARAM_PWM_PER_MPS);
  EXPECT_EQ(FW_PARAM_ID_YAW_KP, FW_PARAM_YAW_KP);
  EXPECT_EQ(FW_PARAM_ID_YAW_KI, FW_PARAM_YAW_KI);
  EXPECT_EQ(FW_PARAM_ID_YAW_TRIM_LIMIT_MPS, FW_PARAM_YAW_TRIM_LIMIT_MPS);
  EXPECT_EQ(FW_PARAM_ID_YAW_LOOP_ENABLED, FW_PARAM_YAW_LOOP_ENABLED);
  EXPECT_EQ(FW_PARAM_ID_YAW_GYRO_SIGN, FW_PARAM_YAW_GYRO_SIGN);
  EXPECT_EQ(FW_PARAM_ID_YAW_GYRO_BIAS_RADPS, FW_PARAM_YAW_GYRO_BIAS_RADPS);
  EXPECT_EQ(FW_PARAM_ID_MAX_MPS, FW_PARAM_MAX_MPS);
  EXPECT_EQ(FW_PARAM_ID_WHEEL_BASE, FW_PARAM_WHEEL_BASE);
  EXPECT_EQ(FW_PARAM_ID_MAX_CHARGE_VOLTAGE, FW_PARAM_MAX_CHARGE_VOLTAGE);
  EXPECT_EQ(FW_PARAM_ID_MAX_CHARGE_CURRENT, FW_PARAM_MAX_CHARGE_CURRENT);
  EXPECT_EQ(FW_PARAM_ID_ONE_WHEEL_LIFT_MS, FW_PARAM_ONE_WHEEL_LIFT_MS);
  EXPECT_EQ(FW_PARAM_ID_BOTH_WHEELS_LIFT_MS, FW_PARAM_BOTH_WHEELS_LIFT_MS);
  EXPECT_EQ(FW_PARAM_ID_TILT_MS, FW_PARAM_TILT_MS);
  EXPECT_EQ(FW_PARAM_ID_STOP_BUTTON_MS, FW_PARAM_STOP_BUTTON_MS);
  EXPECT_EQ(FW_PARAM_ID_PLAY_CLEAR_MS, FW_PARAM_PLAY_CLEAR_MS);
  EXPECT_EQ(FW_PARAM_ID_IMU_INCLINATION_THRESHOLD, FW_PARAM_IMU_INCLINATION_THRESHOLD);
}

TEST(FwParamCatalog, ShippedSafetyDefaultsLieInsideTheEnvelope)
{
  // A default outside the envelope would be silently coerced at boot.
  EXPECT_EQ(coerce(FW_PARAM_MAX_CHARGE_VOLTAGE, MAX_CHARGE_VOLTAGE), MAX_CHARGE_VOLTAGE);
  EXPECT_EQ(coerce(FW_PARAM_MAX_CHARGE_CURRENT, MAX_CHARGE_CURRENT), MAX_CHARGE_CURRENT);
  EXPECT_EQ(coerce(FW_PARAM_ONE_WHEEL_LIFT_MS, ONE_WHEEL_LIFT_EMERGENCY_MILLIS),
            ONE_WHEEL_LIFT_EMERGENCY_MILLIS);
  EXPECT_EQ(coerce(FW_PARAM_BOTH_WHEELS_LIFT_MS, BOTH_WHEELS_LIFT_EMERGENCY_MILLIS),
            BOTH_WHEELS_LIFT_EMERGENCY_MILLIS);
  EXPECT_EQ(coerce(FW_PARAM_TILT_MS, TILT_EMERGENCY_MILLIS), TILT_EMERGENCY_MILLIS);
  EXPECT_EQ(coerce(FW_PARAM_STOP_BUTTON_MS, STOP_BUTTON_EMERGENCY_MILLIS),
            STOP_BUTTON_EMERGENCY_MILLIS);
  EXPECT_EQ(coerce(FW_PARAM_PLAY_CLEAR_MS, PLAY_BUTTON_CLEAR_EMERGENCY_MILLIS),
            PLAY_BUTTON_CLEAR_EMERGENCY_MILLIS);
  EXPECT_EQ(coerce(FW_PARAM_IMU_INCLINATION_THRESHOLD, IMU_ONBOARD_INCLINATION_THRESHOLD),
            IMU_ONBOARD_INCLINATION_THRESHOLD);
  EXPECT_EQ(coerce(FW_PARAM_MAX_MPS, 0.5f), 0.5f);  // board.h MAX_MPS
}

TEST(FwParamCatalog, ChargeCeilingCanNeverExceedThePackLimit)
{
  fw_param_status_t status;
  EXPECT_EQ(coerce(FW_PARAM_MAX_CHARGE_VOLTAGE, 42.0f, &status), 29.4f);
  EXPECT_EQ(status, FW_PARAM_STATUS_CLAMPED);
  EXPECT_EQ(coerce(FW_PARAM_MAX_CHARGE_CURRENT, 5.0f, &status), 1.2f);
  EXPECT_EQ(status, FW_PARAM_STATUS_CLAMPED);
  // Tightening stays possible.
  EXPECT_EQ(coerce(FW_PARAM_MAX_CHARGE_VOLTAGE, 28.0f, &status), 28.0f);
  EXPECT_EQ(status, FW_PARAM_STATUS_OK);
}

TEST(FwParamCatalog, EmergencyTimingsCanBeLoosenedOnlyInsideTheEnvelope)
{
  fw_param_status_t status;
  // Looser than the shipped default, inside the envelope: accepted.
  EXPECT_EQ(coerce(FW_PARAM_TILT_MS, 800.0f, &status), 800.0f);
  EXPECT_EQ(status, FW_PARAM_STATUS_OK);
  EXPECT_EQ(coerce(FW_PARAM_PLAY_CLEAR_MS, 1000.0f, &status), 1000.0f);
  // Beyond it: clamped, never disabled.
  EXPECT_EQ(coerce(FW_PARAM_TILT_MS, 60000.0f, &status), FW_ENVELOPE_TILT_MAX_MS);
  EXPECT_EQ(status, FW_PARAM_STATUS_CLAMPED);
  EXPECT_EQ(coerce(FW_PARAM_STOP_BUTTON_MS, 0.0f), FW_ENVELOPE_TRIP_MIN_MS);
  EXPECT_EQ(coerce(FW_PARAM_STOP_BUTTON_MS, 1e9f), FW_ENVELOPE_STOP_BUTTON_MAX_MS);
  EXPECT_EQ(coerce(FW_PARAM_PLAY_CLEAR_MS, 0.0f), FW_ENVELOPE_PLAY_CLEAR_MIN_MS);
}

TEST(FwParamCatalog, NonFiniteIsRejectedAndLeavesTheOutputUntouched)
{
  for (float bad : {std::numeric_limits<float>::quiet_NaN(),
                    std::numeric_limits<float>::infinity(),
                    -std::numeric_limits<float>::infinity()})
  {
    fw_param_status_t status;
    EXPECT_EQ(coerce(FW_PARAM_TILT_MS, bad, &status), -12345.0f);
    EXPECT_EQ(status, FW_PARAM_STATUS_REJECTED);
  }
}

TEST(FwParamCatalog, KindsAreCoerced)
{
  fw_param_status_t status;
  EXPECT_EQ(coerce(FW_PARAM_TILT_MS, 300.4f, &status), 300.0f);
  EXPECT_EQ(status, FW_PARAM_STATUS_CLAMPED);  // rounded = not what was asked
  EXPECT_EQ(coerce(FW_PARAM_YAW_LOOP_ENABLED, 0.3f), 1.0f);
  EXPECT_EQ(coerce(FW_PARAM_YAW_LOOP_ENABLED, 0.0f), 0.0f);
  EXPECT_EQ(coerce(FW_PARAM_YAW_GYRO_SIGN, -0.2f), -1.0f);
  EXPECT_EQ(coerce(FW_PARAM_YAW_GYRO_SIGN, 0.0f), 1.0f);
  EXPECT_EQ(coerce(FW_PARAM_YAW_GYRO_SIGN, 7.0f), 1.0f);
}

TEST(FwParamCatalog, OnlyTheGyroBiasIsVolatile)
{
  for (std::size_t i = 0; i < FW_PARAM_COUNT; ++i)
  {
    const bool is_volatile = (FW_PARAM_SPECS[i].flags & FW_PARAM_FLAG_VOLATILE) != 0u;
    EXPECT_EQ(is_volatile, FW_PARAM_SPECS[i].id == FW_PARAM_YAW_GYRO_BIAS_RADPS)
        << "id " << FW_PARAM_SPECS[i].id;
  }
}

TEST(FwParamCatalog, EnvelopesAreWellFormed)
{
  for (std::size_t i = 0; i < FW_PARAM_COUNT; ++i)
  {
    EXPECT_LE(FW_PARAM_SPECS[i].min, FW_PARAM_SPECS[i].max) << "id " << FW_PARAM_SPECS[i].id;
    EXPECT_LT(FW_PARAM_SPECS[i].group, FW_PARAM_GROUP_COUNT) << "id " << FW_PARAM_SPECS[i].id;
  }
  EXPECT_LE(FW_PARAM_COUNT, 32u);  // the firmware's report/dirty bitmasks
}

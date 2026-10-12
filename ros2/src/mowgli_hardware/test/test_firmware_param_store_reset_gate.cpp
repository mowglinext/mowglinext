// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0

#include <array>
#include <cstdint>

#include "mowgli_hardware/firmware_param_store_reset_gate.hpp"
#include "mowgli_hardware/ll_datatypes.hpp"
#include <gtest/gtest.h>

using mowgli_hardware::FirmwareParamStoreResetBlockReason;
using mowgli_hardware::FirmwareParamStoreResetState;

namespace
{

FirmwareParamStoreResetState safe_state()
{
  FirmwareParamStoreResetState state;
  state.serial_open = true;
  state.serial_fresh = true;
  state.protocol_compatible = true;
  state.board_status_fresh = true;
  state.board_initialized = true;
  state.high_level_status_fresh = true;
  state.idle = true;
  state.odometry_fresh = true;
  state.wheel_targets_zero = true;
  state.wheels_stationary = true;
  state.actual_wheel_speeds_zero = true;
  state.blade_status_fresh = true;
  state.blade_target_off = true;
  state.blade_inactive = true;
  state.blade_rpm_zero = true;
  return state;
}

}  // namespace

TEST(FirmwareParamStoreResetWire, ProtocolV8RequestAndStatusLayoutsArePinned)
{
  using namespace mowgli_hardware;

  EXPECT_EQ(kMowgliProtocolVersion, 8u);
  EXPECT_EQ(PACKET_ID_LL_PARAM_STORE_RESET, 0x5Bu);
  EXPECT_EQ(kLlParamStoreResetMagic, 0xD3u);
  EXPECT_EQ(PARAM_COMMIT_RESET_PENDING, 6u);
  EXPECT_EQ(sizeof(LlParamStoreReset), 8u);
  EXPECT_EQ(offsetof(LlParamStoreReset, request_id), 2u);
  EXPECT_EQ(sizeof(LlParamStoreStatus), 13u);
  EXPECT_EQ(offsetof(LlParamStoreStatus, reset_request_id), 7u);

  const LlParamStoreReset request{PACKET_ID_LL_PARAM_STORE_RESET,
                                  kLlParamStoreResetMagic,
                                  0x12345678u,
                                  0u};
  const auto* bytes = reinterpret_cast<const uint8_t*>(&request);
  EXPECT_EQ(bytes[0], 0x5Bu);
  EXPECT_EQ(bytes[1], 0xD3u);
  EXPECT_EQ(bytes[2], 0x78u);
  EXPECT_EQ(bytes[3], 0x56u);
  EXPECT_EQ(bytes[4], 0x34u);
  EXPECT_EQ(bytes[5], 0x12u);
}

TEST(FirmwareParamStoreResetGate, AllowsOnlyFreshCompatibleIdleStoppedState)
{
  EXPECT_EQ(mowgli_hardware::firmware_param_store_reset_block_reason(safe_state()),
            FirmwareParamStoreResetBlockReason::kNone);
}

TEST(FirmwareParamStoreResetGate, RejectsUnknownOrStaleBoardAndMotionState)
{
  const std::array cases = {
      std::pair{&FirmwareParamStoreResetState::serial_open,
                FirmwareParamStoreResetBlockReason::kSerialClosed},
      std::pair{&FirmwareParamStoreResetState::serial_fresh,
                FirmwareParamStoreResetBlockReason::kSerialStale},
      std::pair{&FirmwareParamStoreResetState::protocol_compatible,
                FirmwareParamStoreResetBlockReason::kProtocolIncompatible},
      std::pair{&FirmwareParamStoreResetState::board_status_fresh,
                FirmwareParamStoreResetBlockReason::kBoardStatusStale},
      std::pair{&FirmwareParamStoreResetState::board_initialized,
                FirmwareParamStoreResetBlockReason::kBoardUninitialized},
      std::pair{&FirmwareParamStoreResetState::high_level_status_fresh,
                FirmwareParamStoreResetBlockReason::kHighLevelStatusStale},
      std::pair{&FirmwareParamStoreResetState::idle, FirmwareParamStoreResetBlockReason::kNotIdle},
      std::pair{&FirmwareParamStoreResetState::odometry_fresh,
                FirmwareParamStoreResetBlockReason::kOdometryStale},
      std::pair{&FirmwareParamStoreResetState::wheel_targets_zero,
                FirmwareParamStoreResetBlockReason::kWheelTargetActive},
      std::pair{&FirmwareParamStoreResetState::wheels_stationary,
                FirmwareParamStoreResetBlockReason::kWheelsMoving},
      std::pair{&FirmwareParamStoreResetState::actual_wheel_speeds_zero,
                FirmwareParamStoreResetBlockReason::kWheelsMoving},
      std::pair{&FirmwareParamStoreResetState::blade_status_fresh,
                FirmwareParamStoreResetBlockReason::kBladeStatusStale},
      std::pair{&FirmwareParamStoreResetState::blade_target_off,
                FirmwareParamStoreResetBlockReason::kBladeTargetActive},
      std::pair{&FirmwareParamStoreResetState::blade_inactive,
                FirmwareParamStoreResetBlockReason::kBladeActive},
      std::pair{&FirmwareParamStoreResetState::blade_rpm_zero,
                FirmwareParamStoreResetBlockReason::kBladeRpmNonzero},
  };

  for (const auto& [field, expected] : cases)
  {
    auto state = safe_state();
    state.*field = false;
    EXPECT_EQ(mowgli_hardware::firmware_param_store_reset_block_reason(state), expected);
  }
}

TEST(FirmwareParamStoreResetGate, RefusesToReplaceAnAlreadyPendingRequest)
{
  auto state = safe_state();
  state.reset_already_pending = true;
  EXPECT_EQ(mowgli_hardware::firmware_param_store_reset_block_reason(state),
            FirmwareParamStoreResetBlockReason::kResetAlreadyPending);
}

TEST(FirmwareParamStoreResetGate, MovingWheelsAndActiveBladeAreRejected)
{
  auto state = safe_state();
  state.wheels_stationary = false;
  EXPECT_EQ(mowgli_hardware::firmware_param_store_reset_block_reason(state),
            FirmwareParamStoreResetBlockReason::kWheelsMoving);

  state = safe_state();
  state.blade_inactive = false;
  EXPECT_EQ(mowgli_hardware::firmware_param_store_reset_block_reason(state),
            FirmwareParamStoreResetBlockReason::kBladeActive);
}

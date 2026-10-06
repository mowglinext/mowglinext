// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0
//
// Fail-closed host gate for the explicit STM32 parameter-store reset action.

#pragma once

#include <string_view>

namespace mowgli_hardware
{

struct FirmwareParamStoreResetState
{
  bool serial_open{false};
  bool serial_fresh{false};
  bool protocol_compatible{false};
  bool board_status_fresh{false};
  bool board_initialized{false};
  bool high_level_status_fresh{false};
  bool idle{false};
  bool odometry_fresh{false};
  bool wheel_targets_zero{false};
  bool wheels_stationary{false};
  bool actual_wheel_speeds_zero{false};
  bool blade_status_fresh{false};
  bool blade_target_off{false};
  bool blade_inactive{false};
  bool blade_rpm_zero{false};
  bool reset_already_pending{false};
};

enum class FirmwareParamStoreResetBlockReason
{
  kNone,
  kSerialClosed,
  kSerialStale,
  kProtocolIncompatible,
  kBoardStatusStale,
  kBoardUninitialized,
  kHighLevelStatusStale,
  kNotIdle,
  kOdometryStale,
  kWheelTargetActive,
  kWheelsMoving,
  kBladeStatusStale,
  kBladeTargetActive,
  kBladeActive,
  kBladeRpmNonzero,
  kResetAlreadyPending,
};

inline FirmwareParamStoreResetBlockReason firmware_param_store_reset_block_reason(
    const FirmwareParamStoreResetState& state)
{
  if (!state.serial_open)
    return FirmwareParamStoreResetBlockReason::kSerialClosed;
  if (!state.serial_fresh)
    return FirmwareParamStoreResetBlockReason::kSerialStale;
  if (!state.protocol_compatible)
    return FirmwareParamStoreResetBlockReason::kProtocolIncompatible;
  if (!state.board_status_fresh)
    return FirmwareParamStoreResetBlockReason::kBoardStatusStale;
  if (!state.board_initialized)
    return FirmwareParamStoreResetBlockReason::kBoardUninitialized;
  if (!state.high_level_status_fresh)
    return FirmwareParamStoreResetBlockReason::kHighLevelStatusStale;
  if (!state.idle)
    return FirmwareParamStoreResetBlockReason::kNotIdle;
  if (!state.odometry_fresh)
    return FirmwareParamStoreResetBlockReason::kOdometryStale;
  if (!state.wheel_targets_zero)
    return FirmwareParamStoreResetBlockReason::kWheelTargetActive;
  if (!state.wheels_stationary || !state.actual_wheel_speeds_zero)
    return FirmwareParamStoreResetBlockReason::kWheelsMoving;
  if (!state.blade_status_fresh)
    return FirmwareParamStoreResetBlockReason::kBladeStatusStale;
  if (!state.blade_target_off)
    return FirmwareParamStoreResetBlockReason::kBladeTargetActive;
  if (!state.blade_inactive)
    return FirmwareParamStoreResetBlockReason::kBladeActive;
  if (!state.blade_rpm_zero)
    return FirmwareParamStoreResetBlockReason::kBladeRpmNonzero;
  if (state.reset_already_pending)
    return FirmwareParamStoreResetBlockReason::kResetAlreadyPending;
  return FirmwareParamStoreResetBlockReason::kNone;
}

inline std::string_view firmware_param_store_reset_block_message(
    FirmwareParamStoreResetBlockReason reason)
{
  switch (reason)
  {
    case FirmwareParamStoreResetBlockReason::kNone:
      return {};
    case FirmwareParamStoreResetBlockReason::kSerialClosed:
      return "serial port is not open";
    case FirmwareParamStoreResetBlockReason::kSerialStale:
      return "serial telemetry is stale";
    case FirmwareParamStoreResetBlockReason::kProtocolIncompatible:
      return "firmware protocol is not confirmed compatible";
    case FirmwareParamStoreResetBlockReason::kBoardStatusStale:
      return "board status is stale";
    case FirmwareParamStoreResetBlockReason::kBoardUninitialized:
      return "firmware board is not initialized";
    case FirmwareParamStoreResetBlockReason::kHighLevelStatusStale:
      return "high-level state is stale";
    case FirmwareParamStoreResetBlockReason::kNotIdle:
      return "mower must be in IDLE";
    case FirmwareParamStoreResetBlockReason::kOdometryStale:
      return "wheel telemetry is stale";
    case FirmwareParamStoreResetBlockReason::kWheelTargetActive:
      return "wheel targets are not confirmed stopped";
    case FirmwareParamStoreResetBlockReason::kWheelsMoving:
      return "wheel telemetry reports motion";
    case FirmwareParamStoreResetBlockReason::kBladeStatusStale:
      return "blade telemetry is stale";
    case FirmwareParamStoreResetBlockReason::kBladeTargetActive:
      return "blade command must be off";
    case FirmwareParamStoreResetBlockReason::kBladeActive:
      return "blade telemetry reports active";
    case FirmwareParamStoreResetBlockReason::kBladeRpmNonzero:
      return "blade RPM is not zero";
    case FirmwareParamStoreResetBlockReason::kResetAlreadyPending:
      return "a parameter-store reset is already pending";
  }
  return "firmware parameter-store reset is unavailable";
}

}  // namespace mowgli_hardware

// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0
//
// Host-side bookkeeping for the STM32 runtime parameters (protocol v7).
//
// The bridge sends every parameter with SET_PARAM on each (re)connect burst and
// whenever a ROS parameter changes; the firmware coerces each value into its
// absolute envelope and reports what it applied (PARAM_VALUE). This tracker
// keeps "what we asked" next to "what the board runs", so a clamped value is
// visible (log + /hardware_bridge/firmware_params) instead of silently differing.
// Pure C++ (no rclcpp) so it is unit-tested directly (test_firmware_params.cpp).

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "mowgli_hardware/ll_datatypes.hpp"

namespace mowgli_hardware
{

/// A firmware parameter and the hardware_bridge ROS parameter that feeds it.
struct FirmwareParamName
{
  uint16_t id;
  const char* ros_name;
};

/// Every parameter this image sends, in wire-id order. The ids mirror
/// fw_param_catalog.h (pinned by test_fw_param_catalog.cpp).
inline const std::vector<FirmwareParamName>& firmware_param_names()
{
  static const std::vector<FirmwareParamName> kNames = {
      {FW_PARAM_ID_TICKS_PER_METER, "ticks_per_meter"},
      {FW_PARAM_ID_WHEEL_KP, "wheel_pid_kp"},
      {FW_PARAM_ID_WHEEL_KI, "wheel_pid_ki"},
      {FW_PARAM_ID_WHEEL_KD, "wheel_pid_kd"},
      {FW_PARAM_ID_WHEEL_INTEGRAL_LIMIT, "wheel_pid_integral_limit"},
      {FW_PARAM_ID_PWM_PER_MPS, "wheel_pid_pwm_per_mps"},
      {FW_PARAM_ID_YAW_KP, "yaw_kp"},
      {FW_PARAM_ID_YAW_KI, "yaw_ki"},
      {FW_PARAM_ID_YAW_TRIM_LIMIT_MPS, "yaw_trim_limit_mps"},
      {FW_PARAM_ID_YAW_LOOP_ENABLED, "yaw_loop_enabled"},
      {FW_PARAM_ID_YAW_GYRO_SIGN, "yaw_gyro_sign"},
      {FW_PARAM_ID_YAW_GYRO_BIAS_RADPS, "imu_gyro_bias_z"},
      {FW_PARAM_ID_MAX_MPS, "max_mps"},
      {FW_PARAM_ID_WHEEL_BASE, "wheel_track"},
      {FW_PARAM_ID_MAX_CHARGE_VOLTAGE, "max_charge_voltage"},
      {FW_PARAM_ID_MAX_CHARGE_CURRENT, "max_charge_current"},
      {FW_PARAM_ID_ONE_WHEEL_LIFT_MS, "one_wheel_lift_emergency_ms"},
      {FW_PARAM_ID_BOTH_WHEELS_LIFT_MS, "both_wheels_lift_emergency_ms"},
      {FW_PARAM_ID_TILT_MS, "tilt_emergency_ms"},
      {FW_PARAM_ID_STOP_BUTTON_MS, "stop_button_emergency_ms"},
      {FW_PARAM_ID_PLAY_CLEAR_MS, "play_button_clear_emergency_ms"},
      {FW_PARAM_ID_IMU_INCLINATION_THRESHOLD, "imu_inclination_threshold"},
  };
  return kNames;
}

/// ROS parameter name for @p id, or "param_<id>" for an id this image does
/// not know (a newer firmware).
inline std::string firmware_param_name(uint16_t id)
{
  for (const auto& entry : firmware_param_names())
  {
    if (entry.id == id)
    {
      return entry.ros_name;
    }
  }
  return "param_" + std::to_string(id);
}

struct FirmwareParamState
{
  uint16_t id{0};
  bool requested_valid{false};
  float requested{NAN};
  bool reported{false};
  float applied{NAN};
  float default_value{NAN};
  float min_value{NAN};
  float max_value{NAN};
  uint8_t status{FW_PARAM_STATUS_OK};
  bool persisted{false};
  bool is_volatile{false};
};

/// What a firmware report changed, so the caller can log once per change.
enum class FirmwareParamReport
{
  kUnchanged,  ///< same as the previous report
  kUpdated,  ///< new information, applied value matches the request
  kDiffersFromRequest,  ///< the firmware runs something other than we asked
  kUnknownId,  ///< the firmware does not have this parameter
};

class FirmwareParamTracker
{
public:
  static constexpr uint8_t kBootUnknown = 255u;

  /// Forget every firmware report (the board rebooted or was reflashed); the
  /// requested values stay, they are re-sent by the next burst.
  void reset_reports()
  {
    for (auto& [id, state] : states_)
    {
      state.reported = false;
      state.applied = NAN;
    }
    boot_source_ = kBootUnknown;
    last_commit_ = PARAM_COMMIT_NONE;
    records_left_ = 0;
  }

  void set_requested(uint16_t id, float value)
  {
    auto& state = state_for(id);
    state.requested_valid = true;
    state.requested = value;
  }

  FirmwareParamReport on_value(const LlParamValue& pkt)
  {
    if (pkt.status == FW_PARAM_STATUS_UNKNOWN_ID)
    {
      return FirmwareParamReport::kUnknownId;
    }
    auto& state = state_for(pkt.param_id);
    const FirmwareParamState before = state;
    state.reported = true;
    state.applied = pkt.value;
    state.default_value = pkt.default_value;
    state.min_value = pkt.min_value;
    state.max_value = pkt.max_value;
    state.status = pkt.status;
    state.persisted = (pkt.flags & PARAM_VALUE_FLAG_PERSISTED) != 0u;
    state.is_volatile = (pkt.flags & PARAM_VALUE_FLAG_VOLATILE) != 0u;
    const bool changed = !before.reported || !same(before.applied, state.applied) ||
                         before.status != state.status || before.persisted != state.persisted ||
                         !same(before.min_value, state.min_value) ||
                         !same(before.max_value, state.max_value);
    if (!changed)
    {
      return FirmwareParamReport::kUnchanged;
    }
    return differs_from_request(state) ? FirmwareParamReport::kDiffersFromRequest
                                       : FirmwareParamReport::kUpdated;
  }

  /// @return true when the store status changed.
  bool on_store_status(const LlParamStoreStatus& pkt)
  {
    const bool changed = boot_source_ != pkt.boot_source || last_commit_ != pkt.last_commit ||
                         records_left_ != pkt.records_left;
    boot_source_ = pkt.boot_source;
    last_commit_ = pkt.last_commit;
    records_left_ = pkt.records_left;
    return changed;
  }

  /// Parameters we sent that the firmware has not reported since the reset.
  std::vector<uint16_t> unreported() const
  {
    std::vector<uint16_t> ids;
    for (const auto& [id, state] : states_)
    {
      if (state.requested_valid && !state.reported)
      {
        ids.push_back(id);
      }
    }
    return ids;
  }

  /// The applied value is not what we asked. Integer-valued parameters are
  /// rounded by the firmware, so compare with a tolerance relative to the
  /// value; a float round trip is exact otherwise.
  static bool differs_from_request(const FirmwareParamState& state)
  {
    if (!state.requested_valid || !state.reported)
    {
      return false;
    }
    const float tolerance = 1e-6f * std::max(1.0f, std::fabs(state.requested));
    return !(std::fabs(state.applied - state.requested) <= tolerance);
  }

  const std::map<uint16_t, FirmwareParamState>& states() const
  {
    return states_;
  }
  uint8_t boot_source() const
  {
    return boot_source_;
  }
  uint8_t last_commit() const
  {
    return last_commit_;
  }
  uint16_t records_left() const
  {
    return records_left_;
  }

private:
  static bool same(float a, float b)
  {
    return (std::isnan(a) && std::isnan(b)) || a == b;
  }

  FirmwareParamState& state_for(uint16_t id)
  {
    auto& state = states_[id];
    state.id = id;
    return state;
  }

  std::map<uint16_t, FirmwareParamState> states_;
  uint8_t boot_source_{kBootUnknown};
  uint8_t last_commit_{PARAM_COMMIT_NONE};
  uint16_t records_left_{0};
};

}  // namespace mowgli_hardware

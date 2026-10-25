// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Decoder for the PREVIOUS mainboard protocol's (v8) odometry packet.
// Pure logic, no ROS, so it is unit-testable standalone
// (see test_legacy_odometry.cpp).
//
// ── Why a v9 host still reads v8 odometry ───────────────────────────────────
// Container updates never flash the STM32. On the update-first transition of
// docs/UPDATES.md the new host therefore runs against the OLD firmware until
// the operator flashes the matching image from the new GUI. The updater only
// accepts that state when the mower readiness proves fresh status, behaviour
// AND wheel telemetry, idle, stationary and blade-off, with the bridge's
// firmware-compatibility flag as the ONLY missing bit
// (gui/pkg/updater/firmware_protocol.go, expectedFirmwareMismatch).
//
// Protocol v9 grew LlOdometry 17 -> 19 bytes (dt_millis u16 -> u32
// acquisition_millis). A v9 host that dropped every 17-byte v8 packet never
// published /wheel_odom, the readiness reported stale wheel telemetry, and the
// update rolled back (field, 2026-10-25) — the operator could never reach the
// GUI that flashes v9. Decoding the previous layout keeps wheel telemetry and
// the stationary proof alive across that window.
//
// ── NOT a compatibility claim ───────────────────────────────────────────────
// This changes nothing about fw_compatible_: it is still exactly
// (board protocol == kMowgliProtocolVersion), so PreFlightCheck keeps refusing
// to mow on a v8 board. The decoder only keeps telemetry readable.
//
// ── Conversion ──────────────────────────────────────────────────────────────
// The v8 packet carries the firmware-measured interval since the previous
// SENT packet (dt_millis) instead of an absolute acquisition time. Summing it
// yields a monotonic synthetic acquisition clock that is exactly what the v9
// path differentiates again, so the publisher sees the same elapsed time the
// v8 host used. Ticks are re-expressed as a synthetic cumulative count built
// from deltas unwrapped the way the v8 host did (a ±65536 jump from the
// motor-controller's 16-bit counter folds back to the small physical delta);
// the publisher's own spike rejection then applies unchanged.

#pragma once

#include <cstddef>
#include <cstdint>

#include "mowgli_hardware/ll_datatypes.hpp"

namespace mowgli_hardware
{

/// The protocol whose odometry layout LlOdometryV8 describes. Kept one version
/// behind kMowgliProtocolVersion (see docs/UPDATES.md, protocol bump checklist).
static constexpr uint8_t kLegacyOdometryProtocolVersion = 8u;
static_assert(kLegacyOdometryProtocolVersion + 1u == kMowgliProtocolVersion,
              "legacy odometry must describe the protocol directly before the current one; "
              "update LlOdometryV8 (or drop it) when bumping the protocol");

#pragma pack(push, 1)

/// Protocol v8 odometry packet (PACKET_ID_LL_ODOMETRY, 17 bytes on the wire).
struct LlOdometryV8
{
  uint8_t type;  ///< Must equal PACKET_ID_LL_ODOMETRY
  uint16_t dt_millis;  ///< Firmware-measured interval since the previous sent packet [ms]
  int32_t left_ticks;  ///< Signed cumulative left encoder ticks
  int32_t right_ticks;  ///< Signed cumulative right encoder ticks
  int16_t left_velocity_mm_s;  ///< Signed left wheel velocity [mm/s]
  int16_t right_velocity_mm_s;  ///< Signed right wheel velocity [mm/s]
  uint16_t crc;  ///< CRC-16 CCITT over all preceding bytes
};

#pragma pack(pop)

static_assert(sizeof(LlOdometryV8) == 17u, "LlOdometryV8 layout mismatch");

/// Which odometry layout a decoded (CRC-verified) packet of @p len bytes is.
enum class OdometryWireFormat
{
  kCurrent,  ///< LlOdometry (current protocol)
  kLegacyV8,  ///< LlOdometryV8 (previous protocol)
  kInvalid,  ///< Neither: drop it
};

[[nodiscard]] inline OdometryWireFormat classify_odometry_packet(std::size_t len) noexcept
{
  if (len >= sizeof(LlOdometry))
  {
    return OdometryWireFormat::kCurrent;
  }
  if (len == sizeof(LlOdometryV8))
  {
    return OdometryWireFormat::kLegacyV8;
  }
  return OdometryWireFormat::kInvalid;
}

/// The v8 host's recovery of a 16-bit motor-controller counter wrap: any
/// delta closer to ±65536 than to 0 is the small physical delta plus a wrap.
[[nodiscard]] inline int32_t unwrap_legacy_16bit_delta(int32_t d) noexcept
{
  if (d > 32768)
  {
    return d - 65536;
  }
  if (d < -32768)
  {
    return d + 65536;
  }
  return d;
}

/// Converts a stream of v8 packets into current-protocol LlOdometry packets
/// with a synthetic acquisition clock and synthetic cumulative ticks.
class LegacyOdometryDecoder
{
public:
  void reset() noexcept
  {
    initialized_ = false;
    acquisition_ms_ = 0u;
    prev_raw_left_ = 0;
    prev_raw_right_ = 0;
    left_ = 0u;
    right_ = 0u;
  }

  [[nodiscard]] LlOdometry convert(const LlOdometryV8& in) noexcept
  {
    if (!initialized_)
    {
      // The first packet only seeds the baselines, exactly like the
      // publisher's own first packet: no interval or delta is invented.
      initialized_ = true;
      left_ = static_cast<uint32_t>(in.left_ticks);
      right_ = static_cast<uint32_t>(in.right_ticks);
    }
    else
    {
      acquisition_ms_ += in.dt_millis;  // u32: wraps like the HAL tick it stands in for
      left_ +=
          static_cast<uint32_t>(unwrap_legacy_16bit_delta(delta(in.left_ticks, prev_raw_left_)));
      right_ +=
          static_cast<uint32_t>(unwrap_legacy_16bit_delta(delta(in.right_ticks, prev_raw_right_)));
    }
    prev_raw_left_ = in.left_ticks;
    prev_raw_right_ = in.right_ticks;

    LlOdometry out{};
    out.type = in.type;
    out.acquisition_millis = acquisition_ms_;
    out.left_ticks = static_cast<int32_t>(left_);
    out.right_ticks = static_cast<int32_t>(right_);
    out.left_velocity_mm_s = in.left_velocity_mm_s;
    out.right_velocity_mm_s = in.right_velocity_mm_s;
    out.crc = in.crc;
    return out;
  }

private:
  /// Modular int32 difference (no signed-overflow UB on a counter wrap).
  [[nodiscard]] static int32_t delta(int32_t now, int32_t prev) noexcept
  {
    return static_cast<int32_t>(static_cast<uint32_t>(now) - static_cast<uint32_t>(prev));
  }

  bool initialized_{false};
  uint32_t acquisition_ms_{0u};
  int32_t prev_raw_left_{0};
  int32_t prev_raw_right_{0};
  // Synthetic cumulative counts, kept unsigned so accumulation wraps defined.
  uint32_t left_{0u};
  uint32_t right_{0u};
};

}  // namespace mowgli_hardware

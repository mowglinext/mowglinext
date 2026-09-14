// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Thin RAII wrapper over a Linux spidev character device.
//
// This is the ONLY part of mowgli_leds that touches hardware and therefore the
// only part that is not unit-tested: everything upstream of it (status ->
// pixels, pixels -> SPI bytes) is pure and covered by gtest. Keep it that way
// -- no display logic belongs in here.
//
// Every failure path returns a message instead of throwing or logging, so the
// caller owns the "warn once, then stand down quietly" policy (the node does);
// a missing /dev/spidev must never take the node down or spam the log.

#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace mowgli_leds
{

/// Outcome of an SpiDevice call. `ok == false` always carries a human-readable
/// `error` naming the syscall and the errno text.
struct SpiResult
{
  bool ok = false;
  std::string error;

  static SpiResult Success()
  {
    return SpiResult{true, {}};
  }

  static SpiResult Failure(std::string message)
  {
    return SpiResult{false, std::move(message)};
  }
};

class SpiDevice
{
public:
  SpiDevice() = default;
  ~SpiDevice();

  SpiDevice(const SpiDevice&) = delete;
  SpiDevice& operator=(const SpiDevice&) = delete;
  SpiDevice(SpiDevice&&) = delete;
  SpiDevice& operator=(SpiDevice&&) = delete;

  /// Open `path` and configure it for WS2812: SPI mode 0 (CPOL=0/CPHA=0, so
  /// MOSI idles LOW), 8 bits per word, MSB first, `speed_hz` clock.
  /// Closes any previously open descriptor first. Never throws.
  SpiResult Open(const std::string& path, std::uint32_t speed_hz);

  bool IsOpen() const
  {
    return fd_ >= 0;
  }

  const std::string& path() const
  {
    return path_;
  }

  void Close();

  /// Write the whole buffer. A short write is reported as a failure -- a
  /// truncated WS2812 frame leaves the strip mid-bit, so it is never "fine".
  SpiResult Write(const std::vector<std::uint8_t>& bytes);

private:
  int fd_ = -1;
  std::string path_;
};

}  // namespace mowgli_leds

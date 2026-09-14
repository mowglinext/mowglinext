// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// WS2812/WS2812B wire encoding over SPI MOSI. Pure logic, no ROS and no
// hardware, so it is unit-testable standalone (see test_ws2812_encoder.cpp) --
// same shape as mowgli_hardware/dig_detector.hpp.
//
// -- Why SPI and not UART/PWM/PIO -------------------------------------------
// The ring's DATA line is soldered to 40-pin header pin 19 on the Orange Pi 5B,
// which is simultaneously I2C3_SCL_M0, UART3_TX_M0 and SPI4_MOSI_M0. We drive
// it as SPI4 MOSI. A UART TX line IDLES HIGH; WS2812 requires the line to idle
// LOW (a high idle is read as a stream of garbage bits and the reset gap never
// happens), so the UART route needs an external inverter. SPI MOSI in mode 0
// idles LOW and matches the protocol natively. Same physical wire, no rewiring.
//
// -- Why 3 SPI bits per WS2812 bit at 2.4 MHz --------------------------------
// One SPI bit at 2.4 MHz is 416.67 ns, so three of them are 1.25 us -- exactly
// the WS2812B bit period. The two symbols are:
//
//   WS2812 "0" -> 0b100 :  T0H = 417 ns, T0L = 833 ns
//   WS2812 "1" -> 0b110 :  T1H = 833 ns, T1L = 417 ns
//
// WS2812B datasheet windows (+/-150 ns): T0H 400, T0L 850, T1H 800, T1L 450.
// All four land within ~35 ns of nominal -- dead centre, with the full +/-150 ns
// of margin available for SPI clock error.
//
// The common alternative is 4 SPI bits at 3.2 MHz (312.5 ns/bit). It is worse
// here on two counts: the natural symbols 0b1000/0b1100 put T1H at 625 ns,
// BELOW the 650 ns lower bound (you must use 0b1110 instead, which then pushes
// T1L to 312 ns near ITS lower bound of 300 ns), and it costs 12 bytes per
// pixel instead of 9 -- 33 % more SPI traffic for no timing benefit.
//
// The 3-bit scheme is also byte-aligned by construction: 8 WS2812 bits expand
// to 24 SPI bits = exactly 3 bytes, so a colour byte never straddles an output
// byte boundary and the expansion is a plain per-byte lookup. The 24 SPI bits
// of a pixel's G/R/B therefore occupy 9 bytes with no bit packing at all.

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace mowgli_leds
{

/// A single pixel in absolute colour space (before brightness scaling).
/// Deliberately NOT in wire order -- the GRB swap happens in Encode(), once,
/// where it can be tested. Getting that backwards is the classic WS2812 bug.
struct Rgb
{
  std::uint8_t r = 0;
  std::uint8_t g = 0;
  std::uint8_t b = 0;
};

inline bool operator==(const Rgb& lhs, const Rgb& rhs)
{
  return lhs.r == rhs.r && lhs.g == rhs.g && lhs.b == rhs.b;
}

inline bool operator!=(const Rgb& lhs, const Rgb& rhs)
{
  return !(lhs == rhs);
}

namespace ws2812
{

/// SPI clock the 3-bit-per-WS2812-bit scheme requires (see header comment).
constexpr std::uint32_t kSpiClockHz = 2400000u;

/// SPI bits emitted per WS2812 bit.
constexpr std::size_t kSpiBitsPerLedBit = 3u;

/// One colour byte (8 WS2812 bits) -> 24 SPI bits -> 3 bytes, byte-aligned.
constexpr std::size_t kBytesPerColorByte = 3u;

/// Three colour bytes (G, R, B) per pixel.
constexpr std::size_t kBytesPerPixel = 3u * kBytesPerColorByte;

/// Trailing all-zero bytes that hold DATA low to latch the frame.
///
/// The original WS2812B datasheet asks for RES > 50 us; the widely-shipped
/// WS2812B-V5 revision needs > 280 us and will merge frames below that. At
/// 2.4 MHz one byte is 3.33 us, so 90 bytes = 300 us covers BOTH parts. It
/// costs 90 bytes on a ~144-byte frame, which is free at this data rate --
/// there is no reason to shave it down to the older 50 us figure.
constexpr std::size_t kResetLowBytes = 90u;

/// Bytes a full frame occupies on the wire, reset gap included.
constexpr std::size_t EncodedSize(std::size_t led_count)
{
  return led_count * kBytesPerPixel + kResetLowBytes;
}

/// Expand one colour byte (MSB first) into its 3 SPI bytes.
constexpr std::array<std::uint8_t, kBytesPerColorByte> ExpandByte(std::uint8_t value)
{
  std::uint32_t bits = 0u;
  for (int shift = 7; shift >= 0; --shift)
  {
    const bool bit_set = ((value >> shift) & 0x01u) != 0u;
    bits = (bits << kSpiBitsPerLedBit) | (bit_set ? 0b110u : 0b100u);
  }
  return {
      static_cast<std::uint8_t>((bits >> 16) & 0xFFu),
      static_cast<std::uint8_t>((bits >> 8) & 0xFFu),
      static_cast<std::uint8_t>(bits & 0xFFu),
  };
}

/// Linear brightness scale of one channel. `brightness` is clamped to [0, 1];
/// a non-finite value is treated as 0 (dark) rather than propagating NaN into
/// the byte cast, which would be undefined behaviour.
inline std::uint8_t ScaleChannel(std::uint8_t value, float brightness)
{
  if (!std::isfinite(brightness))
  {
    return 0u;
  }
  const float clamped = std::clamp(brightness, 0.0f, 1.0f);
  const float scaled = static_cast<float>(value) * clamped;
  return static_cast<std::uint8_t>(std::lround(scaled));
}

/// Encode a pixel buffer into the SPI byte stream to write to MOSI.
///
/// Emits GRB per pixel -- WS2812 wire order, NOT RGB -- then the reset gap.
/// `brightness` scales every channel linearly and is clamped to [0, 1].
inline std::vector<std::uint8_t> Encode(const std::vector<Rgb>& pixels, float brightness)
{
  std::vector<std::uint8_t> out;
  out.reserve(EncodedSize(pixels.size()));

  for (const Rgb& pixel : pixels)
  {
    const std::array<std::uint8_t, 3> wire = {
        ScaleChannel(pixel.g, brightness),
        ScaleChannel(pixel.r, brightness),
        ScaleChannel(pixel.b, brightness),
    };
    for (const std::uint8_t color_byte : wire)
    {
      const auto expanded = ExpandByte(color_byte);
      out.insert(out.end(), expanded.begin(), expanded.end());
    }
  }

  out.insert(out.end(), kResetLowBytes, 0u);
  return out;
}

}  // namespace ws2812

}  // namespace mowgli_leds

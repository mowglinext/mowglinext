// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Unit tests for the pure WS2812-over-SPI encoding. No ROS, no hardware.

#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include "mowgli_leds/ws2812_encoder.hpp"
#include <gtest/gtest.h>

namespace mowgli_leds
{
namespace
{

/// Read SPI bit `index` (MSB first) out of an encoded byte stream.
bool BitAt(const std::vector<std::uint8_t>& bytes, std::size_t index)
{
  const std::size_t byte_index = index / 8u;
  const std::size_t shift = 7u - (index % 8u);
  return ((bytes[byte_index] >> shift) & 0x01u) != 0u;
}

TEST(Ws2812Encoder, ExpandsZeroBitsToTheOneZeroZeroSymbol)
{
  // 0x00 -> eight 0b100 symbols -> 100100100100100100100100
  const auto expanded = ws2812::ExpandByte(0x00u);
  EXPECT_EQ(expanded[0], 0x92u);
  EXPECT_EQ(expanded[1], 0x49u);
  EXPECT_EQ(expanded[2], 0x24u);
}

TEST(Ws2812Encoder, ExpandsOneBitsToTheOneOneZeroSymbol)
{
  // 0xFF -> eight 0b110 symbols -> 110110110110110110110110
  const auto expanded = ws2812::ExpandByte(0xFFu);
  EXPECT_EQ(expanded[0], 0xDBu);
  EXPECT_EQ(expanded[1], 0x6Du);
  EXPECT_EQ(expanded[2], 0xB6u);
}

TEST(Ws2812Encoder, ExpandsMostSignificantBitFirst)
{
  // 0x80 is "one, then seven zeros": the FIRST symbol must be 0b110.
  const auto expanded = ws2812::ExpandByte(0x80u);
  EXPECT_EQ(expanded[0], 0xD2u);
  EXPECT_EQ(expanded[1], 0x49u);
  EXPECT_EQ(expanded[2], 0x24u);

  // ...and the mirror case, 0x01, must put its 0b110 symbol LAST.
  const auto low_bit = ws2812::ExpandByte(0x01u);
  EXPECT_EQ(low_bit[0], 0x92u);
  EXPECT_EQ(low_bit[1], 0x49u);
  EXPECT_EQ(low_bit[2], 0x26u);
}

TEST(Ws2812Encoder, EverySymbolStartsHighAndEndsLow)
{
  // Structural invariant of both symbols (0b100 and 0b110): the line always
  // rises at the start of a bit period and is always low at its end. If this
  // ever fails the strip sees a merged or truncated bit.
  std::vector<Rgb> pixels;
  pixels.push_back(Rgb{0x5Au, 0xA5u, 0x3Cu});
  const auto bytes = ws2812::Encode(pixels, 1.0f);

  const std::size_t led_bits = 24u;  // one pixel
  for (std::size_t bit = 0; bit < led_bits; ++bit)
  {
    const std::size_t base = bit * ws2812::kSpiBitsPerLedBit;
    EXPECT_TRUE(BitAt(bytes, base)) << "bit " << bit << " did not rise";
    EXPECT_FALSE(BitAt(bytes, base + 2u)) << "bit " << bit << " did not return low";
  }
}

TEST(Ws2812Encoder, EncodesPixelsInGrbWireOrderNotRgb)
{
  // Pure red at full brightness. WS2812 wants GREEN first, so the first three
  // encoded bytes must be the expansion of 0x00 and the SECOND three the
  // expansion of 0xFF. Getting this backwards is the classic WS2812 bug.
  const std::vector<Rgb> pixels{Rgb{255u, 0u, 0u}};
  const auto bytes = ws2812::Encode(pixels, 1.0f);

  const auto zero = ws2812::ExpandByte(0x00u);
  const auto full = ws2812::ExpandByte(0xFFu);

  EXPECT_EQ(bytes[0], zero[0]);  // G
  EXPECT_EQ(bytes[1], zero[1]);
  EXPECT_EQ(bytes[2], zero[2]);
  EXPECT_EQ(bytes[3], full[0]);  // R
  EXPECT_EQ(bytes[4], full[1]);
  EXPECT_EQ(bytes[5], full[2]);
  EXPECT_EQ(bytes[6], zero[0]);  // B
  EXPECT_EQ(bytes[7], zero[1]);
  EXPECT_EQ(bytes[8], zero[2]);
}

TEST(Ws2812Encoder, EncodedFrameIsNineBytesPerPixelPlusTheResetGap)
{
  const std::vector<Rgb> pixels(16u, Rgb{1u, 2u, 3u});
  const auto bytes = ws2812::Encode(pixels, 1.0f);

  EXPECT_EQ(bytes.size(), ws2812::EncodedSize(16u));
  EXPECT_EQ(bytes.size(), 16u * 9u + ws2812::kResetLowBytes);
}

TEST(Ws2812Encoder, ResetGapIsAllZerosAndLongerThanTwoHundredEightyMicroseconds)
{
  const std::vector<Rgb> pixels(4u, Rgb{255u, 255u, 255u});
  const auto bytes = ws2812::Encode(pixels, 1.0f);

  for (std::size_t i = pixels.size() * ws2812::kBytesPerPixel; i < bytes.size(); ++i)
  {
    EXPECT_EQ(bytes[i], 0u) << "reset gap byte " << i << " is not low";
  }

  // The WS2812B-V5 revision needs RES > 280 us; the original part needs > 50 us.
  const double reset_us =
      static_cast<double>(ws2812::kResetLowBytes) * 8.0 * 1e6 / ws2812::kSpiClockHz;
  EXPECT_GT(reset_us, 280.0);
}

TEST(Ws2812Encoder, SpiClockGivesTheWs2812BitPeriodOfOnePointTwoFiveMicroseconds)
{
  const double bit_period_us =
      static_cast<double>(ws2812::kSpiBitsPerLedBit) * 1e6 / ws2812::kSpiClockHz;
  EXPECT_NEAR(bit_period_us, 1.25, 1e-9);
}

TEST(Ws2812Encoder, BrightnessScalesEveryChannelLinearly)
{
  const std::vector<Rgb> pixels{Rgb{200u, 100u, 50u}};
  const auto bytes = ws2812::Encode(pixels, 0.5f);

  EXPECT_EQ(bytes[0], ws2812::ExpandByte(50u)[0]);  // G: 100 * 0.5
  EXPECT_EQ(bytes[3], ws2812::ExpandByte(100u)[0]);  // R: 200 * 0.5
  EXPECT_EQ(bytes[6], ws2812::ExpandByte(25u)[0]);  // B: 50 * 0.5
}

TEST(Ws2812Encoder, ZeroBrightnessProducesOnlyZeroColorBytes)
{
  const std::vector<Rgb> pixels(8u, Rgb{255u, 255u, 255u});
  const auto bytes = ws2812::Encode(pixels, 0.0f);

  const auto zero = ws2812::ExpandByte(0x00u);
  for (std::size_t i = 0; i < pixels.size() * ws2812::kBytesPerPixel; i += 3u)
  {
    EXPECT_EQ(bytes[i], zero[0]);
    EXPECT_EQ(bytes[i + 1u], zero[1]);
    EXPECT_EQ(bytes[i + 2u], zero[2]);
  }
}

TEST(Ws2812Encoder, BrightnessIsClampedToTheUnitRange)
{
  EXPECT_EQ(ws2812::ScaleChannel(200u, 5.0f), 200u);
  EXPECT_EQ(ws2812::ScaleChannel(200u, -3.0f), 0u);
}

TEST(Ws2812Encoder, NonFiniteBrightnessGoesDarkInsteadOfCastingNan)
{
  // A NaN would be undefined behaviour in the uint8_t cast, which on ARM
  // silently produces garbage colours rather than crashing.
  EXPECT_EQ(ws2812::ScaleChannel(200u, std::numeric_limits<float>::quiet_NaN()), 0u);
  EXPECT_EQ(ws2812::ScaleChannel(200u, std::numeric_limits<float>::infinity()), 0u);
}

TEST(Ws2812Encoder, EmptyPixelBufferStillEmitsTheResetGap)
{
  const auto bytes = ws2812::Encode({}, 1.0f);
  EXPECT_EQ(bytes.size(), ws2812::kResetLowBytes);
  for (const std::uint8_t byte : bytes)
  {
    EXPECT_EQ(byte, 0u);
  }
}

}  // namespace
}  // namespace mowgli_leds

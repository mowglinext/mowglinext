// Copyright 2026 MowgliNext contributors
//
// ParseIndicatorIds is the only free-text input the LED node accepts (the GUI
// forwards the operator's comma-separated list verbatim), so its tolerance
// rules are pinned here: well-formed tokens survive whitespace, and every
// malformed token is dropped on its own without taking the rest with it.

#include <cstddef>
#include <string>
#include <vector>

#include "mowgli_leds/indicator_ids.hpp"
#include <gtest/gtest.h>

namespace mowgli_leds
{
namespace
{

using Ids = std::vector<std::size_t>;

TEST(ParseIndicatorIds, WellFormedListSurvivesWhitespaceAndKeepsOrder)
{
  EXPECT_EQ(ParseIndicatorIds("0, 4, 8, 12"), (Ids{0u, 4u, 8u, 12u}));
  EXPECT_EQ(ParseIndicatorIds(" \t7\t,3,, 11 ,"), (Ids{7u, 3u, 11u}));
  EXPECT_EQ(ParseIndicatorIds("5"), (Ids{5u}));
  EXPECT_EQ(ParseIndicatorIds("2,2"), (Ids{2u, 2u}));  // duplicates are the renderer's business
}

TEST(ParseIndicatorIds, EmptyAndBlankInputsYieldNoIds)
{
  EXPECT_TRUE(ParseIndicatorIds("").empty());
  EXPECT_TRUE(ParseIndicatorIds("   ").empty());
  EXPECT_TRUE(ParseIndicatorIds(",,,").empty());
}

TEST(ParseIndicatorIds, MalformedTokensAreSkippedIndividually)
{
  // Negative, non-numeric, trailing garbage, floats, hex and an overflowing
  // literal are each dropped; the valid neighbours are kept.
  EXPECT_EQ(ParseIndicatorIds("-1, 4, abc, 8x, 2.5, 0x10, 12, 99999999999999999999, 3"),
            (Ids{4u, 12u, 3u}));
  EXPECT_TRUE(ParseIndicatorIds("-0").empty() || ParseIndicatorIds("-0") == Ids{0u});
  EXPECT_EQ(ParseIndicatorIds("+6"), (Ids{6u}));  // std::stol accepts an explicit plus sign
}

}  // namespace
}  // namespace mowgli_leds

// Copyright 2026 MowgliNext contributors
//
// Pure, header-only parser for the `led_charge_complete_indicator_ids`
// parameter. Kept out of the rclcpp translation unit so it can be unit-tested
// like the rest of the LED logic (see test/test_indicator_ids.cpp).

#pragma once

#include <cstddef>
#include <exception>
#include <sstream>
#include <string>
#include <vector>

namespace mowgli_leds
{

/// Parses a comma-separated list of non-negative pixel indices, e.g.
/// "0, 4, 8, 12". A malformed, negative, or out-of-range token is silently
/// skipped rather than rejecting the whole list -- a typo in one ID should
/// degrade that one pixel, not fall back to the count-based spacing for all
/// of them. Duplicates are kept; the renderer paints the same pixel twice.
inline std::vector<std::size_t> ParseIndicatorIds(const std::string& raw)
{
  std::vector<std::size_t> ids;
  std::stringstream stream(raw);
  std::string token;
  while (std::getline(stream, token, ','))
  {
    const auto first = token.find_first_not_of(" \t");
    if (first == std::string::npos)
    {
      continue;
    }
    const auto last = token.find_last_not_of(" \t");
    token = token.substr(first, last - first + 1);
    try
    {
      std::size_t consumed = 0;
      const long value = std::stol(token, &consumed);
      if (value >= 0 && consumed == token.size())
      {
        ids.push_back(static_cast<std::size_t>(value));
      }
    }
    catch (const std::exception&)
    {
      // Not a number, or does not fit in a long -- skip it, see above.
    }
  }
  return ids;
}

}  // namespace mowgli_leds

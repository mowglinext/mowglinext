// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <random>
#include <vector>

#include "gtest/gtest.h"
#include "mowgli_localization/deskew_history_cursor.hpp"

namespace
{
struct Sample
{
  double t_s;
  double value;
};

// Unmodified scalar interpolation from scan_deskew_node at dev 6445b2c01.
bool Scalar(const std::deque<Sample>& history, double t_s, double& out)
{
  if (history.empty())
  {
    out = 0.0;
    return false;
  }
  if (t_s <= history.front().t_s)
  {
    out = history.front().value;
    return false;
  }
  if (t_s >= history.back().t_s)
  {
    out = history.back().value;
    return false;
  }
  for (std::size_t i = 1; i < history.size(); ++i)
  {
    const auto& b = history[i];
    if (b.t_s >= t_s)
    {
      const auto& a = history[i - 1];
      const double dt = b.t_s - a.t_s;
      if (dt <= 0.0)
      {
        out = b.value;
        return true;
      }
      const double f = (t_s - a.t_s) / dt;
      out = a.value + f * (b.value - a.value);
      return true;
    }
  }
  out = history.back().value;
  return false;
}

void Compare(const std::deque<Sample>& history, const std::vector<double>& queries)
{
  mowgli_localization::DeskewHistoryCursor<Sample> cursor(history, &Sample::value);
  for (const double query : queries)
  {
    double expected = -123.0, actual = 456.0;
    ASSERT_EQ(Scalar(history, query, expected), cursor.Interpolate(query, actual)) << query;
    ASSERT_EQ(std::bit_cast<std::uint64_t>(expected), std::bit_cast<std::uint64_t>(actual))
        << query;
  }
}

TEST(DeskewHistoryCursor, EndpointsDuplicatesAndBackwardsQueries)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  const std::vector<double> queries{
      -inf, 0, 1, 1.5, 2, 2, 2.5, 3, 4, inf, 2.5, 2, 1.5, nan, 1.5, 2.5, 1, 3};
  for (const std::deque<Sample>& history :
       std::vector<std::deque<Sample>>{{},
                                       {{1, -0.0}},
                                       {{1, 2}, {2, 4}, {2, 8}, {3, 16}},
                                       {{1, 2}, {2.5, 4}, {1.5, 8}, {3, 16}},
                                       {{1, 2}, {nan, 4}, {2, 8}, {3, 16}}})
    Compare(history, queries);
}

TEST(DeskewHistoryCursor, SeededMonotoneReverseAndUnorderedHistories)
{
  std::mt19937 rng(20261113);
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  for (int seed = 0; seed < 500; ++seed)
  {
    std::deque<Sample> history;
    double t = 1000.0;
    for (unsigned k = 0, n = rng() % 201; k < n; ++k)
    {
      t += unit(rng) * 0.02;
      history.push_back({t, unit(rng) * 2.0 - 1.0});
      if (k > 0 && k % 11 == 0)
        history.back().t_s = history[k - 1].t_s;
      if (seed % 3 == 0 && k % 7 == 0)
        history.back().t_s -= 0.1;
    }
    std::vector<double> queries;
    for (unsigned k = 0; k < 1440; ++k)
      queries.push_back(999.9 + k * 0.002);
    Compare(history, queries);
    std::reverse(queries.begin(), queries.end());
    Compare(history, queries);
    std::shuffle(queries.begin(), queries.end(), rng);
    Compare(history, queries);
  }
}

TEST(DeskewHistoryCursor, ANewScanStartsAfterBufferTrim)
{
  std::deque<Sample> history{{1, 1}, {2, 2}, {3, 3}, {4, 4}};
  Compare(history, {3.5});
  history.pop_front();
  history.pop_front();
  history.push_back({5, 5});
  Compare(history, {3.1, 3.5, 4.5});
}
}  // namespace

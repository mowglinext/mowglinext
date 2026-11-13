// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#include <array>
#include <cmath>
#include <random>
#include <vector>

#include "footprint_reference.hpp"
#include "gtest/gtest.h"

namespace
{
using mowgli_nav2_plugins::BoundaryGuard;
using mowgli_nav2_plugins::ObstacleDeviation;

ObstacleDeviation::Footprint Body(bool concave)
{
  ObstacleDeviation::Footprint body;
  for (const auto& xy : std::vector<std::pair<double, double>>{
           {0.54, 0.335}, {0.54, -0.335}, {-0.18, -0.335}, {-0.18, 0.335}})
  {
    geometry_msgs::msg::Point p;
    p.x = xy.first;
    p.y = xy.second;
    body.push_back(p);
  }
  if (concave)
  {
    geometry_msgs::msg::Point notch;
    notch.x = 0.10;
    body.insert(body.begin() + 1, notch);
  }
  return body;
}

TEST(FootprintRasterParity, SeededCellsThresholdsGuardsAndTranslatedGrids)
{
  // Includes normal grid sizes and origins at which cell centres collapse or
  // map back to a neighbouring cell. The latter must use the scalar path.
  std::mt19937 rng(20261113);
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  for (const double origin : {-3.0, -1000000.0, 1e12, 1e15})
  {
    for (const double res : {0.025, 0.05, 0.08})
    {
      nav2_costmap_2d::Costmap2D local(96, 80, res, origin, origin, 0);
      nav2_costmap_2d::Costmap2D zone(96, 80, res, origin, origin, 0);
      for (unsigned int y = 0; y < 80; ++y)
      {
        for (unsigned int x = 0; x < 96; ++x)
        {
          constexpr std::array<unsigned char, 7> costs{0, 1, 50, 100, 253, 254, 255};
          local.setCost(x, y, costs[rng() % costs.size()]);
          zone.setCost(x, y, rng() % 5 == 0 ? 254 : 0);
        }
      }
      for (int q = 0; q < 1000; ++q)
      {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = origin - 0.5 + unit(rng) * (96 * res + 1.0);
        pose.pose.position.y = origin - 0.5 + unit(rng) * (80 * res + 1.0);
        const double yaw = unit(rng) * 6.283185307179586;
        pose.pose.orientation.z = std::sin(yaw / 2);
        pose.pose.orientation.w = std::cos(yaw / 2);
        BoundaryGuard guard;
        if (q % 5 == 0)
          guard.costmap = &zone;
        if (q % 11 == 0)
          guard.deny_all = true;
        if (q % 5 == 0 && origin == -3.0)
        {
          guard.tx = 0.1;
          guard.ty = -0.1;
          guard.cos_yaw = std::cos(0.3);
          guard.sin_yaw = std::sin(0.3);
        }
        const auto body = Body(q % 2 == 0);
        const unsigned char threshold =
            std::array<unsigned char, 5>{0, 1, 253, 254, 255}[(q / 5) % 5];
        const double offset = unit(rng) * 2.0 - 1.0;
        ASSERT_EQ(
            footprint_reference::FootprintBlocked(local, pose, offset, body, guard, threshold),
            ObstacleDeviation::footprintBlocked(local, pose, offset, body, guard, threshold))
            << "origin=" << origin << " resolution=" << res << " query=" << q;
      }
    }
  }
}

TEST(FootprintRasterParity, EveryCellAlongRotatedAndOffGridBodies)
{
  nav2_costmap_2d::Costmap2D local(32, 24, 0.05, -0.8, -0.6, 0);
  for (double yaw : {0.0, 0.3, 1.5707963267948966, -2.1})
  {
    for (double x : {-0.9, 0.0, 0.9})
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = x;
      pose.pose.orientation.z = std::sin(yaw / 2);
      pose.pose.orientation.w = std::cos(yaw / 2);
      for (unsigned y = 0; y < 24; ++y)
      {
        for (unsigned col = 0; col < 32; ++col)
        {
          local.setCost(col, y, 254);
          ASSERT_EQ(footprint_reference::FootprintBlocked(local, pose, 0.0, Body(false), {}, 254),
                    ObstacleDeviation::footprintBlocked(local, pose, 0.0, Body(false), {}, 254));
          local.setCost(col, y, 0);
        }
      }
    }
  }
}
}  // namespace

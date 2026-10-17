// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <vector>

namespace mowgli_interfaces::motion
{
struct Point
{
  double x, y;
};
using Ring = std::vector<Point>;
struct Pose
{
  double x, y, yaw;
};
}  // namespace mowgli_interfaces::motion

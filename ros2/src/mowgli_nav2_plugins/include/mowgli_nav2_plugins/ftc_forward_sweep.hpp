// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include "geometry_msgs/msg/twist.hpp"
#include "mowgli_interfaces/motion_geometry_types.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace mowgli_interfaces::motion
{
struct Snapshot;
}

namespace mowgli_nav2_plugins
{
// Forward-only execution envelope. Assumes no speed/yaw overshoot beyond the
// measured/proposed extrema during command_horizon, then remaining integrals
// of speed/yaw are bounded by coast_time_bound times those extrema. This covers
// first-order zero-command decay with tau<=coast_time_bound, but is an ASSUMED
// actuator contract, not a physical stopping guarantee or a command lease.
bool ForwardSweepBlocked(const nav2_costmap_2d::Costmap2D& costmap,
                         const mowgli_interfaces::motion::Ring& footprint,
                         mowgli_interfaces::motion::Pose pose,
                         const geometry_msgs::msg::Twist& proposed,
                         const geometry_msgs::msg::Twist& measured,
                         double command_horizon,
                         double coast_time_bound);

// The same assumed forward response against recorded geometry: the reference
// stays in the mowing union, while only raw holes constrain the body. STOP and
// non-forward proposals retain their separate authorization/rotation guards.
bool ForwardResponsePermitted(const mowgli_interfaces::motion::Snapshot& geometry,
                              const mowgli_interfaces::motion::Ring& footprint,
                              mowgli_interfaces::motion::Pose pose,
                              const geometry_msgs::msg::Twist& proposed,
                              const geometry_msgs::msg::Twist& measured,
                              double command_horizon,
                              double coast_time_bound);
}  // namespace mowgli_nav2_plugins

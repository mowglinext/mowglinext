// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Small shared helpers for the fusion_graph_node translation units. The node
// implementation is split across several .cpp files to stay within the
// project's 600-line-per-file budget; these inline helpers (formerly an
// anonymous namespace in fusion_graph_node.cpp) are shared by all of them.

#pragma once

#include <cmath>

#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace fusion_graph
{

constexpr double kEarthRadius = 6378137.0;  // WGS84 equatorial, metres.

// Extract yaw from a geometry_msgs Quaternion.
inline double YawFromQuat(const geometry_msgs::msg::Quaternion& q)
{
  tf2::Quaternion tq(q.x, q.y, q.z, q.w);
  double r, p, y;
  tf2::Matrix3x3(tq).getRPY(r, p, y);
  return y;
}

inline geometry_msgs::msg::Quaternion QuatFromYaw(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  geometry_msgs::msg::Quaternion m;
  m.x = q.x();
  m.y = q.y();
  m.z = q.z();
  m.w = q.w();
  return m;
}

// Forward-propagate a dead-reckoned pose (x, y, yaw) by `lead` seconds under a
// constant-velocity model (yaw rate gz, body-forward velocity vx_eff). Used so
// a TF stamped at now()+lead is an HONEST prediction of the pose at that
// instant instead of the current pose mislabelled into the future — the latter
// injects ~wz·lead of yaw error during pivots (≈4° at 1.5 rad/s with a 50 ms
// lead), which fed straight into the controller's heading tracking. Mirrors the
// DR integration order (yaw first, then translate along the updated heading).
inline void ForwardStampPose(
    double lead, double gz, double vx_eff, double& x, double& y, double& yaw)
{
  if (lead <= 0.0)
    return;
  yaw += gz * lead;
  x += vx_eff * std::cos(yaw) * lead;
  y += vx_eff * std::sin(yaw) * lead;
}

}  // namespace fusion_graph

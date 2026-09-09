// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <Eigen/Geometry>

namespace mowgli_nav2_plugins
{

/// Build the command carrot from its canonical path pose. Returning a copy is
/// deliberate: obstacle deviation must never accumulate in the nominal carrot
/// that advances along the coverage path on the following control tick.
inline Eigen::Affine3d LaterallyDeviatedCarrot(const Eigen::Affine3d& nominal,
                                               const double lateral_deviation)
{
  Eigen::Affine3d deviated = nominal;
  deviated.translation() += deviated.linear() * Eigen::Vector3d(0.0, lateral_deviation, 0.0);
  return deviated;
}

}  // namespace mowgli_nav2_plugins

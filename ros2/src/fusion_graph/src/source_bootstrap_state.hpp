// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "source_bootstrap_builder.hpp"

namespace fusion_graph
{
// A future consumed endpoint belongs to this transaction, never to the public
// cadence index. Once promoted, the family retains the first public S..V edge
// and its private acquisitions until that entire edge is marginalized.
struct SourceBootstrapState
{
  bool provisional = true;
  int64_t initial_end_ns = 0;
  SourceBootstrapBuildRequest request;
  std::shared_ptr<SharedSourceProcessFamily> family;
  std::map<gtsam::Key, int64_t> auxiliary_poses;
};
}  // namespace fusion_graph

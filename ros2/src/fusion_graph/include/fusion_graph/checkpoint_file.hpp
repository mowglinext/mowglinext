// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <string>

namespace fusion_graph::checkpoint
{
struct Payload
{
  std::string graph;
  std::string metadata;
};

// One writer per prefix (the node's async-save guard owns serialization).
// Every published file contains a complete graph/metadata generation.
// False after publication means durability is uncertain; the previous
// generation remains available, and either complete generation may recover.
// backup_current=false protects a recovery file when the caller's semantic
// validation (beyond framing/checksum) rejects the current generation.
bool Save(const std::string& prefix, const Payload& payload, bool backup_current = true);
bool Read(const std::string& path, Payload& payload);
bool Clear(const std::string& prefix);
}  // namespace fusion_graph::checkpoint

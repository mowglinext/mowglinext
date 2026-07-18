// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// In-place scalar update for the runtime mowgli_robot.yaml. Extracted
// (task #42, from the #40 dock-calibration audit) from three independent,
// byte-for-byte-identical copies of the same splice algorithm that had
// accumulated in mowgli_localization/calibrate_imu_yaw_node.cpp,
// mowgli_map/area_manager.cpp, and mowgli_behavior/calibration_nodes.cpp —
// all three of which write dock_pose_x/y/yaw and all three of which already
// depend on mowgli_interfaces, making it a natural shared home alongside
// gnss_status_utils.hpp.
//
// Rewrites scalar values via per-line substring splicing rather than a YAML
// library round-trip, so comments and surrounding structure are preserved
// (mowgli_robot.yaml is a hand-maintained, heavily-commented config — see
// CLAUDE.md Inv 15). Writes are atomic via tmp+rename.

#pragma once

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <system_error>

namespace mowgli_interfaces::robot_yaml_scalar
{

// Splice a new numeric value into a "<indent><key>:<spaces><number><rest>"
// line, anchored on the indent so a key whose name happens to contain ours
// (e.g. dock_pose_x_offset) is not matched. Returns true if the key was
// found and replaced, false if no matching line exists (content left
// unchanged in that case).
inline bool SpliceScalar(std::string& content, const std::string& key, const std::string& new_value)
{
  size_t scan = 0;
  while (scan < content.size())
  {
    const size_t line_start = scan;
    size_t cursor = line_start;
    while (cursor < content.size() && (content[cursor] == ' ' || content[cursor] == '\t'))
      ++cursor;
    const size_t indent_end = cursor;
    if (indent_end > line_start && cursor + key.size() < content.size() &&
        content.compare(cursor, key.size(), key) == 0 && content[cursor + key.size()] == ':')
    {
      cursor += key.size() + 1;
      while (cursor < content.size() && (content[cursor] == ' ' || content[cursor] == '\t'))
        ++cursor;
      const size_t val_start = cursor;
      while (cursor < content.size())
      {
        const char c = content[cursor];
        const bool is_num =
            (c >= '0' && c <= '9') || c == '.' || c == '-' || c == '+' || c == 'e' || c == 'E';
        if (!is_num)
          break;
        ++cursor;
      }
      if (cursor > val_start)
      {
        content.replace(val_start, cursor - val_start, new_value);
        return true;
      }
    }
    const size_t nl = content.find('\n', line_start);
    if (nl == std::string::npos)
      break;
    scan = nl + 1;
  }
  return false;
}

// Format a double at the fixed 6-decimal precision used for the persisted
// dock_pose_x/y/yaw values.
inline std::string FormatScalar(double v)
{
  std::ostringstream s;
  s << std::fixed << std::setprecision(6) << v;
  return s.str();
}

// Read `content`, tmp-write it back with `edit` applied, then atomically
// rename over `path`. Shared by UpdateDockPose and PersistScalar below so
// the file I/O + atomicity dance lives in exactly one place.
template <typename EditFn>
inline bool ReadEditWrite(const std::string& path, EditFn&& edit)
{
  std::ifstream in(path);
  if (!in.good())
    return false;
  std::stringstream buf;
  buf << in.rdbuf();
  std::string content = buf.str();
  in.close();

  if (!edit(content))
    return false;

  const std::string tmp_path = path + ".tmp";
  {
    std::ofstream out(tmp_path, std::ios::trunc);
    if (!out.good())
      return false;
    out << content;
    if (!out.good())
      return false;
  }
  std::error_code ec;
  std::filesystem::rename(tmp_path, path, ec);
  return !ec;
}

// Update dock_pose_x/y/yaw in `path` in place. Used by calibrate_imu_yaw_node
// (dock pre-phase) and map_server's on_set_docking_point (GUI "pin dock").
// NOTE on behavior preservation: a key that isn't found in the file is a
// silent no-op for that key (SpliceScalar's per-key bool is intentionally
// ignored) — this matches the exact behavior of the two original per-file
// copies this replaces. Only the overall file read/write success gates the
// return value.
inline bool UpdateDockPose(const std::string& path, double x, double y, double yaw_rad)
{
  return ReadEditWrite(path,
                       [&](std::string& content)
                       {
                         SpliceScalar(content, "dock_pose_x", FormatScalar(x));
                         SpliceScalar(content, "dock_pose_y", FormatScalar(y));
                         SpliceScalar(content, "dock_pose_yaw", FormatScalar(yaw_rad));
                         return true;
                       });
}

// Splice a single scalar `key` to `value` in `path` and persist. Used by
// CalibrateHeadingFromUndock's persist_dock_pose_yaw (yaw-only EMA writeback).
// NOTE on behavior preservation: unlike UpdateDockPose above, a missing key
// IS treated as failure here — this matches the original per-file behavior
// of calibration_nodes.cpp's copy, which checked SpliceScalar's return value.
// The two writers are deliberately left with their original, different
// failure semantics rather than unified, since unifying them would be a
// behavior change beyond this consolidation's scope.
inline bool PersistScalar(const std::string& path, const std::string& key, double value)
{
  return ReadEditWrite(path,
                       [&](std::string& content)
                       {
                         return SpliceScalar(content, key, FormatScalar(value));
                       });
}

}  // namespace mowgli_interfaces::robot_yaml_scalar

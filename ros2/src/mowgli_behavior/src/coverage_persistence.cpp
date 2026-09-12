// Copyright 2026 Mowgli Project
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include "mowgli_behavior/coverage_persistence.hpp"

#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <map>
#include <set>
#include <sstream>
#include <string>

#include "mowgli_behavior/bt_context.hpp"

namespace mowgli_behavior
{

namespace
{
// Bump when the on-disk layout changes incompatibly; an unrecognised header is
// treated as "no state" (start fresh) rather than a parse error.
constexpr const char* kHeader = "mowgli_coverage_resume v2";
// The phase-only snapshot never carries a command or cursor, so it cannot
// auto-start the mower. Older readers ignore these optional rows.
void writeCrossHatch(std::ostream& out, const std::map<uint32_t, CrossHatch>& areas)
{
  for (const auto& [index, state] : areas)
  {
    out << "cross_hatch_area " << index << ' ' << state.next_perpendicular << ' '
        << (state.session_perpendicular ? static_cast<int>(*state.session_perpendicular) : -1)
        << ' ' << state.alternate_session << ' ' << state.used << ' '
        << (state.next_override ? static_cast<int>(*state.next_override) : -1) << ' '
        << state.failed_sessions << ' ' << state.planning_failed << '\n';
  }
}

bool writeSnapshot(const BTContext& ctx, const std::string& contents)
{
  // Atomic replace: write a sibling temp file then rename over the target so a
  // reader never sees a half-written file (mirrors persist_dock_pose_yaw).
  const std::string tmp_path = ctx.coverage_resume_path + ".tmp";
  {
    std::ofstream f(tmp_path, std::ios::trunc);
    if (!f)
    {
      return false;
    }
    f << contents;
    f.flush();
    if (!f)
    {
      return false;
    }
  }
  return std::rename(tmp_path.c_str(), ctx.coverage_resume_path.c_str()) == 0;
}
}  // namespace

bool beginCoverageOrientation(BTContext& ctx, uint32_t area)
{
  auto it = ctx.cross_hatch.find(area);
  if (it == ctx.cross_hatch.end() &&
      (!ctx.mow_cross_hatch || ctx.base_orientation_areas.count(area)))
  {
    ctx.base_orientation_areas.insert(area);
    return false;
  }
  if (it == ctx.cross_hatch.end())
    it = ctx.cross_hatch.emplace(area, CrossHatch{}).first;
  const bool first = !it->second.session_perpendicular.has_value();
  const bool perpendicular =
      it->second.begin(ctx.mow_cross_hatch && !ctx.base_orientation_areas.count(area));
  if (first && !ctx.coverage_resume_path.empty() && !saveCoverageResumeState(ctx) && ctx.node)
    RCLCPP_WARN(ctx.node->get_logger(),
                "Cross-hatch area %u: could not persist session orientation to '%s'",
                area,
                ctx.coverage_resume_path.c_str());
  return perpendicular;
}

void markCoverageStarted(BTContext& ctx, uint32_t area)
{
  const auto it = ctx.cross_hatch.find(area);
  if (it == ctx.cross_hatch.end() || !it->second.session_perpendicular || it->second.used)
    return;
  it->second.used = true;
  it->second.failed_sessions = 0;
  it->second.planning_failed = false;
  if (!ctx.coverage_resume_path.empty() && !saveCoverageResumeState(ctx) && ctx.node)
    RCLCPP_WARN(ctx.node->get_logger(),
                "Cross-hatch area %u: could not persist coverage start to '%s'",
                area,
                ctx.coverage_resume_path.c_str());
}

bool saveCoverageResumeState(const BTContext& ctx)
{
  if (ctx.coverage_resume_path.empty())
  {
    return false;
  }

  // Union of every area index that carries any resume-relevant state.
  std::set<uint32_t> areas;
  for (const auto& [idx, _] : ctx.area_path_pose_count)
    areas.insert(idx);
  for (const auto& [idx, _] : ctx.area_plan_fingerprint)
    areas.insert(idx);
  for (const auto& [idx, _] : ctx.area_resume_pose_index)
    areas.insert(idx);
  for (const auto& [idx, _] : ctx.area_completed_swaths)
    areas.insert(idx);
  for (uint32_t idx : ctx.completed_areas)
    areas.insert(idx);

  std::ostringstream out;
  out << kHeader << '\n';
  writeCrossHatch(out, ctx.cross_hatch);
  for (const auto area : ctx.base_orientation_areas)
    out << "base_orientation_area " << area << '\n';
  // The active high-level command (COMMAND_START etc.). Persisted so a mid-run
  // container restart can auto-re-enter MowingSequence instead of coming up IDLE
  // (current_command defaults to 0). Cast through unsigned so the uint8_t is
  // written as a number, not a raw char. EndSession clears this command and
  // the cursors, retaining only cross-hatch metadata when needed.
  out << "current_command " << static_cast<unsigned>(ctx.current_command) << '\n';
  // Single-area mode (a ~/start_in_area targeted run). Persisted for the same
  // reason as current_command: a restart mid-run auto-re-enters MowingSequence,
  // and without this the restored run would silently widen into a mow-the-whole-
  // lawn run the moment the targeted area finished. Absent line = not targeted.
  if (ctx.single_area_target.has_value())
  {
    out << "single_area_target " << *ctx.single_area_target << '\n';
  }
  out << "current_area " << ctx.current_area << '\n';
  out << "completed_areas";
  for (uint32_t idx : ctx.completed_areas)
    out << ' ' << idx;
  out << '\n';
  for (uint32_t idx : areas)
  {
    std::size_t pose_count = 0;
    if (auto it = ctx.area_path_pose_count.find(idx); it != ctx.area_path_pose_count.end())
      pose_count = it->second;
    // Plan-geometry fingerprint (0 = none recorded). The resume-cursor staleness
    // key: a persisted cursor is only reused when this matches the re-planned
    // geometry (see FollowStrip::onStart / hashPlanGeometry).
    uint64_t fingerprint = 0;
    if (auto it = ctx.area_plan_fingerprint.find(idx); it != ctx.area_plan_fingerprint.end())
      fingerprint = it->second;
    // -1 sentinel = no live resume cursor (area finished or never interrupted).
    int64_t resume = -1;
    if (auto it = ctx.area_resume_pose_index.find(idx); it != ctx.area_resume_pose_index.end())
      resume = static_cast<int64_t>(it->second);

    out << "area " << idx << ' ' << pose_count << ' ' << fingerprint << ' ' << resume
        << " completed";
    if (auto it = ctx.area_completed_swaths.find(idx); it != ctx.area_completed_swaths.end())
      for (std::size_t s : it->second)
        out << ' ' << s;
    out << '\n';
  }

  return writeSnapshot(ctx, out.str());
}

bool loadCoverageResumeState(BTContext& ctx)
{
  if (ctx.coverage_resume_path.empty())
  {
    return false;
  }
  std::ifstream f(ctx.coverage_resume_path);
  if (!f)
  {
    return false;
  }

  std::string header;
  if (!std::getline(f, header) || header != kHeader)
  {
    return false;  // absent / empty / unknown version → start fresh
  }

  std::string line;
  while (std::getline(f, line))
  {
    std::istringstream ls(line);
    std::string tag;
    if (!(ls >> tag))
    {
      continue;
    }
    if (tag == "cross_hatch_area")
    {
      uint32_t index;
      int next, active, alternate, used, override_next;
      if (ls >> index >> next >> active >> alternate >> used >> override_next &&
          (next == 0 || next == 1) && active >= -1 && active <= 1 &&
          (alternate == 0 || alternate == 1) && (used == 0 || used == 1) && override_next >= -1 &&
          override_next <= 1)
      {
        auto& state = ctx.cross_hatch[index];
        state.next_perpendicular = next != 0;
        state.session_perpendicular = active < 0 ? std::nullopt : std::optional<bool>(active != 0);
        state.alternate_session = alternate != 0;
        state.used = used != 0;
        state.next_override =
            override_next < 0 ? std::nullopt : std::optional<bool>(override_next != 0);
        unsigned failures = 0;
        int failed = 0;
        if (ls >> failures >> failed && failures <= 1000 && (failed == 0 || failed == 1))
        {
          state.failed_sessions = failures;
          state.planning_failed = failed != 0;
        }
      }
    }
    else if (tag == "base_orientation_area")
    {
      uint32_t area;
      if (ls >> area)
        ctx.base_orientation_areas.insert(area);
    }
    else if (tag == "current_command")
    {
      // Backward-compatible: older files predate this line, so an absent tag
      // simply leaves current_command at its default (0 / IDLE).
      unsigned v;
      if (ls >> v)
        ctx.current_command = static_cast<uint8_t>(v);
    }
    else if (tag == "single_area_target")
    {
      // Absent in files written before targeted runs became session state (and
      // in every non-targeted run) → single_area_target stays empty, i.e. the
      // normal all-areas iteration.
      uint32_t v;
      if (ls >> v)
        ctx.single_area_target = v;
    }
    else if (tag == "current_area")
    {
      int v;
      if (ls >> v)
        ctx.current_area = v;
    }
    else if (tag == "completed_areas")
    {
      uint32_t idx;
      while (ls >> idx)
        ctx.completed_areas.insert(idx);
    }
    else if (tag == "area")
    {
      uint32_t idx;
      std::size_t pose_count;
      uint64_t fingerprint;
      int64_t resume;
      std::string completed_tag;
      if (!(ls >> idx >> pose_count >> fingerprint >> resume >> completed_tag))
      {
        continue;  // malformed row — skip, don't abort the whole load
      }
      ctx.area_path_pose_count[idx] = pose_count;
      if (fingerprint != 0)
      {
        ctx.area_plan_fingerprint[idx] = fingerprint;
      }
      if (resume >= 0)
      {
        ctx.area_resume_pose_index[idx] = static_cast<std::size_t>(resume);
      }
      auto& done = ctx.area_completed_swaths[idx];
      std::size_t s;
      while (ls >> s)
        done.insert(s);
    }
  }
  return true;
}

bool clearCoverageResumeState(const BTContext& ctx)
{
  if (ctx.coverage_resume_path.empty())
  {
    return true;
  }
  // Invalidate the old START command/cursors BEFORE trying to retain metadata.
  // An atomic replacement alone leaves the active snapshot intact if the disk
  // is full or the temp file cannot be written. A restart would then resume a
  // session which has already ended. Losing phase history is safer than that.
  if (std::remove(ctx.coverage_resume_path.c_str()) != 0 && errno != ENOENT)
  {
    return false;
  }
  if (!ctx.cross_hatch.empty() || !ctx.base_orientation_areas.empty())
  {
    std::ostringstream out;
    out << kHeader << '\n';
    writeCrossHatch(out, ctx.cross_hatch);
    for (const auto area : ctx.base_orientation_areas)
      out << "base_orientation_area " << area << '\n';
    return writeSnapshot(ctx, out.str());
  }
  // A leftover temp file is never loaded; cleanup is best-effort.
  std::remove((ctx.coverage_resume_path + ".tmp").c_str());
  return true;
}

}  // namespace mowgli_behavior

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
//
// CRUD for LidarIgnoreCorridorEntry — see its doc comment
// (map_server_node.hpp) for what the concept is and why it is kept
// deliberately separate from AreaEntry/area_manager.cpp: a corridor is not
// classified, is not part of any keepout mask, and its only consumer is
// costmap_scan_filter_node over lidar_ignore_corridors_pub_.

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <mutex>
#include <set>

#include "mowgli_map/internal_helpers.hpp"
#include "mowgli_map/map_server_node.hpp"

namespace mowgli_map
{
namespace
{
using Replace = mowgli_interfaces::srv::ReplaceLidarIgnoreCorridors;
using Corridor = mowgli_interfaces::msg::LidarIgnoreCorridor;

/// Names are normalised (surrounding whitespace trimmed) on every input path —
/// add, replace and load — instead of being rejected, so a legacy name with a
/// trailing blank can never make a whole-list GUI replace fail validation.
Corridor normalised(Corridor c)
{
  c.name = trim_ascii_whitespace(c.name);
  return c;
}

/// nullptr when `c` (already normalised) is valid, else why it is not.
const char* corridor_rejection(const Corridor& c)
{
  if (c.polyline.points.size() < 2)
    return "polyline has fewer than 2 points";
  if (!std::isfinite(c.width_m))
    return "width_m is not finite";
  if (c.name.find_first_of("\r\n") != std::string::npos || c.name.find('\0') != std::string::npos)
    return "name contains a line break or NUL";
  bool segment = false;
  for (size_t i = 0; i < c.polyline.points.size(); ++i)
  {
    const auto& p = c.polyline.points[i];
    // areas.dat is explicitly 2D. Never acknowledge a Z coordinate that the writer drops.
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) || p.z != 0)
      return "point is not finite or has a non-zero z";
    if (i)
    {
      const auto& prev = c.polyline.points[i - 1];
      const double dx = double{p.x} - prev.x, dy = double{p.y} - prev.y;
      segment |= dx * dx + dy * dy > 1e-12;  // same degeneracy scale as the consumer
    }
  }
  return segment ? nullptr : "polyline has no non-degenerate segment";
}

/// True when committing `candidate` can only make the scan filter suppress
/// FEWER beams than `current`: every candidate entry is an existing corridor
/// (same id, same polyline) at the same or a narrower width. Clear and a
/// replace that only drops lines qualify; any new, re-shaped or widened line
/// does not. Names and order do not affect what the filter drops.
template <class Entries>
bool only_removes_suppression(const Entries& candidate, const Entries& current)
{
  return std::all_of(candidate.begin(),
                     candidate.end(),
                     [&](const auto& next)
                     {
                       return std::any_of(current.begin(),
                                          current.end(),
                                          [&](const auto& now)
                                          {
                                            return now.id == next.id &&
                                                   now.polyline == next.polyline &&
                                                   next.width_m <= now.width_m;
                                          });
                     });
}
}  // namespace

std::vector<mowgli_interfaces::msg::LidarIgnoreCorridor> MapServerNode::corridor_snapshot() const
{
  std::lock_guard<std::recursive_mutex> state_lock(areas_state_mutex_);
  std::vector<mowgli_interfaces::msg::LidarIgnoreCorridor> result;
  result.reserve(lidar_ignore_corridors_.size());
  for (const auto& e : lidar_ignore_corridors_)
  {
    mowgli_interfaces::msg::LidarIgnoreCorridor c;
    c.name = e.name;
    c.polyline = e.polyline;
    c.width_m = e.width_m;
    c.id = e.id;
    result.push_back(std::move(c));
  }
  return result;
}

void MapServerNode::commit_corridors(std::vector<LidarIgnoreCorridorEntry> candidate,
                                     uint64_t next_id,
                                     Replace::Response& result,
                                     bool require_persistence)
{
  // All serialized-state writers hold this lock before map_mutex_. No shared .tmp collision,
  // no stale area/obstacle/coverage/datum snapshot, and one publication order per instance.
  std::lock_guard<std::recursive_mutex> state_lock(areas_state_mutex_);
  result.server_instance_id = corridor_server_instance_id_;
  result.revision = corridor_revision_;
  result.corridors = corridor_snapshot();
  if (require_persistence && areas_file_path_.empty())
  {
    result.outcome = "PERSISTENCE_NOT_CONFIGURED";
    result.message = "areas_file_path is empty; no persistent replacement was applied";
    return;
  }
  // Allocate response/publication BEFORE Rename. Only swaps and scalar writes install the
  // prepared state. A publisher exception is handled after commit, never as a rollback.
  mowgli_interfaces::msg::LidarIgnoreCorridorArray message;
  message.header.frame_id = "map";
  message.header.stamp = get_clock()->now();
  message.corridors.reserve(candidate.size());
  for (const auto& e : candidate)
  {
    mowgli_interfaces::msg::LidarIgnoreCorridor c;
    c.name = e.name;
    c.polyline = e.polyline;
    c.width_m = e.width_m;
    c.id = e.id;
    message.corridors.push_back(std::move(c));
  }
  auto canonical = message.corridors;
  bool durable = false;
  std::string write_error;
  try
  {
    if (!areas_file_path_.empty())
      durable = write_areas_candidate(areas_file_path_, false, candidate, next_id);
  }
  catch (const std::exception& e)
  {
    write_error = e.what();
  }
  if (!write_error.empty())
  {
    // A corridor makes collision_monitor blind along its line. A change that only
    // REMOVES suppression must therefore never be blocked by the disk (full SD
    // card): it takes effect in memory and is published, and the file keeps the
    // old list until the next successful areas.dat write (every writer serializes
    // the live corridor list). Additions and widenings stay all-or-nothing.
    if (!only_removes_suppression(candidate, lidar_ignore_corridors_))
    {
      result.outcome = "PERSISTENCE_FAILED_BEFORE_COMMIT";
      result.message = write_error;
      return;
    }
    RCLCPP_WARN(get_logger(),
                "Corridor removal applied in memory but NOT persisted (%s); %s still holds the "
                "previous list until the next successful map save, and a restart before then "
                "restores the removed line(s).",
                write_error.c_str(),
                areas_file_path_.c_str());
  }
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    lidar_ignore_corridors_.swap(candidate);
    next_lidar_corridor_id_ = next_id;
  }
  result.corridors.swap(canonical);
  result.committed = true;
  result.persisted = durable;
  result.revision = ++corridor_revision_;
  corridor_publication_pending_ = true;
  try
  {
    lidar_ignore_corridors_pub_->publish(message);
    corridor_publication_pending_ = false;
    result.published = true;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(get_logger(), "Corridor commit completed; publication pending: %s", e.what());
  }
  if (!write_error.empty())
  {
    result.outcome = "COMMITTED_DURABILITY_UNCONFIRMED";
    result.message = std::string("removal committed in memory") +
                     (result.published ? " and published" : ", publication pending") +
                     "; areas.dat write failed (" + write_error +
                     "), durability unconfirmed — a restart before the next successful save "
                     "restores it";
  }
  else if (!durable && !areas_file_path_.empty())
  {
    result.outcome = "COMMITTED_DURABILITY_UNCONFIRMED";
    result.message = "replacement committed; directory sync failed, durability unconfirmed";
  }
  else if (!result.published)
  {
    result.outcome = "COMMITTED_PUBLICATION_PENDING";
    result.message =
        "replacement committed; publication will be retried without replaying the write";
  }
  else
  {
    result.success = true;
    result.outcome = "COMMITTED";
    result.message =
        areas_file_path_.empty() ? "legacy memory-only operation" : "replacement persisted";
  }
}

void MapServerNode::on_replace_lidar_ignore_corridors(const Replace::Request::SharedPtr req,
                                                      Replace::Response::SharedPtr res)
{
  std::lock_guard<std::recursive_mutex> state_lock(areas_state_mutex_);
  res->client_operation_id = req->client_operation_id;
  res->server_instance_id = corridor_server_instance_id_;
  res->revision = corridor_revision_;
  res->corridors = corridor_snapshot();
  res->outcome = "VALIDATION_REJECTED";
  res->message = "invalid 2D corridor, duplicate/exhausted ID, or staging payload exceeds 64 KiB";
  size_t remaining = Replace::Request::MAX_PAYLOAD_BYTES;
  if (req->client_operation_id.size() > remaining ||
      req->corridors.size() > remaining / sizeof(LidarIgnoreCorridorEntry))
    return;
  remaining -= req->client_operation_id.size();
  std::set<uint32_t> ids;
  // Reserve ALL explicit identities before minting even the first zero ID.
  for (const auto& c : req->corridors)
  {
    if (sizeof(LidarIgnoreCorridorEntry) > remaining)
      return;
    remaining -= sizeof(LidarIgnoreCorridorEntry);
    if (c.name.size() > remaining)
      return;
    remaining -= c.name.size();
    if (c.polyline.points.size() > remaining / sizeof(geometry_msgs::msg::Point32))
      return;
    remaining -= c.polyline.points.size() * sizeof(geometry_msgs::msg::Point32);
    const auto normal = normalised(c);
    if (const char* reason = corridor_rejection(normal))
    {
      res->message = std::string("invalid corridor '") + normal.name + "': " + reason;
      RCLCPP_WARN(get_logger(), "Rejected corridor replacement: %s", res->message.c_str());
      return;
    }
    if (c.id && !ids.insert(c.id).second)
    {
      res->message = "duplicate corridor id " + std::to_string(c.id);
      RCLCPP_WARN(get_logger(), "Rejected corridor replacement: %s", res->message.c_str());
      return;
    }
  }
  uint64_t next = next_lidar_corridor_id_;
  for (auto id : ids)
    next = std::max(next, uint64_t{id} + 1);
  std::vector<LidarIgnoreCorridorEntry> candidate;
  candidate.reserve(req->corridors.size());
  for (const auto& c : req->corridors)
  {
    uint64_t id = c.id;
    if (!id)
    {
      if (next > UINT32_MAX)
        return;
      id = next++;
    }
    candidate.push_back(
        {trim_ascii_whitespace(c.name),
         c.polyline,
         std::clamp(c.width_m, kMinLidarIgnoreCorridorWidthM, kMaxLidarIgnoreCorridorWidthM),
         static_cast<uint32_t>(id)});
  }
  commit_corridors(std::move(candidate), next, *res, true);
}

void MapServerNode::on_add_lidar_ignore_corridor(
    const mowgli_interfaces::srv::AddLidarIgnoreCorridor::Request::SharedPtr req,
    mowgli_interfaces::srv::AddLidarIgnoreCorridor::Response::SharedPtr res)
{
  std::lock_guard<std::recursive_mutex> state_lock(areas_state_mutex_);
  const auto c = normalised(req->corridor);
  if (const char* reason = corridor_rejection(c))
  {
    RCLCPP_WARN(get_logger(), "Rejected LiDAR-ignore corridor '%s': %s", c.name.c_str(), reason);
    return;
  }
  auto candidate = lidar_ignore_corridors_;
  uint64_t next = next_lidar_corridor_id_;
  uint64_t id = c.id;
  if (!id || std::any_of(candidate.begin(),
                         candidate.end(),
                         [&](const auto& e)
                         {
                           return e.id == id;
                         }))
  {
    if (next > UINT32_MAX)
      return;
    id = next++;
  }
  next = std::max(next, id + 1);
  candidate.push_back(
      {c.name,
       c.polyline,
       std::clamp(c.width_m, kMinLidarIgnoreCorridorWidthM, kMaxLidarIgnoreCorridorWidthM),
       static_cast<uint32_t>(id)});
  Replace::Response result;
  commit_corridors(std::move(candidate), next, result, false);
  res->success = result.success;
  if (result.committed)
    res->id = static_cast<uint32_t>(id);
}

void MapServerNode::on_get_lidar_ignore_corridors(
    const mowgli_interfaces::srv::GetLidarIgnoreCorridors::Request::SharedPtr,
    mowgli_interfaces::srv::GetLidarIgnoreCorridors::Response::SharedPtr res)
{
  res->corridors = corridor_snapshot();
  res->success = true;
}

void MapServerNode::on_clear_lidar_ignore_corridors(
    const mowgli_interfaces::srv::ClearLidarIgnoreCorridors::Request::SharedPtr,
    mowgli_interfaces::srv::ClearLidarIgnoreCorridors::Response::SharedPtr res)
{
  std::lock_guard<std::recursive_mutex> state_lock(areas_state_mutex_);
  Replace::Response result;
  commit_corridors({}, next_lidar_corridor_id_, result, false);
  // Clearing only ever un-blinds the scan filter: report success once the empty
  // list is live and published, even if areas.dat could not be rewritten
  // (commit_corridors logs that durability is unconfirmed).
  res->success = result.committed && result.published;
}

void MapServerNode::publish_lidar_ignore_corridors()
{
  std::lock_guard<std::recursive_mutex> state_lock(areas_state_mutex_);
  mowgli_interfaces::msg::LidarIgnoreCorridorArray msg;
  msg.header.stamp = get_clock()->now();
  msg.header.frame_id = "map";
  msg.corridors = corridor_snapshot();
  lidar_ignore_corridors_pub_->publish(msg);
  corridor_publication_pending_ = false;
}
}  // namespace mowgli_map

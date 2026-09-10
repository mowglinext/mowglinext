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

#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "mowgli_behavior/bt_context.hpp"
#include "mowgli_interfaces/srv/add_mowing_area.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

class RecordAreaAlgorithmTest;

namespace mowgli_behavior
{

// ---------------------------------------------------------------------------
// RecordArea — record robot trajectory and save as mowing area polygon
// ---------------------------------------------------------------------------

/// Records the robot's position while the user drives along the boundary. On
/// finish (COMMAND_RECORD_FINISH=5), simplifies the trajectory using the
/// Douglas-Peucker algorithm and saves it as a mowing area via the
/// map_server_node/add_area service.
///
/// On cancel (COMMAND_RECORD_CANCEL=6), discards the recording.
///
/// Returns RUNNING while recording, SUCCESS on finish, FAILURE on cancel or error.
///
/// BOUNDARY RESOLUTION — the three limiters, and why they are set where they are.
/// A field recording produced only 24 vertices for a 38.13 m perimeter (~1.6 m
/// between vertices on a hand-driven garden boundary). Two knobs caused it:
///
///  1. `simplification_tolerance` was 0.2 m. Douglas-Peucker guarantees only
///     that the stored polygon stays WITHIN the tolerance of the driven path,
///     so the saved boundary could be wrong by 0.2 m — wider than a full
///     cutting swath (`tool_width`, 0.18 m default, 0.15 m on some builds).
///     A boundary error larger than one swath is the wrong order of magnitude:
///     it can cost (or over-cut) an entire outermost pass.
///  2. `record_rate_hz` was 2.0. kMinSampleSpacingM (below) is the INTENDED
///     resolution floor, but at 2 Hz and any real driving speed the samples
///     arrive 0.15-0.25 m apart, so the gate never bit and the SAMPLE RATE was
///     the true limiter — DP was simplifying an already-coarse polyline.
///
/// The values are now derived rather than picked:
///  * Rate: the gate binds when v / rate <= kMinSampleSpacingM. `max_mps`
///    (0.5 m/s) is a hard runtime wheel-speed ceiling pushed to the STM32, so
///    the robot CANNOT be driven faster than that while recording. This gives
///    rate >= 0.5 / 0.05 = 10 Hz. That is also the ceiling: onRunning() only
///    executes when the tree ticks, so `tick_rate` (10 Hz) caps the achievable
///    rate — a higher record_rate_hz is silently ineffective, and is warned
///    about at onStart() using the `bt_tick_rate` blackboard entry.
///  * Tolerance: the useful window is [kMinSampleSpacingM, tool_width). Below
///    the sampling gate DP cannot recover detail that was never sampled — it
///    only preserves jitter — and above tool_width the error exceeds a swath.
///    kMinSampleSpacingM (0.05 m) is where "as fine as the data supports" and
///    "comfortably under a swath" (0.05 m is tool_width / 3) coincide.
///
/// The third limiter is downstream and needs no change: `mowgli_coverage`'s
/// dedupClosedRing() collapses the ingested boundary ring at 1 cm (metric
/// dedup) / 5 mm (near-collinear spike removal). Both sit an order of magnitude
/// below the 5 cm sampling floor, so they cost no recorded detail — and the
/// collinear pass doubles as the second-stage collapse of straight runs that
/// keeps the vertex count off F2C. map_server's on_add_area() stores the
/// polygon verbatim (no re-simplification, no clamping, no resampling).
///
/// Input ports:
///   simplification_tolerance (double, default kDefaultSimplificationToleranceM)
///       — Douglas-Peucker tolerance in metres.
///   min_vertices (uint32_t, default "3") — minimum polygon vertices after simplification.
///   min_area (double, default "1.0") — minimum polygon area in square metres.
///   record_rate_hz (double, default kDefaultRecordRateHz) — position sampling frequency.
class RecordArea : public BT::StatefulActionNode
{
  friend class ::RecordAreaAlgorithmTest;  // test access to private static methods

public:
  /// Minimum spacing between two consecutive RECORDED trajectory points (m).
  /// This is the intended boundary resolution floor: a sample closer than this
  /// to the previous kept point is dropped as a duplicate. It also sets the
  /// floor for a useful `simplification_tolerance` (see the class comment).
  static constexpr double kMinSampleSpacingM = 0.05;

  /// Default Douglas-Peucker tolerance (m). Equal to kMinSampleSpacingM — the
  /// finest tolerance the sampled data can actually support, and one third of
  /// the 0.18 m default `tool_width`.
  static constexpr double kDefaultSimplificationToleranceM = kMinSampleSpacingM;

  /// Default position sampling rate (Hz). Chosen so kMinSampleSpacingM is the
  /// binding limiter at `max_mps` (0.5 m/s), the hard wheel-speed ceiling.
  /// Also equals the default BT `tick_rate`, which is the achievable maximum.
  static constexpr double kDefaultRecordRateHz = 10.0;

  /// Rate (Hz) at which the live trajectory preview Path is republished to the
  /// GUI. Deliberately DECOUPLED from (and far below) the sampling rate: each
  /// publish rebuilds the entire trajectory, so publishing every sample is
  /// O(n^2) allocation work over a recording — ~700 poses at 10 Hz — for a
  /// preview no operator can perceive at more than a few Hz.
  static constexpr double kPreviewPublishRateHz = 2.0;

  RecordArea(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
        BT::InputPort<double>("simplification_tolerance",
                              kDefaultSimplificationToleranceM,
                              "Douglas-Peucker tolerance (m)"),
        BT::InputPort<uint32_t>("min_vertices", 3u, "Minimum polygon vertices"),
        BT::InputPort<double>("min_area", 1.0, "Minimum polygon area (m^2)"),
        BT::InputPort<double>("record_rate_hz", kDefaultRecordRateHz, "Recording frequency (Hz)"),
        BT::InputPort<bool>("is_exclusion_zone", false, "Record as exclusion zone"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  /// Record current robot position into trajectory_.
  void record_position();

  /// True when `candidate` is at least kMinSampleSpacingM from `last` and so
  /// should be appended to the trajectory. This is the minimum-spacing gate —
  /// the intended boundary-resolution floor.
  static bool is_sample_far_enough(const geometry_msgs::msg::Point32& last,
                                   const geometry_msgs::msg::Point32& candidate);

  /// Apply Douglas-Peucker simplification to the trajectory.
  static std::vector<geometry_msgs::msg::Point32> douglas_peucker(
      const std::vector<geometry_msgs::msg::Point32>& points, double tolerance);

  /// Recursive Douglas-Peucker helper.
  static void dp_recursive(const std::vector<geometry_msgs::msg::Point32>& points,
                           double tolerance,
                           size_t start,
                           size_t end,
                           std::vector<bool>& keep);

  /// Perpendicular distance from a point to a line segment.
  static double perpendicular_distance(const geometry_msgs::msg::Point32& pt,
                                       const geometry_msgs::msg::Point32& line_start,
                                       const geometry_msgs::msg::Point32& line_end);

  /// Compute polygon area using the shoelace formula.
  static double polygon_area(const std::vector<geometry_msgs::msg::Point32>& points);

  /// Save the simplified polygon as a mowing area.
  bool save_area(const std::vector<geometry_msgs::msg::Point32>& points, bool is_exclusion_zone);

  /// Recorded trajectory points in map frame.
  std::vector<geometry_msgs::msg::Point32> trajectory_;

  /// Publish the live trajectory preview Path (all points recorded so far).
  void publish_preview();

  /// Timestamp of last recorded position.
  std::chrono::steady_clock::time_point last_record_time_;

  /// Timestamp of last published trajectory preview.
  std::chrono::steady_clock::time_point last_preview_time_;

  /// Position sampling interval (computed from record_rate_hz).
  std::chrono::milliseconds record_interval_{100};

  /// Trajectory-preview publish interval (kPreviewPublishRateHz).
  std::chrono::milliseconds preview_interval_{static_cast<int>(1000.0 / kPreviewPublishRateHz)};

  /// Publisher for live trajectory preview.
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;

  /// Service client for adding the area.
  rclcpp::Client<mowgli_interfaces::srv::AddMowingArea>::SharedPtr add_area_client_;

  /// Area counter for auto-naming.
  static int area_counter_;
};

}  // namespace mowgli_behavior

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

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "mowgli_behavior/action_nodes.hpp"
#include "mowgli_behavior/bt_context.hpp"
#include "mowgli_behavior/condition_nodes.hpp"
#include "mowgli_behavior/coverage_persistence.hpp"
#include "mowgli_interfaces/gnss_status_utils.hpp"
#include "mowgli_interfaces/msg/absolute_pose.hpp"
#include "mowgli_interfaces/msg/emergency.hpp"
#include "mowgli_interfaces/msg/gnss_status.hpp"
#include "mowgli_interfaces/msg/power.hpp"
#include "mowgli_interfaces/msg/status.hpp"
#include "mowgli_interfaces/srv/high_level_control.hpp"
#include "mowgli_interfaces/srv/start_in_area.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/undock_robot.hpp"
#include "nav2_msgs/msg/collision_monitor_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/exceptions.hpp"

using namespace std::chrono_literals;

namespace mowgli_behavior
{

// ---------------------------------------------------------------------------
// BehaviorTreeNode
// ---------------------------------------------------------------------------

class BehaviorTreeNode : public rclcpp::Node
{
public:
  explicit BehaviorTreeNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{})
      : rclcpp::Node("mowgli_behavior_node", options), context_(std::make_shared<BTContext>())
  {
    // context_->node is set in init() after shared_from_this() becomes valid.

    RCLCPP_INFO(get_logger(), "Initializing mowgli_behavior_node");
  }

  /// Must be called once after construction (shared_from_this() is valid).
  void init()
  {
    context_->node = shared_from_this();
    context_->tf_buffer = std::make_shared<tf2_ros::Buffer>(get_clock());
    context_->tf_listener = std::make_shared<tf2_ros::TransformListener>(*context_->tf_buffer);
    context_->helper_node = rclcpp::Node::make_shared("_bt_helper_node");

    // Disk-backed coverage resume: where FollowStrip persists per-area progress
    // so an interrupted mow survives a full process/container restart (reboot,
    // crash, docker restart, power-cycle after an emergency stop) — the in-RAM
    // BTContext resume only survives a live BT halt. Default lives on the
    // bind-mounted maps volume so it outlives the container. Empty disables it.
    context_->coverage_resume_path =
        declare_parameter<std::string>("coverage_resume_path", "/ros2_ws/maps/coverage_resume.txt");
    if (loadCoverageResumeState(*context_))
    {
      RCLCPP_INFO(get_logger(),
                  "Restored coverage resume state from %s (current_area=%d, %zu area(s) with a "
                  "resume cursor, %zu completed)",
                  context_->coverage_resume_path.c_str(),
                  context_->current_area,
                  context_->area_resume_pose_index.size(),
                  context_->completed_areas.size());
    }

    setupSubscribers();
    setupServiceServer();
    setupBehaviorTree();
    setupTimer();
    startNav2WaitTimer();

    RCLCPP_INFO(get_logger(), "mowgli_behavior_node ready");
  }

  std::shared_ptr<BTContext> context() const
  {
    return context_;
  }

private:
  // ------------------------------------------------------------------
  // ROS2 infrastructure
  // ------------------------------------------------------------------

  void setupSubscribers()
  {
    using namespace mowgli_interfaces::msg;

    status_sub_ =
        create_subscription<Status>("/hardware_bridge/status",
                                    10,
                                    [this](Status::ConstSharedPtr msg)
                                    {
                                      std::lock_guard<std::mutex> lock(context_->context_mutex);
                                      context_->latest_status = *msg;
                                    });

    emergency_sub_ =
        create_subscription<Emergency>("/hardware_bridge/emergency",
                                       10,
                                       [this](Emergency::ConstSharedPtr msg)
                                       {
                                         std::lock_guard<std::mutex> lock(context_->context_mutex);
                                         context_->latest_emergency = *msg;
                                         context_->last_emergency_time =
                                             std::chrono::steady_clock::now();
                                       });

    power_sub_ =
        create_subscription<Power>("/hardware_bridge/power",
                                   10,
                                   [this](Power::ConstSharedPtr msg)
                                   {
                                     std::lock_guard<std::mutex> lock(context_->context_mutex);
                                     context_->latest_power = *msg;

                                     // Derive battery_percent from voltage using
                                     // configurable thresholds from ROS parameters.
                                     const float v_max = battery_full_voltage_;
                                     const float v_min = battery_empty_voltage_;
                                     const float clamped = std::clamp(msg->v_battery, v_min, v_max);
                                     const float range = v_max - v_min;
                                     context_->battery_percent =
                                         (range > 0.01f) ? 100.0f * (clamped - v_min) / range
                                                         : 0.0f;
                                   });

    // Replan / boundary signals from map_server_node
    replan_needed_sub_ =
        create_subscription<std_msgs::msg::Bool>("/map_server_node/replan_needed",
                                                 rclcpp::QoS(1),
                                                 [this](std_msgs::msg::Bool::ConstSharedPtr msg)
                                                 {
                                                   std::lock_guard<std::mutex> lock(
                                                       context_->context_mutex);
                                                   context_->replan_needed = msg->data;
                                                 });

    // Must match the publisher in mowgli_map/map_server_node.cpp, which uses
    // ~/boundary_violation → resolves to /map_server_node/boundary_violation.
    // An earlier version of this subscription used /map_server/… which had
    // zero publishers, so the BoundaryGuard silently never tripped and the
    // robot was free to drive outside the defined mowing area.
    boundary_violation_sub_ =
        create_subscription<std_msgs::msg::Bool>("/map_server_node/boundary_violation",
                                                 10,
                                                 [this](std_msgs::msg::Bool::ConstSharedPtr msg)
                                                 {
                                                   std::lock_guard<std::mutex> lock(
                                                       context_->context_mutex);
                                                   context_->boundary_violation = msg->data;
                                                 });

    // Lethal boundary: outside the allowed area by more than the
    // configured margin (map_server_node's lethal_boundary_margin_m).
    // When this trips, BoundaryGuard emergency-stops instead of trying
    // to navigate back inside — too far gone for safe recovery.
    lethal_boundary_violation_sub_ =
        create_subscription<std_msgs::msg::Bool>("/map_server_node/lethal_boundary_violation",
                                                 10,
                                                 [this](std_msgs::msg::Bool::ConstSharedPtr msg)
                                                 {
                                                   std::lock_guard<std::mutex> lock(
                                                       context_->context_mutex);
                                                   context_->lethal_boundary_violation = msg->data;
                                                 });

    // GPS position for heading calibration during undock. RTK/fix-state comes
    // from /gps/status so covariance fallout on /gps/fix does not masquerade
    // as "no RTK fix" in the behavior tree.
    gps_sub_ = create_subscription<mowgli_interfaces::msg::AbsolutePose>(
        "/gps/absolute_pose",
        10,
        [this](mowgli_interfaces::msg::AbsolutePose::ConstSharedPtr msg)
        {
          std::lock_guard<std::mutex> lock(context_->context_mutex);
          using AP = mowgli_interfaces::msg::AbsolutePose;

          context_->gps_x = msg->pose.pose.position.x;
          context_->gps_y = msg->pose.pose.position.y;

          // Buffer GPS samples while the undock BackUp is in flight so
          // CalibrateHeadingFromUndock can fit a line through the whole
          // trajectory (much tighter yaw than the endpoint-only method).
          // Drop samples that haven't moved at least 5 cm so a stalled
          // chassis doesn't bloat the buffer; cap the buffer to
          // kUndockGpsSamplesCap and drop the oldest on overflow.
          if (context_->undock_start_recorded)
          {
            auto& buf = context_->undock_gps_samples;
            const double x = context_->gps_x;
            const double y = context_->gps_y;
            bool keep = true;
            if (!buf.empty())
            {
              const double dx = x - buf.back().first;
              const double dy = y - buf.back().second;
              if ((dx * dx + dy * dy) < (0.05 * 0.05))
              {
                keep = false;
              }
            }
            if (keep)
            {
              if (buf.size() >= BTContext::kUndockGpsSamplesCap)
              {
                buf.erase(buf.begin());
              }
              buf.emplace_back(x, y);
            }
          }

          if (!has_authoritative_gnss_status_)
          {
            // Fallback for legacy bring-up before /gps/status arrives.
            if (msg->flags & AP::FLAG_GPS_RTK_FIXED)
            {
              context_->gps_fix_type = 4;
            }
            else if (msg->flags & AP::FLAG_GPS_RTK_FLOAT)
            {
              context_->gps_fix_type = 3;
            }
            else if (msg->flags & AP::FLAG_GPS_RTK)
            {
              context_->gps_fix_type = 2;
            }
            else
            {
              context_->gps_fix_type = 0;
            }
            context_->gps_is_fixed =
                (context_->gps_fix_type >= 4) && (msg->position_accuracy < 0.1f);
            context_->gps_quality = std::clamp(1.0f - msg->position_accuracy, 0.0f, 1.0f);
          }
        });

    gnss_status_sub_ = create_subscription<mowgli_interfaces::msg::GnssStatus>(
        "/gps/status",
        10,
        [this](mowgli_interfaces::msg::GnssStatus::ConstSharedPtr msg)
        {
          std::lock_guard<std::mutex> lock(context_->context_mutex);
          has_authoritative_gnss_status_ = true;
          context_->gps_fix_type = mowgli_interfaces::gnss_status_utils::BehaviorTreeFixType(*msg);
          context_->gps_quality = mowgli_interfaces::gnss_status_utils::NormalizedQuality(*msg);

          // Debounce RTK-fixed transitions so the BT does not chatter during
          // short-lived fix-state flicker while still trusting the typed
          // /gps/status contract rather than /gps/absolute_pose covariance.
          constexpr double kGpsFixDebounceSec = 2.0;
          const bool raw_fixed = mowgli_interfaces::gnss_status_utils::BehaviorTreeRtkFixed(*msg);
          const rclcpp::Time gps_now = this->now();
          if (!gps_fix_debounce_init_)
          {
            gps_fix_debounce_init_ = true;
            gps_fix_candidate_ = raw_fixed;
            gps_fix_candidate_since_ = gps_now;
            context_->gps_is_fixed = raw_fixed;
          }
          else
          {
            if (raw_fixed != gps_fix_candidate_)
            {
              gps_fix_candidate_ = raw_fixed;
              gps_fix_candidate_since_ = gps_now;
            }
            if ((gps_now - gps_fix_candidate_since_).seconds() >= kGpsFixDebounceSec)
            {
              context_->gps_is_fixed = gps_fix_candidate_;
            }
          }
        });

    // collision_monitor state — used by IsObstacleStuck to detect when
    // the robot is wedged on an obstacle (PolygonStop active for ≥5 s).
    // Latched into BTContext so the condition tick is a pure read.
    collision_monitor_sub_ = create_subscription<nav2_msgs::msg::CollisionMonitorState>(
        "/collision_monitor_state",
        10,
        [this](nav2_msgs::msg::CollisionMonitorState::ConstSharedPtr msg)
        {
          std::lock_guard<std::mutex> lock(context_->context_mutex);
          const uint8_t prev = context_->collision_action_type;
          context_->collision_action_type = msg->action_type;

          // Stamp the entry-into-STOP transition so IsObstacleStuck can
          // measure duration. On STOP exit, clear collision_stop_since AND
          // record the exit time in last_collision_stop_end so
          // WasRecentlyInCollisionStop can guard MarkBlockedAndSkip against
          // marking cells DEAD just because a dynamic obstacle wedged the
          // robot for a few seconds and then walked off.
          if (msg->action_type == nav2_msgs::msg::CollisionMonitorState::STOP)
          {
            if (prev != nav2_msgs::msg::CollisionMonitorState::STOP)
            {
              context_->collision_stop_since = std::chrono::steady_clock::now();
            }
          }
          else
          {
            if (prev == nav2_msgs::msg::CollisionMonitorState::STOP)
            {
              context_->last_collision_stop_end = std::chrono::steady_clock::now();
            }
            context_->collision_stop_since = std::chrono::steady_clock::time_point{};
          }
        });

    // Motion integrals for IsWheelSlipStuck. Two "trying to move" signals:
    // /cmd_vel_monitored (post-collision_monitor command — accumulates even
    // when the wheels are STALLED against a root, the case wheel odom misses)
    // and /wheel_odom (accumulates when the wheels SPIN in place, the case a
    // near-zero commanded speed could under-report). The condition node takes
    // max(cmd, wheel) over its window. dt is wall-clock between messages,
    // clamped so a stream hiccup can't inject a huge phantom distance.
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        "/cmd_vel_monitored",
        10,
        [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
        {
          const auto now = std::chrono::steady_clock::now();
          std::lock_guard<std::mutex> lock(context_->context_mutex);
          if (last_cmd_vel_time_.time_since_epoch().count() != 0)
          {
            const double dt =
                std::min(kMotionIntegrationMaxDtSec,
                         std::chrono::duration<double>(now - last_cmd_vel_time_).count());
            context_->cmd_dist_accum += std::fabs(msg->twist.linear.x) * dt;
          }
          last_cmd_vel_time_ = now;
        });

    wheel_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/wheel_odom",
        rclcpp::SensorDataQoS(),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg)
        {
          const auto now = std::chrono::steady_clock::now();
          std::lock_guard<std::mutex> lock(context_->context_mutex);
          if (last_wheel_odom_time_.time_since_epoch().count() != 0)
          {
            const double dt =
                std::min(kMotionIntegrationMaxDtSec,
                         std::chrono::duration<double>(now - last_wheel_odom_time_).count());
            context_->wheel_dist_accum += std::fabs(msg->twist.twist.linear.x) * dt;
          }
          last_wheel_odom_time_ = now;
        });

    // 5 Hz snapshot of the motion integrals + the GPS/graph-anchored
    // map→base_footprint position into the rolling window IsWheelSlipStuck
    // reads. TF unavailability is recorded (map_valid=false) rather than
    // skipped so the condition can refuse to fire on a stale window.
    motion_window_timer_ = create_wall_timer(
        std::chrono::milliseconds(200),
        [this]()
        {
          BTContext::MotionSample sample;
          sample.t = std::chrono::steady_clock::now();
          try
          {
            const auto tf =
                context_->tf_buffer->lookupTransform("map", "base_footprint", tf2::TimePointZero);
            sample.map_x = tf.transform.translation.x;
            sample.map_y = tf.transform.translation.y;
            sample.map_valid = true;
          }
          catch (const tf2::TransformException&)
          {
            sample.map_valid = false;
          }
          std::lock_guard<std::mutex> lock(context_->context_mutex);
          sample.cmd_dist = context_->cmd_dist_accum;
          sample.wheel_dist = context_->wheel_dist_accum;
          context_->motion_window.push_back(sample);
          // Keep ~2× the largest window IsWheelSlipStuck can be configured
          // with (30 s) so the oldest-sample lookup always has slack.
          const auto horizon = sample.t - std::chrono::seconds(60);
          while (!context_->motion_window.empty() && context_->motion_window.front().t < horizon)
          {
            context_->motion_window.pop_front();
          }
        });

    RCLCPP_DEBUG(get_logger(), "Topic subscribers created");
  }

  void setupServiceServer()
  {
    using HighLevelControl = mowgli_interfaces::srv::HighLevelControl;

    high_level_control_srv_ = create_service<HighLevelControl>(
        "~/high_level_control",
        [this](const HighLevelControl::Request::SharedPtr req,
               HighLevelControl::Response::SharedPtr resp)
        {
          RCLCPP_INFO(get_logger(), "HighLevelControl: received command=%u", req->command);
          // COMMAND_S2 (4, "mow next area" — the GUI's onMowNextArea button) has
          // no dedicated MainLogic branch: in this architecture mowing always
          // resumes from the next UN-mowed area (GetNextUnmowedArea), so "mow
          // next area" is functionally COMMAND_START. Normalise 4 -> 1 here so the
          // button actually mows instead of falling through to IdleSequence
          // (which stops the robot). If a distinct "skip current, jump to next"
          // semantic is ever needed, give it its own branch instead.
          uint8_t cmd = req->command;
          if (cmd == HighLevelControl::Request::COMMAND_S2)
          {
            cmd = HighLevelControl::Request::COMMAND_START;
            RCLCPP_INFO(get_logger(),
                        "HighLevelControl: COMMAND_S2 normalised to COMMAND_START (mow next area)");
          }
          {
            std::lock_guard<std::mutex> lock(context_->context_mutex);
            context_->current_command = cmd;
          }
          resp->success = true;
        });

    RCLCPP_DEBUG(get_logger(), "~/high_level_control service server created");

    // ~/start_in_area: GUI hook for "mow this specific area only".
    // Pre-loads target_area_index for the next GetNextUnmowedArea call,
    // then issues COMMAND_START so MowingSequence picks it up. The BT
    // exits after that single area is done (no roll-over to other areas).
    using StartInArea = mowgli_interfaces::srv::StartInArea;
    start_in_area_srv_ = create_service<StartInArea>(
        "~/start_in_area",
        [this](const StartInArea::Request::SharedPtr req, StartInArea::Response::SharedPtr resp)
        {
          RCLCPP_INFO(get_logger(), "StartInArea: received area=%u", req->area);
          {
            std::lock_guard<std::mutex> lock(context_->context_mutex);
            context_->target_area_index = static_cast<int>(req->area);
            context_->current_command = 1;  // COMMAND_START
          }
          resp->success = true;
        });

    RCLCPP_DEBUG(get_logger(), "~/start_in_area service server created");

    // ~/clear_coverage_resume: "Start fresh" — discard any persisted mowing
    // progress so the NEXT COMMAND_START begins at the first line instead of
    // resuming mid-path. The GUI offers this vs "Resume" when
    // coverage_resume_available is true (a prior session was interrupted without
    // reaching a dock/EndSession boundary). This is the operator's explicit
    // resume-vs-restart choice (issue: "starts at 2nd/3rd line"); the automatic
    // in-session resume after an e-stop is unaffected.
    // The clear is DEFERRED to the BT tick thread (processed at the top of
    // tickTree) rather than done inline: the BT nodes (FollowStrip,
    // GetNextUnmowedArea, EndSession) read/write these maps WITHOUT the context
    // mutex — safe today only because every callback of this node shares the
    // default MutuallyExclusive callback group, so tick/service/timers are
    // serialized even under the MultiThreadedExecutor. Deferring keeps every
    // map mutation on the tick thread, so a future Reentrant group / callback
    // re-homing can't silently turn this into a data race against a mid-tick
    // `const auto& done = ctx->area_completed_swaths[...]` reference.
    clear_coverage_resume_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/clear_coverage_resume",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
               std_srvs::srv::Trigger::Response::SharedPtr resp)
        {
          clear_resume_requested_.store(true);
          RCLCPP_INFO(get_logger(),
                      "Coverage resume clear requested — applied before the next BT tick");
          resp->success = true;
          resp->message = "coverage resume state cleared";
        });

    // Latched signal the GUI reads to decide whether to offer "Resume vs Start
    // fresh". True when a prior session left recoverable progress.
    resume_available_pub_ = create_publisher<std_msgs::msg::Bool>("~/coverage_resume_available",
                                                                  rclcpp::QoS(1).transient_local());
    publishResumeAvailable();
    resume_available_timer_ = create_wall_timer(1s,
                                                [this]()
                                                {
                                                  publishResumeAvailable();
                                                });

    RCLCPP_DEBUG(get_logger(), "~/clear_coverage_resume service + resume-available signal created");
  }

  // Publish whether a coverage session can be resumed (a persisted resume cursor
  // or completed-area set survives from an interrupted session). Republished on
  // change; latched so a late GUI subscriber always gets the current value.
  void publishResumeAvailable()
  {
    if (!resume_available_pub_)
    {
      return;
    }
    bool available;
    {
      std::lock_guard<std::mutex> lock(context_->context_mutex);
      available = !context_->area_resume_pose_index.empty() || !context_->completed_areas.empty();
    }
    if (available == last_resume_available_ && resume_available_published_)
    {
      return;
    }
    std_msgs::msg::Bool msg;
    msg.data = available;
    resume_available_pub_->publish(msg);
    last_resume_available_ = available;
    resume_available_published_ = true;
  }

  // Non-blocking check for Nav2 action servers.  The BT tick loop starts
  // immediately so that idle-state publishers (high_level_status, etc.) are
  // created right away.  A periodic timer polls for Nav2 readiness and
  // sets a blackboard flag that BT action nodes can check before sending
  // goals.
  void startNav2WaitTimer()
  {
    using nav2_msgs::action::NavigateToPose;
    using nav2_msgs::action::UndockRobot;

    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");
    undock_client_ = rclcpp_action::create_client<UndockRobot>(this, "/undock_robot");

    nav2_wait_deadline_ = std::chrono::steady_clock::now() + std::chrono::seconds(120);

    RCLCPP_INFO(get_logger(),
                "Waiting for Nav2 action servers (/navigate_to_pose, /undock_robot)...");

    nav2_wait_timer_ = create_wall_timer(2s,
                                         [this]()
                                         {
                                           checkNav2Ready();
                                         });
  }

  void checkNav2Ready()
  {
    if (!nav_ready_)
    {
      nav_ready_ = nav_client_->action_server_is_ready();
    }
    if (!undock_ready_)
    {
      undock_ready_ = undock_client_->action_server_is_ready();
    }

    if (nav_ready_ && undock_ready_)
    {
      RCLCPP_INFO(get_logger(), "Nav2 action servers are available");
      nav2_wait_timer_->cancel();
      return;
    }

    if (std::chrono::steady_clock::now() >= nav2_wait_deadline_)
    {
      RCLCPP_WARN(get_logger(),
                  "Timed out waiting for Nav2 action servers — BT continues without them");
      nav2_wait_timer_->cancel();
      return;
    }

    RCLCPP_INFO_THROTTLE(get_logger(),
                         *get_clock(),
                         10000,
                         "Still waiting for Nav2 (navigate=%s, undock=%s)...",
                         nav_ready_ ? "ok" : "waiting",
                         undock_ready_ ? "ok" : "waiting");
  }

  void setupBehaviorTree()
  {
    // Register all custom nodes
    mowgli_behavior::registerAllNodes(factory_);

    // Resolve tree file path
    std::string tree_file = declare_parameter<std::string>("tree_file", "");

    if (tree_file.empty())
    {
      try
      {
        const std::string pkg_share =
            ament_index_cpp::get_package_share_directory("mowgli_behavior");
        tree_file = pkg_share + "/trees/main_tree.xml";
      }
      catch (const std::exception& ex)
      {
        throw std::runtime_error(std::string("Cannot locate default tree file: ") + ex.what());
      }
    }

    if (!std::filesystem::exists(tree_file))
    {
      throw std::runtime_error("Tree file not found: " + tree_file);
    }

    RCLCPP_INFO(get_logger(), "Loading behavior tree from: %s", tree_file.c_str());

    // Build blackboard and store shared context
    blackboard_ = BT::Blackboard::create();
    blackboard_->set("context", context_);

    // Default poses (x;y;yaw format) — overridable via parameters.
    const std::string dock_pose = declare_parameter<std::string>("dock_pose", "0.0;0.0;0.0");
    const std::string undock_pose = declare_parameter<std::string>("undock_pose", "1.0;0.0;0.0");
    blackboard_->set("dock_pose", dock_pose);
    blackboard_->set("undock_pose", undock_pose);

    // Undock reverse speed and distance, sourced from mowgli_robot.yaml and
    // consumed by the BackUp BT instances in main_tree.xml via the
    // {undock_speed} / {undock_distance} blackboard references. Previously
    // both were hardcoded in the BT XML, so editing the YAML had no effect.
    // Recovery-side BackUps (e.g. OBSTACLE_BACKOFF) intentionally stay
    // hardcoded; they are not undocking and must not shift when the operator
    // tunes undock distance. See issue #191.
    const double undock_speed = declare_parameter<double>("undock_speed", 0.15);
    blackboard_->set("undock_speed", undock_speed);
    const double undock_distance = declare_parameter<double>("undock_distance", 1.0);
    blackboard_->set("undock_distance", undock_distance);

    // Wheel-slip / stall stuck detection (IsWheelSlipStuck, StuckBackoff).
    // Template defaults in mowgli_bringup/config/mowgli_robot.yaml (GUI:
    // Settings → Obstacles); forwarded here by full_system.launch.py. Clamps
    // mirror the template comments so a stray override cannot make the
    // detector hair-triggered (window < 2 s) or dead (displacement gate 0).
    const bool stuck_enabled = declare_parameter<bool>("stuck_detection_enabled", true);
    blackboard_->set("stuck_detection_enabled", stuck_enabled);
    const double stuck_window_sec =
        std::clamp(declare_parameter<double>("stuck_window_sec", 4.0), 2.0, 30.0);
    blackboard_->set("stuck_window_sec", stuck_window_sec);
    const double stuck_min_commanded_m =
        std::clamp(declare_parameter<double>("stuck_min_commanded_m", 0.15), 0.05, 1.0);
    blackboard_->set("stuck_min_commanded_m", stuck_min_commanded_m);
    const double stuck_max_displacement_m =
        std::clamp(declare_parameter<double>("stuck_max_displacement_m", 0.05), 0.01, 0.3);
    blackboard_->set("stuck_max_displacement_m", stuck_max_displacement_m);

    // idle_nav2_suspend (default false): when true, the BT PAUSEs the Nav2
    // lifecycle stack (via SetNav2Lifecycle) while parked on the dock to cut
    // the idle CPU/thermal load of the always-looping costmaps, and RESUMEs
    // it (root Nav2ResumeGuard) before any motion. Default-off so enabling
    // it is a deliberate, per-site operator decision. Read by the
    // SetNav2Lifecycle nodes from the blackboard.
    const bool idle_nav2_suspend = declare_parameter<bool>("idle_nav2_suspend", false);
    blackboard_->set("idle_nav2_suspend", idle_nav2_suspend);

    // Transit / mowing speeds, sourced from mowgli_robot.yaml and applied to
    // the live controllers by SetNavMode (FollowPath.desired_linear_vel for the
    // RPP transit controller, FollowCoveragePath.speed_fast for FTC coverage).
    // Stored on the shared BTContext so SetNavMode's tick is a pure read.
    // Previously SetNavMode hardcoded 0.5 (precise) / 0.25 (degraded), which
    // stomped the launch-injected values — the configured speeds never applied.
    context_->transit_speed = declare_parameter<double>("transit_speed", 0.25);
    context_->mowing_speed = declare_parameter<double>("mowing_speed", 0.2);

    // Rain delay: parameter in minutes, blackboard in seconds.
    const double rain_delay_minutes = declare_parameter<double>("rain_delay_minutes", 30.0);
    blackboard_->set("rain_delay_sec", rain_delay_minutes * 60.0);

    // Rain handling mode: 0 = off, 1 = pause-in-place, 2 = dock-and-pause.
    // Read by IsNewRain (mode 0 short-circuits to FAILURE) and
    // IsRainModeAtLeast (gates the dock branch in the BT XML so mode 1
    // skips the round trip to the dock).
    const int rain_mode = static_cast<int>(declare_parameter<int>("rain_mode", 2));
    blackboard_->set("rain_mode", rain_mode);

    // Rain debounce window: rain must be detected continuously for this
    // many seconds before IsNewRain trips. Filters short pulses (a leaf
    // brushing the sensor, a single drop) that would otherwise abort
    // mowing. 0 disables debounce (legacy behaviour).
    const double rain_debounce_sec = declare_parameter<double>("rain_debounce_sec", 0.0);
    blackboard_->set("rain_debounce_sec", rain_debounce_sec);

    declare_parameter<double>("tick_rate", 10.0);

    // Robot footprint — used by PlanCoverageArea to inset the perimeter
    // ring path so the chassis (wider than the mower deck) stays inside
    // the polygon. Default matches YardForce 500 (mowgli_robot.yaml).
    declare_parameter<double>("chassis_width", 0.40);
    declare_parameter<double>("chassis_length", 0.60);

    // Battery voltage curve — configurable via mowgli_robot.yaml.
    // YardForce500 SLA packs top out around 28.0 V on the dock; the previous
    // default of 28.5 V capped the displayed percent at 88.9 % when full.
    battery_full_voltage_ =
        static_cast<float>(declare_parameter<double>("battery_full_voltage", 28.0));
    battery_empty_voltage_ =
        static_cast<float>(declare_parameter<double>("battery_empty_voltage", 24.0));

    // Battery percent thresholds — operator-facing knobs from the GUI's
    // BatterySection. Pushed onto the blackboard so the BT XML can pull
    // them with {key} substitution instead of carrying hardcoded values.
    const double battery_low_pct = declare_parameter<double>("battery_low_percent", 20.0);
    const double battery_critical_pct = declare_parameter<double>("battery_critical_percent", 10.0);
    const double battery_full_pct = declare_parameter<double>("battery_full_percent", 95.0);
    const double battery_critical_voltage =
        declare_parameter<double>("battery_critical_voltage", 0.0);
    // Hysteresis recovery threshold for the critical-battery handler: the
    // robot enters critical-dock at battery_critical_percent but only leaves
    // it (back to IDLE_DOCKED) once charged above this higher level. Without
    // the upper threshold the critical state flapped at the entry boundary
    // and re-ran DockRobot every tick while the pack crawled back up. Clamp
    // to strictly above the entry threshold so the band can never invert.
    double battery_critical_recovery_pct =
        declare_parameter<double>("battery_critical_recovery_percent", 30.0);
    if (battery_critical_recovery_pct <= battery_critical_pct)
    {
      battery_critical_recovery_pct = battery_critical_pct + 10.0;
      RCLCPP_WARN(get_logger(),
                  "battery_critical_recovery_percent must exceed "
                  "battery_critical_percent (%.1f); clamped to %.1f",
                  battery_critical_pct,
                  battery_critical_recovery_pct);
    }
    blackboard_->set("battery_low_pct", static_cast<float>(battery_low_pct));
    blackboard_->set("battery_critical_pct", static_cast<float>(battery_critical_pct));
    blackboard_->set("battery_full_pct", static_cast<float>(battery_full_pct));
    blackboard_->set("battery_critical_voltage", static_cast<float>(battery_critical_voltage));
    blackboard_->set("battery_critical_recovery_pct",
                     static_cast<float>(battery_critical_recovery_pct));

    tree_ = factory_.createTreeFromFile(tree_file, blackboard_);

    // Optionally attach a console logger for debugging BT state transitions.
    const bool bt_debug_logging = declare_parameter<bool>("bt_debug_logging", false);
    if (bt_debug_logging)
    {
      logger_ = std::make_unique<BT::StdCoutLogger>(tree_);
      RCLCPP_INFO(get_logger(), "BT StdCoutLogger enabled (bt_debug_logging=true)");
    }

    RCLCPP_INFO(get_logger(), "Behavior tree loaded successfully");
  }

  void setupTimer()
  {
    const double tick_rate = get_parameter("tick_rate").as_double();
    const auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / tick_rate));
    RCLCPP_INFO(get_logger(),
                "Behavior tree tick rate: %.1f Hz (%ld ms)",
                tick_rate,
                period.count());
    tick_timer_ = create_wall_timer(period,
                                    [this]()
                                    {
                                      tickTree();
                                    });
  }

  void tickTree()
  {
    // Apply a pending "Start fresh" clear BEFORE ticking, on the tick thread —
    // every coverage-map mutation stays on this thread (see the service
    // registration comment). Between ticks no BT node holds a reference into
    // the maps, so clearing here is race-free by construction.
    if (clear_resume_requested_.exchange(false))
    {
      std::lock_guard<std::mutex> lock(context_->context_mutex);
      context_->area_completed_swaths.clear();
      context_->area_swath_count.clear();
      context_->area_resume_pose_index.clear();
      context_->area_path_pose_count.clear();
      context_->completed_areas.clear();
      context_->attempted_areas.clear();
      context_->area_attempt_count.clear();
      context_->area_last_coverage.clear();
      clearCoverageResumeState(*context_);
      RCLCPP_INFO(get_logger(),
                  "Cleared coverage resume state on request — next start begins fresh");
    }
    try
    {
      const BT::NodeStatus status = tree_.tickOnce();

      if (status == BT::NodeStatus::FAILURE)
      {
        RCLCPP_WARN_THROTTLE(get_logger(),
                             *get_clock(),
                             5000,
                             "Behavior tree root returned FAILURE");
      }
    }
    catch (const std::exception& ex)
    {
      RCLCPP_ERROR(get_logger(), "Exception during tree tick: %s", ex.what());
    }
  }

  // ------------------------------------------------------------------
  // Data members
  // ------------------------------------------------------------------

  std::shared_ptr<BTContext> context_;

  // GPS-fixed debounce state (see the /gps callback): rides through the F9P
  // per-epoch Fixed↔Float flicker so gps_is_fixed — and thus SetNavMode — does
  // not chatter and reset the MPPI optimizer.
  bool gps_fix_debounce_init_{false};
  bool gps_fix_candidate_{false};
  rclcpp::Time gps_fix_candidate_since_;
  bool has_authoritative_gnss_status_{false};

  // Subscribers
  rclcpp::Subscription<mowgli_interfaces::msg::Status>::SharedPtr status_sub_;
  rclcpp::Subscription<mowgli_interfaces::msg::Emergency>::SharedPtr emergency_sub_;
  rclcpp::Subscription<mowgli_interfaces::msg::Power>::SharedPtr power_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr replan_needed_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr boundary_violation_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lethal_boundary_violation_sub_;
  rclcpp::Subscription<mowgli_interfaces::msg::AbsolutePose>::SharedPtr gps_sub_;
  rclcpp::Subscription<mowgli_interfaces::msg::GnssStatus>::SharedPtr gnss_status_sub_;
  rclcpp::Subscription<nav2_msgs::msg::CollisionMonitorState>::SharedPtr collision_monitor_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_sub_;
  // Wall-clock stamps of the previous cmd_vel / wheel_odom message, for the
  // |v|·dt motion integrals (guarded by context_mutex alongside the accums).
  std::chrono::steady_clock::time_point last_cmd_vel_time_{};
  std::chrono::steady_clock::time_point last_wheel_odom_time_{};
  // Cap per-message integration dt: a stream that stalls then resumes must
  // not inject a phantom |v|·gap distance into the stuck detector.
  static constexpr double kMotionIntegrationMaxDtSec = 0.5;

  // Service server
  rclcpp::Service<mowgli_interfaces::srv::HighLevelControl>::SharedPtr high_level_control_srv_;
  rclcpp::Service<mowgli_interfaces::srv::StartInArea>::SharedPtr start_in_area_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_coverage_resume_srv_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr resume_available_pub_;
  rclcpp::TimerBase::SharedPtr resume_available_timer_;
  rclcpp::TimerBase::SharedPtr motion_window_timer_;
  // Set by the ~/clear_coverage_resume service, consumed by tickTree() so the
  // actual map clearing happens on the BT tick thread (see the service comment).
  std::atomic<bool> clear_resume_requested_{false};
  // Only touched from this node's mutually-exclusive callback group (timer +
  // service + init), so plain bools would work today — atomic future-proofs
  // them against a Reentrant-group conversion, same rationale as the deferral.
  std::atomic<bool> last_resume_available_{false};
  std::atomic<bool> resume_available_published_{false};

  // BehaviorTree.CPP
  BT::BehaviorTreeFactory factory_;
  BT::Blackboard::Ptr blackboard_;
  BT::Tree tree_;
  std::unique_ptr<BT::StdCoutLogger> logger_;

  // Tick timer
  rclcpp::TimerBase::SharedPtr tick_timer_;

  // Nav2 readiness polling
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_;
  rclcpp_action::Client<nav2_msgs::action::UndockRobot>::SharedPtr undock_client_;
  rclcpp::TimerBase::SharedPtr nav2_wait_timer_;
  std::chrono::steady_clock::time_point nav2_wait_deadline_;
  bool nav_ready_{false};
  bool undock_ready_{false};

  // Battery voltage curve parameters
  float battery_full_voltage_{28.0f};
  float battery_empty_voltage_{24.0f};
};

}  // namespace mowgli_behavior

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<mowgli_behavior::BehaviorTreeNode>();

  // init() must be called after the node is managed by a shared_ptr so that
  // shared_from_this() is valid.
  node->init();

  // Use MultiThreadedExecutor so both the main BT node and the helper
  // node (used for service clients from BT tick callbacks) get spun.
  // Without spinning the helper, async service responses never reach
  // the future, so GetCoverageStatus / GetNextStrip / etc. all time out
  // — symptom: `GetNextUnmowedArea: all areas complete` immediately on
  // start because the service future is never ready.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(node->context()->helper_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}

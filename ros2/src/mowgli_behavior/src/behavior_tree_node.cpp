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

#include <chrono>
#include <filesystem>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"
#include "mowgli_behavior/action_nodes.hpp"
#include "mowgli_behavior/bt_context.hpp"
#include "mowgli_behavior/condition_nodes.hpp"
#include "mowgli_interfaces/msg/absolute_pose.hpp"
#include "mowgli_interfaces/msg/emergency.hpp"
#include "mowgli_interfaces/msg/power.hpp"
#include "mowgli_interfaces/msg/status.hpp"
#include "mowgli_interfaces/srv/high_level_control.hpp"
#include "mowgli_interfaces/srv/start_in_area.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/undock_robot.hpp"
#include "nav2_msgs/msg/collision_monitor_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/bool.hpp"

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

    // GPS position and quality for heading calibration during undock
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

          // Derive fix type from flags. The ordinal MUST be monotonic in
          // quality (higher = better) because the undock/preflight gates use
          // ">= min_fix_type". RTK-Fixed is the best fix, so it must be the
          // highest value — earlier code ranked Float (5) ABOVE Fixed (4),
          // which let a robot in RTK-Float pass a "require RTK-Fixed" (min=4)
          // gate and undock/mow on Float-quality GPS (σ 10-50 cm). Ordering:
          //   FLAG_GPS_RTK_FIXED → 4 (RTK fixed — best)
          //   FLAG_GPS_RTK_FLOAT → 3 (RTK float — worse than fixed)
          //   FLAG_GPS_RTK       → 2 (DGPS/RTK)
          //   otherwise          → 0 (no fix / autonomous)
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

          // RTK fixed (fix_type >= 4) with reasonable accuracy → GPS is fixed.
          context_->gps_is_fixed = (context_->gps_fix_type >= 4) && (msg->position_accuracy < 0.1f);
          context_->gps_quality = std::clamp(1.0f - msg->position_accuracy, 0.0f, 1.0f);
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
          {
            std::lock_guard<std::mutex> lock(context_->context_mutex);
            context_->current_command = req->command;
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
    blackboard_->set("battery_low_pct", static_cast<float>(battery_low_pct));
    blackboard_->set("battery_critical_pct", static_cast<float>(battery_critical_pct));
    blackboard_->set("battery_full_pct", static_cast<float>(battery_full_pct));
    blackboard_->set("battery_critical_voltage", static_cast<float>(battery_critical_voltage));

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

  // Subscribers
  rclcpp::Subscription<mowgli_interfaces::msg::Status>::SharedPtr status_sub_;
  rclcpp::Subscription<mowgli_interfaces::msg::Emergency>::SharedPtr emergency_sub_;
  rclcpp::Subscription<mowgli_interfaces::msg::Power>::SharedPtr power_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr replan_needed_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr boundary_violation_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lethal_boundary_violation_sub_;
  rclcpp::Subscription<mowgli_interfaces::msg::AbsolutePose>::SharedPtr gps_sub_;
  rclcpp::Subscription<nav2_msgs::msg::CollisionMonitorState>::SharedPtr collision_monitor_sub_;

  // Service server
  rclcpp::Service<mowgli_interfaces::srv::HighLevelControl>::SharedPtr high_level_control_srv_;
  rclcpp::Service<mowgli_interfaces::srv::StartInArea>::SharedPtr start_in_area_srv_;

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

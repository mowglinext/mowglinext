// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// retrace_server — a drop-in replacement for Nav2's BackUp in the navigate-to-pose recovery.
//
// It serves nav2_msgs/action/BackUp on `/retrace` (the BT BackUp node selects it with
// server_name="retrace"), but instead of reversing blindly along the current heading it undoes the
// last piece the robot actually drove: straight back over breadcrumbs, never turning, never driving
// forward, never further than the recorded path. See mowgli_nav2_plugins/retrace.hpp for why.
//
// Motion goes out on /cmd_vel_nav like every other Nav2 server, so collision_monitor still filters
// it. Anything the node cannot do cleanly it REFUSES, and the recovery round-robin moves on to
// clearing the costmaps and waiting.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "mowgli_nav2_plugins/retrace.hpp"
#include "nav2_msgs/action/back_up.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace mowgli_nav2_plugins
{
namespace
{
// Result.error_code values of the Nav2 behaviours (BackUp: UNKNOWN=710, TIMEOUT=711 ...). Spelled
// out so the node does not depend on the exact constant names of one Nav2 release.
constexpr uint16_t kErrorUnknown = 710;
constexpr uint16_t kErrorTimeout = 711;
}  // namespace

class RetraceServer : public rclcpp::Node
{
public:
  using BackUp = nav2_msgs::action::BackUp;
  using GoalHandle = rclcpp_action::ServerGoalHandle<BackUp>;

  RetraceServer() : rclcpp::Node("retrace_server")
  {
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel_nav");
    max_distance_m_ = declare_parameter<double>("max_distance_m", 0.40);
    max_speed_mps_ = declare_parameter<double>("max_speed_mps", 0.20);
    min_speed_mps_ = declare_parameter<double>("min_speed_mps", 0.08);
    record_hz_ = declare_parameter<double>("record_hz", 20.0);
    stall_timeout_s_ = declare_parameter<double>("stall_timeout_s", 2.0);
    retrace::Limits limits;
    limits.min_step_m = declare_parameter<double>("min_step_m", limits.min_step_m);
    limits.history_m = declare_parameter<double>("history_m", limits.history_m);
    limits.min_distance_m = declare_parameter<double>("min_distance_m", limits.min_distance_m);
    limits.max_heading_error_rad =
        declare_parameter<double>("max_heading_error_rad", limits.max_heading_error_rad);
    limits.max_lateral_dev_m =
        declare_parameter<double>("max_lateral_dev_m", limits.max_lateral_dev_m);
    crumbs_ = retrace::Breadcrumbs(limits);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(cmd_vel_topic_, rclcpp::QoS(1));

    record_timer_ =
        create_wall_timer(std::chrono::duration<double>(1.0 / std::max(record_hz_, 1.0)),
                          [this]()
                          {
                            retrace::Pose2D pose;
                            if (!lookupPose(pose))
                              return;
                            std::lock_guard<std::mutex> lock(mutex_);
                            // While a retrace runs the robot drives back over its own breadcrumbs:
                            // do not record that, or the history would describe the undo as if it
                            // were new driving.
                            if (!executing_)
                              crumbs_.add(pose);
                          });

    server_ = rclcpp_action::create_server<BackUp>(
        this,
        "retrace",
        [this](const rclcpp_action::GoalUUID&, std::shared_ptr<const BackUp::Goal>)
        {
          return executing_.load() ? rclcpp_action::GoalResponse::REJECT
                                   : rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](std::shared_ptr<GoalHandle>)
        {
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](std::shared_ptr<GoalHandle> handle)
        {
          executing_ = true;
          std::thread(
              [this, handle]()
              {
                execute(handle);
              })
              .detach();
        });

    RCLCPP_INFO(get_logger(),
                "retrace_server ready: /retrace undoes the last <= %.2f m driven, at <= %.2f m/s, "
                "straight back only (%s → %s)",
                max_distance_m_,
                max_speed_mps_,
                odom_frame_.c_str(),
                base_frame_.c_str());
  }

private:
  bool lookupPose(retrace::Pose2D& pose)
  {
    try
    {
      const auto tf = tf_buffer_->lookupTransform(odom_frame_, base_frame_, tf2::TimePointZero);
      pose.x = tf.transform.translation.x;
      pose.y = tf.transform.translation.y;
      const auto& q = tf.transform.rotation;
      pose.yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      return true;
    }
    catch (const std::exception&)
    {
      return false;
    }
  }

  void publish(double vx)
  {
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = now();
    cmd.header.frame_id = base_frame_;
    cmd.twist.linear.x = vx;
    cmd_pub_->publish(cmd);
  }

  void finish(const std::shared_ptr<GoalHandle>& handle,
              bool success,
              uint16_t code,
              const std::string& message)
  {
    publish(0.0);
    auto result = std::make_shared<BackUp::Result>();
    result->error_code = success ? 0 : code;
    result->error_msg = message;
    if (success)
      handle->succeed(result);
    else if (handle->is_canceling())
      handle->canceled(result);
    else
      handle->abort(result);
    executing_ = false;
  }

  void execute(const std::shared_ptr<GoalHandle>& handle)
  {
    const auto goal = handle->get_goal();
    const double requested = std::abs(static_cast<double>(goal->target.x));
    const double distance = std::min(requested, max_distance_m_);
    const double speed =
        std::clamp(static_cast<double>(goal->speed), min_speed_mps_, max_speed_mps_);
    const double allowance_s = rclcpp::Duration(goal->time_allowance).seconds() > 0.0
                                   ? rclcpp::Duration(goal->time_allowance).seconds()
                                   : distance / speed * 3.0 + 2.0;

    retrace::Pose2D start;
    if (!lookupPose(start))
    {
      finish(handle,
             false,
             kErrorUnknown,
             "retrace: no " + odom_frame_ + "→" + base_frame_ + " transform");
      return;
    }
    retrace::Plan plan;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      plan = crumbs_.plan(start, distance);
    }
    if (plan.refusal != retrace::Refusal::kNone)
    {
      const char* why =
          plan.refusal == retrace::Refusal::kNoHistory ? "not enough recorded path to undo"
          : plan.refusal == retrace::Refusal::kWrongDirection
              ? "the last piece was driven in reverse, undoing it means driving forward"
              : "the last piece was not a straight line behind the robot";
      RCLCPP_WARN(get_logger(), "retrace refused: %s", why);
      finish(handle, false, kErrorUnknown, std::string("retrace refused: ") + why);
      return;
    }

    RCLCPP_INFO(get_logger(),
                "retrace: undoing %.2f m straight back at %.2f m/s (target %.2f, %.2f)",
                plan.distance_m,
                speed,
                plan.target.x,
                plan.target.y);

    const auto begin = now();
    auto last_progress_time = begin;
    double last_progress = 0.0;
    rclcpp::Rate rate(20.0);
    while (rclcpp::ok())
    {
      if (handle->is_canceling())
      {
        finish(handle, false, kErrorUnknown, "retrace canceled");
        return;
      }
      retrace::Pose2D pose;
      if (!lookupPose(pose))
      {
        finish(handle, false, kErrorUnknown, "retrace: transform lost");
        return;
      }
      const double progress = retrace::progressAlong(start, pose, plan.ux, plan.uy);
      if (progress >= plan.distance_m - 0.01)
      {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          crumbs_.retreat(plan.distance_m);
        }
        finish(handle, true, 0, "");
        return;
      }
      if ((now() - begin).seconds() > allowance_s)
      {
        finish(handle, false, kErrorTimeout, "retrace timed out");
        return;
      }
      if (progress > last_progress + 0.01)
      {
        last_progress = progress;
        last_progress_time = now();
      }
      else if ((now() - last_progress_time).seconds() > stall_timeout_s_)
      {
        finish(handle, false, kErrorUnknown, "retrace made no progress");
        return;
      }
      // Straight back: no steering, so the manoeuvre can only move along the recorded line.
      publish(-speed);
      rate.sleep();
    }
    finish(handle, false, kErrorUnknown, "retrace: shutting down");
  }

  std::string odom_frame_;
  std::string base_frame_;
  std::string cmd_vel_topic_;
  double max_distance_m_{0.40};
  double max_speed_mps_{0.20};
  double min_speed_mps_{0.08};
  double record_hz_{20.0};
  double stall_timeout_s_{2.0};

  std::mutex mutex_;
  std::atomic<bool> executing_{false};
  retrace::Breadcrumbs crumbs_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr record_timer_;
  rclcpp_action::Server<BackUp>::SharedPtr server_;
};

}  // namespace mowgli_nav2_plugins

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mowgli_nav2_plugins::RetraceServer>());
  rclcpp::shutdown();
  return 0;
}

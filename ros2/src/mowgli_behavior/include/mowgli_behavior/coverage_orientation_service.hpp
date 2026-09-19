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
#include <deque>
#include <exception>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>

#include "mowgli_behavior/bt_context.hpp"
#include "mowgli_behavior/coverage_persistence.hpp"
#include "mowgli_interfaces/srv/coverage_orientation.hpp"
#include "mowgli_interfaces/srv/get_mowing_area.hpp"

namespace mowgli_behavior
{
// Callbacks only enqueue. Context access and persistence belong to processPending,
// called before each BT tick. A successful deferred response means saved, not queued.
class CoverageOrientationService
{
  using Service = mowgli_interfaces::srv::CoverageOrientation;
  using Area = mowgli_interfaces::srv::GetMowingArea;
  struct Pending
  {
    std::shared_ptr<rmw_request_id_t> header;
    Service::Request::SharedPtr request;
  };

public:
  CoverageOrientationService(rclcpp::Node& owner, const std::shared_ptr<BTContext>& context)
      : context_(context)
  {
    area_client_ = context->helper_node->create_client<Area>("/map_server_node/get_mowing_area");
    service_ = owner.create_service<Service>(
        "~/coverage_orientation",
        [this](std::shared_ptr<rmw_request_id_t> header, Service::Request::SharedPtr request)
        {
          std::lock_guard<std::mutex> lock(queue_mutex_);
          if (queue_.size() >= 32)
          {
            Service::Response response;
            response.message = "Coverage orientation queue is busy; retry later";
            service_->send_response(*header, response);
            return;
          }
          queue_.push_back({std::move(header), std::move(request)});
        },
        rclcpp::ServicesQoS().keep_last(64));
  }

  ~CoverageOrientationService()
  {
    if (area_future_)
      area_client_->remove_pending_request(*area_future_);
    // Destruction runs after the owning executor stops. Complete deferred
    // clients when ROS is still live; after ROS shutdown there is no transport.
    if (!rclcpp::ok())
      return;
    try
    {
      if (active_)
        reply("Coverage orientation service is stopping");
      for (const auto& pending : queue_)
      {
        Service::Response response;
        response.message = "Coverage orientation service is stopping";
        service_->send_response(*pending.header, response);
      }
    }
    catch (const std::exception&)
    {
      // ROS may shut down between the validity check and send_response.
    }
  }

  // Must have the same single owner as BT ticks; never call from a service callback.
  void processPending()
  {
    if (!active_)
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      if (queue_.empty())
        return;
      active_ = std::move(queue_.front());
      queue_.pop_front();
    }
    auto ctx = context_.lock();
    if (!ctx)
      return reply("Behavior context is unavailable");
    if (!area_future_)
    {
      if (!area_client_->service_is_ready())
        return reply("Map area validation is unavailable");
      auto request = std::make_shared<Area::Request>();
      request->index = active_->request->area_index;
      area_future_.emplace(area_client_->async_send_request(request));
      deadline_ = std::chrono::steady_clock::now() + std::chrono::seconds(2);
      return;
    }
    if (area_future_->future.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
    {
      if (std::chrono::steady_clock::now() >= deadline_)
      {
        area_client_->remove_pending_request(*area_future_);
        reply("Map area validation timed out");
      }
      return;
    }
    auto area = area_future_->future.get();
    area_future_.reset();
    if (!area->success || area->area.is_navigation_area)
      return reply("Area is missing or is navigation-only");
    const auto& req = *active_->request;
    if (req.set_next)
    {
      const auto it = ctx->cross_hatch.find(req.area_index);
      const auto previous = it == ctx->cross_hatch.end() ? std::optional<CrossHatch>{} : it->second;
      auto& state = ctx->cross_hatch[req.area_index];
      // A base-only disabled session must not be rotated by a next-session edit.
      if (ctx->base_orientation_areas.count(req.area_index) && !state.session_perpendicular)
        state.begin(false);
      state.next_override = req.perpendicular;
      state.failed_sessions = 0;
      state.planning_failed = false;
      if (!saveCoverageResumeState(*ctx))
      {
        if (previous)
          ctx->cross_hatch[req.area_index] = *previous;
        else
          ctx->cross_hatch.erase(req.area_index);
        return reply("Could not persist the next coverage orientation");
      }
    }
    const auto it = ctx->cross_hatch.find(req.area_index);
    const auto state = it == ctx->cross_hatch.end() ? CrossHatch{} : it->second;
    Service::Response response;
    response.success = true;
    response.enabled = ctx->mow_cross_hatch;
    ctx->node->get_parameter_or("mow_angle_deg", response.base_angle_deg, -1.0);
    response.current_active = state.session_perpendicular.has_value() ||
                              ctx->base_orientation_areas.count(req.area_index);
    response.current_perpendicular = state.session_perpendicular.value_or(false);
    response.next_perpendicular = state.next();
    service_->send_response(*active_->header, response);
    active_.reset();
  }

private:
  void reply(const std::string& message)
  {
    Service::Response response;
    response.message = message;
    service_->send_response(*active_->header, response);
    active_.reset();
    area_future_.reset();
  }

  std::weak_ptr<BTContext> context_;
  std::mutex queue_mutex_;
  std::deque<Pending> queue_;
  std::optional<Pending> active_;
  rclcpp::Client<Area>::SharedPtr area_client_;
  std::optional<rclcpp::Client<Area>::FutureAndRequestId> area_future_;
  std::chrono::steady_clock::time_point deadline_;
  rclcpp::Service<Service>::SharedPtr service_;
};
}  // namespace mowgli_behavior

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

#include <memory>
#include <mutex>

#include "mowgli_behavior/bt_context.hpp"
#include "mowgli_behavior/coverage_persistence.hpp"
#include "mowgli_interfaces/srv/fleet_yield.hpp"
#include "mowgli_interfaces/update_maintenance.hpp"

namespace mowgli_behavior
{
// Caller holds context_mutex. Maintenance retains its existing terminal STOP
// policy; no fleet request or clearing of the gate can restart the mower.
inline void refreshFleetYieldLocked(BTContext& ctx)
{
  const bool maintenance = mowgli_interfaces::updateMaintenanceActive();
  if (maintenance)
    ctx.current_command = 8;
  ctx.fleet_yield.observeInterlock(
      maintenance || ctx.latest_emergency.active_emergency ||
      ctx.latest_emergency.latched_emergency || ctx.latest_status.is_charging ||
      ctx.critical_charge_stop_latched || ctx.critical_dock_failure_latched);
}

class FleetYieldService
{
public:
  using Service = mowgli_interfaces::srv::FleetYield;
  FleetYieldService(rclcpp::Node& owner, const std::shared_ptr<BTContext>& context)
  {
    service_ = owner.create_service<Service>(
        "~/fleet_yield",
        [weak = std::weak_ptr<BTContext>(context)](const Service::Request::SharedPtr req,
                                                   Service::Response::SharedPtr resp)
        {
          auto ctx = weak.lock();
          if (!ctx)
          {
            resp->result = Service::Response::INELIGIBLE;
            return;
          }
          bool persist_stop = false;
          {
            std::lock_guard<std::mutex> lock(ctx->context_mutex);
            // Completed duplicates acknowledge without ANY context mutation.
            if (req->operation == Service::Request::RESUME &&
                ctx->fleet_yield.completed(req->epoch, req->request_id))
            {
              resp->result = Service::Response::ALREADY_RESUMED;
              resp->epoch = ctx->fleet_yield.epoch();
              resp->held = ctx->fleet_yield.held();
              return;
            }
            refreshFleetYieldLocked(*ctx);
            const bool was_held = ctx->fleet_yield.held();
            resp->result = ctx->fleet_yield.apply(req->operation,
                                                  req->epoch,
                                                  req->request_id,
                                                  ctx->current_command);
            persist_stop = resp->result == Service::Response::HELD && !was_held;
          }
          // The default MutuallyExclusive callback group serializes this
          // snapshot with the tick, operator services and subscriber callbacks.
          // Do not hold context_mutex across disk I/O or tick the tree here.
          // Replace a previous persisted START before acknowledging PAUSE so
          // process restart cannot auto-continue the held run. Atomic rename
          // is not a claim of fsync/power-loss durability.
          if (persist_stop && !ctx->coverage_resume_path.empty() && !saveCoverageResumeState(*ctx))
          {
            std::lock_guard<std::mutex> lock(ctx->context_mutex);
            ctx->fleet_yield.revoke();
            resp->result = Service::Response::INELIGIBLE;
            resp->message = "STOP retained in memory; pause checkpoint failed; ownership revoked";
          }
          {
            std::lock_guard<std::mutex> lock(ctx->context_mutex);
            resp->epoch = ctx->fleet_yield.epoch();
            resp->held = ctx->fleet_yield.held();
          }
        },
        rclcpp::ServicesQoS(),
        owner.get_node_base_interface()->get_default_callback_group());
  }

private:
  rclcpp::Service<Service>::SharedPtr service_;
};
}  // namespace mowgli_behavior

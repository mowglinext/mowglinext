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
#include <string>

#include "mowgli_behavior/bt_context.hpp"
#include "mowgli_interfaces/srv/blade_control.hpp"
#include "mowgli_interfaces/srv/mower_control.hpp"

namespace mowgli_behavior
{

/// Shares the tick/start handlers' default MutuallyExclusive group. Policy
/// mutation AND command dispatch must be serialized, not separately locked.
class BladeControlService
{
public:
  using Control = mowgli_interfaces::srv::BladeControl;
  using Hardware = mowgli_interfaces::srv::MowerControl;

  BladeControlService(rclcpp::Node& owner, const std::shared_ptr<BTContext>& context)
  {
    auto handle = [weak = std::weak_ptr<BTContext>(
                       context)](uint8_t enabled, uint8_t direction, Control::Response& resp)
    {
      auto ctx = weak.lock();
      if (!ctx)
      {
        resp.message = "Blade controller is unavailable";
        return;
      }
      if (enabled > 1u || (enabled != 0u && direction > 1u))
      {
        resp.message = "Blade enable and ON direction must be 0 or 1";
        return;
      }
      auto client = ctx->bladeClient();
      if (enabled && !client->service_is_ready())
      {
        resp.message = "Hardware bridge is unavailable; direction was not changed";
        return;
      }
      // OFF ignores direction and always latches, including during an outage.
      const auto command = ctx->blade_direction.forOperatorCommand(enabled != 0u, direction);
      resp.success = true;
      resp.message =
          enabled ? (command.enabled
                         ? "Blade direction requested"
                         : "Direction saved; the behavior tree currently keeps the blade off")
                  : "Blade OFF latched";
      if (!client->service_is_ready())
      {
        resp.message = "Blade OFF latched; hardware bridge is unavailable";
        return;
      }
      auto request = std::make_shared<Hardware::Request>();
      request->mow_enabled = command.enabled;
      request->mow_direction = command.direction;
      try
      {
        client->async_send_request(request);
        resp.forwarded = true;
      }
      catch (const std::exception& e)
      {
        resp.message += std::string("; hardware request could not be queued: ") + e.what();
      }
    };
    auto group = owner.get_node_base_interface()->get_default_callback_group();
    service_ = owner.create_service<Control>(
        "~/blade_control",
        [handle](const Control::Request::SharedPtr req, Control::Response::SharedPtr resp)
        {
          handle(req->mow_enabled, req->mow_direction, *resp);
        },
        rclcpp::ServicesQoS(),
        group);
  }

private:
  rclcpp::Service<Control>::SharedPtr service_;
};
}  // namespace mowgli_behavior

// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0

#include "mowgli_openmower_bridge/xesc_2040_link.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <utility>

namespace mowgli_openmower_bridge
{

namespace
{
constexpr int kXescBaud = 115200;
constexpr std::size_t kReadChunk = 256u;
constexpr std::chrono::milliseconds kSettingsResendPeriod{500};
}  // namespace

Xesc2040Link::Xesc2040Link(std::string port, Xesc2040Settings settings, LogFn warn)
    : port_(std::move(port)), settings_(settings), warn_(std::move(warn)), serial_(port_, kXescBaud)
{
}

void Xesc2040Link::Poll(SteadyClock::time_point now)
{
  if (!serial_.is_open())
  {
    telemetry_.connected = false;
    if (now - last_reconnect_attempt_ < kMotorReconnectPeriod)
    {
      return;
    }
    last_reconnect_attempt_ = now;
    if (!serial_.open())
    {
      return;
    }
    packets_.reset_receive_state();
    telemetry_.has_status = false;
    have_previous_status_ = false;
    SendSettings(now);
  }

  uint8_t buf[kReadChunk];
  while (true)
  {
    const ssize_t n = serial_.read(buf, kReadChunk);
    if (n < 0)
    {
      warn_("xESC 2040 read error on " + port_ + " — reopening");
      serial_.close();
      telemetry_.connected = false;
      return;
    }
    if (n == 0)
    {
      break;
    }
    packets_.set_callback(
        [this, now](const uint8_t* data, std::size_t len)
        {
          OnPacket(data, len, now);
        });
    packets_.feed(buf, static_cast<std::size_t>(n));
  }

  telemetry_.connected =
      telemetry_.has_status && (now - telemetry_.last_status) <= kMotorStatusTimeout;
}

void Xesc2040Link::OnPacket(const uint8_t* data, std::size_t len, SteadyClock::time_point now)
{
  if (len == 0u || data[0] != xesc2040::kMsgTypeStatus)
  {
    return;
  }
  if (len != sizeof(xesc2040::StatusPacket))
  {
    warn_("xESC 2040 status packet with wrong size on " + port_);
    return;
  }
  xesc2040::StatusPacket pkt{};
  std::memcpy(&pkt, data, sizeof(pkt));
  HandleStatus(pkt, now);
}

void Xesc2040Link::HandleStatus(const xesc2040::StatusPacket& pkt, SteadyClock::time_point now)
{
  // tacho_absolute only ever grows; the controller tells us which way it was
  // turning. Turn that into a signed cumulative count so the bridge sees the
  // same shape from both controller flavours.
  if (have_previous_status_)
  {
    const uint32_t delta = pkt.tacho_absolute - previous_tacho_absolute_;
    const int64_t signed_delta =
        pkt.direction ? -static_cast<int64_t>(delta) : static_cast<int64_t>(delta);
    telemetry_.signed_ticks += signed_delta;
  }
  previous_tacho_absolute_ = pkt.tacho_absolute;
  have_previous_status_ = true;

  telemetry_.has_status = true;
  telemetry_.last_status = now;
  telemetry_.fw_major = pkt.fw_version_major;
  telemetry_.fw_minor = pkt.fw_version_minor;
  telemetry_.voltage_in = pkt.voltage_input;
  telemetry_.temp_pcb = pkt.temperature_pcb;
  telemetry_.temp_motor = pkt.temperature_motor;
  telemetry_.current_in = pkt.current_input;
  telemetry_.duty = pkt.duty_cycle;
  telemetry_.rpm = 0.0;  // the 2040 does not report shaft speed
  telemetry_.fault_code = static_cast<uint32_t>(pkt.fault_code);
  telemetry_.connected = true;

  if ((telemetry_.fault_code & xesc2040::kFaultUninitialized) != 0u &&
      now - last_settings_sent_ >= kSettingsResendPeriod)
  {
    SendSettings(now);
  }
}

bool Xesc2040Link::SendDuty(double duty)
{
  if (!std::isfinite(duty))
  {
    duty = 0.0;
  }
  xesc2040::ControlPacket pkt{};
  pkt.message_type = xesc2040::kMsgTypeControl;
  pkt.duty_cycle = std::clamp(duty, -1.0, 1.0);
  return SendRaw(reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt) - sizeof(uint16_t));
}

void Xesc2040Link::SendSettings(SteadyClock::time_point now)
{
  last_settings_sent_ = now;
  xesc2040::SettingsPacket pkt{};
  pkt.message_type = xesc2040::kMsgTypeSettings;
  std::memcpy(pkt.hall_table, settings_.hall_table, sizeof(pkt.hall_table));
  pkt.motor_current_limit = settings_.motor_current_limit;
  pkt.acceleration = settings_.acceleration;
  pkt.has_motor_temp = settings_.has_motor_temp;
  pkt.min_motor_temp = settings_.min_motor_temp;
  pkt.max_motor_temp = settings_.max_motor_temp;
  pkt.min_pcb_temp = settings_.min_pcb_temp;
  pkt.max_pcb_temp = settings_.max_pcb_temp;
  SendRaw(reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt) - sizeof(uint16_t));
}

bool Xesc2040Link::SendRaw(const uint8_t* payload, std::size_t len)
{
  if (!serial_.is_open())
  {
    return false;
  }
  const std::vector<uint8_t> frame = packets_.encode_packet(payload, len);
  const ssize_t written = serial_.write_all(frame.data(), frame.size());
  if (written < 0 || static_cast<std::size_t>(written) != frame.size())
  {
    warn_("xESC 2040 short write on " + port_ + " — reopening");
    serial_.close();
    telemetry_.connected = false;
    return false;
  }
  return true;
}

}  // namespace mowgli_openmower_bridge

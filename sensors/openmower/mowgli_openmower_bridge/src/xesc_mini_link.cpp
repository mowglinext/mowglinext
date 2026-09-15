// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0

#include "mowgli_openmower_bridge/xesc_mini_link.hpp"

#include <algorithm>
#include <utility>

namespace mowgli_openmower_bridge
{

namespace
{
constexpr int kXescBaud = 115200;
constexpr std::size_t kReadChunk = 256u;
constexpr std::chrono::milliseconds kFwRequestPeriod{1000};
}  // namespace

XescMiniLink::XescMiniLink(std::string port, int motor_pole_pairs, LogFn warn)
    : port_(std::move(port)),
      pole_pairs_(std::max(1, motor_pole_pairs)),
      warn_(std::move(warn)),
      serial_(port_, kXescBaud)
{
}

void XescMiniLink::Poll(SteadyClock::time_point now)
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
    deframer_ = vesc::Deframer{};
    telemetry_.has_status = false;
    fw_known_ = false;
  }

  uint8_t buf[kReadChunk];
  while (true)
  {
    const ssize_t n = serial_.read(buf, kReadChunk);
    if (n < 0)
    {
      warn_("xESC mini read error on " + port_ + " — reopening");
      serial_.close();
      telemetry_.connected = false;
      return;
    }
    if (n == 0)
    {
      break;
    }
    deframer_.Feed(buf, static_cast<std::size_t>(n));
  }

  while (auto payload = deframer_.NextPayload())
  {
    if (const auto parsed = vesc::ParsePayload(payload->data(), payload->size()))
    {
      HandlePayload(*parsed, now);
    }
  }

  // The VESC protocol is request/response: keep asking. Firmware version
  // first so the log names what is on the wire, then the values stream.
  if (!fw_known_)
  {
    if (now - last_fw_request_ >= kFwRequestPeriod)
    {
      last_fw_request_ = now;
      SendFrame(vesc::BuildFwVersionRequest());
    }
  }
  else
  {
    SendFrame(vesc::BuildGetValuesRequest());
  }

  telemetry_.connected =
      telemetry_.has_status && (now - telemetry_.last_status) <= kMotorStatusTimeout;
}

void XescMiniLink::HandlePayload(const vesc::ParsedPayload& parsed, SteadyClock::time_point now)
{
  switch (parsed.kind)
  {
    case vesc::PayloadKind::kFwVersion:
      telemetry_.fw_major = parsed.fw.major;
      telemetry_.fw_minor = parsed.fw.minor;
      fw_known_ = true;
      return;
    case vesc::PayloadKind::kValues:
      telemetry_.has_status = true;
      telemetry_.last_status = now;
      telemetry_.voltage_in = parsed.values.voltage_in;
      telemetry_.temp_pcb = parsed.values.temp_pcb;
      telemetry_.temp_motor = parsed.values.temp_motor;
      telemetry_.current_in = parsed.values.current_in;
      telemetry_.duty = parsed.values.duty;
      telemetry_.rpm = parsed.values.erpm / static_cast<double>(pole_pairs_);
      telemetry_.fault_code = parsed.values.fault_code;
      // The VESC tachometer is already a signed cumulative count.
      telemetry_.signed_ticks = parsed.values.tacho;
      telemetry_.connected = true;
      return;
    case vesc::PayloadKind::kOther:
      return;
  }
}

bool XescMiniLink::SendDuty(double duty)
{
  return SendFrame(vesc::BuildSetDuty(duty));
}

bool XescMiniLink::SendFrame(const std::vector<uint8_t>& frame)
{
  if (!serial_.is_open())
  {
    return false;
  }
  const ssize_t written = serial_.write_all(frame.data(), frame.size());
  if (written < 0 || static_cast<std::size_t>(written) != frame.size())
  {
    warn_("xESC mini short write on " + port_ + " — reopening");
    serial_.close();
    telemetry_.connected = false;
    return false;
  }
  return true;
}

}  // namespace mowgli_openmower_bridge

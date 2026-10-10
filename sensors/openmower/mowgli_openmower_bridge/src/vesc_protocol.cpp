// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0

#include "mowgli_openmower_bridge/vesc_protocol.hpp"

#include <algorithm>
#include <cmath>

namespace mowgli_openmower_bridge::vesc
{

namespace
{

constexpr std::size_t kSmallHeaderSize = 2u;  // SOF + 1-byte length
constexpr std::size_t kLargeHeaderSize = 3u;  // SOF + 2-byte length
constexpr std::size_t kTrailerSize = 3u;  // CRC hi, CRC lo, EOF
constexpr std::size_t kMinFrameSize = kSmallHeaderSize + kTrailerSize;

int16_t ReadInt16(const uint8_t* p) noexcept
{
  return static_cast<int16_t>((static_cast<uint16_t>(p[0]) << 8u) | p[1]);
}

int32_t ReadInt32(const uint8_t* p) noexcept
{
  return static_cast<int32_t>((static_cast<uint32_t>(p[0]) << 24u) |
                              (static_cast<uint32_t>(p[1]) << 16u) |
                              (static_cast<uint32_t>(p[2]) << 8u) | p[3]);
}

void WriteInt32(std::vector<uint8_t>& out, int32_t value)
{
  const auto v = static_cast<uint32_t>(value);
  out.push_back(static_cast<uint8_t>((v >> 24u) & 0xFFu));
  out.push_back(static_cast<uint8_t>((v >> 16u) & 0xFFu));
  out.push_back(static_cast<uint8_t>((v >> 8u) & 0xFFu));
  out.push_back(static_cast<uint8_t>(v & 0xFFu));
}

}  // namespace

uint16_t crc16_xmodem(const uint8_t* data, std::size_t len) noexcept
{
  uint16_t crc = 0x0000u;
  for (std::size_t i = 0; i < len; ++i)
  {
    crc ^= static_cast<uint16_t>(data[i]) << 8u;
    for (int bit = 0; bit < 8; ++bit)
    {
      crc = (crc & 0x8000u) != 0u ? static_cast<uint16_t>((crc << 1u) ^ 0x1021u)
                                  : static_cast<uint16_t>(crc << 1u);
    }
  }
  return crc;
}

std::vector<uint8_t> BuildFrame(const std::vector<uint8_t>& payload)
{
  std::vector<uint8_t> frame;
  frame.reserve(payload.size() + kLargeHeaderSize + kTrailerSize);
  if (payload.size() < 256u)
  {
    frame.push_back(kSofSmallFrame);
    frame.push_back(static_cast<uint8_t>(payload.size()));
  }
  else
  {
    frame.push_back(kSofLargeFrame);
    frame.push_back(static_cast<uint8_t>((payload.size() >> 8u) & 0xFFu));
    frame.push_back(static_cast<uint8_t>(payload.size() & 0xFFu));
  }
  frame.insert(frame.end(), payload.begin(), payload.end());
  const uint16_t crc = crc16_xmodem(payload.data(), payload.size());
  frame.push_back(static_cast<uint8_t>(crc >> 8u));
  frame.push_back(static_cast<uint8_t>(crc & 0xFFu));
  frame.push_back(kEof);
  return frame;
}

std::vector<uint8_t> BuildFwVersionRequest()
{
  return BuildFrame({COMM_FW_VERSION});
}

std::vector<uint8_t> BuildGetValuesRequest()
{
  return BuildFrame({COMM_GET_VALUES});
}

std::vector<uint8_t> BuildSetDuty(double duty)
{
  if (!std::isfinite(duty))
  {
    duty = 0.0;
  }
  duty = std::clamp(duty, -1.0, 1.0);
  std::vector<uint8_t> payload{COMM_SET_DUTY};
  WriteInt32(payload, static_cast<int32_t>(duty * 100000.0));
  return BuildFrame(payload);
}

std::optional<ParsedPayload> ParsePayload(const uint8_t* payload, std::size_t len)
{
  if (payload == nullptr || len == 0u)
  {
    return std::nullopt;
  }
  ParsedPayload out{};
  switch (payload[0])
  {
    case COMM_GET_VALUES:
    {
      if (len < kValuesPayloadMinSize)
      {
        return std::nullopt;
      }
      out.kind = PayloadKind::kValues;
      out.values.temp_pcb = ReadInt16(payload + kTempMos) / 10.0;
      out.values.temp_motor = ReadInt16(payload + kTempMotor) / 10.0;
      out.values.current_motor = ReadInt32(payload + kCurrentMotor) / 100.0;
      out.values.current_in = ReadInt32(payload + kCurrentIn) / 100.0;
      out.values.duty = ReadInt16(payload + kDutyNow) / 1000.0;
      out.values.erpm = static_cast<double>(ReadInt32(payload + kErpm));
      out.values.voltage_in = ReadInt16(payload + kVoltageIn) / 10.0;
      out.values.tacho = ReadInt32(payload + kTachometer);
      out.values.tacho_abs = ReadInt32(payload + kTachometerAbs);
      out.values.fault_code = payload[kFaultCode];
      return out;
    }
    case COMM_FW_VERSION:
    {
      if (len < 3u)
      {
        return std::nullopt;
      }
      out.kind = PayloadKind::kFwVersion;
      out.fw.major = payload[1];
      out.fw.minor = payload[2];
      return out;
    }
    default:
      out.kind = PayloadKind::kOther;
      return out;
  }
}

void Deframer::Feed(const uint8_t* data, std::size_t len)
{
  if (data == nullptr || len == 0u)
  {
    return;
  }
  buffer_.insert(buffer_.end(), data, data + len);
  if (buffer_.size() > kMaxBufferedBytes)
  {
    const std::size_t excess = buffer_.size() - kMaxBufferedBytes;
    buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(excess));
    resync_drops_ += excess;
  }
}

namespace
{

enum class Candidate
{
  kInvalid,  ///< not a frame here — skip this byte
  kIncomplete,  ///< could be a frame, needs more bytes
  kBadCrc,  ///< complete frame shape, but the CRC or EOF byte is wrong
  kValid,  ///< complete frame with a good CRC and EOF
};

struct FrameView
{
  Candidate state{Candidate::kInvalid};
  std::size_t payload_offset{0u};
  std::size_t payload_len{0u};
  std::size_t frame_len{0u};
};

FrameView InspectAt(const std::vector<uint8_t>& buf, std::size_t at)
{
  FrameView v{};
  const std::size_t avail = buf.size() - at;
  if (avail < kMinFrameSize)
  {
    v.state = (buf[at] == kSofSmallFrame || buf[at] == kSofLargeFrame) ? Candidate::kIncomplete
                                                                       : Candidate::kInvalid;
    return v;
  }
  std::size_t header = 0u;
  if (buf[at] == kSofSmallFrame)
  {
    header = kSmallHeaderSize;
    v.payload_len = buf[at + 1];
  }
  else if (buf[at] == kSofLargeFrame)
  {
    if (avail < kLargeHeaderSize + kTrailerSize)
    {
      v.state = Candidate::kIncomplete;
      return v;
    }
    header = kLargeHeaderSize;
    v.payload_len = (static_cast<std::size_t>(buf[at + 1]) << 8u) | buf[at + 2];
  }
  else
  {
    return v;
  }
  if (v.payload_len == 0u || v.payload_len > kMaxPayloadSize)
  {
    return v;
  }
  v.payload_offset = at + header;
  v.frame_len = header + v.payload_len + kTrailerSize;
  if (avail < v.frame_len)
  {
    v.state = Candidate::kIncomplete;
    return v;
  }
  const uint8_t* payload = buf.data() + v.payload_offset;
  const std::size_t crc_at = v.payload_offset + v.payload_len;
  const uint16_t crc_wire = static_cast<uint16_t>((buf[crc_at] << 8u) | buf[crc_at + 1u]);
  if (buf[crc_at + 2u] != kEof || crc_wire != crc16_xmodem(payload, v.payload_len))
  {
    v.state = Candidate::kBadCrc;
    return v;
  }
  v.state = Candidate::kValid;
  return v;
}

}  // namespace

std::optional<std::vector<uint8_t>> Deframer::NextPayload()
{
  std::optional<std::size_t> first_incomplete;
  for (std::size_t at = 0; at < buffer_.size(); ++at)
  {
    const FrameView v = InspectAt(buffer_, at);
    if (v.state == Candidate::kValid)
    {
      std::vector<uint8_t> out(buffer_.begin() + static_cast<std::ptrdiff_t>(v.payload_offset),
                               buffer_.begin() +
                                   static_cast<std::ptrdiff_t>(v.payload_offset + v.payload_len));
      resync_drops_ += at;  // everything before the frame was noise
      buffer_.erase(buffer_.begin(),
                    buffer_.begin() + static_cast<std::ptrdiff_t>(at + v.frame_len));
      return out;
    }
    if (v.state == Candidate::kIncomplete && !first_incomplete)
    {
      first_incomplete = at;
    }
    if (v.state == Candidate::kBadCrc)
    {
      ++crc_errors_;
    }
  }
  // No complete frame: keep only what might still become one.
  const std::size_t keep_from = first_incomplete.value_or(buffer_.size());
  resync_drops_ += keep_from;
  buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(keep_from));
  return std::nullopt;
}

}  // namespace mowgli_openmower_bridge::vesc

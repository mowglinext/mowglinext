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

// SPDX-License-Identifier: GPL-3.0
#include "mowgli_hardware/packet_handler.hpp"

#include <cstring>
#include <limits>
#include <stdexcept>

#include "mowgli_hardware/cobs.hpp"
#include "mowgli_hardware/crc16.hpp"

namespace mowgli_hardware
{

void PacketHandler::set_callback(PacketCallback cb)
{
  callback_ = std::move(cb);
}

void PacketHandler::feed(const uint8_t* data, std::size_t len)
{
  for (std::size_t i = 0; i < len; ++i)
  {
    const uint8_t byte = data[i];

    if (byte == 0x00)
    {
      // Packet delimiter: attempt to decode whatever is in the rx buffer.
      if (!rx_buf_.empty())
      {
        dispatch_frame();
      }
      rx_buf_.clear();
      overflowed_ = false;
    }
    else
    {
      if (overflowed_)
      {
        // Keep consuming bytes until the next delimiter, but do not store.
        continue;
      }
      if (rx_buf_.size() >= kMaxPacketBytes)
      {
        ++rx_overflow_;
        overflowed_ = true;
        rx_buf_.clear();
        continue;
      }
      rx_buf_.push_back(byte);
    }
  }
}

void PacketHandler::reset_receive_state()
{
  rx_buf_.clear();
  overflowed_ = false;
}

void PacketHandler::dispatch_frame()
{
  // Decode COBS.  The decoded output is at most as large as the encoded input.
  std::vector<uint8_t> decoded(rx_buf_.size());
  const std::size_t decoded_len = cobs_decode(rx_buf_.data(), rx_buf_.size(), decoded.data());

  if (decoded_len == 0)
  {
    ++rx_cobs_errors_;
    return;
  }

  // Minimum valid packet: 1 type byte + 2 CRC bytes.
  if (decoded_len < 3u)
  {
    ++rx_crc_errors_;
    return;
  }

  if (!verify_crc(decoded.data(), decoded_len))
  {
    ++rx_crc_errors_;
    return;
  }

  ++rx_ok_;

  if (callback_)
  {
    callback_(decoded.data(), decoded_len);
  }
}

std::vector<uint8_t> PacketHandler::encode_packet(const uint8_t* data, std::size_t len) const
{
  // Reject invalid input.
  if (len > 0 && data == nullptr)
  {
    throw std::invalid_argument("encode_packet: data is null while len > 0");
  }

  // Assemble payload + 2-byte CRC placeholder.
  if (len > std::numeric_limits<std::size_t>::max() - 2u)
  {
    throw std::overflow_error("encode_packet: payload size overflow");
  }

  const std::size_t payload_len = len + 2u;
  std::vector<uint8_t> payload(payload_len);

  if (len > 0)
  {
    std::memcpy(payload.data(), data, len);
  }

  append_crc(payload.data(), payload_len);

  // COBS-encode.
  const std::size_t encoded_cap = cobs_max_encoded_size(payload_len);
  std::vector<uint8_t> encoded(encoded_cap);
  const std::size_t encoded_len = cobs_encode(payload.data(), payload_len, encoded.data());

  // Frame: 0x00 + COBS bytes + 0x00.
  if (encoded_len > std::numeric_limits<std::size_t>::max() - 2u)
  {
    throw std::overflow_error("encode_packet: frame size overflow");
  }

  std::vector<uint8_t> frame;
  frame.reserve(encoded_len + 2u);
  frame.push_back(0x00);
  frame.insert(frame.end(), encoded.data(), encoded.data() + encoded_len);
  frame.push_back(0x00);

  return frame;
}

bool PacketHandler::verify_crc(const uint8_t* data, std::size_t len) noexcept
{
  if (len < 3u)
  {
    return false;
  }

  const std::size_t payload_len = len - 2u;
  const uint16_t computed = crc16_ccitt(data, payload_len);

  // CRC is stored little-endian in the last two bytes.
  const uint16_t received =
      static_cast<uint16_t>(data[len - 2u]) | (static_cast<uint16_t>(data[len - 1u]) << 8u);

  return computed == received;
}

void PacketHandler::append_crc(uint8_t* data, std::size_t len) noexcept
{
  if (len < 2u)
  {
    return;
  }

  const std::size_t payload_len = len - 2u;
  const uint16_t crc = crc16_ccitt(data, payload_len);

  // Store little-endian.
  data[len - 2u] = static_cast<uint8_t>(crc & 0xFFu);
  data[len - 1u] = static_cast<uint8_t>((crc >> 8u) & 0xFFu);
}

}  // namespace mowgli_hardware

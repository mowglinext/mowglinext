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
/**
 * @file packet_handler.hpp
 * @brief COBS packet framing, deframing, and CRC verification.
 *
 * The wire format is:
 *   0x00 | COBS-encoded payload | 0x00
 *
 * where the payload = raw_data_bytes + uint16_t CRC (little-endian).
 *
 * PacketHandler maintains a receive buffer.  Call feed() with raw serial
 * bytes as they arrive; it accumulates them and, on receipt of a 0x00
 * delimiter, COBS-decodes the accumulated frame and invokes the registered
 * PacketCallback with the decoded bytes (including the CRC at the end).
 *
 * For transmit, encode_packet() takes a raw payload (without CRC) and
 * returns a fully-framed byte vector ready to send over the wire.
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

namespace mowgli_hardware
{

/**
 * @brief Stateful COBS framer / deframer with CRC verification.
 */
class PacketHandler
{
public:
  /// Maximum raw packet size accepted (guards against run-away frames).
  static constexpr std::size_t kMaxPacketBytes = 512u;

  /**
   * @brief Callback signature invoked on each successfully decoded packet.
   *
   * @param data Pointer to decoded bytes (including the 2-byte trailing CRC).
   * @param len  Number of decoded bytes.
   *
   * The buffer pointed to by @p data is valid only for the duration of the
   * callback.  Copy if longer retention is needed.
   */
  using PacketCallback = std::function<void(const uint8_t* data, std::size_t len)>;

  PacketHandler() = default;

  /**
   * @brief Register the callback to invoke when a valid packet is received.
   *
   * Replacing the callback is safe only when feed() is not executing.
   *
   * @param cb Callback to invoke; may be nullptr to disable dispatching.
   */
  void set_callback(PacketCallback cb);

  /**
   * @brief Feed raw bytes from the serial port into the framer.
   *
   * Scans for 0x00 delimiters.  Each time a delimiter is found the
   * accumulated COBS frame is decoded; if the CRC checks out the
   * PacketCallback is called.  Corrupt or oversized frames are silently
   * discarded (error statistics are updated internally).
   *
   * @param data Pointer to freshly-read serial bytes.
   * @param len  Number of bytes to process.
   */
  void feed(const uint8_t* data, std::size_t len);

  /// Drop any partial frame currently buffered between delimiters.
  void reset_receive_state();

  /**
   * @brief Encode a raw payload into a fully-framed COBS packet.
   *
   * Computes and appends the CRC-16, COBS-encodes the result, then wraps
   * it in leading and trailing 0x00 delimiter bytes.
   *
   * @param data Pointer to the raw payload (type byte first, NO CRC).
   * @param len  Payload length in bytes.
   * @return Framed byte vector ready to pass to SerialPort::write().
   */
  [[nodiscard]] std::vector<uint8_t> encode_packet(const uint8_t* data, std::size_t len) const;

  /**
   * @brief Verify the CRC-16 appended to a decoded packet.
   *
   * The last two bytes of @p data are treated as a little-endian CRC-16
   * CCITT value.  The CRC is recomputed over bytes [0 … len-3] and compared.
   *
   * @param data Decoded packet bytes (payload + 2-byte CRC).
   * @param len  Total length including the CRC.
   * @return true if the CRC matches.
   */
  [[nodiscard]] static bool verify_crc(const uint8_t* data, std::size_t len) noexcept;

  /**
   * @brief Compute and write the CRC-16 into the last two bytes of @p data.
   *
   * The CRC is computed over bytes [0 … len-3] and stored in little-endian
   * order at bytes [len-2] and [len-1].  The caller must have allocated
   * those two trailing bytes.
   *
   * @param data Buffer of @p len bytes; last two bytes are overwritten.
   * @param len  Total buffer size including the two CRC bytes.
   */
  static void append_crc(uint8_t* data, std::size_t len) noexcept;

  // ---------------------------------------------------------------------------
  // Diagnostic counters (monotonically increasing, never reset)
  // ---------------------------------------------------------------------------

  /// Number of packets successfully decoded and dispatched.
  [[nodiscard]] uint64_t rx_ok() const noexcept
  {
    return rx_ok_;
  }

  /// Number of frames discarded due to CRC mismatch.
  [[nodiscard]] uint64_t rx_crc_errors() const noexcept
  {
    return rx_crc_errors_;
  }

  /// Number of frames discarded because they exceeded kMaxPacketBytes.
  [[nodiscard]] uint64_t rx_overflow() const noexcept
  {
    return rx_overflow_;
  }

  /// Number of COBS decoding failures.
  [[nodiscard]] uint64_t rx_cobs_errors() const noexcept
  {
    return rx_cobs_errors_;
  }

private:
  /// Process one complete COBS frame (contents of rx_buf_ up to rx_len_).
  void dispatch_frame();

  PacketCallback callback_{nullptr};

  // Receive accumulation buffer (raw COBS bytes between delimiters).
  std::vector<uint8_t> rx_buf_;
  bool overflowed_{false};

  uint64_t rx_ok_{0};
  uint64_t rx_crc_errors_{0};
  uint64_t rx_overflow_{0};
  uint64_t rx_cobs_errors_{0};
};

}  // namespace mowgli_hardware

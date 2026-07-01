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
 * @file serial_port.hpp
 * @brief POSIX termios serial port RAII wrapper.
 *
 * Provides non-blocking reads and blocking writes over a serial device
 * (e.g. /dev/mowgli or /dev/ttyACM0).  The destructor closes the file
 * descriptor, so the port is never leaked.
 */

#pragma once

#include <cstdint>
#include <string>

#include <sys/types.h>

namespace mowgli_hardware
{

/**
 * @brief RAII wrapper around a POSIX serial port using termios.
 *
 * Thread safety: instances are NOT thread-safe.  External synchronisation
 * is required if read() and write() are called from different threads.
 */
class SerialPort
{
public:
  /**
   * @brief Construct a SerialPort descriptor (does NOT open the device yet).
   *
   * @param device    Path to the serial device (e.g. "/dev/mowgli").
   * @param baud_rate Baud rate; standard values: 9600, 19200, 115200, 460800.
   *                  Defaults to 115200.
   */
  explicit SerialPort(const std::string& device, int baud_rate = 115200);

  /// Destructor — closes the port if still open.
  ~SerialPort();

  // Non-copyable; move is allowed.
  SerialPort(const SerialPort&) = delete;
  SerialPort& operator=(const SerialPort&) = delete;
  SerialPort(SerialPort&& other) noexcept;
  SerialPort& operator=(SerialPort&& other) noexcept;

  /**
   * @brief Open and configure the serial port.
   *
   * Configures 8N1, no flow control, with the requested baud rate.
   * The port is opened in raw (non-canonical) mode with O_NONBLOCK so that
   * read() returns immediately when no data is available.
   *
   * @return true on success; false on failure (errno is set).
   */
  bool open();

  /// Close the port (safe to call on an already-closed port).
  void close();

  /// @return true if the file descriptor is valid (port is open).
  [[nodiscard]] bool is_open() const noexcept;

  /**
   * @brief Read up to @p max_len bytes from the serial port.
   *
   * This is a non-blocking read; it returns 0 immediately if no data is
   * available, and -1 (with errno set) on error.
   *
   * @param buffer  Destination buffer.
   * @param max_len Capacity of the destination buffer.
   * @return Number of bytes read (≥ 0), or -1 on error.
   */
  ssize_t read(uint8_t* buffer, std::size_t max_len);

  /**
   * @brief Write @p len bytes to the serial port.
   *
   * Performs a single write(2) syscall; on most Linux serial drivers this is
   * sufficient for small packets.  Returns the number of bytes actually written
   * or -1 on error.
   *
   * @param data Pointer to the data to send.
   * @param len  Number of bytes to send.
   * @return Number of bytes written, or -1 on error.
   */
  ssize_t write(const uint8_t* data, std::size_t len);

  /**
   * @brief Write the complete buffer, retrying short non-blocking writes briefly.
   *
   * The serial fd is opened O_NONBLOCK. A single write(2) can therefore return
   * EAGAIN or write only part of a packet during USB CDC back-pressure. This
   * helper keeps small COBS frames atomic from the caller's point of view, while
   * still returning quickly on a dead device.
   *
   * @return Number of bytes written, or -1 if no byte could be written.
   */
  ssize_t write_all(const uint8_t* data, std::size_t len);

  /// @return The device path this port was constructed with.
  [[nodiscard]] const std::string& device() const noexcept;

  /// @return The configured baud rate.
  [[nodiscard]] int baud_rate() const noexcept;

private:
  /// Convert an integer baud rate to the corresponding termios B* constant.
  /// Returns B0 if the value is unsupported.
  [[nodiscard]] static int to_termios_baud(int baud_rate) noexcept;

  std::string device_;
  int baud_rate_;
  int fd_{-1};
};

}  // namespace mowgli_hardware

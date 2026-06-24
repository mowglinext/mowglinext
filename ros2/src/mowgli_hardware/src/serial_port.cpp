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
#include "mowgli_hardware/serial_port.hpp"

#include <cerrno>
#include <cstring>
#include <utility>

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

namespace mowgli_hardware
{

SerialPort::SerialPort(const std::string& device, int baud_rate)
    : device_(device), baud_rate_(baud_rate)
{
}

SerialPort::~SerialPort()
{
  close();
}

SerialPort::SerialPort(SerialPort&& other) noexcept
    : device_(std::move(other.device_)), baud_rate_(other.baud_rate_), fd_(other.fd_)
{
  other.fd_ = -1;
}

SerialPort& SerialPort::operator=(SerialPort&& other) noexcept
{
  if (this != &other)
  {
    close();
    device_ = std::move(other.device_);
    baud_rate_ = other.baud_rate_;
    fd_ = other.fd_;
    other.fd_ = -1;
  }
  return *this;
}

bool SerialPort::open()
{
  if (fd_ >= 0)
  {
    // Already open.
    return true;
  }

  // O_NONBLOCK: read() returns immediately when no data is available.
  fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0)
  {
    return false;
  }

  struct termios tty
  {
  };
  if (::tcgetattr(fd_, &tty) != 0)
  {
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  // ---- Output flags ----
  tty.c_oflag = 0;

  // ---- Input flags ----
  // No SW flow control, no NL/CR translations, no parity stripping.
  tty.c_iflag &= ~(
      static_cast<tcflag_t>(IXON) | static_cast<tcflag_t>(IXOFF) | static_cast<tcflag_t>(IXANY) |
      static_cast<tcflag_t>(ICRNL) | static_cast<tcflag_t>(INLCR) | static_cast<tcflag_t>(IGNCR) |
      static_cast<tcflag_t>(ISTRIP) | static_cast<tcflag_t>(INPCK) | static_cast<tcflag_t>(IGNBRK));

  // ---- Control flags ----
  // 8 data bits, no parity, 1 stop bit, enable receiver, no HW flow control.
  tty.c_cflag &= ~(static_cast<tcflag_t>(PARENB) | static_cast<tcflag_t>(CSTOPB) |
                   static_cast<tcflag_t>(CSIZE) | static_cast<tcflag_t>(CRTSCTS));
  tty.c_cflag |=
      static_cast<tcflag_t>(CS8) | static_cast<tcflag_t>(CREAD) | static_cast<tcflag_t>(CLOCAL);

  // ---- Local flags ----
  // Raw mode: no echo, no signals, no canonical processing.
  tty.c_lflag &= ~(static_cast<tcflag_t>(ECHO) | static_cast<tcflag_t>(ECHOE) |
                   static_cast<tcflag_t>(ECHONL) | static_cast<tcflag_t>(ICANON) |
                   static_cast<tcflag_t>(ISIG) | static_cast<tcflag_t>(IEXTEN));

  // ---- VMIN / VTIME ----
  // Non-blocking: return immediately with however many bytes are available.
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  // ---- Baud rate ----
  const speed_t speed = static_cast<speed_t>(to_termios_baud(baud_rate_));
  if (speed == B0)
  {
    ::close(fd_);
    fd_ = -1;
    errno = EINVAL;
    return false;
  }
  ::cfsetispeed(&tty, speed);
  ::cfsetospeed(&tty, speed);

  if (::tcsetattr(fd_, TCSANOW, &tty) != 0)
  {
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  // Flush any stale data from the kernel buffers.
  ::tcflush(fd_, TCIOFLUSH);

  return true;
}

void SerialPort::close()
{
  if (fd_ >= 0)
  {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SerialPort::is_open() const noexcept
{
  return fd_ >= 0;
}

ssize_t SerialPort::read(uint8_t* buffer, std::size_t max_len)
{
  if (fd_ < 0)
  {
    errno = EBADF;
    return -1;
  }
  return ::read(fd_, buffer, max_len);
}

ssize_t SerialPort::write(const uint8_t* data, std::size_t len)
{
  if (fd_ < 0)
  {
    errno = EBADF;
    return -1;
  }
  return ::write(fd_, data, len);
}

ssize_t SerialPort::write_all(const uint8_t* data, std::size_t len)
{
  if (fd_ < 0)
  {
    errno = EBADF;
    return -1;
  }
  if (data == nullptr && len > 0)
  {
    errno = EINVAL;
    return -1;
  }

  std::size_t offset = 0;
  int backpressure_retries = 0;
  constexpr int kMaxBackpressureRetries = 4;
  constexpr int kPollTimeoutMs = 5;

  while (offset < len)
  {
    const ssize_t n = ::write(fd_, data + offset, len - offset);
    if (n > 0)
    {
      offset += static_cast<std::size_t>(n);
      backpressure_retries = 0;
      continue;
    }
    if (n == 0)
    {
      break;
    }
    if (errno == EINTR)
    {
      continue;
    }
    if ((errno == EAGAIN || errno == EWOULDBLOCK) &&
        backpressure_retries < kMaxBackpressureRetries)
    {
      ++backpressure_retries;
      struct pollfd pfd
      {
      };
      pfd.fd = fd_;
      pfd.events = POLLOUT;
      const int poll_result = ::poll(&pfd, 1, kPollTimeoutMs);
      if (poll_result < 0 && errno != EINTR)
      {
        break;
      }
      continue;
    }
    break;
  }

  if (offset == 0)
  {
    return -1;
  }
  return static_cast<ssize_t>(offset);
}

const std::string& SerialPort::device() const noexcept
{
  return device_;
}

int SerialPort::baud_rate() const noexcept
{
  return baud_rate_;
}

int SerialPort::to_termios_baud(int baud_rate) noexcept
{
  switch (baud_rate)
  {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
#ifdef B460800
    case 460800:
      return B460800;
#endif
#ifdef B921600
    case 921600:
      return B921600;
#endif
    default:
      return B0;
  }
}

}  // namespace mowgli_hardware

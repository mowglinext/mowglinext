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

#include <array>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <vector>

#include "mowgli_hardware/serial_port.hpp"
#include <gtest/gtest.h>
#include <poll.h>
#include <pty.h>
#include <unistd.h>

using mowgli_hardware::SerialPort;

TEST(SerialPort, WriteAllWritesCompleteSmallFrameToPty)
{
  int master_fd = -1;
  int slave_fd = -1;
  std::array<char, 128> slave_name{};
  ASSERT_EQ(::openpty(&master_fd, &slave_fd, slave_name.data(), nullptr, nullptr), 0)
      << std::strerror(errno);
  ::close(slave_fd);

  SerialPort port(slave_name.data(), 115200);
  ASSERT_TRUE(port.open()) << std::strerror(errno);

  const std::vector<uint8_t> frame{0x00, 0x11, 0x22, 0x33, 0x00};
  EXPECT_EQ(port.write_all(frame.data(), frame.size()), static_cast<ssize_t>(frame.size()));

  struct pollfd pfd
  {
  };
  pfd.fd = master_fd;
  pfd.events = POLLIN;
  ASSERT_GT(::poll(&pfd, 1, 100), 0) << std::strerror(errno);

  std::array<uint8_t, 16> received{};
  const ssize_t n = ::read(master_fd, received.data(), received.size());
  ASSERT_EQ(n, static_cast<ssize_t>(frame.size())) << std::strerror(errno);
  EXPECT_EQ(std::vector<uint8_t>(received.begin(), received.begin() + n), frame);

  port.close();
  ::close(master_fd);
}

TEST(SerialPort, WriteAllOnClosedPortFails)
{
  SerialPort port("/dev/does-not-matter", 115200);
  const uint8_t byte = 0x42;
  EXPECT_EQ(port.write_all(&byte, 1), -1);
  EXPECT_EQ(errno, EBADF);
}

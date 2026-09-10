// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later

#include "mowgli_leds/spi_device.hpp"

#include <cerrno>
#include <cstddef>
#include <cstring>
#include <string>

#ifdef __linux__
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#endif

namespace mowgli_leds
{

namespace
{

#ifdef __linux__
std::string Errno(const std::string& what)
{
  return what + ": " + std::strerror(errno);
}
#endif

}  // namespace

SpiDevice::~SpiDevice()
{
  Close();
}

void SpiDevice::Close()
{
#ifdef __linux__
  if (fd_ >= 0)
  {
    ::close(fd_);
  }
#endif
  fd_ = -1;
  path_.clear();
}

SpiResult SpiDevice::Open(const std::string& path, std::uint32_t speed_hz)
{
#ifdef __linux__
  Close();

  const int fd = ::open(path.c_str(), O_WRONLY | O_CLOEXEC);
  if (fd < 0)
  {
    return SpiResult::Failure(Errno("open(" + path + ")"));
  }

  // Mode 0: CPOL=0 so the clock (and, with no transfer in flight, MOSI) idles
  // LOW. WS2812 reads a high idle line as garbage bits and never sees the
  // reset gap -- this single ioctl is why SPI works on this pin where the
  // UART alternate function would need an external inverter.
  const std::uint8_t mode = SPI_MODE_0;
  if (::ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0)
  {
    const std::string error = Errno("ioctl(SPI_IOC_WR_MODE)");
    ::close(fd);
    return SpiResult::Failure(error);
  }

  const std::uint8_t bits_per_word = 8u;
  if (::ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word) < 0)
  {
    const std::string error = Errno("ioctl(SPI_IOC_WR_BITS_PER_WORD)");
    ::close(fd);
    return SpiResult::Failure(error);
  }

  const std::uint8_t lsb_first = 0u;
  if (::ioctl(fd, SPI_IOC_WR_LSB_FIRST, &lsb_first) < 0)
  {
    const std::string error = Errno("ioctl(SPI_IOC_WR_LSB_FIRST)");
    ::close(fd);
    return SpiResult::Failure(error);
  }

  std::uint32_t speed = speed_hz;
  if (::ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0)
  {
    const std::string error = Errno("ioctl(SPI_IOC_WR_MAX_SPEED_HZ)");
    ::close(fd);
    return SpiResult::Failure(error);
  }

  fd_ = fd;
  path_ = path;
  return SpiResult::Success();
#else
  (void)path;
  (void)speed_hz;
  return SpiResult::Failure("spidev is only available on Linux");
#endif
}

SpiResult SpiDevice::Write(const std::vector<std::uint8_t>& bytes)
{
  if (!IsOpen())
  {
    return SpiResult::Failure("device is not open");
  }
  if (bytes.empty())
  {
    return SpiResult::Success();
  }

#ifdef __linux__
  const ssize_t written = ::write(fd_, bytes.data(), bytes.size());
  if (written < 0)
  {
    return SpiResult::Failure(Errno("write(" + path_ + ")"));
  }
  if (static_cast<std::size_t>(written) != bytes.size())
  {
    return SpiResult::Failure("short write: " + std::to_string(written) + " of " +
                              std::to_string(bytes.size()) + " bytes");
  }
  return SpiResult::Success();
#else
  return SpiResult::Failure("spidev is only available on Linux");
#endif
}

}  // namespace mowgli_leds

// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Thin, ROS-free wrapper over the Linux sysfs PWM interface
// (/sys/class/pwm/pwmchipN/pwmM/{period,duty_cycle,enable}). Deliberately
// separate from pwm_controller.hpp's pure state machine so the decision
// logic stays unit-testable without touching a filesystem, and this class
// stays unit-testable without real PWM hardware (base_path is injectable —
// point it at a scratch directory in tests).
//
// The exact pwmchipN/pwmM indices for a given GPIO are NOT fixed across
// Raspberry Pi device-tree/overlay versions — resolve them on the real
// hardware (`ls /sys/class/pwm/`, per the lidar_pwm_gpio_pin settings
// description) rather than assuming a value here.

#ifndef MOWGLI_LIDAR_PWM__SYSFS_PWM_HPP_
#define MOWGLI_LIDAR_PWM__SYSFS_PWM_HPP_

#include <sys/stat.h>
#include <time.h>

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <string>

namespace mowgli_lidar_pwm
{

class SysfsPwm
{
public:
  /// @param base_path sysfs PWM class root, default the real one. Tests
  ///                   point this at a scratch directory instead.
  /// @param chip       pwmchip index (pwmchipN)
  /// @param channel    channel index within the chip (pwmM)
  /// @param period_ns  fixed PWM period in nanoseconds. Set once at Open()
  ///                   and never changed afterward — the LD19's documented
  ///                   30 kHz carrier is constant; only duty cycle varies.
  SysfsPwm(std::string base_path, int chip, int channel, std::int64_t period_ns)
  : chip_dir_(base_path + "/pwmchip" + std::to_string(chip)),
    pwm_dir_(chip_dir_ + "/pwm" + std::to_string(channel)),
    channel_(channel),
    period_ns_(period_ns)
  {
  }

  /// Exports the channel (if not already exported) and sets the fixed
  /// period. Throws std::runtime_error on failure — the caller (the node)
  /// treats that as a startup fault, not something to retry silently.
  void Open()
  {
    if (!PathExists(pwm_dir_)) {
      WriteFile(chip_dir_ + "/export", std::to_string(channel_));
      // The kernel creates pwmM asynchronously; a handful of retries covers
      // the normal case without an arbitrary fixed sleep.
      for (int attempt = 0; attempt < 50 && !PathExists(pwm_dir_); ++attempt) {
        struct timespec ts{0, 10'000'000};  // 10 ms
        nanosleep(&ts, nullptr);
      }
      if (!PathExists(pwm_dir_)) {
        throw std::runtime_error("pwm channel did not appear after export: " + pwm_dir_);
      }
    }
    WriteFile(pwm_dir_ + "/period", std::to_string(period_ns_));
    opened_ = true;
  }

  /// duty_cycle_fraction clamped to [0, 1] internally by the caller
  /// (pwm_controller.hpp already clamps its PwmCommand).
  void SetDutyCycle(double duty_cycle_fraction)
  {
    EnsureOpen();
    const auto duty_ns =
      static_cast<std::int64_t>(static_cast<double>(period_ns_) * duty_cycle_fraction);
    WriteFile(pwm_dir_ + "/duty_cycle", std::to_string(duty_ns));
  }

  void SetEnabled(bool enabled)
  {
    EnsureOpen();
    WriteFile(pwm_dir_ + "/enable", enabled ? "1" : "0");
  }

  /// Best-effort: disables and unexports. Never throws — this runs from
  /// shutdown paths where a failure has nothing useful left to do about it.
  void Close() noexcept
  {
    if (!opened_) {
      return;
    }
    try {
      WriteFile(pwm_dir_ + "/enable", "0");
    } catch (...) {
    }
    try {
      WriteFile(chip_dir_ + "/unexport", std::to_string(channel_));
    } catch (...) {
    }
    opened_ = false;
  }

  ~SysfsPwm()
  {
    Close();
  }

  SysfsPwm(const SysfsPwm &) = delete;
  SysfsPwm & operator=(const SysfsPwm &) = delete;

private:
  void EnsureOpen()
  {
    if (!opened_) {
      throw std::logic_error("SysfsPwm used before Open()");
    }
  }

  static bool PathExists(const std::string & path)
  {
    struct stat st{};
    return ::stat(path.c_str(), &st) == 0;
  }

  static void WriteFile(const std::string & path, const std::string & value)
  {
    std::ofstream f(path);
    if (!f.is_open()) {
      throw std::runtime_error("failed to open " + path + ": " + std::strerror(errno));
    }
    f << value;
    if (!f.good()) {
      throw std::runtime_error("failed to write " + value + " to " + path);
    }
  }

  std::string chip_dir_;
  std::string pwm_dir_;
  int channel_;
  std::int64_t period_ns_;
  bool opened_ = false;
};

}  // namespace mowgli_lidar_pwm

#endif  // MOWGLI_LIDAR_PWM__SYSFS_PWM_HPP_

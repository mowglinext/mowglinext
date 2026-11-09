// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0
// Simulation-only observation. No code or IO in a default production build.
#pragma once

#ifdef OPENMOWER_SIMULATION_TRACE
#include <array>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>

#include <fcntl.h>
#include <time.h>
#include <unistd.h>

namespace mowgli_openmower_bridge
{
inline long long SimulationMonotonicNs()
{
  timespec stamp{};
  if (::clock_gettime(CLOCK_MONOTONIC, &stamp) != 0)
  {
    throw std::runtime_error("simulation monotonic clock failed");
  }
  return static_cast<long long>(stamp.tv_sec) * 1000000000LL + stamp.tv_nsec;
}

class SimulationTrace
{
public:
  SimulationTrace()
  {
    const char* path = std::getenv("OPENMOWER_SIMULATION_TRACE_PATH");
    if (!path || (fd_ = ::open(path, O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0600)) < 0)
    {
      throw std::runtime_error(
          "simulation trace requires a new writable OPENMOWER_SIMULATION_TRACE_PATH");
    }
    Append("identity", "\"schema\":1,\"pid\":%ld", static_cast<long>(::getpid()));
  }
  ~SimulationTrace()
  {
    ::close(fd_);
  }
  SimulationTrace(const SimulationTrace&) = delete;
  SimulationTrace& operator=(const SimulationTrace&) = delete;

  // Called only by the node's mutually exclusive callbacks. A short/error
  // write aborts the simulation instead of silently losing safety evidence.
  template <typename... Args>
  void Append(const char* event, const char* format, Args... args)
  {
    std::array<char, 2048> line{};
    const int prefix = std::snprintf(line.data(),
                                     line.size(),
                                     "{\"seq\":%llu,\"event\":\"%s\",\"mono_ns\":%lld,",
                                     sequence_++,
                                     event,
                                     SimulationMonotonicNs());
    if (prefix < 0 || static_cast<std::size_t>(prefix) >= line.size())
    {
      throw std::runtime_error("simulation trace prefix overflow");
    }
    const int body = std::snprintf(line.data() + prefix, line.size() - prefix, format, args...);
    if (body < 0 || static_cast<std::size_t>(prefix + body + 2) >= line.size())
    {
      throw std::runtime_error("simulation trace record overflow");
    }
    const auto length = static_cast<std::size_t>(prefix + body);
    line[length] = '}';
    line[length + 1] = '\n';
    if (::write(fd_, line.data(), length + 2) != static_cast<ssize_t>(length + 2))
    {
      throw std::runtime_error("simulation trace write failed");
    }
  }

private:
  int fd_{-1};
  unsigned long long sequence_{0};
};

inline SimulationTrace& GetSimulationTrace()
{
  static SimulationTrace trace;
  return trace;
}
}  // namespace mowgli_openmower_bridge
#define OM_SIM_TRACE(...) ::mowgli_openmower_bridge::GetSimulationTrace().Append(__VA_ARGS__)
#else
#define OM_SIM_TRACE(...) \
  do                      \
  {                       \
  } while (false)
#endif

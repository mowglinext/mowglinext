// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
// Test-only preload: hold the real node at checkpoint file fsync so the test
// can observe executor progress and the deferred response deterministically.
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <dlfcn.h>
#include <fcntl.h>
#include <unistd.h>

extern "C" int fsync(int fd)
{
  const auto real = reinterpret_cast<int (*)(int)>(::dlsym(RTLD_NEXT, "fsync"));
  const char* control = std::getenv("CHECKPOINT_TEST_CONTROL");
  char path[4096];
  char descriptor[64];
  ::snprintf(descriptor, sizeof(descriptor), "/proc/self/fd/%d", fd);
  const auto count = ::readlink(descriptor, path, sizeof(path) - 1);
  if (control && count > 0)
  {
    path[count] = '\0';
    if (::strstr(path, ".checkpoint.tmp."))
    {
      const int marker = ::open(control, O_WRONLY | O_CREAT | O_CLOEXEC, 0600);
      if (marker >= 0)
        ::close(marker);
      // Removing the marker releases the save. Bound a failed test's worker.
      for (int i = 0; i < 1000 && ::access(control, F_OK) == 0; ++i)
        ::usleep(10000);
      if (::access(control, F_OK) == 0)
      {
        errno = EIO;
        return -1;
      }
    }
  }
  return real(fd);
}

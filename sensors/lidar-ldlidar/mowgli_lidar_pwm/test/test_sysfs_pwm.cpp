// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Exercises the sysfs file-writing behaviour against a scratch directory
// standing in for /sys/class/pwm — no real PWM hardware needed or assumed.

#include <sys/stat.h>

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>

#include "gtest/gtest.h"
#include "mowgli_lidar_pwm/sysfs_pwm.hpp"

using mowgli_lidar_pwm::SysfsPwm;

namespace
{

std::string MakeScratchDir()
{
  char tmpl[] = "/tmp/mowgli_lidar_pwm_test_XXXXXX";
  const char * dir = mkdtemp(tmpl);
  if (dir == nullptr) {
    throw std::runtime_error("mkdtemp failed");
  }
  return std::string(dir);
}

std::string ReadFile(const std::string & path)
{
  std::ifstream f(path);
  std::ostringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

}  // namespace

TEST(SysfsPwm, OpenSkipsExportWhenChannelAlreadyExists)
{
  const auto base = MakeScratchDir();
  const std::string pwm_dir = base + "/pwmchip0/pwm0";
  ASSERT_EQ(system(("mkdir -p " + pwm_dir).c_str()), 0);

  SysfsPwm pwm(base, /*chip=*/0, /*channel=*/0, /*period_ns=*/33333);
  EXPECT_NO_THROW(pwm.Open());

  EXPECT_EQ(ReadFile(pwm_dir + "/period"), "33333");

  system(("rm -rf " + base).c_str());
}

TEST(SysfsPwm, OpenThrowsWhenChannelNeverAppears)
{
  const auto base = MakeScratchDir();
  ASSERT_EQ(system(("mkdir -p " + base + "/pwmchip0").c_str()), 0);
  // Deliberately do NOT create pwmchip0/pwm0 — nothing will ever export it
  // in this test, so Open() must give up after its bounded retry loop
  // rather than hang.

  SysfsPwm pwm(base, /*chip=*/0, /*channel=*/0, /*period_ns=*/33333);
  EXPECT_THROW(pwm.Open(), std::runtime_error);

  system(("rm -rf " + base).c_str());
}

TEST(SysfsPwm, SetDutyCycleWritesScaledNanoseconds)
{
  const auto base = MakeScratchDir();
  const std::string pwm_dir = base + "/pwmchip0/pwm0";
  ASSERT_EQ(system(("mkdir -p " + pwm_dir).c_str()), 0);

  SysfsPwm pwm(base, 0, 0, /*period_ns=*/100000);
  pwm.Open();
  pwm.SetDutyCycle(0.5);

  EXPECT_EQ(ReadFile(pwm_dir + "/duty_cycle"), "50000");

  system(("rm -rf " + base).c_str());
}

TEST(SysfsPwm, SetEnabledWritesOneAndZero)
{
  const auto base = MakeScratchDir();
  const std::string pwm_dir = base + "/pwmchip0/pwm0";
  ASSERT_EQ(system(("mkdir -p " + pwm_dir).c_str()), 0);

  SysfsPwm pwm(base, 0, 0, 33333);
  pwm.Open();

  pwm.SetEnabled(true);
  EXPECT_EQ(ReadFile(pwm_dir + "/enable"), "1");

  pwm.SetEnabled(false);
  EXPECT_EQ(ReadFile(pwm_dir + "/enable"), "0");

  system(("rm -rf " + base).c_str());
}

TEST(SysfsPwm, UsingBeforeOpenThrows)
{
  const auto base = MakeScratchDir();
  SysfsPwm pwm(base, 0, 0, 33333);

  EXPECT_THROW(pwm.SetDutyCycle(0.5), std::logic_error);
  EXPECT_THROW(pwm.SetEnabled(true), std::logic_error);

  system(("rm -rf " + base).c_str());
}

TEST(SysfsPwm, CloseWritesDisableAndUnexport)
{
  const auto base = MakeScratchDir();
  const std::string chip_dir = base + "/pwmchip0";
  const std::string pwm_dir = chip_dir + "/pwm0";
  ASSERT_EQ(system(("mkdir -p " + pwm_dir).c_str()), 0);

  {
    SysfsPwm pwm(base, 0, 0, 33333);
    pwm.Open();
    pwm.SetEnabled(true);
  }  // destructor calls Close()

  EXPECT_EQ(ReadFile(pwm_dir + "/enable"), "0");
  EXPECT_EQ(ReadFile(chip_dir + "/unexport"), "0");

  system(("rm -rf " + base).c_str());
}

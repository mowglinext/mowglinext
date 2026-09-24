// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0
//
// Unit tests for the firmware's append-only parameter log format
// (firmware/stm32/ros_usbnode/include/fw_param_log.h), compiled directly from
// the firmware tree like the other pure firmware headers. The flash-facing
// half (fw_param_store.c / fw_params.c) relies on exactly these guarantees:
// the newest committed record wins, a torn write is invisible, and anything
// that is not a record forces an erase at boot instead of an append on top.

#include <array>
#include <cmath>
#include <vector>

#include "fw_param_log.h"
#include <gtest/gtest.h>

namespace
{

constexpr std::size_t kAreaWords = 256;

std::vector<uint32_t> erased_area()
{
  return std::vector<uint32_t>(kAreaWords, FW_PARAM_LOG_ERASED);
}

// Append a record at `at`, as the firmware would; returns its size in words.
std::size_t append(std::vector<uint32_t>& area,
                   std::size_t at,
                   const std::vector<uint16_t>& ids,
                   const std::vector<float>& values)
{
  std::array<uint32_t, 64> record{};
  const std::size_t words =
      fw_param_log_encode(ids.data(), values.data(), ids.size(), record.data(), record.size());
  EXPECT_GT(words, 0u);
  for (std::size_t w = 0; w < words; ++w)
  {
    area[at + w] = record[w];
  }
  return words;
}

float value_of(const uint32_t* record, uint16_t wanted)
{
  for (std::size_t e = 0; e < fw_param_log_entry_count(record); ++e)
  {
    uint16_t id = 0;
    float value = 0.0f;
    fw_param_log_entry(record, e, &id, &value);
    if (id == wanted)
    {
      return value;
    }
  }
  return NAN;
}

}  // namespace

TEST(FwParamLog, CrcMatchesTheStandardCrc32)
{
  // Words are fed byte by byte, least significant first, so one word holding
  // '1','2','3','4' must give the reference CRC-32 of the string "1234".
  const uint32_t word = 0x34333231u;  // bytes '1','2','3','4'
  EXPECT_EQ(fw_param_log_crc32(&word, 1), 0x9BE3E0A3u);
}

TEST(FwParamLog, EmptyAreaHasNoRecordAndAppendsAtZero)
{
  const auto area = erased_area();
  const fw_param_log_scan_t scan = fw_param_log_scan(area.data(), area.size());
  EXPECT_EQ(scan.last_valid, -1);
  EXPECT_EQ(scan.next_free, 0u);
  EXPECT_EQ(scan.valid_records, 0u);
  EXPECT_FALSE(scan.needs_erase);
}

TEST(FwParamLog, RoundTripsIdsAndValuesBitExactly)
{
  auto area = erased_area();
  append(area, 0, {1, 40, 50}, {312.5f, 1500.0f, 44.0f});
  const fw_param_log_scan_t scan = fw_param_log_scan(area.data(), area.size());
  ASSERT_EQ(scan.last_valid, 0);
  EXPECT_EQ(scan.next_free, fw_param_log_record_words(3));
  EXPECT_EQ(value_of(&area[0], 1), 312.5f);
  EXPECT_EQ(value_of(&area[0], 40), 1500.0f);
  EXPECT_EQ(value_of(&area[0], 50), 44.0f);
}

TEST(FwParamLog, NewestCommittedRecordWins)
{
  auto area = erased_area();
  std::size_t at = append(area, 0, {40}, {2000.0f});
  at += append(area, at, {40}, {800.0f});
  const fw_param_log_scan_t scan = fw_param_log_scan(area.data(), area.size());
  ASSERT_GE(scan.last_valid, 0);
  EXPECT_EQ(scan.valid_records, 2u);
  EXPECT_EQ(value_of(&area[static_cast<std::size_t>(scan.last_valid)], 40), 800.0f);
  EXPECT_EQ(scan.next_free, at);
  EXPECT_FALSE(scan.needs_erase);
}

TEST(FwParamLog, RecordWithoutCommitWordIsIgnoredButKeepsItsSlot)
{
  // Power lost after everything but the commit word was programmed.
  auto area = erased_area();
  std::size_t at = append(area, 0, {40}, {2000.0f});
  const std::size_t torn = at;
  at += append(area, at, {40}, {800.0f});
  area[at - 1] = FW_PARAM_LOG_ERASED;
  const fw_param_log_scan_t scan = fw_param_log_scan(area.data(), area.size());
  EXPECT_EQ(scan.last_valid, 0);
  EXPECT_EQ(value_of(&area[0], 40), 2000.0f);
  EXPECT_EQ(scan.next_free, at);  // the next append goes AFTER the torn slot
  EXPECT_GT(scan.next_free, torn);
  EXPECT_FALSE(scan.needs_erase);
}

TEST(FwParamLog, CorruptedPayloadFailsTheCrc)
{
  auto area = erased_area();
  std::size_t at = append(area, 0, {40}, {2000.0f});
  append(area, at, {40}, {800.0f});
  area[at + FW_PARAM_LOG_HEADER_WORDS + 1] ^= 0x1u;  // flip a value bit
  const fw_param_log_scan_t scan = fw_param_log_scan(area.data(), area.size());
  EXPECT_EQ(scan.last_valid, 0);
  EXPECT_EQ(scan.valid_records, 1u);
}

TEST(FwParamLog, TornHeaderForcesAnEraseButKeepsTheLastGoodRecord)
{
  // Only the magic of the second record made it to flash: its length is
  // unknown, so nothing can be appended after it safely.
  auto area = erased_area();
  const std::size_t at = append(area, 0, {40}, {2000.0f});
  area[at] = FW_PARAM_LOG_MAGIC;
  const fw_param_log_scan_t scan = fw_param_log_scan(area.data(), area.size());
  EXPECT_TRUE(scan.needs_erase);
  EXPECT_EQ(scan.last_valid, 0);
  EXPECT_EQ(value_of(&area[0], 40), 2000.0f);
}

TEST(FwParamLog, ForeignDataForcesAnErase)
{
  auto area = erased_area();
  area[0] = 0x20005000u;  // e.g. a stack pointer from another image
  const fw_param_log_scan_t scan = fw_param_log_scan(area.data(), area.size());
  EXPECT_TRUE(scan.needs_erase);
  EXPECT_EQ(scan.last_valid, -1);
}

TEST(FwParamLog, DataAfterTheEndOfLogForcesAnErase)
{
  auto area = erased_area();
  const std::size_t at = append(area, 0, {40}, {2000.0f});
  area[at + 5] = 0x12345678u;
  const fw_param_log_scan_t scan = fw_param_log_scan(area.data(), area.size());
  EXPECT_TRUE(scan.needs_erase);
  EXPECT_EQ(scan.last_valid, 0);
}

TEST(FwParamLog, EncodeRefusesAnOversizedRecord)
{
  std::array<uint32_t, 5> small{};
  const uint16_t id = 1;
  const float value = 1.0f;
  EXPECT_EQ(fw_param_log_encode(&id, &value, 1, small.data(), small.size()), 0u);
  std::vector<uint16_t> ids(FW_PARAM_LOG_MAX_ENTRIES + 1, 1);
  std::vector<float> values(ids.size(), 1.0f);
  std::vector<uint32_t> big(fw_param_log_record_words(ids.size()));
  EXPECT_EQ(fw_param_log_encode(ids.data(), values.data(), ids.size(), big.data(), big.size()), 0u);
}

TEST(FwParamLog, FullAreaReportsNoRoomWithoutErasing)
{
  // A log packed with valid records up to the last word is full, not corrupt:
  // the boot code decides to erase because next_free + record > area.
  std::vector<uint32_t> area(fw_param_log_record_words(1) * 3, FW_PARAM_LOG_ERASED);
  std::size_t at = 0;
  for (int i = 0; i < 3; ++i)
  {
    at += append(area, at, {40}, {static_cast<float>(1000 + i)});
  }
  const fw_param_log_scan_t scan = fw_param_log_scan(area.data(), area.size());
  EXPECT_FALSE(scan.needs_erase);
  EXPECT_EQ(scan.next_free, area.size());
  EXPECT_EQ(value_of(&area[static_cast<std::size_t>(scan.last_valid)], 40), 1002.0f);
}

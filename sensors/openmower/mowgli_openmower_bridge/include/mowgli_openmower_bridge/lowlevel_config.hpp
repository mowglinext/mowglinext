// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0
/**
 * @file lowlevel_config.hpp
 * @brief The OpenMower LowLevel ↔ high-level configuration packet
 *        (PACKET_ID_LL_HIGH_LEVEL_CONFIG_REQ / _RSP, 0x11 / 0x12).
 *
 * Mirrors `ll_high_level_config` from open_mower_ros/mower_comms_v1
 * byte for byte. It is a FLEXIBLE-LENGTH struct: either side may be longer
 * than the other, so a receiver copies min(payload, sizeof) bytes and keeps
 * its defaults for the rest. The Mowgli STM32 answers the same packet id with
 * its own (different) LlConfigRsp — that layout lives in mowgli_hardware and
 * is deliberately NOT used here.
 */

#pragma once

#include <algorithm>
#include <cctype>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace mowgli_openmower_bridge::lowlevel
{

enum class OptionState : unsigned int
{
  OFF = 0,
  ON,
  UNDEFINED
};

enum class HallMode : unsigned int
{
  OFF = 0,
  LIFT_TILT,
  STOP,
  UNDEFINED
};

constexpr std::size_t kMaxHallInputs = 10u;

#pragma pack(push, 1)

struct ConfigOptions
{
  OptionState dfp_is_5v : 2;
  OptionState background_sounds : 2;
  OptionState ignore_charging_current : 2;
  OptionState reserved1 : 2;
  OptionState reserved2 : 2;
  OptionState reserved3 : 2;
  OptionState reserved4 : 2;
  OptionState reserved5 : 2;
};

struct HallConfig
{
  HallMode mode : 3;
  bool active_low : 1;
};

struct HighLevelConfig
{
  ConfigOptions options{OptionState::OFF,
                        OptionState::OFF,
                        OptionState::OFF,
                        OptionState::UNDEFINED,
                        OptionState::UNDEFINED,
                        OptionState::UNDEFINED,
                        OptionState::UNDEFINED,
                        OptionState::UNDEFINED};
  uint16_t rain_threshold{0xFFFFu};  ///< stock CoverUI rain ADC threshold
  float v_charge_cutoff{-1.0f};  ///< [V] charger off above this (-1 = unknown)
  float i_charge_cutoff{-1.0f};  ///< [A]
  float v_battery_cutoff{-1.0f};  ///< [V] battery full-protection cutoff
  float v_battery_empty{-1.0f};  ///< [V] 0 % point of the SoC estimate
  float v_battery_full{-1.0f};  ///< [V] 100 % point
  uint16_t lift_period{0xFFFFu};  ///< [ms] >= 2 wheels lifted → emergency
  uint16_t tilt_period{0xFFFFu};  ///< [ms] one wheel lifted → emergency
  uint8_t shutdown_esc_max_pitch{0xFFu};
  char language[2]{'e', 'n'};
  uint8_t volume{0xFFu};  ///< 0xFF = do not change
  HallConfig hall_configs[kMaxHallInputs]{};
};

#pragma pack(pop)

static_assert(sizeof(ConfigOptions) == 2u, "ConfigOptions must stay 2 bytes on the wire");
static_assert(sizeof(HallConfig) == 1u, "HallConfig must stay 1 byte on the wire");
static_assert(sizeof(HighLevelConfig) ==
                  2u + 2u + 5u * 4u + 2u + 2u + 1u + 2u + 1u + kMaxHallInputs,
              "HighLevelConfig layout drifted from OpenMower's ll_high_level_config");

/// Fresh config with every hall input UNDEFINED (board keeps its own).
[[nodiscard]] inline HighLevelConfig DefaultConfig()
{
  HighLevelConfig cfg{};
  for (auto& hall : cfg.hall_configs)
  {
    hall.mode = HallMode::UNDEFINED;
    hall.active_low = false;
  }
  return cfg;
}

/**
 * @brief Parse OpenMower's emergency input string, e.g. "!L,!L,S,S".
 *
 * Comma-separated, one entry per hall input in order (OM-Hall1..4 then the
 * six stock CoverUI inputs). Letters: I = ignore, L = lift/tilt, S = stop,
 * U = undefined; a leading '!' marks the input active-low. Missing entries
 * stay UNDEFINED so the board keeps its compiled default for them.
 */
inline void ApplyHallConfigString(HighLevelConfig& cfg, const std::string& spec)
{
  std::size_t index = 0u;
  std::size_t start = 0u;
  while (index < kMaxHallInputs && start <= spec.size())
  {
    const std::size_t comma = spec.find(',', start);
    const std::string token =
        spec.substr(start, comma == std::string::npos ? std::string::npos : comma - start);
    bool low = false;
    for (const char raw : token)
    {
      switch (static_cast<char>(std::toupper(static_cast<unsigned char>(raw))))
      {
        case '!':
          low = true;
          break;
        case 'I':
          cfg.hall_configs[index] = HallConfig{HallMode::OFF, low};
          break;
        case 'L':
          cfg.hall_configs[index] = HallConfig{HallMode::LIFT_TILT, low};
          break;
        case 'S':
          cfg.hall_configs[index] = HallConfig{HallMode::STOP, low};
          break;
        case 'U':
          cfg.hall_configs[index] = HallConfig{HallMode::UNDEFINED, low};
          break;
        default:
          break;
      }
    }
    ++index;
    if (comma == std::string::npos)
    {
      break;
    }
    start = comma + 1u;
  }
}

/// `type` + struct bytes; the CRC is appended by PacketHandler::encode_packet.
[[nodiscard]] inline std::vector<uint8_t> BuildConfigPayload(const HighLevelConfig& cfg,
                                                             uint8_t packet_type)
{
  std::vector<uint8_t> payload(1u + sizeof(HighLevelConfig));
  payload[0] = packet_type;
  std::memcpy(payload.data() + 1, &cfg, sizeof(HighLevelConfig));
  return payload;
}

/**
 * @brief Decode a received config packet (type byte first, CRC included at
 *        the end) over @p defaults: only the bytes the sender actually sent
 *        replace the defaults.
 */
[[nodiscard]] inline HighLevelConfig ParseConfigPayload(const uint8_t* data,
                                                        std::size_t len,
                                                        const HighLevelConfig& defaults)
{
  HighLevelConfig cfg = defaults;
  if (data == nullptr || len < 3u)
  {
    return cfg;
  }
  const std::size_t payload_size = std::min(sizeof(HighLevelConfig), len - 3u);
  std::memcpy(&cfg, data + 1, payload_size);
  return cfg;
}

}  // namespace mowgli_openmower_bridge::lowlevel

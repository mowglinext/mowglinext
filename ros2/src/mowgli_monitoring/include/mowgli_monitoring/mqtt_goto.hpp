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
 * @file mqtt_goto.hpp
 * @brief Pure helpers for the MQTT GOTO topics of mqtt_bridge_node.
 *
 * <prefix>/goto carries {"request_id", "issued_at", "lat", "lon", "yaw"}; the
 * bridge turns it into a ~/goto action goal. These helpers hold everything that
 * needs no ROS: a flat JSON object reader (the package deliberately has no JSON
 * library), the request and timestamp grammar, the freshness and clock checks,
 * the request store persisted across restarts, and the event payload.
 */

#pragma once

#include <array>
#include <cctype>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <map>
#include <optional>
#include <random>
#include <sstream>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include <fcntl.h>
#include <sys/timex.h>
#include <unistd.h>

namespace mowgli_monitoring::mqtt_goto
{

// ---------------------------------------------------------------------------
// Flat JSON object
// ---------------------------------------------------------------------------

/// A value of a flat JSON object: string, number, boolean or null (monostate).
using JsonValue = std::variant<std::monostate, std::string, double, bool>;

/// Parses a single JSON object whose values are strings, numbers, booleans or
/// null — no nested objects or arrays. Duplicate keys are rejected. Returns
/// nullopt on any syntax error.
inline std::optional<std::map<std::string, JsonValue>> parse_flat_json_object(
    const std::string& text)
{
  std::size_t i = 0;
  const auto skip_ws = [&]()
  {
    while (i < text.size() &&
           (text[i] == ' ' || text[i] == '\t' || text[i] == '\r' || text[i] == '\n'))
    {
      ++i;
    }
  };
  const auto parse_string = [&](std::string& out) -> bool
  {
    if (i >= text.size() || text[i] != '"')
    {
      return false;
    }
    ++i;
    while (i < text.size())
    {
      const char c = text[i++];
      if (c == '"')
      {
        return true;
      }
      if (static_cast<unsigned char>(c) < 0x20)
      {
        return false;
      }
      if (c != '\\')
      {
        out.push_back(c);
        continue;
      }
      if (i >= text.size())
      {
        return false;
      }
      const char e = text[i++];
      switch (e)
      {
        case '"':
        case '\\':
        case '/':
          out.push_back(e);
          break;
        case 'b':
          out.push_back('\b');
          break;
        case 'f':
          out.push_back('\f');
          break;
        case 'n':
          out.push_back('\n');
          break;
        case 'r':
          out.push_back('\r');
          break;
        case 't':
          out.push_back('\t');
          break;
        default:
          return false;  // \uXXXX is not needed by this contract
      }
    }
    return false;
  };

  std::map<std::string, JsonValue> object;
  skip_ws();
  if (i >= text.size() || text[i] != '{')
  {
    return std::nullopt;
  }
  ++i;
  skip_ws();
  if (i < text.size() && text[i] == '}')
  {
    ++i;
    skip_ws();
    return i == text.size() ? std::optional(object) : std::nullopt;
  }
  while (true)
  {
    skip_ws();
    std::string key;
    if (!parse_string(key))
    {
      return std::nullopt;
    }
    skip_ws();
    if (i >= text.size() || text[i] != ':')
    {
      return std::nullopt;
    }
    ++i;
    skip_ws();
    JsonValue value;
    if (i < text.size() && text[i] == '"')
    {
      std::string s;
      if (!parse_string(s))
      {
        return std::nullopt;
      }
      value = std::move(s);
    }
    else if (text.compare(i, 4, "true") == 0)
    {
      value = true;
      i += 4;
    }
    else if (text.compare(i, 5, "false") == 0)
    {
      value = false;
      i += 5;
    }
    else if (text.compare(i, 4, "null") == 0)
    {
      i += 4;
    }
    else
    {
      // JSON number: -?(0|[1-9][0-9]*)(\.[0-9]+)?([eE][+-]?[0-9]+)?
      const std::size_t start = i;
      const auto digits = [&]()
      {
        const std::size_t first = i;
        while (i < text.size() && std::isdigit(static_cast<unsigned char>(text[i])))
        {
          ++i;
        }
        return i - first;
      };
      if (i < text.size() && text[i] == '-')
      {
        ++i;
      }
      const std::size_t int_start = i;
      const std::size_t int_digits = digits();
      if (int_digits == 0 || (int_digits > 1 && text[int_start] == '0'))
      {
        return std::nullopt;
      }
      if (i < text.size() && text[i] == '.')
      {
        ++i;
        if (digits() == 0)
        {
          return std::nullopt;
        }
      }
      if (i < text.size() && (text[i] == 'e' || text[i] == 'E'))
      {
        ++i;
        if (i < text.size() && (text[i] == '+' || text[i] == '-'))
        {
          ++i;
        }
        if (digits() == 0)
        {
          return std::nullopt;
        }
      }
      double number = 0.0;
      const auto [end, error] = std::from_chars(text.data() + start, text.data() + i, number);
      if (start == i || error != std::errc{} || end != text.data() + i || !std::isfinite(number))
      {
        return std::nullopt;
      }
      value = number;
    }
    if (!object.emplace(std::move(key), std::move(value)).second)
    {
      return std::nullopt;
    }
    skip_ws();
    if (i < text.size() && text[i] == ',')
    {
      ++i;
      continue;
    }
    if (i < text.size() && text[i] == '}')
    {
      ++i;
      skip_ws();
      return i == text.size() ? std::optional(object) : std::nullopt;
    }
    return std::nullopt;
  }
}

/// Escapes a string for a JSON string value.
inline std::string json_escape(const std::string& in)
{
  std::string out;
  out.reserve(in.size());
  for (const char c : in)
  {
    switch (c)
    {
      case '"':
        out += "\\\"";
        break;
      case '\\':
        out += "\\\\";
        break;
      case '\n':
        out += "\\n";
        break;
      case '\r':
        out += "\\r";
        break;
      case '\t':
        out += "\\t";
        break;
      default:
        if (static_cast<unsigned char>(c) < 0x20)
        {
          char buf[8];
          std::snprintf(buf, sizeof(buf), "\\u%04x", static_cast<unsigned>(c));
          out += buf;
        }
        else
        {
          out.push_back(c);
        }
    }
  }
  return out;
}

// ---------------------------------------------------------------------------
// Timestamps (ISO 8601 UTC, "Z" only)
// ---------------------------------------------------------------------------

/// Parses "YYYY-MM-DDTHH:MM:SS[.fff…]Z" to milliseconds since the Unix epoch.
inline std::optional<int64_t> parse_iso_utc_ms(const std::string& text)
{
  // Fixed layout first, so sscanf never sees a sign or a space in a field.
  if (text.size() < 20)
  {
    return std::nullopt;
  }
  for (std::size_t k = 0; k < 19; ++k)
  {
    const char expected = k == 4 || k == 7 ? '-' : k == 10 ? 'T' : k == 13 || k == 16 ? ':' : '0';
    if (expected == '0' ? !std::isdigit(static_cast<unsigned char>(text[k])) : text[k] != expected)
    {
      return std::nullopt;
    }
  }
  int year = 0, month = 0, day = 0, hour = 0, minute = 0, second = 0;
  int consumed = 0;
  if (std::sscanf(text.c_str(),
                  "%4d-%2d-%2dT%2d:%2d:%2d%n",
                  &year,
                  &month,
                  &day,
                  &hour,
                  &minute,
                  &second,
                  &consumed) != 6 ||
      consumed != 19)
  {
    return std::nullopt;
  }
  std::size_t i = 19;
  int64_t millis = 0;
  if (i < text.size() && text[i] == '.')
  {
    ++i;
    int digits = 0;
    while (i < text.size() && std::isdigit(static_cast<unsigned char>(text[i])))
    {
      if (digits < 3)
      {
        millis = millis * 10 + (text[i] - '0');
      }
      ++digits;
      ++i;
    }
    if (digits == 0)
    {
      return std::nullopt;
    }
    for (; digits < 3; ++digits)
    {
      millis *= 10;
    }
  }
  if (i + 1 != text.size() || text[i] != 'Z')
  {
    return std::nullopt;
  }
  if (month < 1 || month > 12 || day < 1 || day > 31 || hour > 23 || minute > 59 || second > 60)
  {
    return std::nullopt;
  }
  std::tm tm{};
  tm.tm_year = year - 1900;
  tm.tm_mon = month - 1;
  tm.tm_mday = day;
  tm.tm_hour = hour;
  tm.tm_min = minute;
  tm.tm_sec = second;
  const time_t seconds = timegm(&tm);
  if (seconds == static_cast<time_t>(-1))
  {
    return std::nullopt;
  }
  return static_cast<int64_t>(seconds) * 1000 + millis;
}

/// Formats milliseconds since the Unix epoch as "YYYY-MM-DDTHH:MM:SS.mmmZ".
inline std::string format_iso_utc_ms(int64_t ms)
{
  const time_t seconds = static_cast<time_t>(ms / 1000);
  std::tm tm{};
  gmtime_r(&seconds, &tm);
  // Sized for any int field values, so the compiler can prove no truncation.
  char buf[80];
  std::snprintf(buf,
                sizeof(buf),
                "%04d-%02d-%02dT%02d:%02d:%02d.%03dZ",
                tm.tm_year + 1900,
                tm.tm_mon + 1,
                tm.tm_mday,
                tm.tm_hour,
                tm.tm_min,
                tm.tm_sec,
                static_cast<int>(ms % 1000));
  return buf;
}

// ---------------------------------------------------------------------------
// Requests
// ---------------------------------------------------------------------------

/// A request_id: 1 to 64 characters from [A-Za-z0-9_-] (a UUID fits).
inline bool is_valid_request_id(const std::string& id)
{
  if (id.empty() || id.size() > 64)
  {
    return false;
  }
  for (const char c : id)
  {
    if (!std::isalnum(static_cast<unsigned char>(c)) && c != '-' && c != '_')
    {
      return false;
    }
  }
  return true;
}

struct GotoRequest
{
  std::string request_id;
  int64_t issued_at_ms{0};
  double lat{0.0};
  double lon{0.0};
  double yaw{0.0};
};

/// Parses a <prefix>/goto payload. On failure returns the refusal reason
/// ("invalid_payload") and, when it could be read, the request_id.
struct GotoRequestParse
{
  std::optional<GotoRequest> request;
  std::string request_id;  ///< set whenever a valid request_id was read
  std::string error;  ///< empty on success
};

inline GotoRequestParse parse_goto_request(const std::string& payload)
{
  GotoRequestParse out;
  const auto object = parse_flat_json_object(payload);
  if (!object)
  {
    out.error = "invalid_payload";
    return out;
  }
  const auto text = [&](const char* key) -> std::optional<std::string>
  {
    const auto it = object->find(key);
    if (it == object->end() || !std::holds_alternative<std::string>(it->second))
    {
      return std::nullopt;
    }
    return std::get<std::string>(it->second);
  };
  const auto number = [&](const char* key) -> std::optional<double>
  {
    const auto it = object->find(key);
    if (it == object->end() || !std::holds_alternative<double>(it->second))
    {
      return std::nullopt;
    }
    return std::get<double>(it->second);
  };
  const auto id = text("request_id");
  if (id && is_valid_request_id(*id))
  {
    out.request_id = *id;
  }
  const auto issued = text("issued_at");
  const auto issued_ms = issued ? parse_iso_utc_ms(*issued) : std::nullopt;
  const auto lat = number("lat");
  const auto lon = number("lon");
  const auto yaw = number("yaw");
  if (out.request_id.empty() || !issued_ms || !lat || !lon || !yaw || *lat < -90.0 || *lat > 90.0 ||
      *lon < -180.0 || *lon > 180.0 || std::abs(*yaw) > 2.0 * M_PI)
  {
    out.error = "invalid_payload";
    return out;
  }
  out.request = GotoRequest{out.request_id, *issued_ms, *lat, *lon, *yaw};
  return out;
}

/// Parses a <prefix>/goto/cancel payload: {"request_id": "..."}. Returns the
/// request_id, or an empty string when absent or invalid (the cancel is then
/// answered invalid_payload and cancels nothing).
inline std::string parse_cancel_request_id(const std::string& payload)
{
  const auto object = parse_flat_json_object(payload);
  if (!object)
  {
    return {};
  }
  const auto it = object->find("request_id");
  if (it == object->end() || !std::holds_alternative<std::string>(it->second) ||
      !is_valid_request_id(std::get<std::string>(it->second)))
  {
    return {};
  }
  return std::get<std::string>(it->second);
}

/// "" when issued_at is within [now - max_age, now + max_future], else "stale"
/// or "future".
inline std::string check_freshness(int64_t issued_ms,
                                   int64_t now_ms,
                                   double max_age_s,
                                   double max_future_s)
{
  if (issued_ms < now_ms - static_cast<int64_t>(max_age_s * 1000.0))
  {
    return "stale";
  }
  if (issued_ms > now_ms + static_cast<int64_t>(max_future_s * 1000.0))
  {
    return "future";
  }
  return {};
}

// ---------------------------------------------------------------------------
// Clock synchronization
// ---------------------------------------------------------------------------

struct KernelClockState
{
  bool synchronized{false};
  double max_error_s{0.0};
};

/// Reads the kernel NTP state without changing it (adjtimex with modes = 0).
/// Synchronized means STA_UNSYNC is clear and adjtimex does not report
/// TIME_ERROR — the state an NTP daemon maintains once it has disciplined the
/// clock.
inline KernelClockState read_kernel_clock_state()
{
  timex tx{};
  tx.modes = 0;
  const int state = adjtimex(&tx);
  KernelClockState out;
  out.synchronized = state != -1 && state != TIME_ERROR && (tx.status & STA_UNSYNC) == 0;
  out.max_error_s = static_cast<double>(tx.maxerror) / 1e6;  // maxerror is in microseconds
  return out;
}

/// "" when a GOTO may rely on the system clock, else "clock_unsynced".
inline std::string check_clock(const KernelClockState& clock, double max_error_s)
{
  return clock.synchronized && clock.max_error_s <= max_error_s ? std::string{}
                                                                : std::string{"clock_unsynced"};
}

// ---------------------------------------------------------------------------
// Request store (persisted across restarts)
// ---------------------------------------------------------------------------

/// A phase is terminal when the bridge no longer follows the request (for
/// LOST, the goal may still be running: see <prefix>/high_level_status).
inline bool is_terminal_phase(const std::string& phase)
{
  return phase == "REFUSED" || phase == "SUCCEEDED" || phase == "ABORTED" || phase == "CANCELED";
}

/// 32 lowercase hex digits: a ROS action goal ID.
inline bool is_goal_id_hex(const std::string& hex)
{
  if (hex.size() != 32)
  {
    return false;
  }
  for (const char c : hex)
  {
    if (!std::isdigit(static_cast<unsigned char>(c)) && (c < 'a' || c > 'f'))
    {
      return false;
    }
  }
  return true;
}

/// The 16 bytes of a goal ID as 32 lowercase hex digits, and back.
inline std::string goal_id_to_hex(const std::array<uint8_t, 16>& id)
{
  std::string hex;
  hex.reserve(32);
  char byte[3];
  for (const uint8_t b : id)
  {
    std::snprintf(byte, sizeof(byte), "%02x", static_cast<unsigned>(b));
    hex += byte;
  }
  return hex;
}

inline std::array<uint8_t, 16> goal_id_from_hex(const std::string& hex)
{
  std::array<uint8_t, 16> id{};
  for (std::size_t k = 0; k < id.size() && 2 * k + 1 < hex.size(); ++k)
  {
    unsigned value = 0;
    std::sscanf(hex.c_str() + 2 * k, "%2x", &value);
    id[k] = static_cast<uint8_t>(value);
  }
  return id;
}

/// A random goal ID (version 4 UUID layout).
inline std::array<uint8_t, 16> new_goal_id()
{
  static std::mt19937_64 engine{std::random_device{}()};
  std::array<uint8_t, 16> id{};
  for (std::size_t k = 0; k < id.size(); k += 8)
  {
    const uint64_t r = engine();
    for (std::size_t j = 0; j < 8; ++j)
    {
      id[k + j] = static_cast<uint8_t>(r >> (8 * j));
    }
  }
  id[6] = static_cast<uint8_t>((id[6] & 0x0F) | 0x40);
  id[8] = static_cast<uint8_t>((id[8] & 0x3F) | 0x80);
  return id;
}

struct StoredRequest
{
  std::string phase;  ///< SUBMITTED, ACCEPTED, ACTIVE, PAUSED or a terminal phase
  int64_t updated_ms{0};  ///< last change, Unix ms
  uint32_t seq{0};  ///< seq of the last event
  std::string goal_id;  ///< the ~/goto goal ID (32 hex digits), chosen before the goal is sent
  /// The goal must still be canceled: set when a restart ended the request
  /// while its goal could still run, cleared once the server confirms it stopped.
  bool cancel_pending{false};
  std::string last_event;  ///< last <prefix>/goto/event payload for this request
};

/// request_id → last known state, written to disk on every phase change so a
/// duplicate is recognised after a restart, and a goal left running by a
/// previous run can be canceled by its own ID. One line per request:
/// "v1<TAB>request_id<TAB>phase<TAB>updated_ms<TAB>seq<TAB>goal_id|-<TAB>cancel_pending<TAB>
/// event_json", cancel_pending being 0 or 1;
/// the event JSON never contains a tab or a newline (json_escape).
class RequestStore
{
public:
  /// Loads `path`. A missing file is an empty store; an unreadable or
  /// malformed file returns false and leaves the store empty.
  bool load(const std::string& path)
  {
    entries_.clear();
    std::error_code ec;
    if (!std::filesystem::exists(path, ec))
    {
      return !ec;
    }
    std::ifstream in(path);
    if (!in)
    {
      return false;
    }
    std::string line;
    while (std::getline(in, line))
    {
      if (line.empty())
      {
        continue;
      }
      std::vector<std::string> fields;
      std::size_t start = 0;
      for (int k = 0; k < 7; ++k)
      {
        const std::size_t tab = line.find('\t', start);
        if (tab == std::string::npos)
        {
          entries_.clear();
          return false;
        }
        fields.push_back(line.substr(start, tab - start));
        start = tab + 1;
      }
      fields.push_back(line.substr(start));
      int64_t updated = 0;
      uint32_t seq = 0;
      const auto parsed_updated =
          std::from_chars(fields[3].data(), fields[3].data() + fields[3].size(), updated);
      const auto parsed_seq =
          std::from_chars(fields[4].data(), fields[4].data() + fields[4].size(), seq);
      const std::string goal_id = fields[5] == "-" ? std::string{} : fields[5];
      if (fields[0] != "v1" || !is_valid_request_id(fields[1]) || fields[2].empty() ||
          parsed_updated.ec != std::errc{} ||
          parsed_updated.ptr != fields[3].data() + fields[3].size() ||
          parsed_seq.ec != std::errc{} || parsed_seq.ptr != fields[4].data() + fields[4].size() ||
          (!goal_id.empty() && !is_goal_id_hex(goal_id)) || (fields[6] != "0" && fields[6] != "1"))
      {
        entries_.clear();
        return false;
      }
      entries_[fields[1]] =
          StoredRequest{fields[2], updated, seq, goal_id, fields[6] == "1", fields[7]};
    }
    return !in.bad();
  }

  /// Writes the store to `path` through a temporary file that is flushed to
  /// disk, renamed over `path`, and the directory synced: a power cut leaves
  /// either the old or the new file.
  bool save(const std::string& path) const
  {
    const std::string tmp = path + ".tmp";
    std::FILE* file = std::fopen(tmp.c_str(), "w");
    if (file == nullptr)
    {
      return false;
    }
    bool ok = true;
    for (const auto& [id, entry] : entries_)
    {
      ok = ok && std::fprintf(file,
                              "v1\t%s\t%s\t%lld\t%u\t%s\t%d\t%s\n",
                              id.c_str(),
                              entry.phase.c_str(),
                              static_cast<long long>(entry.updated_ms),
                              static_cast<unsigned>(entry.seq),
                              entry.goal_id.empty() ? "-" : entry.goal_id.c_str(),
                              entry.cancel_pending ? 1 : 0,
                              entry.last_event.c_str()) > 0;
    }
    ok = ok && std::fflush(file) == 0 && ::fsync(::fileno(file)) == 0;
    ok = std::fclose(file) == 0 && ok;
    if (!ok || std::rename(tmp.c_str(), path.c_str()) != 0)
    {
      return false;
    }
    const std::string dir = std::filesystem::path(path).parent_path().string();
    const int dir_fd = ::open(dir.empty() ? "." : dir.c_str(), O_RDONLY | O_DIRECTORY);
    if (dir_fd >= 0)
    {
      ::fsync(dir_fd);
      ::close(dir_fd);
    }
    return true;
  }

  /// Drops terminal entries older than ttl_ms, then the oldest terminal
  /// entries beyond max_entries. Entries changed within keep_ms — a request
  /// that could still be re-sent as fresh — and non-terminal entries are never
  /// dropped, so a request is never forgotten while it could run again.
  void prune(int64_t now_ms, int64_t ttl_ms, std::size_t max_entries, int64_t keep_ms)
  {
    const auto droppable = [&](const StoredRequest& entry)
    {
      return is_terminal_phase(entry.phase) && !entry.cancel_pending &&
             now_ms - entry.updated_ms > keep_ms;
    };
    for (auto it = entries_.begin(); it != entries_.end();)
    {
      it = droppable(it->second) && now_ms - it->second.updated_ms > ttl_ms ? entries_.erase(it)
                                                                            : std::next(it);
    }
    while (entries_.size() > max_entries)
    {
      auto oldest = entries_.end();
      for (auto it = entries_.begin(); it != entries_.end(); ++it)
      {
        if (droppable(it->second) &&
            (oldest == entries_.end() || it->second.updated_ms < oldest->second.updated_ms))
        {
          oldest = it;
        }
      }
      if (oldest == entries_.end())
      {
        break;
      }
      entries_.erase(oldest);
    }
  }

  /// The request_ids still non-terminal (e.g. after a restart mid-mission).
  std::vector<std::string> active_ids() const
  {
    std::vector<std::string> ids;
    for (const auto& [id, entry] : entries_)
    {
      if (!is_terminal_phase(entry.phase))
      {
        ids.push_back(id);
      }
    }
    return ids;
  }

  /// The request_ids whose goal must still be canceled.
  std::vector<std::string> cancel_pending_ids() const
  {
    std::vector<std::string> ids;
    for (const auto& [id, entry] : entries_)
    {
      if (entry.cancel_pending)
      {
        ids.push_back(id);
      }
    }
    return ids;
  }

  const StoredRequest* find(const std::string& id) const
  {
    const auto it = entries_.find(id);
    return it == entries_.end() ? nullptr : &it->second;
  }

  void put(const std::string& id, StoredRequest entry)
  {
    entries_[id] = std::move(entry);
  }

  std::size_t size() const
  {
    return entries_.size();
  }

private:
  std::map<std::string, StoredRequest> entries_;
};

// ---------------------------------------------------------------------------
// Event payload
// ---------------------------------------------------------------------------

struct WgsPose
{
  double lat{0.0};
  double lon{0.0};
  double yaw{0.0};
  bool valid{false};
  double age_s{0.0};
};

struct GotoEvent
{
  std::string request_id;
  uint32_t seq{0};
  std::string phase;
  std::string error_code{"NONE"};
  std::string error_msg;
  std::string state_name;
  std::optional<WgsPose> current;
  std::optional<double> distance_remaining_m;
  std::optional<WgsPose> final_pose;  ///< terminal phases only
  bool settled{false};  ///< with final_pose
  double datum_lat{0.0};
  double datum_lon{0.0};
  int64_t stamp_ms{0};
  bool durable{true};  ///< false when the bridge could not save this event to disk
};

/// {"lat","lon","yaw","valid"} followed by `tail` (already serialised members,
/// each starting with a comma) and the closing brace. A non-finite coordinate
/// is written as an invalid zero pose, so the payload stays valid JSON.
inline std::string serialise_pose(WgsPose pose, const std::string& tail)
{
  if (!std::isfinite(pose.lat) || !std::isfinite(pose.lon) || !std::isfinite(pose.yaw))
  {
    pose = WgsPose{};
  }
  char buf[160];
  std::snprintf(buf,
                sizeof(buf),
                "{\"lat\":%.8f,\"lon\":%.8f,\"yaw\":%.4f,\"valid\":%s",
                pose.lat,
                pose.lon,
                pose.yaw,
                pose.valid ? "true" : "false");
  return std::string(buf) + tail + "}";
}

/// The <prefix>/goto/event (and goto/state) payload. `final` is null outside
/// terminal phases; `settled` is only reported inside it.
inline std::string serialise_goto_event(const GotoEvent& event)
{
  std::ostringstream out;
  out << "{\"request_id\":\"" << json_escape(event.request_id) << "\",\"seq\":" << event.seq
      << ",\"phase\":\"" << event.phase << "\",\"error_code\":\"" << event.error_code
      << "\",\"error_msg\":\"" << json_escape(event.error_msg) << "\",\"state_name\":\""
      << json_escape(event.state_name) << "\",\"current\":";
  if (event.current)
  {
    char age[32];
    const double age_s = std::isfinite(event.current->age_s) ? event.current->age_s : 0.0;
    std::snprintf(age, sizeof(age), ",\"age_s\":%.2f", age_s);
    out << serialise_pose(*event.current, age);
  }
  else
  {
    out << "null";
  }
  out << ",\"distance_remaining_m\":";
  if (event.distance_remaining_m && std::isfinite(*event.distance_remaining_m))
  {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.2f", *event.distance_remaining_m);
    out << buf;
  }
  else
  {
    out << "null";
  }
  out << ",\"final\":";
  if (event.final_pose && is_terminal_phase(event.phase))
  {
    out << serialise_pose(*event.final_pose,
                          event.settled ? ",\"settled\":true" : ",\"settled\":false");
  }
  else
  {
    out << "null";
  }
  char datum[96];
  std::snprintf(
      datum, sizeof(datum), "{\"lat\":%.8f,\"lon\":%.8f}", event.datum_lat, event.datum_lon);
  out << ",\"datum\":" << datum << ",\"stamp\":\"" << format_iso_utc_ms(event.stamp_ms)
      << "\",\"durable\":" << (event.durable ? "true" : "false") << "}";
  return out.str();
}

}  // namespace mowgli_monitoring::mqtt_goto

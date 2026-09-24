/* SPDX-License-Identifier: GPL-3.0 */
/**
 * @file fw_param_log.h
 * @brief Append-only record format for the parameters persisted in flash.
 *
 * Pure C over a word array (no HAL), so the host test suite compiles it
 * directly (ros2/src/mowgli_hardware/test/test_fw_param_log.cpp). The
 * flash-facing half lives in src/fw_param_store.c.
 *
 * WHY APPEND-ONLY: this board runs a ~40 ms window watchdog, and erasing flash
 * stalls the CPU for 20-40 ms (F103 page) up to seconds (F401 128 KB sector).
 * So the running firmware NEVER erases: it only programs one 32-bit word at a
 * time into already-erased space (tens of microseconds each). Erasing happens
 * at boot, before the watchdog is armed, and only when the log is full or
 * holds something that is not a valid record (e.g. leftovers of the stock
 * firmware).
 *
 * Record layout, in 32-bit words (erased flash reads 0xFFFFFFFF):
 *   [0]              FW_PARAM_LOG_MAGIC
 *   [1]              (entry_count << 16) | FW_PARAM_LOG_FORMAT
 *   [2 .. 2+2n)      entry pairs: id (low 16 bits), value (IEEE-754 bits)
 *   [2+2n]           CRC-32 over words [0 .. 2+2n)
 *   [3+2n]           FW_PARAM_LOG_COMMITTED, programmed LAST
 * A record whose commit word is not programmed (power lost mid-write) or whose
 * CRC does not match is skipped; the newest valid record wins. Entries are
 * (id, value) pairs rather than a positional snapshot so a firmware update that
 * adds, removes or reorders parameters still loads every id it knows.
 */
#ifndef FW_PARAM_LOG_H
#define FW_PARAM_LOG_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FW_PARAM_LOG_MAGIC 0x4D50524Du /* "MPRM" */
#define FW_PARAM_LOG_FORMAT 1u
#define FW_PARAM_LOG_COMMITTED 0x00000000u
#define FW_PARAM_LOG_ERASED 0xFFFFFFFFu
#define FW_PARAM_LOG_MAX_ENTRIES 64u
#define FW_PARAM_LOG_HEADER_WORDS 2u
#define FW_PARAM_LOG_TRAILER_WORDS 2u

/** Size of a record holding @p entry_count entries, in 32-bit words. */
static inline size_t fw_param_log_record_words(size_t entry_count) {
  return FW_PARAM_LOG_HEADER_WORDS + 2u * entry_count + FW_PARAM_LOG_TRAILER_WORDS;
}

/** Standard reflected CRC-32 (poly 0xEDB88320), bitwise: no table in flash. */
static inline uint32_t fw_param_log_crc32(const uint32_t *words, size_t count) {
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t w = 0; w < count; ++w) {
    uint32_t word = words[w];
    for (int byte = 0; byte < 4; ++byte) {
      crc ^= (word >> (8 * byte)) & 0xFFu;
      for (int bit = 0; bit < 8; ++bit) {
        crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
      }
    }
  }
  return ~crc;
}

static inline uint32_t fw_param_log_float_bits(float value) {
  uint32_t bits;
  memcpy(&bits, &value, sizeof(bits));
  return bits;
}

static inline float fw_param_log_bits_float(uint32_t bits) {
  float value;
  memcpy(&value, &bits, sizeof(value));
  return value;
}

/**
 * Encode a record into @p out (capacity @p out_words). Returns the record size
 * in words, or 0 if it does not fit or has too many entries.
 */
static inline size_t fw_param_log_encode(const uint16_t *ids, const float *values,
                                         size_t entry_count, uint32_t *out,
                                         size_t out_words) {
  const size_t words = fw_param_log_record_words(entry_count);
  if (entry_count > FW_PARAM_LOG_MAX_ENTRIES || words > out_words) {
    return 0u;
  }
  out[0] = FW_PARAM_LOG_MAGIC;
  out[1] = ((uint32_t)entry_count << 16) | FW_PARAM_LOG_FORMAT;
  for (size_t i = 0; i < entry_count; ++i) {
    out[FW_PARAM_LOG_HEADER_WORDS + 2u * i] = ids[i];
    out[FW_PARAM_LOG_HEADER_WORDS + 2u * i + 1u] = fw_param_log_float_bits(values[i]);
  }
  const size_t crc_at = FW_PARAM_LOG_HEADER_WORDS + 2u * entry_count;
  out[crc_at] = fw_param_log_crc32(out, crc_at);
  out[crc_at + 1u] = FW_PARAM_LOG_COMMITTED;
  return words;
}

typedef struct {
  /** Word offset of the newest valid record, or -1 when there is none. */
  long last_valid;
  /** Word offset where the next record would start. */
  size_t next_free;
  /** Valid records found (diagnostics). */
  size_t valid_records;
  /** The area holds something that is not a well-formed record (or data
   *  after the first erased word): it must be erased before appending. */
  uint8_t needs_erase;
} fw_param_log_scan_t;

/** Walk the log and locate the newest valid record and the append point. */
static inline fw_param_log_scan_t fw_param_log_scan(const uint32_t *area, size_t area_words) {
  fw_param_log_scan_t scan = {-1, 0u, 0u, 0u};
  size_t off = 0u;
  while (off < area_words) {
    const uint32_t magic = area[off];
    if (magic == FW_PARAM_LOG_ERASED) {
      /* End of log: everything after it must still be erased, otherwise an
       * append would program over old data. */
      for (size_t w = off; w < area_words; ++w) {
        if (area[w] != FW_PARAM_LOG_ERASED) {
          scan.needs_erase = 1u;
          break;
        }
      }
      scan.next_free = off;
      return scan;
    }
    if (magic != FW_PARAM_LOG_MAGIC || off + FW_PARAM_LOG_HEADER_WORDS > area_words) {
      scan.needs_erase = 1u;
      scan.next_free = area_words;
      return scan;
    }
    const uint32_t header = area[off + 1u];
    const size_t entries = header >> 16;
    const size_t words = fw_param_log_record_words(entries);
    if (header == FW_PARAM_LOG_ERASED || (header & 0xFFFFu) != FW_PARAM_LOG_FORMAT ||
        entries > FW_PARAM_LOG_MAX_ENTRIES || off + words > area_words) {
      /* Torn or foreign header: the record length is unknown, so nothing
       * after it can be located. */
      scan.needs_erase = 1u;
      scan.next_free = area_words;
      return scan;
    }
    const size_t crc_at = off + FW_PARAM_LOG_HEADER_WORDS + 2u * entries;
    if (area[crc_at] == fw_param_log_crc32(&area[off], crc_at - off) &&
        area[crc_at + 1u] == FW_PARAM_LOG_COMMITTED) {
      scan.last_valid = (long)off;
      ++scan.valid_records;
    }
    off += words;
  }
  scan.next_free = area_words;
  return scan;
}

/** Entry count of the (valid) record at @p record. */
static inline size_t fw_param_log_entry_count(const uint32_t *record) {
  return record[1] >> 16;
}

/** Entry @p index of the (valid) record at @p record. */
static inline void fw_param_log_entry(const uint32_t *record, size_t index, uint16_t *id,
                                      float *value) {
  *id = (uint16_t)(record[FW_PARAM_LOG_HEADER_WORDS + 2u * index] & 0xFFFFu);
  *value = fw_param_log_bits_float(record[FW_PARAM_LOG_HEADER_WORDS + 2u * index + 1u]);
}

#ifdef __cplusplus
}
#endif

#endif /* FW_PARAM_LOG_H */

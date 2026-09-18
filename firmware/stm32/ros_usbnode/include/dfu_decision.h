#ifndef MOWGLI_DFU_DECISION_H
#define MOWGLI_DFU_DECISION_H

#include <stdbool.h>
#include <stdint.h>

#define DFU_STOP_WINDOW_MS 500u
#define DFU_MARKER_MAGIC 0x44465531u

static inline bool DFU_ReadyToReset(uint32_t start_tick, uint32_t now_tick)
{
  return (uint32_t)(now_tick - start_tick) >= DFU_STOP_WINDOW_MS;
}

/* A SYSRESETREQ may legitimately retain PINRSTF on STM32F4. DFU needs an
 * observed software reset, but must reject reset causes that can indicate an
 * interrupted or unsafe transition. The masks are arguments so this stays
 * pure and native-testable without CMSIS register headers. */
static inline bool DFU_ResetCauseAllowsEntry(uint32_t reset_cause,
                                             uint32_t software_reset_mask,
                                             uint32_t rejected_reset_mask)
{
  return (reset_cause & software_reset_mask) != 0u &&
         (reset_cause & rejected_reset_mask) == 0u;
}

static inline bool DFU_MarkerIsValid(uint32_t marker,
                                     uint32_t inverse,
                                     bool reset_cause_allows_entry)
{
  return reset_cause_allows_entry && marker == DFU_MARKER_MAGIC &&
         inverse == ~DFU_MARKER_MAGIC;
}

#endif

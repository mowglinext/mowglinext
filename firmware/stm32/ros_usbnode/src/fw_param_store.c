/* SPDX-License-Identifier: GPL-3.0 */
/**
 * @file fw_param_store.c
 * @brief HAL flash access for the runtime parameter log. See fw_param_store.h.
 */
#include "fw_param_store.h"

#include "board.h"
#include "main.h"

#if BOARD_YARDFORCE500_VARIANT_ORIG
/* STM32F103VC: 256 KB, 2 KB pages. Last 4 pages. */
#define FW_PARAM_STORE_BASE 0x0803E000u
#define FW_PARAM_STORE_BYTES (4u * 2048u)
#elif BOARD_YARDFORCE500_VARIANT_B
/* STM32F401VC: 256 KB; sector 5 is the last one (128 KB). */
#define FW_PARAM_STORE_BASE 0x08020000u
#define FW_PARAM_STORE_BYTES (128u * 1024u)
#else
#error "fw_param_store: no parameter flash region defined for this board"
#endif

const uint32_t *fw_param_store_area(void) {
  return (const uint32_t *)FW_PARAM_STORE_BASE;
}

size_t fw_param_store_area_words(void) { return FW_PARAM_STORE_BYTES / 4u; }

static void fw_param_store_clear_errors(void) {
#if BOARD_YARDFORCE500_VARIANT_ORIG
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
#else
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                         FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
#endif
}

int fw_param_store_erase(void) {
  FLASH_EraseInitTypeDef erase = {0};
  uint32_t fault = 0u;
#if BOARD_YARDFORCE500_VARIANT_ORIG
  erase.TypeErase = FLASH_TYPEERASE_PAGES;
  erase.PageAddress = FW_PARAM_STORE_BASE;
  erase.NbPages = FW_PARAM_STORE_BYTES / 2048u;
#else
  erase.TypeErase = FLASH_TYPEERASE_SECTORS;
  erase.Sector = FLASH_SECTOR_5;
  erase.NbSectors = 1u;
  erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
#endif
  HAL_FLASH_Unlock();
  fw_param_store_clear_errors();
  const HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase, &fault);
  HAL_FLASH_Lock();
  return (status == HAL_OK && fault == 0xFFFFFFFFu) ? 0 : -1;
}

int fw_param_store_program_word(size_t word_offset, uint32_t value) {
  if (word_offset >= fw_param_store_area_words()) {
    return -1;
  }
  const uint32_t address = FW_PARAM_STORE_BASE + (uint32_t)(word_offset * 4u);
  HAL_FLASH_Unlock();
  fw_param_store_clear_errors();
  const HAL_StatusTypeDef status =
      HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, (uint64_t)value);
  HAL_FLASH_Lock();
  return (status == HAL_OK) ? 0 : -1;
}

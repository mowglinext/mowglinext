#include "dfu_boot.h"

#include "board.h"
#include "dfu_decision.h"

#if BOARD_YARDFORCE500_VARIANT_B

#include "stm32f4xx.h"

typedef struct
{
  volatile uint32_t marker;
  volatile uint32_t inverse;
} dfu_marker_t;

/* The custom F401 linker script locates this exact 8-byte section at the top
 * of the official 64 KiB SRAM and marks it NOLOAD. */
__attribute__((section(".dfu_magic"), used)) static dfu_marker_t g_dfu_marker;

#define STM32F401_SYSTEM_MEMORY_BASE 0x1FFF0000u
#define STM32F401_SYSTEM_MEMORY_END 0x1FFF8000u
#define STM32F401_SRAM_BASE 0x20000000u
#define STM32F401_SRAM_END 0x20010000u

static int dfu_rom_vector_is_valid(uint32_t sp, uint32_t entry)
{
  return sp >= STM32F401_SRAM_BASE && sp < STM32F401_SRAM_END && (sp & 0x7u) == 0u &&
         (entry & 1u) != 0u && (entry & ~1u) >= STM32F401_SYSTEM_MEMORY_BASE &&
         (entry & ~1u) < STM32F401_SYSTEM_MEMORY_END;
}

static void dfu_assert_hardware_inhibit(void)
{
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
  (void)RCC->AHB1ENR;
  /* Load the safe output latch values while both pins are still inputs. In
   * particular PE15 is active-low drive enable: configuring output mode first
   * would briefly drive the reset-default low latch and enable traction. */
  GPIOE->BSRR = (1u << (14u + 16u)) | (1u << 15u);
  GPIOE->OTYPER &= ~((1u << 14u) | (1u << 15u));
  GPIOE->PUPDR &= ~((3u << (14u * 2u)) | (3u << (15u * 2u)));
  GPIOE->MODER = (GPIOE->MODER & ~((3u << (14u * 2u)) | (3u << (15u * 2u)))) | (1u << (14u * 2u)) |
                 (1u << (15u * 2u));
}

/* Changing MSP inside an ordinary C function would let its epilogue access the
 * new ROM stack. Keep the final transfer as an ABI-sized naked trampoline:
 * r0=ROM MSP, r1=ROM reset entry. Interrupts are re-enabled only after MSP is
 * valid, immediately before the non-returning branch. */
__attribute__((naked, noreturn)) static void dfu_branch_to_rom(uint32_t sp, uint32_t entry)
{
  __asm volatile(
      "msr msp, r0\n"
      "cpsie i\n"
      "bx r1\n");
}

static void dfu_jump_to_rom(void)
{
  const uint32_t rom_sp = *(const volatile uint32_t *)STM32F401_SYSTEM_MEMORY_BASE;
  const uint32_t rom_entry = *(const volatile uint32_t *)(STM32F401_SYSTEM_MEMORY_BASE + 4u);
  if (!dfu_rom_vector_is_valid(rom_sp, rom_entry))
  {
    return;
  }

  __disable_irq();
  SysTick->CTRL = 0u;
  SysTick->LOAD = 0u;
  SysTick->VAL = 0u;
  for (uint32_t i = 0u; i < 8u; ++i)
  {
    NVIC->ICER[i] = 0xFFFFFFFFu;
    NVIC->ICPR[i] = 0xFFFFFFFFu;
  }
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
  SYSCFG->MEMRMP = SYSCFG_MEMRMP_MEM_MODE_0;
  SCB->VTOR = STM32F401_SYSTEM_MEMORY_BASE;
  __DSB();
  __ISB();
  dfu_branch_to_rom(rom_sp, rom_entry);
}

void DFU_PreRuntime(void)
{
  const uint32_t marker = g_dfu_marker.marker;
  const uint32_t inverse = g_dfu_marker.inverse;
  const uint32_t reset_cause = RCC->CSR;
  g_dfu_marker.marker = 0u;
  g_dfu_marker.inverse = 0u;

  /* PINRSTF may accompany a valid SYSRESETREQ on this MCU, so it is
   * deliberately not part of the reject set. PIN-only still fails because
   * SFTRSTF is required. */
  const uint32_t rejected_reset_flags = RCC_CSR_LPWRRSTF | RCC_CSR_WWDGRSTF |
                                        RCC_CSR_IWDGRSTF | RCC_CSR_PORRSTF |
                                        RCC_CSR_BORRSTF;
  const bool reset_cause_allows_entry = DFU_ResetCauseAllowsEntry(
      reset_cause, RCC_CSR_SFTRSTF, rejected_reset_flags);

  if (!DFU_MarkerIsValid(marker, inverse, reset_cause_allows_entry))
  {
    return;
  }

  dfu_assert_hardware_inhibit();
  dfu_jump_to_rom();
}

void DFU_ArmAndReset(void)
{
  /* Remove sticky historical reset flags immediately before arming. The next
   * boot requires the fresh SFTRSTF produced below and rejects mixed watchdog,
   * brownout, power or low-power history. PINRSTF may accompany SYSRESETREQ. */
  RCC->CSR |= RCC_CSR_RMVF;
  __DSB();
  g_dfu_marker.marker = DFU_MARKER_MAGIC;
  g_dfu_marker.inverse = ~DFU_MARKER_MAGIC;
  __DSB();
  NVIC_SystemReset();
  for (;;)
  {
  }
}

#else

void DFU_PreRuntime(void)
{
}
void DFU_ArmAndReset(void)
{
}

#endif

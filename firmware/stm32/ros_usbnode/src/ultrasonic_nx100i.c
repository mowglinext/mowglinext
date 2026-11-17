/**
 * @file ultrasonic_nx100i.c
 * @brief YardForce NX100i hood ultrasonic board ("RM CSB") driver.
 *
 * The NX100i hood carries a small ultrasonic board with two sensors that the
 * mainboard reads as an I2C slave (7-bit address 0x51) over a bit-banged bus on
 * PC10 (SCL) / PC11 (SDA). Reverse-engineered from the stock firmware
 * ("GRM V4.03_2021"), which:
 *   - reads register 0x22, 26 bytes: ASCII ID starting "RM CSB" (bring-up),
 *   - then every 50 ms reads register 0x20, 4 bytes: two big-endian u16
 *     distances in 0.1 mm (d0, d1),
 *   - treats 2 s without a valid read as "sensor gone" (distances 0, re-probe).
 * The stock bit-bangs push-pull at roughly 15-18 kHz; this driver uses real
 * open-drain outputs with the internal pull-ups and ~25 kHz.
 *
 * It exposes the ULTRASONICSENSOR_* interface of ultrasonic_sensor.c so the
 * existing OPTION_ULTRASONIC plumbing (50 ms NBT in main.c, uss_ranges_m[0..1]
 * in the status packet) works unchanged. Values are in 0.1 mm like the UART
 * backend. Which sensor is left/right has not been verified on hardware; d0
 * is reported as left (same convention as the LUV1000RI module).
 *
 * Timing: a 4-byte register read takes ~3 ms, the one-off 26-byte ID read
 * ~11 ms, both well inside the WWDG window. No interrupts are disabled.
 */
#include "board.h"

#if (OPTION_ULTRASONIC == 1) && defined(ULTRASONIC_TYPE) && (ULTRASONIC_TYPE == ULTRASONIC_TYPE_NX100I_I2C)

#include <string.h>
#include "stm32f_board_hal.h"
#include "main.h"
#include "ultrasonic_sensor.h"

#define CSB_ADDR7 0x51u
#define CSB_REG_ID 0x22u
#define CSB_REG_RANGES 0x20u
#define CSB_ID_LEN 26u
#define CSB_ID_PREFIX "RM CSB"
#define CSB_TIMEOUT_MS 2000u
#define CSB_HALF_PERIOD_US 20u /* ~25 kHz SCL */
#define CSB_STRETCH_LIMIT_US 1000u

typedef enum {
  CSB_STATE_PROBE,
  CSB_STATE_RUN
} csb_state_t;

static csb_state_t csb_state = CSB_STATE_PROBE;
static uint32_t csb_left_01mm = 0;
static uint32_t csb_right_01mm = 0;
static uint32_t csb_last_valid_tick = 0;
static uint8_t csb_fresh = 0;
static char csb_id[CSB_ID_LEN + 1] = {0};

/* ---- microsecond delay on the DWT cycle counter ------------------------- */
static void csb_dwt_init(void) {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void csb_delay_us(uint32_t us) {
  const uint32_t start = DWT->CYCCNT;
  const uint32_t cycles = us * (SystemCoreClock / 1000000u);
  while ((DWT->CYCCNT - start) < cycles) {
  }
}

/* ---- open-drain bit-bang primitives -------------------------------------- */
static inline void csb_scl(uint8_t high) {
  HAL_GPIO_WritePin(NX100I_USS_SCL_PORT, NX100I_USS_SCL_PIN, high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
static inline void csb_sda(uint8_t high) {
  HAL_GPIO_WritePin(NX100I_USS_SDA_PORT, NX100I_USS_SDA_PIN, high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
static inline uint8_t csb_sda_read(void) {
  return HAL_GPIO_ReadPin(NX100I_USS_SDA_PORT, NX100I_USS_SDA_PIN) == GPIO_PIN_SET ? 1u : 0u;
}
static inline uint8_t csb_scl_read(void) {
  return HAL_GPIO_ReadPin(NX100I_USS_SCL_PORT, NX100I_USS_SCL_PIN) == GPIO_PIN_SET ? 1u : 0u;
}

/* Release SCL and wait for it to actually rise (clock stretching), bounded. */
static uint8_t csb_scl_release(void) {
  csb_scl(1);
  uint32_t waited = 0;
  while (!csb_scl_read()) {
    csb_delay_us(1);
    if (++waited > CSB_STRETCH_LIMIT_US) return 0;
  }
  csb_delay_us(CSB_HALF_PERIOD_US);
  return 1;
}

static void csb_pins_init(void) {
  GPIO_InitTypeDef gpio = {0};
  NX100I_USS_GPIO_CLK_ENABLE();
  csb_scl(1);
  csb_sda(1);
  gpio.Mode = GPIO_MODE_OUTPUT_OD;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  gpio.Pin = NX100I_USS_SCL_PIN;
  HAL_GPIO_Init(NX100I_USS_SCL_PORT, &gpio);
  gpio.Pin = NX100I_USS_SDA_PIN;
  HAL_GPIO_Init(NX100I_USS_SDA_PORT, &gpio);
}

static void csb_start(void) {
  csb_sda(1);
  csb_scl(1);
  csb_delay_us(CSB_HALF_PERIOD_US);
  csb_sda(0);
  csb_delay_us(CSB_HALF_PERIOD_US);
  csb_scl(0);
  csb_delay_us(CSB_HALF_PERIOD_US);
}

static void csb_stop(void) {
  csb_sda(0);
  csb_delay_us(CSB_HALF_PERIOD_US);
  csb_scl(1);
  csb_delay_us(CSB_HALF_PERIOD_US);
  csb_sda(1);
  csb_delay_us(CSB_HALF_PERIOD_US);
}

/* Returns 1 when the slave ACKed. */
static uint8_t csb_write_byte(uint8_t b) {
  for (uint8_t i = 0; i < 8u; i++) {
    csb_sda((b & 0x80u) ? 1u : 0u);
    b <<= 1;
    csb_delay_us(CSB_HALF_PERIOD_US);
    if (!csb_scl_release()) return 0;
    csb_scl(0);
  }
  csb_sda(1); /* release for ACK */
  csb_delay_us(CSB_HALF_PERIOD_US);
  if (!csb_scl_release()) return 0;
  const uint8_t ack = csb_sda_read() ? 0u : 1u;
  csb_scl(0);
  csb_delay_us(CSB_HALF_PERIOD_US);
  return ack;
}

static uint8_t csb_read_byte(uint8_t ack, uint8_t *out) {
  uint8_t v = 0;
  csb_sda(1);
  for (uint8_t i = 0; i < 8u; i++) {
    csb_delay_us(CSB_HALF_PERIOD_US);
    if (!csb_scl_release()) return 0;
    v = (uint8_t)((v << 1) | csb_sda_read());
    csb_scl(0);
  }
  csb_sda(ack ? 0u : 1u);
  csb_delay_us(CSB_HALF_PERIOD_US);
  if (!csb_scl_release()) return 0;
  csb_scl(0);
  csb_sda(1);
  csb_delay_us(CSB_HALF_PERIOD_US);
  *out = v;
  return 1;
}

/* START, W addr, W reg, repeated START, R addr, read n bytes, STOP. */
static uint8_t csb_read_reg(uint8_t reg, uint8_t *buf, uint8_t n) {
  csb_start();
  if (!csb_write_byte((uint8_t)(CSB_ADDR7 << 1))) { csb_stop(); return 0; }
  if (!csb_write_byte(reg)) { csb_stop(); return 0; }
  csb_start();
  if (!csb_write_byte((uint8_t)((CSB_ADDR7 << 1) | 1u))) { csb_stop(); return 0; }
  for (uint8_t i = 0; i < n; i++) {
    if (!csb_read_byte((i + 1u < n) ? 1u : 0u, &buf[i])) { csb_stop(); return 0; }
  }
  csb_stop();
  return 1;
}

/* ---- ULTRASONICSENSOR_* interface ---------------------------------------- */
void ULTRASONICSENSOR_Init(void) {
  csb_dwt_init();
  csb_pins_init();
  csb_state = CSB_STATE_PROBE;
  csb_left_01mm = 0;
  csb_right_01mm = 0;
  csb_fresh = 0;
  csb_last_valid_tick = HAL_GetTick();
  memset(csb_id, 0, sizeof(csb_id));
}

/* Called from the main loop every 50 ms (main_ultrasonicsensor_nbt). */
void ULTRASONICSENSOR_App(void) {
  uint8_t buf[CSB_ID_LEN];

  switch (csb_state) {
    case CSB_STATE_PROBE:
      if (csb_read_reg(CSB_REG_ID, buf, CSB_ID_LEN) &&
          memcmp(buf, CSB_ID_PREFIX, sizeof(CSB_ID_PREFIX) - 1u) == 0) {
        memcpy(csb_id, buf, CSB_ID_LEN);
        csb_id[CSB_ID_LEN] = '\0';
        csb_last_valid_tick = HAL_GetTick();
        csb_state = CSB_STATE_RUN;
        DB_TRACE(" * NX100i ultrasonic board found: %s\r\n", csb_id);
      }
      break;

    case CSB_STATE_RUN:
    default:
      if (csb_read_reg(CSB_REG_RANGES, buf, 4u)) {
        csb_left_01mm = ((uint32_t)buf[0] << 8) | buf[1];
        csb_right_01mm = ((uint32_t)buf[2] << 8) | buf[3];
        csb_last_valid_tick = HAL_GetTick();
        csb_fresh = 1;
      } else if ((uint32_t)(HAL_GetTick() - csb_last_valid_tick) > CSB_TIMEOUT_MS) {
        csb_left_01mm = 0;
        csb_right_01mm = 0;
        csb_state = CSB_STATE_PROBE;
        DB_TRACE(" * NX100i ultrasonic board lost, re-probing\r\n");
      }
      break;
  }
}

void ULTRASONICSENSOR_ReceiveIT(void) {
  /* no UART path on this backend */
}

uint32_t ULTRASONIC_MessageReceived(void) {
  if (csb_fresh) {
    csb_fresh = 0;
    return 1;
  }
  return 0;
}

uint32_t ULTRASONICSENSOR_u32GetLeftDistance(void) { return csb_left_01mm; }
uint32_t ULTRASONICSENSOR_u32GetRightDistance(void) { return csb_right_01mm; }

#endif /* NX100i I2C backend */

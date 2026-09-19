/**
  ******************************************************************************
  * @file    i2c.c
  * @author  Georg Swoboda <cn@warp.at>
  * @date    21/09/22
  * @version 1.0.0
  * @brief   I2C Driver for the LIS3DH that is onboard GForce
  ******************************************************************************
  * @attention
  *
  * the setup sequence for tilt sensing and triggering INT1 matches the 
  * original firmware  
  ******************************************************************************
  */

#include <math.h>
#include <string.h>
#include "board.h"
#include "main.h"
#include "i2c.h"
#include "emergency.h"
#include "imu/imu.h"
#include "stm32f_board_hal.h"
#include "i2c_lis3dh.h"

/* I2C1/PB6/PB7 serves the onboard LIS3DH, NOT the external IMU.
 * Only the foreground may transact. USB callbacks consume snapshots below.
 * Busy is checked before HAL (which otherwise has a fixed 25 ms busy wait).
 * Each service pass requests at most one transaction (2 ms HAL timeout) or
 * one GPIO edge. This is not a hard wall-time guarantee for the vendor HAL.
 */
#define SENSOR_IO_MS 2u
#define SENSOR_POLL_MS 10u
#define SENSOR_FRESH_MS 100u
#define SENSOR_RETRY_MS 1000u
#define SENSOR_EDGE_MS 1u

I2C_HandleTypeDef I2C_Handle;
static volatile uint8_t sensor_valid, sensor_tilt;
static volatile uint32_t sensor_sample_tick;
static uint32_t state_tick, poll_tick, audit_tick;
static uint8_t audit_index;
static uint8_t last_was_poll;
static uint8_t pulse_count, config_index;
static enum { SENSOR_ID, SENSOR_WRITE, SENSOR_VERIFY, SENSOR_POLL,
              SENSOR_COOLDOWN, SENSOR_RELEASE, SENSOR_CLOCK_LOW,
              SENSOR_CLOCK_HIGH, SENSOR_STOP_LOW, SENSOR_STOP_CLOCK,
              SENSOR_STOP_DATA, SENSOR_RESTORE } sensor_state;
/* RAM-only diagnostics. Faults also assert the existing emergency indication. */
static volatile struct { uint32_t faults, attempts, recoveries, samples; } onboard_i2c_diag;
static uint8_t recovering;

/* Restore the existing safety configuration, with INT1 routing enabled last.
 * See LIS3DH register definitions: 100 Hz XYZ, +/-2g HR/BDU, temperature ADC,
 * Z-low threshold/duration, pulsed active-high INT1 and bypass FIFO.
 */
static const struct { uint8_t reg, value; } sensor_config[] = {
    {LIS3DH_CTRL_REG3, 0x00},
    {LIS3DH_CTRL_REG1, 0x57},
    {LIS3DH_CTRL_REG2, 0x00},
    {LIS3DH_CTRL_REG4, 0x88},
    {LIS3DH_TEMP_CFG_REG, 0xc0},
    {LIS3DH_CTRL_REG5, 0x00},
    {LIS3DH_INT1_THS, IMU_ONBOARD_INCLINATION_THRESHOLD},
    {LIS3DH_INT1_DURATION, 0x01},
    {LIS3DH_INT1_CFG, 0x10},
    {LIS3DH_CTRL_REG6, 0x00},
    {LIS3DH_FIFO_CTRL_REG, 0x00},
    {LIS3DH_CTRL_REG3, 0x40},
};

static void sensor_fault(void)
{
    sensor_valid = 0;
    Emergency_OnboardSensorFault();
    ++onboard_i2c_diag.faults;
    debug_printf("Onboard tilt sensor unavailable; motion inhibited, recovery pending\r\n");
    sensor_state = SENSOR_COOLDOWN;
    state_tick = HAL_GetTick();
}

static uint8_t sensor_bus_init(void)
{
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();
    __HAL_RCC_I2C1_FORCE_RESET();
    __HAL_RCC_I2C1_RELEASE_RESET();
    gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Mode = GPIO_MODE_AF_OD;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
#if BOARD_YARDFORCE500_VARIANT_B
    gpio.Alternate = GPIO_AF4_I2C1;
#endif
    HAL_GPIO_Init(GPIOB, &gpio);
    memset(&I2C_Handle, 0, sizeof(I2C_Handle));
    I2C_Handle.Instance = I2C1;
    I2C_Handle.Init.ClockSpeed = 400000;
    I2C_Handle.Init.DutyCycle = I2C_DUTYCYCLE_2;
    I2C_Handle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    I2C_Handle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    I2C_Handle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    I2C_Handle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    return HAL_I2C_Init(&I2C_Handle) == HAL_OK;
}

void I2C_Init(void)
{
    sensor_valid = 0;
    sensor_tilt = 0;
    recovering = 0;
    config_index = audit_index = 0;
    last_was_poll = 0;
    audit_tick = HAL_GetTick();
    sensor_state = SENSOR_ID;
    if (!sensor_bus_init()) sensor_fault();
}

static uint8_t sensor_can_transact(void *handle)
{
    return __get_IPSR() == 0 && handle == &I2C_Handle &&
        (sensor_state == SENSOR_ID || sensor_state == SENSOR_WRITE ||
         sensor_state == SENSOR_VERIFY || sensor_state == SENSOR_POLL) &&
        HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_SET &&
        HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET &&
        __HAL_I2C_GET_FLAG(&I2C_Handle, I2C_FLAG_BUSY) == RESET;
}

int32_t I2C_platform_read(void *handle, uint8_t reg, uint8_t *buf, uint16_t len)
{
    if (__get_IPSR() != 0) return -1;
    if (!sensor_can_transact(handle) ||
        HAL_I2C_Mem_Read(handle, LIS3DH_I2C_ADD_L, reg | 0x80,
                         I2C_MEMADD_SIZE_8BIT, buf, len, SENSOR_IO_MS) != HAL_OK) {
        sensor_fault();
        return -1;
    }
    return 0;
}

int32_t I2C_platform_write(void *handle, uint8_t reg, const uint8_t *buf, uint16_t len)
{
    if (__get_IPSR() != 0) return -1;
    if (!sensor_can_transact(handle) ||
        HAL_I2C_Mem_Write(handle, LIS3DH_I2C_ADD_L, reg | 0x80,
                          I2C_MEMADD_SIZE_8BIT, (uint8_t *)buf, len, SENSOR_IO_MS) != HAL_OK) {
        sensor_fault();
        return -1;
    }
    return 0;
}

/* Snapshot age is delivery liveness, not proof of a new physical measurement.
 * Only a successful INT1_SRC transaction publishes this snapshot, and every
 * transaction/configuration failure invalidates it before starting recovery.
 */
static uint8_t sensor_snapshot(uint8_t *tilt)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint8_t valid = sensor_valid;
    uint32_t tick = sensor_sample_tick;
    *tilt = sensor_tilt;
    __set_PRIMASK(primask);
    return valid && (uint32_t)(HAL_GetTick() - tick) <= SENSOR_FRESH_MS;
}

uint8_t I2C_OnboardHealthy(void)
{
    uint8_t tilt;
    return sensor_snapshot(&tilt);
}

uint8_t I2C_TestZLowINT(void)
{
    uint8_t tilt;
    return !sensor_snapshot(&tilt) || tilt;
}

void I2C_Onboard_Service(void)
{
    if (__get_IPSR() != 0) return;
    uint32_t now = HAL_GetTick();
    uint8_t value = 0;
    if (sensor_valid && !I2C_OnboardHealthy()) { sensor_fault(); return; }
    switch (sensor_state) {
    case SENSOR_ID:
        if (I2C_platform_read(&I2C_Handle, LIS3DH_WHO_AM_I, &value, 1)) return;
        if (value != LIS3DH_ID) { sensor_fault(); return; }
        config_index = 0;
        audit_index = 0;
        audit_tick = now;
        last_was_poll = 0;
        sensor_state = SENSOR_WRITE;
        return;
    case SENSOR_WRITE:
        if (I2C_platform_write(&I2C_Handle, sensor_config[config_index].reg,
                               &sensor_config[config_index].value, 1)) return;
        sensor_state = SENSOR_VERIFY;
        return;
    case SENSOR_VERIFY:
        if (I2C_platform_read(&I2C_Handle, sensor_config[config_index].reg, &value, 1)) return;
        if (value != sensor_config[config_index].value) { sensor_fault(); return; }
        if (++config_index == sizeof(sensor_config) / sizeof(sensor_config[0])) {
            sensor_state = SENSOR_POLL;
            poll_tick = now - SENSOR_POLL_MS;
        } else sensor_state = SENSOR_WRITE;
        return;
    case SENSOR_POLL:
        /* Audit identity/configuration a register at a time between polls.
         * This detects a powered-but-reset sensor whose INT source reads zero.
         * Skip the initial routing-disable entry; final CTRL3 is 0x40. */
        if (last_was_poll && (uint32_t)(now - audit_tick) >= 1000u) {
            last_was_poll = 0;
            uint8_t reg = audit_index ? sensor_config[audit_index].reg : LIS3DH_WHO_AM_I;
            uint8_t expected = audit_index ? sensor_config[audit_index].value : LIS3DH_ID;
            if (I2C_platform_read(&I2C_Handle, reg, &value, 1)) return;
            if (value != expected) { sensor_fault(); return; }
            if (++audit_index == sizeof(sensor_config) / sizeof(sensor_config[0])) {
                audit_index = 0;
                audit_tick = now;
            }
            return;
        }
        if ((uint32_t)(now - poll_tick) < SENSOR_POLL_MS) return;
        poll_tick = now;
        if (I2C_platform_read(&I2C_Handle, LIS3DH_INT1_SRC, &value, 1)) return;
        /* Publish validity last; a preempting USB reader sees invalid until all
         * fields belong to this successful transaction. */
        sensor_valid = 0;
        sensor_tilt = (value & 0x50) == 0x50;
        sensor_sample_tick = HAL_GetTick();
        sensor_valid = 1;
        ++onboard_i2c_diag.samples;
        last_was_poll = 1;
        if (recovering) { ++onboard_i2c_diag.recoveries; recovering = 0; }
        return;
    case SENSOR_COOLDOWN:
        if ((uint32_t)(now - state_tick) < SENSOR_RETRY_MS) return;
        ++onboard_i2c_diag.attempts;
        recovering = 1;
        /* Stop the peripheral before taking GPIO ownership. Never drive high. */
        __HAL_RCC_I2C1_FORCE_RESET();
        __HAL_RCC_I2C1_RELEASE_RESET();
        {
            GPIO_InitTypeDef gpio = {0};
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET);
            gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
            gpio.Mode = GPIO_MODE_OUTPUT_OD;
            gpio.Pull = GPIO_PULLUP;
            gpio.Speed = GPIO_SPEED_FREQ_HIGH;
            HAL_GPIO_Init(GPIOB, &gpio);
        }
        pulse_count = 0;
        state_tick = now;
        sensor_state = SENSOR_RELEASE;
        return;
    case SENSOR_RELEASE:
    case SENSOR_CLOCK_HIGH:
        if ((uint32_t)(now - state_tick) < SENSOR_EDGE_MS) return;
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) != GPIO_PIN_SET) goto bus_failed;
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET) {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
            sensor_state = SENSOR_STOP_LOW;
        } else if (pulse_count < 9) {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
            sensor_state = SENSOR_CLOCK_LOW;
        } else goto bus_failed;
        state_tick = now;
        return;
    case SENSOR_CLOCK_LOW:
        if ((uint32_t)(now - state_tick) < SENSOR_EDGE_MS) return;
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
        ++pulse_count;
        sensor_state = SENSOR_CLOCK_HIGH;
        state_tick = now;
        return;
    case SENSOR_STOP_LOW:
        if ((uint32_t)(now - state_tick) < SENSOR_EDGE_MS) return;
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
        sensor_state = SENSOR_STOP_CLOCK;
        state_tick = now;
        return;
    case SENSOR_STOP_CLOCK:
        if ((uint32_t)(now - state_tick) < SENSOR_EDGE_MS) return;
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) != GPIO_PIN_SET) goto bus_failed;
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
        sensor_state = SENSOR_STOP_DATA;
        state_tick = now;
        return;
    case SENSOR_STOP_DATA:
        if ((uint32_t)(now - state_tick) < SENSOR_EDGE_MS) return;
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) != GPIO_PIN_SET ||
            HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) != GPIO_PIN_SET) goto bus_failed;
        sensor_state = SENSOR_RESTORE;
        return;
    case SENSOR_RESTORE:
        if (!sensor_bus_init()) { sensor_fault(); return; }
        sensor_state = SENSOR_ID;
        return;
    }
    return;
bus_failed:
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET);
    sensor_fault();
}

/* Legacy foreground diagnostic API. No reinitialization or fabricated sample
 * on failure; runtime safety reads only the service's qualified snapshot. */
void I2C_ReadAccelerometer(float *x, float *y, float *z)
{
    uint8_t bytes[6];
    *x = *y = *z = NAN;
    if (!I2C_OnboardHealthy() || __get_IPSR() != 0) return;
    if (I2C_platform_read(&I2C_Handle, LIS3DH_OUT_X_L, bytes, sizeof(bytes))) return;
    *x = lis3dh_from_fs2_hr_to_mg((int16_t)(bytes[0] | bytes[1] << 8)) / 1000.0f * MS2_PER_G;
    *y = lis3dh_from_fs2_hr_to_mg((int16_t)(bytes[2] | bytes[3] << 8)) / 1000.0f * MS2_PER_G;
    *z = lis3dh_from_fs2_hr_to_mg((int16_t)(bytes[4] | bytes[5] << 8)) / 1000.0f * MS2_PER_G;
}

float I2C_ReadAccelerometerTemp(void)
{
    uint8_t bytes[2];
    if (!I2C_OnboardHealthy() || __get_IPSR() != 0) return NAN;
    if (I2C_platform_read(&I2C_Handle, LIS3DH_OUT_ADC3_L, bytes, sizeof(bytes))) return NAN;
    return lis3dh_from_lsb_hr_to_celsius((int16_t)(bytes[0] | bytes[1] << 8));
}

uint8_t I2C_Acclerometer_TestDevice(void) { return I2C_OnboardHealthy(); }
void I2C_Accelerometer_Setup(void) { /* I2C_Onboard_Service owns setup. */ }

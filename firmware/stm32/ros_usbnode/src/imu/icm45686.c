/* Minimal ICM-45686 driver using the same simple pattern as MPU6050 driver.
 * No inv_imu vendor stack required. This implementation probes WHO_AM_I and
 * performs simple reads of accel/gyro registers.
 */

#include "imu/imu.h"
#include "imu/icm45686.h"
#include "soft_i2c.h"
#include "main.h"
#include <math.h>
#include <stdint.h>

/* TIMER__Wait_us is implemented in src/soft_i2c.c but not declared in a header
 * (legacy pattern). Provide an extern prototype here to avoid implicit
 * declaration warnings.
 */
extern void TIMER__Wait_us(uint32_t nCount);

/* runtime-selected scale factors (filled in ICM45686_Init) */
static float icm45686_g_per_lsb = 1.0f/16384.0f; /* default for +/-2g */
static float icm45686_deg_per_lsb = 250.0f / 32768.0f; /* default for 250 dps */
static icm45686_accel_fs_sel_t icm45686_accel_fs = ICM45686_ACCEL_FS_SEL_2_G;
static icm45686_gyro_fs_sel_t icm45686_gyro_fs = ICM45686_GYRO_FS_SEL_250_DPS;

/* I2C address the device actually answered on; resolved by ICM45686_TestDevice()
 * (probes ICM45686_ADDRESS then ICM45686_ADDRESS_ALT). Used by Init/reads. */
static uint8_t icm45686_addr = ICM45686_ADDRESS;

/* Registers used by the simple init sequence (from vendor regmap excerpts) */
#define ICM45686_REG_MISC2         0x7F
#define ICM45686_REG_PWR_MGMT0     0x10
#define ICM45686_REG_ACCEL_CONFIG0 0x1B
#define ICM45686_REG_GYRO_CONFIG0  0x1C

#ifndef DISABLE_ICM45686

uint8_t ICM45686_TestDevice(void)
{
  const uint8_t candidates[2] = { ICM45686_ADDRESS, ICM45686_ADDRESS_ALT };
  for (uint8_t i = 0; i < 2; i++) {
    uint8_t val = SW_I2C_UTIL_Read(candidates[i], ICM45686_WHO_AM_I);
    if (val == ICM45686_WHO_AM_I_ID) {
      icm45686_addr = candidates[i];
      debug_printf("    > [ICM-45686] - WHO_AM_I=0x%02x (OK) at I2C addr=0x%02x\r\n", val, icm45686_addr);
      return 1;
    }
    debug_printf("    > [ICM-45686] - probe at I2C addr=0x%02x got 0x%02x\r\n", candidates[i], val);
  }
  debug_printf("    > [ICM-45686] - Error: not found at 0x%02x or 0x%02x\r\n", ICM45686_ADDRESS, ICM45686_ADDRESS_ALT);
  return 0;
}

void ICM45686_Init(void)
{
  uint8_t who = SW_I2C_UTIL_Read(icm45686_addr, ICM45686_WHO_AM_I);
  if (who != ICM45686_WHO_AM_I_ID) {
    debug_printf(" * ICM-45686 not found during init (who=0x%02x)\r\n", who);
    return;
  }

  debug_printf(" * ICM-45686 init: WHO_AM_I=0x%02x\r\n", who);

  /* Soft reset: write soft reset bit in REG_MISC2 (vendor regmap indicates soft reset at REG_MISC2 bit) */
  SW_I2C_UTIL_WRITE(icm45686_addr, ICM45686_REG_MISC2, 0x02);
  /* Short delay for reset to complete */
  TIMER__Wait_us(2000);

  /* Set power management: enable accelerometer and gyro in low-noise mode per datasheet.
   * Writing 0x0F enables both accel and gyro low-noise default mode (vendor datasheet).
   */
  SW_I2C_UTIL_WRITE(icm45686_addr, ICM45686_REG_PWR_MGMT0, ICM45686_PWR_MGMT0_ACCEL_GYRO_LOW_NOISE);

  /* Configure accelerometer and gyro:
   * - accel FSR: +/-2g, ODR: 100Hz
   * - gyro FSR: 250 dps, ODR: 100Hz
   */
  uint8_t accel_cfg = ICM45686_ACCEL_CONFIG0_VALUE(ICM45686_ACCEL_FS_SEL_2_G, ICM45686_ACCEL_ODR_100HZ);
  uint8_t gyro_cfg  = ICM45686_GYRO_CONFIG0_VALUE(ICM45686_GYRO_FS_SEL_250_DPS, ICM45686_GYRO_ODR_100HZ);
  SW_I2C_UTIL_WRITE(icm45686_addr, ICM45686_REG_ACCEL_CONFIG0, accel_cfg);
  SW_I2C_UTIL_WRITE(icm45686_addr, ICM45686_REG_GYRO_CONFIG0, gyro_cfg);

  /* record selected FS and compute scale factors (LSB -> physical units)
   * Accelerometer: assume 16-bit output, so LSB_per_g = 16384 / (FS/2) ???
   * Use canonical formula: LSB_per_g = 16384 / (FS/2) is confusing; instead compute
   * directly: for signed 16-bit, full-scale range +/-FS_g maps to +/-32768 counts,
   * therefore LSB_per_g = 32768 / (2 * FS_g) = 16384 / FS_g.
   */
  icm45686_accel_fs = ICM45686_ACCEL_FS_SEL_2_G;
  switch (icm45686_accel_fs) {
    case ICM45686_ACCEL_FS_SEL_2_G:  icm45686_g_per_lsb = 1.0f / 16384.0f; break; /* 2g */
    case ICM45686_ACCEL_FS_SEL_4_G:  icm45686_g_per_lsb = 1.0f / 8192.0f;  break; /* 4g */
    case ICM45686_ACCEL_FS_SEL_8_G:  icm45686_g_per_lsb = 1.0f / 4096.0f;  break; /* 8g */
    case ICM45686_ACCEL_FS_SEL_16_G: icm45686_g_per_lsb = 1.0f / 2048.0f;  break; /* 16g */
#if 0
    case ICM45686_ACCEL_FS_SEL_32_G: icm45686_g_per_lsb = 1.0f / 1024.0f;  break; /* 32g */
#endif
    default: icm45686_g_per_lsb = 1.0f / 16384.0f; break;
  }

  /* Gyroscope: for 16-bit signed baseline, LSB_per_dps = 32768 / FS_dps
   * thus deg_per_lsb = FS_dps / 32768
   */
  icm45686_gyro_fs = ICM45686_GYRO_FS_SEL_250_DPS;
  switch (icm45686_gyro_fs) {
    case ICM45686_GYRO_FS_SEL_15_625_DPS: icm45686_deg_per_lsb = 15.625f  / 32768.0f; break;
    case ICM45686_GYRO_FS_SEL_31_25_DPS:  icm45686_deg_per_lsb = 31.25f   / 32768.0f; break;
    case ICM45686_GYRO_FS_SEL_62_5_DPS:   icm45686_deg_per_lsb = 62.5f    / 32768.0f; break;
    case ICM45686_GYRO_FS_SEL_125_DPS:    icm45686_deg_per_lsb = 125.0f   / 32768.0f; break;
    case ICM45686_GYRO_FS_SEL_250_DPS:    icm45686_deg_per_lsb = 250.0f   / 32768.0f; break;
    case ICM45686_GYRO_FS_SEL_500_DPS:    icm45686_deg_per_lsb = 500.0f   / 32768.0f; break;
    case ICM45686_GYRO_FS_SEL_1000_DPS:   icm45686_deg_per_lsb = 1000.0f  / 32768.0f; break;
    case ICM45686_GYRO_FS_SEL_2000_DPS:   icm45686_deg_per_lsb = 2000.0f  / 32768.0f; break;
#if 0
    case ICM45686_GYRO_FS_SEL_4000_DPS:   icm45686_deg_per_lsb = 4000.0f  / 32768.0f; break;
#endif
    default: icm45686_deg_per_lsb = 250.0f / 32768.0f; break;
  }

  debug_printf(" * ICM-45686 configured (soft-reset, pwr_mgmt, accel/gyro cfg)\r\n");
}

void ICM45686_ReadAccelerometerRaw(float *x, float *y, float *z)
{
    uint8_t accel_xyz[6];

    SW_I2C_UTIL_Read_Multi(icm45686_addr, ICM45686_ACCEL_XOUT_H, 6, (uint8_t*)&accel_xyz);

  {
    // default is little-endian, combine bytes
    int16_t rx = (int16_t)(accel_xyz[1] << 8 | accel_xyz[0]);
    int16_t ry = (int16_t)(accel_xyz[3] << 8 | accel_xyz[2]);
    int16_t rz = (int16_t)(accel_xyz[5] << 8 | accel_xyz[4]);
    *x = rx * icm45686_g_per_lsb * MS2_PER_G;
    *y = ry * icm45686_g_per_lsb * MS2_PER_G;
    *z = rz * icm45686_g_per_lsb * MS2_PER_G;
  }
}

void ICM45686_ReadGyroRaw(float *x, float *y, float *z)
{
    uint8_t gyro_xyz[6];
    SW_I2C_UTIL_Read_Multi(icm45686_addr, ICM45686_GYRO_XOUT_H, 6, (uint8_t*)&gyro_xyz);

  {
    // default is little-endian, combine bytes
    int16_t gx = (int16_t)(gyro_xyz[1] << 8 | gyro_xyz[0]);
    int16_t gy = (int16_t)(gyro_xyz[3] << 8 | gyro_xyz[2]);
    int16_t gz = (int16_t)(gyro_xyz[5] << 8 | gyro_xyz[4]);
    /* deg/sec per LSB * RAD_PER_DEG (deg->rad) */
    *x = gx * icm45686_deg_per_lsb * RAD_PER_DEG;
    *y = gy * icm45686_deg_per_lsb * RAD_PER_DEG;
    *z = gz * icm45686_deg_per_lsb * RAD_PER_DEG;
  }
}

#endif

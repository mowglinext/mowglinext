/**
 * @file    lis3mdl.c
 * @brief   LIS3MDL magnetometer driver (Pololu AltIMU-10v5 / MinIMU-9v5)
 *
 * Extracted from cloudn1ne/Mowgli altimu-10v5.c
 * Only the magnetometer functions — accel/gyro handled by lsm6.c
 */

#include "imu/lis3mdl.h"
#include "imu/imu.h"
#include "soft_i2c.h"
#include "main.h"

#ifndef DISABLE_LIS3MDL

uint8_t LIS3MDL_TestDevice(void)
{
    uint8_t val = SW_I2C_UTIL_Read(LIS3MDL_ADDRESS, LIS3MDL_WHO_AM_I);
    if (val == LIS3MDL_WHO_ID)
    {
        DB_TRACE("    > [LIS3MDL] Magnetometer FOUND at I2C addr=0x%0x\r\n", LIS3MDL_ADDRESS);
        return 1;
    }
    return 0;
}

void LIS3MDL_Init(void)
{
    /* Ultra-high-performance mode for X/Y, 10 Hz ODR */
    SW_I2C_UTIL_WRITE(LIS3MDL_ADDRESS, LIS3MDL_CTRL_REG1, 0x70);
    /* +/- 4 gauss full scale */
    SW_I2C_UTIL_WRITE(LIS3MDL_ADDRESS, LIS3MDL_CTRL_REG2, 0x00);
    /* Continuous-conversion mode */
    SW_I2C_UTIL_WRITE(LIS3MDL_ADDRESS, LIS3MDL_CTRL_REG3, 0x00);
    /* Ultra-high-performance mode for Z */
    SW_I2C_UTIL_WRITE(LIS3MDL_ADDRESS, LIS3MDL_CTRL_REG4, 0x0C);
}

int LIS3MDL_ReadMagRaw(float *x, float *y, float *z)
{
    uint8_t mag_xyz[6];

    if (!SW_I2C_UTIL_Read_Multi(LIS3MDL_ADDRESS, LIS3MDL_OUT_X_L, 6, mag_xyz)) {
        return 0;
    }

    /* Raw values in LSB, convert to microtesla:
     * 1 Gauss = 100 uT, LIS3MDL_GAUSS_FACTOR = Gauss/LSB
     * So uT = raw * GAUSS_FACTOR * 100 */
    *x = (float)(int16_t)(mag_xyz[1] << 8 | mag_xyz[0]) * LIS3MDL_GAUSS_FACTOR * 100.0f;
    *y = (float)(int16_t)(mag_xyz[3] << 8 | mag_xyz[2]) * LIS3MDL_GAUSS_FACTOR * 100.0f;
    *z = (float)(int16_t)(mag_xyz[5] << 8 | mag_xyz[4]) * LIS3MDL_GAUSS_FACTOR * 100.0f;
    return 1;
}

#endif /* DISABLE_LIS3MDL */

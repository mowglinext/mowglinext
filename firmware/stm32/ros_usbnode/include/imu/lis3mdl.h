#ifndef __LIS3MDL_H
#define __LIS3MDL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* LIS3MDL magnetometer (Pololu AltIMU-10v5 / MinIMU-9v5) */
#define LIS3MDL_ADDRESS         0b0011110   /* 7-bit I2C address (0x1E) */
#define LIS3MDL_WHO_ID          0x3D

#define LIS3MDL_WHO_AM_I        0x0F
#define LIS3MDL_CTRL_REG1       0x20
#define LIS3MDL_CTRL_REG2       0x21
#define LIS3MDL_CTRL_REG3       0x22
#define LIS3MDL_CTRL_REG4       0x23
#define LIS3MDL_OUT_X_L         0x28

/* Gauss/LSB at +/-4 gauss range */
#define LIS3MDL_GAUSS_FACTOR    (1.0f / 6842.0f)

uint8_t LIS3MDL_TestDevice(void);
void LIS3MDL_Init(void);
int LIS3MDL_ReadMagRaw(float *x, float *y, float *z);

#ifdef __cplusplus
}
#endif

#endif /* __LIS3MDL_H */

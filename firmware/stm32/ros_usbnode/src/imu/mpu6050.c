#include "imu/imu.h"
#include "imu/mpu6050.h"
#include "soft_i2c.h"
#include "main.h"
#include <math.h>

#define MPU6050_ADDRESS  0x68

#define MPU6050_SMPRT_DIV    0x19
#define MPU6050_CONFIG       0x1a
#define MPU6050_ACCEL_XOUT_H 0x3b
#define MPU6050_GYRO_XOUT_H  0x43
#define MPU6050_PWR_MGMT_1   0x6b
#define MPU6050_WHO_AM_I     0x75
#define MPU6500_WHO_AM_I     0x70
#define MPU9250_WHO_AM_I     0x71
#define MPU9255_WHO_AM_I     0x73

#define MPU6050_DPS_FACTOR (1/131.0)
#define MPU6050_G_FACTOR   (1/16384.0)

#ifndef DISABLE_MPU6050

/**
  * @brief  Test Device 
  * Perform any tests possible before actually enabling and using the device,
  * for example check the i2c address and whoami registers if existing  
  *
  * @retval          0 -> test failed 1-> test ok, good to init and use
  */
uint8_t MPU6050_TestDevice(void)
{
  uint8_t  val;
  /* Test who am I */
  val = SW_I2C_UTIL_Read(MPU6050_ADDRESS,MPU6050_WHO_AM_I);
  if (val == MPU6500_WHO_AM_I ||  val == MPU6050_ADDRESS || val == MPU9255_WHO_AM_I || val == MPU9250_WHO_AM_I) return 1;
  debug_printf("    > [MPU-6050] - Error probing for (Gyro / Accelerometer) at I2C addr=0x%0x %x\r\n", MPU6050_ADDRESS,val);
  return 0;
}

void MPU6050_Init(void)
{
  // Disable temperature sensor, use gyroscope clock
  SW_I2C_UTIL_WRITE(MPU6050_ADDRESS, MPU6050_PWR_MGMT_1, 0b00001001);
  // Low pass filter 10 Hz
  SW_I2C_UTIL_WRITE(MPU6050_ADDRESS, MPU6050_CONFIG, 0x5);
  // Sample rate divider 10 (=> 1 kHz/(9+1) = 100 Hz)
  SW_I2C_UTIL_WRITE(MPU6050_ADDRESS, MPU6050_SMPRT_DIV, 9);
  // We don't touch the default configuration: 250°/s, +/- 2g
  debug_printf(" * MPU 6050 initialized\r\n");
}

/**
  * @brief  Reads the 3 accelerometer channels and stores them in *x,*y,*z
  * units are m/s^2
  */
int MPU6050_ReadAccelerometerRaw(float *x, float *y, float *z)
{
    uint8_t accel_xyz[6];   // 2 bytes each

    if (!SW_I2C_UTIL_Read_Multi(MPU6050_ADDRESS, MPU6050_ACCEL_XOUT_H, 6, (uint8_t*)&accel_xyz)) {
        return 0;
    }

    *x =  (int16_t)(accel_xyz[0] << 8 | accel_xyz[1]) * MPU6050_G_FACTOR * MS2_PER_G;
    *y =  (int16_t)(accel_xyz[2] << 8 | accel_xyz[3]) * MPU6050_G_FACTOR * MS2_PER_G;
    *z =  (int16_t)(accel_xyz[4] << 8 | accel_xyz[5]) * MPU6050_G_FACTOR * MS2_PER_G;    
    return 1;
}

/**
  * @brief  Reads the 3 gyro channels and stores them in *x,*y,*z
  * units are rad/sec
  */
int MPU6050_ReadGyroRaw(float *x, float *y, float *z)
{
    uint8_t gyro_xyz[6];   // 2 bytes each

    if (!SW_I2C_UTIL_Read_Multi(MPU6050_ADDRESS, MPU6050_GYRO_XOUT_H, 6, (uint8_t*)&gyro_xyz)) {
        return 0;
    }
    
    *x = (int16_t)(gyro_xyz[0] << 8 | gyro_xyz[1]) * MPU6050_DPS_FACTOR * RAD_PER_DEG;
    *y = (int16_t)(gyro_xyz[2] << 8 | gyro_xyz[3]) * MPU6050_DPS_FACTOR * RAD_PER_DEG;
    *z = (int16_t)(gyro_xyz[4] << 8 | gyro_xyz[5]) * MPU6050_DPS_FACTOR * RAD_PER_DEG;    
    return 1;
}

#endif

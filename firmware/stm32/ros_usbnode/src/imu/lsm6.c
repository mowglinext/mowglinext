
/**
  ******************************************************************************
  * @file    lsm6.c
  * @author  Georg Swoboda <cn@warp.at>
  * @brief   Mowgli driver for the LSM6DS33/LSM633 IMU
  ******************************************************************************
  * @attention
  *
  * Some code taken from the various Arduino drivers referenced at 
  * https://www.pololu.com/product/2739
  * 
  * This code supports both of the following gyro/accelerometer combos:
  *     LSM6DS33
  *     LSM6DSO
  ******************************************************************************
  */

#include "imu/imu.h"
#include "imu/lsm6.h"
#include "soft_i2c.h"
#include "main.h"
#include <math.h>

#ifndef DISABLE_LSM6

uint8_t lsm6_address = LSM6_SA0_LOW_ADDRESS;


/**
  * @brief  Test Device 
  * Perform any tests possible before actually enabling and using the device,
  * for example check the i2c address and whoami registers if existing  
  *
  * @param  ctx      read / write interface definitions
  * @param  val      get the values of hpcf in reg CTRL_REG2
  * @retval          0 -> test failed 1-> test ok, good to init and use
  *
  */
uint8_t LSM6_TestDevice(void)
{
    uint8_t  val;

    /* test the SA0 high address*/
    val = SW_I2C_UTIL_Read(LSM6_SA0_LOW_ADDRESS, LSM6_WHO_AM_I);
    if (val == DS33_WHO_ID)
    {
        debug_printf("    > [LSM6] - LSM6DS33 (Gyro / Accelerometer) FOUND at I2C addr=0x%0x\r\n", LSM6_SA0_LOW_ADDRESS);
        lsm6_address = LSM6_SA0_LOW_ADDRESS;
        return 1;
    } else if (val == DSO_WHO_ID)
    {
        debug_printf("    > [LSM6] - LSM6DSO (Gyro / Accelerometer) FOUND at I2C addr=0x%0x\r\n", LSM6_SA0_LOW_ADDRESS);
        lsm6_address = LSM6_SA0_LOW_ADDRESS;
        return 1;
    }

    /* test the SA0 high address*/
    val = SW_I2C_UTIL_Read(LSM6_SA0_HIGH_ADDRESS, LSM6_WHO_AM_I);
    if (val == DS33_WHO_ID)
    {
        debug_printf("    > [LSM6] - LSM6DS33 (Gyro / Accelerometer) FOUND at I2C addr=0x%0x\r\n", LSM6_SA0_HIGH_ADDRESS);
        lsm6_address = LSM6_SA0_HIGH_ADDRESS;
        return 1;
    } else if (val == DSO_WHO_ID)
    {
        debug_printf("    > [LSM6] - LSM6DSO (Gyro / Accelerometer) FOUND at I2C addr=0x%0x\r\n", LSM6_SA0_HIGH_ADDRESS);
        lsm6_address = LSM6_SA0_HIGH_ADDRESS;
        return 1;
    }

    debug_printf("    > [LSM6] - Error probing for LSM6DS33/LSM6DSO (Gyro / Accelerometer) at I2C addr=0x%0x and addr=0x%0x \r\n", LSM6_SA0_LOW_ADDRESS, LSM6_SA0_HIGH_ADDRESS);
    return(0);
}

/**
  * @brief  Initialize IMU
  * LSM6DS33/LSM6DSO +/- 2g acceleration and 245 gps for gyro    
  */
void LSM6_Init(void)
{
    /*******************************/
    /* LSM6DS33 Gyro/Accelerometer */
    /*******************************/

    // ACCLEROMETER
    // 0x80 = 0b10000000
    // ODR = 1000 (1.66 kHz (high performance)); FS_XL = 00 (+/-2 g full scale)
    SW_I2C_UTIL_WRITE(lsm6_address, LSM6_CTRL1_XL, 0x80);
    // GYRO
    // 0x80 = 0b010000000
    // ODR = 1000 (1.66 kHz (high performance)); FS_XL = 00 (245 degree per s)
    SW_I2C_UTIL_WRITE(lsm6_address, LSM6_CTRL2_G, 0x80);
    // ACCELEROMETER + GYRO
    // 0x44 = 0b01000100
    // BDU = 1 (block data update - prevent torn reads between high/low bytes)
    // IF_INC = 1 (automatically increment register address)
    SW_I2C_UTIL_WRITE(lsm6_address, LSM6_CTRL3_C, 0x44);
}

/**
  * @brief  Reads the 3 accelerometer channels and stores them in *x,*y,*z
  * units are m/s^2
  */
int LSM6_ReadAccelerometerRaw(float *x, float *y, float *z)
{
    uint8_t accel_xyz[6];   // 2 bytes each

    uint8_t acked = SW_I2C_UTIL_Read_Multi(lsm6_address, LSM6_OUTX_L_XL, 6, (uint8_t*)&accel_xyz);
/*
    uint8_t i;
    debug_printf("IMU_ReadAccelerometer Raw Bytes: ");
    for (i=0;i<6;i++)
    {
      debug_printf("%02x ", accel_xyz[i]);
    }
    debug_printf("\r\n");
*/
    if (!acked) {
        return 0;
    }

    *x =  (int16_t)(accel_xyz[1] << 8 | accel_xyz[0]) * LSM6_G_FACTOR * MS2_PER_G;
    *y =  (int16_t)(accel_xyz[3] << 8 | accel_xyz[2]) * LSM6_G_FACTOR * MS2_PER_G;
    *z =  (int16_t)(accel_xyz[5] << 8 | accel_xyz[4]) * LSM6_G_FACTOR * MS2_PER_G;
    return 1;
}

/**
  * @brief  Reads the 3 gyro channels and stores them in *x,*y,*z
  * units are rad/sec
  */
int LSM6_ReadGyroRaw(float *x, float *y, float *z)
{
    uint8_t gyro_xyz[6];   // 2 bytes each

    uint8_t acked = SW_I2C_UTIL_Read_Multi(lsm6_address, LSM6_OUTX_L_G, 6, (uint8_t*)&gyro_xyz);
    
    if (!acked) {
      return 0;
    }

    *x = (int16_t)(gyro_xyz[1] << 8 | gyro_xyz[0]) * LSM6_DPS_FACTOR * RAD_PER_DEG;
    *y = (int16_t)(gyro_xyz[3] << 8 | gyro_xyz[2]) * LSM6_DPS_FACTOR * RAD_PER_DEG;
    *z = (int16_t)(gyro_xyz[5] << 8 | gyro_xyz[4]) * LSM6_DPS_FACTOR * RAD_PER_DEG;
    return 1;
}

#endif

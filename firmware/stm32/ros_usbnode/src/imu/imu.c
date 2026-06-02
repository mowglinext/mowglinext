
/**
  ******************************************************************************
  * @file    imu.c
  * @author  Georg Swoboda <cn@warp.at>
  * @brief   Mowgli IMU calibration routines as long as they are IMU independent 
  ******************************************************************************
  * @attention
  *
  * details: https://learn.adafruit.com/adafruit-sensorlab-magnetometer-calibration?view=all
  *          https://makersportal.com/blog/calibration-of-a-magnetometer-with-raspberry-pi
  ******************************************************************************
  */

#include <math.h>

#include "imu/imu.h"
#include "imu/lsm6.h"
#include "imu/mpu6050.h"
#include "imu/wt901.h"
#include "imu/icm45686.h"
#include "imu/lis3mdl.h"
#include "i2c.h"
#include "main.h"

IMU_ReadAccelerometerRaw imuReadAccelerometerRaw=NULL;
IMU_ReadGyroRaw imuReadGyroRaw=NULL;
IMU_ReadGyroRaw imuReadMagRaw=NULL;  /* reuse same typedef: void(*)(float*,float*,float*) */


static int assertAccelerometer() {
  return debug_assert(imuReadAccelerometerRaw!=NULL,"Usage of non installed accelerometer\r\n");
}

static int assertGyro() {
  return debug_assert(imuReadGyroRaw!=NULL,"Usage of non installed gryometer\r\n");
}

int IMU_HasAccelerometer() {
  return imuReadAccelerometerRaw!=NULL;
}

int IMU_HasGyro() {
  return imuReadGyroRaw!=NULL;
}

/**
  * @brief  Reads the 3 accelerometer axis and stores them in *x,*y,*z  
  * 
  * units are m/s^2 uncalibrated
  */ 
void IMU_ReadAccelerometer(float *x, float *y, float *z)
{
  if (assertAccelerometer()) return;
    // Raw values — calibration is handled on the ROS2 side
    imuReadAccelerometerRaw(x, y, z);
}

/**
  * @brief  Reads the 3 accelerometer gyro and stores them in *x,*y,*z  
  * 
  * units are rad/sec uncalibrated
  */ 
void IMU_ReadGyro(float *x, float *y, float *z)
{
  if (assertGyro()) return;
  // Raw values — calibration is handled on the ROS2 side
  imuReadGyroRaw(x, y, z);
}

/**
  * @brief  Reads the 3 magnetometer channels (if available)
  * units are uT (microtesla), uncalibrated
  */
void IMU_ReadMag(float *x, float *y, float *z)
{
  if (imuReadMagRaw != NULL) {
    imuReadMagRaw(x, y, z);
  } else {
    *x = 0.0f;
    *y = 0.0f;
    *z = 0.0f;
  }
}

int IMU_HasMag(void)
{
  return imuReadMagRaw != NULL;
}

/*
 * Read onboard IMU acceleration in ms^2
 */
void IMU_Onboard_ReadAccelerometer(float *x, float *y, float *z)
{
   I2C_ReadAccelerometer(x, y, z);
}

/*
 * Read onboard IMU temperature in °C
 */
float IMU_Onboard_ReadTemp(void)
{
  return(I2C_ReadAccelerometerTemp());
}

void IMU_Normalize( VECTOR* p )
{
    double w = sqrt( p->x * p->x + p->y * p->y + p->z * p->z );
    p->x /= w;
    p->y /= w;
    p->z /= w;
}

void IMU_Init() {
  imuReadAccelerometerRaw=NULL;
  imuReadGyroRaw=NULL;
  imuReadMagRaw=NULL;

  uint32_t l_u32Timestamp = HAL_GetTick();
while (imuReadAccelerometerRaw == NULL && ((HAL_GetTick() - l_u32Timestamp) < 20000) )
{
  #ifndef DISABLE_LSM6
    if (LSM6_TestDevice()) {
      LSM6_Init();
      imuReadAccelerometerRaw=LSM6_ReadAccelerometerRaw;
      imuReadGyroRaw=LSM6_ReadGyroRaw;
    }
  #endif

  #ifndef DISABLE_WT901
    if ((!imuReadGyroRaw || !imuReadAccelerometerRaw) && WT901_TestDevice()) {
      WT901_Init();
      imuReadAccelerometerRaw=WT901_ReadAccelerometerRaw;
      imuReadGyroRaw=WT901_ReadGyroRaw;
      imuReadMagRaw=WT901_ReadMagRaw;
    }
  #endif

  #ifndef DISABLE_MPU6050
    if ((!imuReadGyroRaw || !imuReadAccelerometerRaw) && MPU6050_TestDevice()) {
      MPU6050_Init();
      imuReadAccelerometerRaw=MPU6050_ReadAccelerometerRaw;
      imuReadGyroRaw=MPU6050_ReadGyroRaw;
    }
  #endif

  #ifndef DISABLE_ICM45686
    if ((!imuReadGyroRaw || !imuReadAccelerometerRaw) && ICM45686_TestDevice()) {
      ICM45686_Init();
      imuReadAccelerometerRaw=ICM45686_ReadAccelerometerRaw;
      imuReadGyroRaw=ICM45686_ReadGyroRaw;
    }
  #endif

  HAL_Delay(20);
}

if(imuReadAccelerometerRaw == NULL){
  chirp(10);
}

  /* Magnetometer: try LIS3MDL (separate chip on Pololu boards) if no
   * mag was set by the accel/gyro IMU driver (e.g. WT901 has built-in mag,
   * but LSM6/MPU6050/ICM45686 do not). */
#ifndef DISABLE_LIS3MDL
  if (imuReadMagRaw == NULL && LIS3MDL_TestDevice()) {
    LIS3MDL_Init();
    imuReadMagRaw = LIS3MDL_ReadMagRaw;
  }
#endif

}
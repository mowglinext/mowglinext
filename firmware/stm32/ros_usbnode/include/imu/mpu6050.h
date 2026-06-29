
#ifndef __MPU6050_H
#define __MPU6050_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Test for MPU-6050
 * 
 * @return zero, if device is not found.
 */
uint8_t MPU6050_TestDevice(void);

/**
  * @brief  Initialize MPU-6050
  */
void MPU6050_Init(void);

/**
  * @brief  Reads the 3 accelerometer channels and stores them in *x,*y,*z
  * units are m/s^2
  */
int MPU6050_ReadAccelerometerRaw(float *x, float *y, float *z);

/**
  * @brief  Reads the 3 gyro channels and stores them in *x,*y,*z
  * units are rad/sec
  */
int MPU6050_ReadGyroRaw(float *x, float *y, float *z);

#ifdef __cplusplus
}
#endif

#endif /* __MPU6050_H */

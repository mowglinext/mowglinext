
#ifndef __IMU_H
#define __IMU_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f_board_hal.h"

typedef struct 
{
    double x, y, z;
} VECTOR;

/*
 * Conversions to ROS units.
 *
 * RAD_PER_DEG was previously named RAD_PER_G; the "G" was short for
 * "degrees" (°) but collided visually with the "g" used for gravity.
 * Renamed to avoid confusion — the numeric value (π/180) is unchanged.
 *
 * T_PER_GAUSS is parenthesised with a float literal; the previous
 * "1/10000" was integer division = 0, which silently zeroed any magnetometer
 * conversion that precomputed the factor.
 */
#define RAD_PER_DEG             0.017453292519943295f  /* π/180: deg → rad */
#define MS2_PER_G               9.80665f               /* g → m/s²        */
#define T_PER_GAUSS             (1.0f / 10000.0f)      /* Gauss → Tesla   */

/*
 * IMU functions that a compatible IMU needs to be able to provide
 */

int IMU_TryReadAccelerometer(float *x, float *y, float *z);
void IMU_ReadAccelerometer(float *x, float *y, float *z);
void IMU_Onboard_ReadAccelerometer(float *x, float *y, float *z);
float IMU_Onboard_ReadTemp(void);
int IMU_TryReadGyro(float *x, float *y, float *z);
void IMU_ReadGyro(float *x, float *y, float *z);
typedef float (*IMU_ReadBarometerTemperatureC)(void);
typedef float (*IMU_ReadBarometerAltitudeMeters)(void);
void IMU_Normalize( VECTOR* p );

/* Any external IMU needs to implement the following functions and adhere to the ROS REP 103 standard (https://www.ros.org/reps/rep-0103.html) */
typedef int (*IMU_ReadGyroRaw)(float *x, float *y, float *z);
typedef int (*IMU_ReadAccelerometerRaw)(float *x, float *y, float *z);
typedef int (*IMU_ReadMagRaw)(float *x, float *y, float *z);
/* end of functions to implement for IMU */

void IMU_Init();
int IMU_HasAccelerometer();
int IMU_HasGyro();
int IMU_HasMag();
int IMU_TryReadMag(float *x, float *y, float *z);
void IMU_ReadMag(float *x, float *y, float *z);


#ifdef __cplusplus
}
#endif

#endif /* __IMU_H */

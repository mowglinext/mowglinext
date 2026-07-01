/****************************************************************************
* Title                 :   WT901 driver
* Filename              :   WT901.h
* Author                :   Nekraus
* Origin Date           :   15/09/2022
* Version               :   1.0.0

*****************************************************************************/
/** \file WT901.h
*  \brief 
*
*/
#ifndef __WT901_H
#define __WT901_H

#ifndef DISABLE_WT901

/******************************************************************************
* Includes
*******************************************************************************/
#include "stm32f_board_hal.h"

/**
 * @brief Test for WT901
 * 
 * @return zero, if device is not found.
 */
uint8_t WT901_TestDevice(void);

/**
  * @brief  Initialize WT901
  */
void WT901_Init(void);

/**
  * @brief  Reads the 3 accelerometer channels and stores them in *x,*y,*z
  * units are m/s^2
  */
int WT901_ReadAccelerometerRaw(float *x, float *y, float *z);

/**
  * @brief  Reads the 3 gyro channels and stores them in *x,*y,*z
  * units are rad/sec
  */
int WT901_ReadGyroRaw(float *x, float *y, float *z);

/**
  * @brief  Reads the 3 magnetometer channels and stores them in *x,*y,*z
  * units are uT (microtesla)
  */
int WT901_ReadMagRaw(float *x, float *y, float *z);

#endif
#endif /*WT901_H*/ 

/*** End of File **************************************************************/

/****************************************************************************
* Title                 :   
* Filename              :   wt901.c
* Author                :   Nekraus
* Origin Date           :   15/09/2022
* Version               :   1.0.0

*****************************************************************************/
/** \file wt901.c
*  \brief 
*
*/

#include "board.h"

/******************************************************************************
* Includes
*******************************************************************************/
#include "main.h"
#include "board.h"
#include "soft_i2c.h"

#include "imu/imu.h"
#include "imu/wt901.h"
#include <math.h>

#ifndef DISABLE_WT901

/******************************************************************************
* Module Preprocessor Constants
*******************************************************************************/
#define WT901_ADDRESS 0x50

// WT901 full-scale: ±16 g accel, ±2000 °/s gyro, int16 raw output.
// Parenthesise and use float literals so the expression is self-contained.
// The previous "16/32768" was integer division = 0; it only produced correct
// readings in the existing call sites because C's left-to-right evaluation
// order promoted to float before the /32768 hit. A refactor like
// (WT901_G_FACTOR * MS2_PER_G) would collapse to 0. Fixed.
#define WT901_G_FACTOR   (16.0f / 32768.0f)
#define WT901_DPS_FACTOR (2000.0f / 32768.0f)
#define WT901_T_FACTOR 0.00000015f

#define DIO_MODE_AIN 0
#define DIO_MODE_DIN 1
#define DIO_MODE_DOH 2
#define DIO_MODE_DOL 3
#define DIO_MODE_DOPWM 4
#define DIO_MODE_GPS 5	

/******************************************************************************
* Module Preprocessor Macros
*******************************************************************************/

/******************************************************************************
* Module Typedefs
*******************************************************************************/
typedef enum {
 SAVE       =   0x00,
 CALSW      = 	0x01,
 RSW 	    =	0x02,
 RRATE	    =	0x03,
 BAUD 	    =	0x04,
 AXOFFSET   =	0x05,
 AYOFFSET   =	0x06,
 AZOFFSET   =	0x07,
 GXOFFSET   =	0x08,
 GYOFFSET   =	0x09,
 GZOFFSET   =	0x0a,
 HXOFFSET   =	0x0b,
 HYOFFSET   =	0x0c,
 HZOFFSET   =   0x0d,
 D0MODE     =	0x0e,
 D1MODE     =	0x0f,
 D2MODE     =	0x10,
 D3MODE	    =	0x11,
 D0PWMH	    =	0x12,
 D1PWMH	    =	0x13,
 D2PWMH		=   0x14,
 D3PWMH	    =	0x15,
 D0PWMT	    =	0x16,
 D1PWMT	    =	0x17,
 D2PWMT	    =	0x18,
 D3PWMT	    =	0x19,
 IICADDR	=	0x1a,
 LEDOFF 	=	0x1b,
 GPSBAUD	=	0x1c,
 YYMM		=	0x30,
 DDHH		=	0x31,
 MMSS		=	0x32,
 MS			=	0x33,
 AX			=	0x34,
 AY			=	0x35,
 AZ			=	0x36,
 GX			=	0x37,
 GY			=	0x38,
 GZ			=	0x39,
 HX			=	0x3a,
 HY			=	0x3b,
 HZ			=	0x3c,			
 Roll		=	0x3d,
 Pitch		=	0x3e,
 Yaw		=	0x3f,
 TEMP		=	0x40,
 D0Status	=	0x41,
 D1Status	=	0x42,
 D2Status	=	0x43,
 D3Status	=	0x44,
 PressureL	=	0x45,
 PressureH	=	0x46,
 HeightL	=	0x47,
 HeightH	=	0x48,
 LonL		=	0x49,
 LonH		=	0x4a,
 LatL		=	0x4b,
 LatH		=	0x4c,
 GPSHeight  =   0x4d,
 GPSYAW     =   0x4e,
 GPSVL		=	0x4f,
 GPSVH		=	0x50,
}WT901_RegAdress_e;

/******************************************************************************
* Module Variable Definitions
*******************************************************************************/

/******************************************************************************
* Function Prototypes
*******************************************************************************/

/******************************************************************************
*  Public Functions
*******************************************************************************/

/**
  * @brief  Test Device 
  * Perform any tests possible before actually enabling and using the device,
  * for example check the i2c address and whoami registers if existing  
  *
  * @retval          0 -> test failed 1-> test ok, good to init and use
  *
  */
uint8_t WT901_TestDevice(void)
{
    uint8_t  val;
    uint8_t  l_u8return = 0;

    /* test the LSM6DS33 (gyro and accelerometer) */
    val = SW_I2C_UTIL_Read(WT901_ADDRESS,(uint8_t)IICADDR);
    if (val == (WT901_ADDRESS)) /* should be 0x50 */
    {
        DB_TRACE("    > [WT901] FOUND at I2C addr=0x%0x\r\n", WT901_ADDRESS);
        l_u8return = 1;
    }
    else
    {
        DB_TRACE("    > [WT901] - Error probing for (Gyro / Accelerometer) at I2C addr=0x%0x\r\n", WT901_ADDRESS);
        l_u8return = 0;
    }
 
    // all tests passed
    return(l_u8return); 
}

/**
  * @brief  Initialize IMU
  *  
  */
void WT901_Init(void)
{
    /* default values are OK
    *  maybe set it but need to restart the module 
    */      
}

/**
  * @brief  Reads the 3 accelerometer channels and stores them in *x,*y,*z
  * units are m/s^2
  */
int WT901_ReadAccelerometerRaw(float *x, float *y, float *z)
{
    uint8_t accel_xyz[6];   // 2 bytes each

    if (!SW_I2C_UTIL_Read_Multi(WT901_ADDRESS, AX, 6, (uint8_t*)accel_xyz)) {
        return 0;
    }

    *x =  (float)(int16_t)(accel_xyz[1] << 8 | accel_xyz[0]) * WT901_G_FACTOR * MS2_PER_G;
    *y =  (float)(int16_t)(accel_xyz[3] << 8 | accel_xyz[2]) * WT901_G_FACTOR * MS2_PER_G;
    *z =  (float)(int16_t)(accel_xyz[5] << 8 | accel_xyz[4]) * WT901_G_FACTOR * MS2_PER_G;
    return 1;
}

/**
  * @brief  Reads the 3 gyro channels and stores them in *x,*y,*z
  * units are rad/sec
  */
int WT901_ReadGyroRaw(float *x, float *y, float *z)
{
    uint8_t gyro_xyz[6];   // 2 bytes each

    if (!SW_I2C_UTIL_Read_Multi(WT901_ADDRESS, GX, 6, (uint8_t*)&gyro_xyz)) {
        return 0;
    }
    
    *x = (float)(int16_t)(gyro_xyz[1] << 8 | gyro_xyz[0]) * WT901_DPS_FACTOR * RAD_PER_DEG;
    *y = (float)(int16_t)(gyro_xyz[3] << 8 | gyro_xyz[2]) * WT901_DPS_FACTOR * RAD_PER_DEG;
    *z = (float)(int16_t)(gyro_xyz[5] << 8 | gyro_xyz[4]) * WT901_DPS_FACTOR * RAD_PER_DEG;
    return 1;
}

/**
  * @brief  Reads the raw temp value
  * (internal function only)
  * @retval float temp in °C
  */
float WT901_TempRaw(void)
{
    uint8_t temp[2];   
    float retval; // temp
    
    // assert MSB to enable register address auto-increment
    SW_I2C_UTIL_Read_Multi(WT901_ADDRESS, TEMP, 2, (uint8_t*)&temp);

    retval = (float)(temp[1] << 8 | temp[0])/100;
    return(retval);
}

/**
  * @brief  Reads the 3 magnetometer channels and stores them in *x,*y,*z
  * units are uT (microtesla)
  */
int WT901_ReadMagRaw(float *x, float *y, float *z)
{
    uint8_t mag_xyz[6];   // 2 bytes each

    if (!SW_I2C_UTIL_Read_Multi(WT901_ADDRESS, HX, 6, (uint8_t*)mag_xyz)) {
        return 0;
    }

    *x = (float)(int16_t)(mag_xyz[1] << 8 | mag_xyz[0]);
    *y = (float)(int16_t)(mag_xyz[3] << 8 | mag_xyz[2]);
    *z = (float)(int16_t)(mag_xyz[5] << 8 | mag_xyz[4]);
    return 1;
}

#endif

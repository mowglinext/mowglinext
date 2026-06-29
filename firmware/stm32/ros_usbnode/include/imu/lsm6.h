
#ifndef __LSM6_H
#define __LSM6_H

/* Calibration, Conversion factors */

#define LSM6_G_FACTOR           0.000061f           // LSM6DS33 datasheet (page 15)  0.061 mg/LSB
#define LSM6_DPS_FACTOR         0.00875f            // LSM6DS33 datasheet (page 15)  0.00875 °/sec/LSB 

/* Gyro / Accelerometer */
#define LSM6_SA0_LOW_ADDRESS 0b1101010
#define LSM6_SA0_HIGH_ADDRESS 0b1101011
#define DSO_WHO_ID    0x6C         // WHO_AM_I will report this value
#define DS33_WHO_ID    0x69         // WHO_AM_I will report this value


#define LSM6_FUNC_CFG_ACCESS    0x01
#define LSM6_FIFO_CTRL1         0x06
#define LSM6_FIFO_CTRL2         0x07
#define LSM6_FIFO_CTRL3         0x08
#define LSM6_FIFO_CTRL4         0x09
#define LSM6_FIFO_CTRL5         0x0A
#define LSM6_ORIENT_CFG_G       0x0B
#define LSM6_INT1_CTRL          0x0D
#define LSM6_INT2_CTRL          0x0E
#define LSM6_WHO_AM_I           0x0F
#define LSM6_CTRL1_XL           0x10
#define LSM6_CTRL2_G            0x11
#define LSM6_CTRL3_C            0x12
#define LSM6_CTRL4_C            0x13
#define LSM6_CTRL5_C            0x14
#define LSM6_CTRL6_C            0x15
#define LSM6_CTRL7_G            0x16
#define LSM6_CTRL8_XL           0x17
#define LSM6_CTRL9_XL           0x18
#define LSM6_CTRL10_C           0x19
#define LSM6_WAKE_UP_SRC        0x1B
#define LSM6_TAP_SRC            0x1C
#define LSM6_D6D_SRC            0x1D
#define LSM6_STATUS_REG         0x1E
#define LSM6_OUT_TEMP_L         0x20
#define LSM6_OUT_TEMP_H         0x21
#define LSM6_OUTX_L_G           0x22
#define LSM6_OUTX_H_G           0x23
#define LSM6_OUTY_L_G           0x24
#define LSM6_OUTY_H_G           0x25
#define LSM6_OUTZ_L_G           0x26
#define LSM6_OUTZ_H_G           0x27
#define LSM6_OUTX_L_XL          0x28
#define LSM6_OUTX_H_XL          0x29
#define LSM6_OUTY_L_XL          0x2A
#define LSM6_OUTY_H_XL          0x2B
#define LSM6_OUTZ_L_XL          0x2C
#define LSM6_OUTZ_H_XL          0x2D
#define LSM6_FIFO_STATUS1       0x3A
#define LSM6_FIFO_STATUS2       0x3B
#define LSM6_FIFO_STATUS3       0x3C
#define LSM6_FIFO_STATUS4       0x3D
#define LSM6_FIFO_DATA_OUT_L    0x3E
#define LSM6_FIFO_DATA_OUT_H    0x3F
#define LSM6_TIMESTAMP0_REG     0x40
#define LSM6_TIMESTAMP1_REG     0x41
#define LSM6_TIMESTAMP2_REG     0x42
#define LSM6_STEP_TIMESTAMP_L   0x49
#define LSM6_STEP_TIMESTAMP_H   0x4A
#define LSM6_STEP_COUNTER_L     0x4B
#define LSM6_STEP_COUNTER_H     0x4C
#define LSM6_FUNC_SRC           0x53
#define LSM6_TAP_CFG            0x58
#define LSM6_TAP_THS_6D         0x59
#define LSM6_INT_DUR2           0x5A
#define LSM6_WAKE_UP_THS        0x5B
#define LSM6_WAKE_UP_DUR        0x5C
#define LSM6_FREE_FALL          0x5D
#define LSM6_MD1_CFG            0x5E
#define LSM6_MD2_CFG            0x5F


uint8_t LSM6_TestDevice(void);
/**
  * @brief  Initialize IMU
  * LSM6 +/- 2g acceleration 
  */
void LSM6_Init(void);

/**
  * @brief  Reads the 3 accelerometer channels and stores them in *x,*y,*z
  * units are m/s^2
  */
int LSM6_ReadAccelerometerRaw(float *x, float *y, float *z);

/**
  * @brief  Reads the 3 gyro channels and stores them in *x,*y,*z
  * units are rad/sec
  */
int LSM6_ReadGyroRaw(float *x, float *y, float *z);

#endif /* __LSM6_H */

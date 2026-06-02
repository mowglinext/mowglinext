#ifndef __ICM45686_H
#define __ICM45686_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Minimal ICM-45686 driver API
 *
 * NOTE: This driver is provided as a drop-in template modeled after
 * the existing MPU6050 driver
 */

/* I2C address. The ICM-45686 address LSB is set by the AD0/MISO pin:
 * AD0 low -> 0x68 (primary), AD0 high -> 0x69 (alternate). Some breakouts
 * strap AD0 high, so ICM45686_TestDevice() probes both addresses. */
#ifndef ICM45686_ADDRESS
#define ICM45686_ADDRESS 0x68
#endif

#ifndef ICM45686_ADDRESS_ALT
#define ICM45686_ADDRESS_ALT 0x69
#endif

/* Register addresses (from TDK/InvenSense ICM456xx driver)
 * DREG_BANK1 data registers
 */
#ifndef ICM45686_ACCEL_XOUT_H
#define ICM45686_ACCEL_XOUT_H 0x00 /* ACCEL_DATA_X1_UI */
#endif

#ifndef ICM45686_GYRO_XOUT_H
#define ICM45686_GYRO_XOUT_H  0x06 /* GYRO_DATA_X1_UI */
#endif

/* WHO_AM_I register and expected ID */
#ifndef ICM45686_WHO_AM_I
#define ICM45686_WHO_AM_I 0x72 /* WHO_AM_I register address */
#endif

#ifndef ICM45686_WHO_AM_I_ID
#define ICM45686_WHO_AM_I_ID 0xE9 /* expected device ID */
#endif

/* Configuration bit mappings (conservative, inferred layout)
 * Note: These enums and macros provide a readable mapping for register
 * fields used by the simplified driver.
 */

/* ACCEL_CONFIG0 (ACCEL_CONFIG0 register) */
#define ICM45686_ACCEL_UI_FS_SEL_SHIFT 4
#define ICM45686_ACCEL_UI_FS_SEL_MASK  (0x7 << ICM45686_ACCEL_UI_FS_SEL_SHIFT)
#define ICM45686_ACCEL_ODR_MASK        0x0F

/* Accelerometer full-scale selection and ODR use vendor mapping (hex codes).
 * Values taken from vendor `inv_imu_defs.h` (ACCEL_CONFIG0 field encodings).
 */
typedef enum {
	ICM45686_ACCEL_FS_SEL_2_G  = 0x4,
	ICM45686_ACCEL_FS_SEL_4_G  = 0x3,
	ICM45686_ACCEL_FS_SEL_8_G  = 0x2,
	ICM45686_ACCEL_FS_SEL_16_G = 0x1,
#if 0 /* INV_IMU_HIGH_FSR_SUPPORTED */
	ICM45686_ACCEL_FS_SEL_32_G = 0x0,
#endif
} icm45686_accel_fs_sel_t;

typedef enum {
	ICM45686_ACCEL_ODR_1_5625_HZ = 0x0F,
	ICM45686_ACCEL_ODR_3_125_HZ  = 0x0E,
	ICM45686_ACCEL_ODR_6_25_HZ   = 0x0D,
	ICM45686_ACCEL_ODR_12_5_HZ   = 0x0C,
	ICM45686_ACCEL_ODR_25_HZ     = 0x0B,
	ICM45686_ACCEL_ODR_50_HZ     = 0x0A,
	ICM45686_ACCEL_ODR_100HZ     = 0x09,
	ICM45686_ACCEL_ODR_200HZ     = 0x08,
	ICM45686_ACCEL_ODR_400HZ     = 0x07,
	ICM45686_ACCEL_ODR_800HZ     = 0x06,
	ICM45686_ACCEL_ODR_1600HZ    = 0x05,
	ICM45686_ACCEL_ODR_3200HZ    = 0x04,
	ICM45686_ACCEL_ODR_6400HZ    = 0x03,
	/* remaining codes reserved: 0x02..0x00 */
	ICM45686_ACCEL_ODR_RESERVED_0x02 = 0x02,
	ICM45686_ACCEL_ODR_RESERVED_0x01 = 0x01,
	ICM45686_ACCEL_ODR_RESERVED_0x00 = 0x00,
} icm45686_accel_odr_t;

/* GYRO_CONFIG0 (GYRO_CONFIG0 register) */
/* gyro FS_SEL uses same bit positions as accel (FS_SEL in upper nibble) */
#define ICM45686_GYRO_UI_FS_SEL_SHIFT 4
#define ICM45686_GYRO_UI_FS_SEL_MASK  (0x7 << ICM45686_GYRO_UI_FS_SEL_SHIFT)
#define ICM45686_GYRO_ODR_MASK        0x0F

/* Gyro full-scale selection field (FS_SEL) - use vendor mapping from
 * inv_imu_defs.h. Values are the register codes (hex) that map to DPS.
 * Note: lower numeric code corresponds to larger FSR in vendor mapping.
 */
typedef enum {
	/* 0x08..0x00 mapping per vendor: 0x08 = 15.625 dps, 0x04 = 250 dps, ... */
	ICM45686_GYRO_FS_SEL_15_625_DPS = 0x08,
	ICM45686_GYRO_FS_SEL_31_25_DPS  = 0x07,
	ICM45686_GYRO_FS_SEL_62_5_DPS   = 0x06,
	ICM45686_GYRO_FS_SEL_125_DPS    = 0x05,
	ICM45686_GYRO_FS_SEL_250_DPS    = 0x04,
	ICM45686_GYRO_FS_SEL_500_DPS    = 0x03,
	ICM45686_GYRO_FS_SEL_1000_DPS   = 0x02,
	ICM45686_GYRO_FS_SEL_2000_DPS   = 0x01,
#if 0 /* INV_IMU_HIGH_FSR_SUPPORTED */
	ICM45686_GYRO_FS_SEL_4000_DPS   = 0x00,
#endif
} icm45686_gyro_fs_sel_t;

/* Gyro ODR codes (0..15) - expanded to 16 possible values as per datasheet
 * families. The following names map common ODR settings to codes; please
 * verify the exact rate associated with each code in the official datasheet
 * and adjust as needed.
 */
/* Gyro ODR codes (4-bit) using vendor mapping (codes map to Hz rates). */
typedef enum {
	ICM45686_GYRO_ODR_1_5625_HZ = 0x0F,
	ICM45686_GYRO_ODR_3_125_HZ  = 0x0E,
	ICM45686_GYRO_ODR_6_25_HZ   = 0x0D,
	ICM45686_GYRO_ODR_12_5_HZ   = 0x0C,
	ICM45686_GYRO_ODR_25_HZ     = 0x0B,
	ICM45686_GYRO_ODR_50_HZ     = 0x0A,
	ICM45686_GYRO_ODR_100HZ     = 0x09,
	ICM45686_GYRO_ODR_200HZ     = 0x08,
	ICM45686_GYRO_ODR_400HZ     = 0x07,
	ICM45686_GYRO_ODR_800HZ     = 0x06,
	ICM45686_GYRO_ODR_1600HZ    = 0x05,
	ICM45686_GYRO_ODR_3200HZ    = 0x04,
	ICM45686_GYRO_ODR_6400HZ    = 0x03,
	/* remaining codes reserved: 0x02..0x00 */
	ICM45686_GYRO_ODR_RESERVED_0x02 = 0x02,
	ICM45686_GYRO_ODR_RESERVED_0x01 = 0x01,
	ICM45686_GYRO_ODR_RESERVED_0x00 = 0x00,
} icm45686_gyro_odr_t;

/* Helper macros to compose config register values */
#define ICM45686_ACCEL_CONFIG0_VALUE(fsr, odr) \
	((((uint8_t)(fsr)) << ICM45686_ACCEL_UI_FS_SEL_SHIFT) | ((uint8_t)(odr) & ICM45686_ACCEL_ODR_MASK))

#define ICM45686_GYRO_CONFIG0_VALUE(fsr, odr) \
	((((uint8_t)(fsr)) << ICM45686_GYRO_UI_FS_SEL_SHIFT) | ((uint8_t)(odr) & ICM45686_GYRO_ODR_MASK))

/* Power management bits in PWR_MGMT0 (inferred): accel_mode bits [1:0], gyro_mode bits [3:2] */
#define ICM45686_PWR_MGMT0_ACCEL_MODE_SHIFT 0
#define ICM45686_PWR_MGMT0_GYRO_MODE_SHIFT  2
#define ICM45686_PWR_MGMT0_ACCEL_MODE_MASK  (0x3 << ICM45686_PWR_MGMT0_ACCEL_MODE_SHIFT)
#define ICM45686_PWR_MGMT0_GYRO_MODE_MASK   (0x3 << ICM45686_PWR_MGMT0_GYRO_MODE_SHIFT)

typedef enum {
	ICM45686_PWR_MODE_SLEEP  = 0,
	ICM45686_PWR_MODE_NORMAL = 1,
	ICM45686_PWR_MODE_LOW    = 2,
} icm45686_power_mode_t;

/*
 * Datasheet helper: compose PWR_MGMT0 value
 * Note: the vendor datasheet / reference driver indicates that writing 0x0F
 * to PWR_MGMT0 enables both accelerometer and gyro in low-noise mode for
 * typical device setups. Provide a named constant for that common choice and
 * a generic composer macro for other combinations.
 */
#define ICM45686_PWR_MGMT0_ACCEL_GYRO_LOW_NOISE ((uint8_t)0x0F)

#define ICM45686_PWR_MGMT0_COMPOSE(accel_mode, gyro_mode) \
	(((uint8_t)(accel_mode) << ICM45686_PWR_MGMT0_ACCEL_MODE_SHIFT) | \
	 ((uint8_t)(gyro_mode)  << ICM45686_PWR_MGMT0_GYRO_MODE_SHIFT))


/**
 * Test for ICM-45686
 *
 * @return zero if device is not found, non-zero if found
 */
uint8_t ICM45686_TestDevice(void);

/**
 * Initialize ICM-45686 (basic configuration)
 *
 * Implementation uses conservative defaults; adjust as required.
 */
void ICM45686_Init(void);

/**
 * Read raw accelerometer (m/s^2)
 */
void ICM45686_ReadAccelerometerRaw(float *x, float *y, float *z);

/**
 * Read raw gyroscope (rad/s)
 */
void ICM45686_ReadGyroRaw(float *x, float *y, float *z);

#ifdef __cplusplus
}
#endif

#endif /* __ICM45686_H */

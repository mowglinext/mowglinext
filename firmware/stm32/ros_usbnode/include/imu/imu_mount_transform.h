#ifndef IMU_MOUNT_TRANSFORM_H
#define IMU_MOUNT_TRANSFORM_H

#include <stdint.h>

#define IMU_MOUNT_ROTATION_IDENTITY 0
#define IMU_MOUNT_ROTATION_YAW_180 1
/* Module mounted chip-side DOWN. ROLL_180: its X arrow still points forward
 * (flip about X: y and z negate). PITCH_180: its X arrow points backward
 * (flip about Y: x and z negate). Both are proper rotations (det +1). */
#define IMU_MOUNT_ROTATION_ROLL_180 2
#define IMU_MOUNT_ROTATION_PITCH_180 3

#ifndef IMU_MOUNT_ROTATION
#define IMU_MOUNT_ROTATION IMU_MOUNT_ROTATION_IDENTITY
#endif

#if IMU_MOUNT_ROTATION != IMU_MOUNT_ROTATION_IDENTITY && \
    IMU_MOUNT_ROTATION != IMU_MOUNT_ROTATION_YAW_180 && \
    IMU_MOUNT_ROTATION != IMU_MOUNT_ROTATION_ROLL_180 && \
    IMU_MOUNT_ROTATION != IMU_MOUNT_ROTATION_PITCH_180
#error "Unsupported IMU_MOUNT_ROTATION"
#endif

/* Convert a vector from the sensor's physical mounting frame into the robot
 * base frame. Use the same proper rotation for acceleration, angular velocity,
 * and magnetic field so firmware control and USB telemetry agree. */
static inline void IMU_ApplyMountRotation(uint8_t rotation, float *x, float *y,
                                          float *z)
{
  if (rotation == IMU_MOUNT_ROTATION_YAW_180) {
    *x = -*x;
    *y = -*y;
  } else if (rotation == IMU_MOUNT_ROTATION_ROLL_180) {
    *y = -*y;
    *z = -*z;
  } else if (rotation == IMU_MOUNT_ROTATION_PITCH_180) {
    *x = -*x;
    *z = -*z;
  }
}

static inline void IMU_ApplyConfiguredMountRotation(float *x, float *y, float *z)
{
  IMU_ApplyMountRotation(IMU_MOUNT_ROTATION, x, y, z);
}

#endif /* IMU_MOUNT_TRANSFORM_H */

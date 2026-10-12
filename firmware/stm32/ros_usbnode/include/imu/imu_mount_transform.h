#ifndef IMU_MOUNT_TRANSFORM_H
#define IMU_MOUNT_TRANSFORM_H

#include <stdint.h>

#define IMU_MOUNT_ROTATION_IDENTITY 0
#define IMU_MOUNT_ROTATION_YAW_180 1

#ifndef IMU_MOUNT_ROTATION
#define IMU_MOUNT_ROTATION IMU_MOUNT_ROTATION_IDENTITY
#endif

#if IMU_MOUNT_ROTATION != IMU_MOUNT_ROTATION_IDENTITY && \
    IMU_MOUNT_ROTATION != IMU_MOUNT_ROTATION_YAW_180
#error "Unsupported IMU_MOUNT_ROTATION"
#endif

/* Convert a vector from the sensor's physical mounting frame into the robot
 * base frame. Use the same proper rotation for acceleration, angular velocity,
 * and magnetic field so firmware control and USB telemetry agree. */
static inline void IMU_ApplyMountRotation(uint8_t rotation, float *x, float *y,
                                          float *z)
{
  (void)z;
  if (rotation == IMU_MOUNT_ROTATION_YAW_180) {
    *x = -*x;
    *y = -*y;
  }
}

static inline void IMU_ApplyConfiguredMountRotation(float *x, float *y, float *z)
{
  IMU_ApplyMountRotation(IMU_MOUNT_ROTATION, x, y, z);
}

#endif /* IMU_MOUNT_TRANSFORM_H */

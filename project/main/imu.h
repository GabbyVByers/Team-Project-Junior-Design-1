
/*
 *   Header [imu.h]
 */

#ifndef IMU_HEADER_GUARD
#define IMU_HEADER_GUARD

#include <stdbool.h>

typedef struct {
  float w, x, y, z;  // orientation, device -> world (game rotation vector: gravity-aligned, no magnetometer)
  float ax, ay, az;  // linear acceleration (gravity removed) in the device frame, m/s^2
  float dt;          // seconds since the previous sample, from the sensor's own sample clock
} imu_sample_t;

void imu_init(void);
bool imu_read(imu_sample_t *out);  // pops the next sample; false if none is pending

#endif /* IMU_HEADER_GUARD */

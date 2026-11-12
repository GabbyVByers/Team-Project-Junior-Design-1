
/*
 *   Header [imu.h]
 */

#ifndef IMU_HEADER_GUARD
#define IMU_HEADER_GUARD

typedef struct {
  float w, x, y, z;
  float ax, ay, az;
} imu_data_t;

void imu_init(void);
imu_data_t imu_read(void);

#endif /* IMU_HEADER_GUARD */



/*
 *   Header [imu.h]
 */

#ifndef IMU_HEADER_GUARD
#define IMU_HEADER_GUARD

#include <stdbool.h>

typedef struct {
    float w, x, y, z; // quaternion
    float ax, ay, az; // acceleration
} imu_data_t;

extern imu_data_t imu_data;

void imu_init(void);
bool imu_has_data(void);
void imu_receive_packet(void);

#endif /* IMU_HEADER_GUARD */


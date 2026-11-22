
/*
 *   Header [imu.h]
 */

#ifndef IMU_HEADER_GUARD
#define IMU_HEADER_GUARD

typedef struct { float w, x, y, z; } quat_t;
typedef struct { float x, y, z; } vec3_t;

typedef struct {
	quat_t orientation;   // device -> world (game rotation vector: gravity-aligned, no magnetometer)
	vec3_t linear_accel;  // gravity removed, device frame, m/s^2
	float dt;             // seconds since the previous sample, from the sensor's own sample clock
} imu_sample_t;

void imu_init(void);
imu_sample_t imu_next_sample(void);  // blocks until the next sample arrives

#endif /* IMU_HEADER_GUARD */

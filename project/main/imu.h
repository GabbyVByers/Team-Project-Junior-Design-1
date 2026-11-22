
/*
 *   Header [imu.h]
 */

#ifndef IMU_HEADER_GUARD
#define IMU_HEADER_GUARD

#include <stdbool.h>

typedef struct { float w, x, y, z; } quat_t;
typedef struct { float x, y, z; } vec3_t;

// Pose in the level frame: z is up, x points where the device's x axis pointed during
// calibration, and the origin is where the device sat during calibration
typedef struct {
	vec3_t position;     // m
	quat_t orientation;  // device -> level frame
} imu_pose_t;

typedef struct {
	quat_t orientation;  // device -> world (game rotation vector: gravity-aligned, no magnetometer),
	                     // interpolated to the moment the acceleration was measured
	vec3_t accel;        // calibrated accelerometer (gravity included), device frame, m/s^2
	float angular_rate;  // rad/s, from the change in orientation since the previous sample
	float dt;            // seconds since the previous sample, from the hub's timestamps
} imu_sample_t;

void imu_init(void);
void imu_calibrate(void);     // device must be at rest; blocks for a few seconds
imu_pose_t imu_update(void);  // blocks for the next sample and returns the updated pose

// Lower-level access, used by the sensor calibration procedure
imu_sample_t imu_next_sample(void);  // blocks until the next sample arrives
void imu_enable_dynamic_calibration(bool accel, bool gyro, bool mag);
bool imu_save_calibration(void);     // stores the sensor's calibration in its flash; true on success

#endif /* IMU_HEADER_GUARD */

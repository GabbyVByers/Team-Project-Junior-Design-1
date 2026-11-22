
/*
 *   Header [imu.h]
 */

#ifndef IMU_HEADER_GUARD
#define IMU_HEADER_GUARD

typedef struct { float w, x, y, z; } quat_t;
typedef struct { float x, y, z; } vec3_t;

// Pose in the level frame: z is up, x points where the device's x axis pointed during
// calibration, and the origin is where the device sat during calibration
typedef struct {
	vec3_t position;     // m
	quat_t orientation;  // device -> level frame
} imu_pose_t;

void imu_init(void);
void imu_calibrate(void);     // device must be at rest; blocks for a few seconds
imu_pose_t imu_update(void);  // blocks for the next sample and returns the updated pose

#endif /* IMU_HEADER_GUARD */

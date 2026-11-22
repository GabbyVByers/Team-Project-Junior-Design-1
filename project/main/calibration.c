
/*
 *   Source [calibration.c]
 */

#include "calibration.h"
#include <stdio.h>
#include <math.h>
#include "imu.h"

#define HOLD_TIME_S     1.5f   // the sensor wants each orientation held for about a second
#define HOLD_MAX_RATE   0.1f   // rad/s
#define HOLD_MAX_JITTER 0.15f  // m/s^2, raw reading vs its own short-term average
#define SMOOTHING       0.1f   // per-sample weight of the short-term average
#define AXIS_ALIGNMENT  0.87f  // cos(30 deg): gravity this close to a device axis counts as that pose

typedef enum { POSE_FLAT, POSE_X_UP, POSE_X_DOWN, POSE_Y_UP, POSE_Y_DOWN, POSE_COUNT, POSE_NONE } pose_t;

static const char *pose_names[POSE_COUNT] = {
	"flat (+Z up)", "+X side up", "-X side up", "+Y side up", "-Y side up",
};

// At rest the accelerometer reads +1 g along whichever device axis points up
static pose_t classify_pose(vec3_t accel)
{
	float limit = AXIS_ALIGNMENT * sqrtf(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);

	if ( accel.z > limit) { return POSE_FLAT; }
	if ( accel.x > limit) { return POSE_X_UP; }
	if (-accel.x > limit) { return POSE_X_DOWN; }
	if ( accel.y > limit) { return POSE_Y_UP; }
	if (-accel.y > limit) { return POSE_Y_DOWN; }
	return POSE_NONE;
}

static void print_remaining(const bool captured[POSE_COUNT])
{
	printf("Calibration: still needed:");
	for (int i = 0; i < POSE_COUNT; i++) {
		if (!captured[i]) { printf(" [%s]", pose_names[i]); }
	}
	printf("\n");
}

bool calibration_run(void (*on_pose_captured)(void))
{
	imu_enable_dynamic_calibration(true, true, false);

	printf("Calibration: hold the device still with each side facing up in turn, then set it down flat\n");

	bool captured[POSE_COUNT] = { false };
	int remaining = POSE_COUNT;
	print_remaining(captured);

	vec3_t average = imu_next_sample().accel;
	float hold_time = 0;
	bool hold_used = false;  // each hold counts once, however long it lasts

	while (true) {
		imu_sample_t sample = imu_next_sample();
		vec3_t a = sample.accel;

		average.x += SMOOTHING * (a.x - average.x);
		average.y += SMOOTHING * (a.y - average.y);
		average.z += SMOOTHING * (a.z - average.z);
		vec3_t d = { a.x - average.x, a.y - average.y, a.z - average.z };
		float jitter = sqrtf(d.x * d.x + d.y * d.y + d.z * d.z);

		if (jitter < HOLD_MAX_JITTER && sample.angular_rate < HOLD_MAX_RATE) {
			hold_time += sample.dt;
		} else {
			hold_time = 0;
			hold_used = false;
		}
		if (hold_time < HOLD_TIME_S || hold_used) { continue; }
		hold_used = true;

		pose_t pose = classify_pose(average);
		if (pose == POSE_NONE) { continue; }

		if (!captured[pose]) {
			captured[pose] = true;
			remaining--;
			printf("Calibration: captured %s\n", pose_names[pose]);
			if (remaining > 0) { print_remaining(captured); }
			on_pose_captured();
		}

		// Finish once every side has been seen and the device is resting flat again
		if (remaining == 0 && pose == POSE_FLAT) { break; }
	}

	bool saved = imu_save_calibration();
	printf(saved ? "Calibration: saved to the sensor\n" : "Calibration: saving to the sensor FAILED\n");
	return saved;
}


/*
 *   Header [calibration.h]
 */

#ifndef CALIBRATION_HEADER_GUARD
#define CALIBRATION_HEADER_GUARD

#include <stdbool.h>

// Sensor calibration sweep. Start flat, then hold the device still for a moment with each of
// its four sides facing up in turn (tilt left, right, forward, back by ~90 degrees), then set
// it back down flat. The sensor's own accelerometer calibration learns from those holds, and
// the result is saved in the sensor's flash so later boots use it.
//
// on_pose_captured is called each time a new orientation has been recorded (e.g. to blink an LED).
// Returns true if the calibration was saved.
bool calibration_run(void (*on_pose_captured)(void));

#endif /* CALIBRATION_HEADER_GUARD */

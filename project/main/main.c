
/*
 *   Source [main.c]
 */

#include <stdio.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "imu.h"
#include "calibration.h"
#include "lidar.h"
#include "web.h"

#define PIN_LED_CALIBRATING GPIO_NUM_4
#define PIN_LED_READY       GPIO_NUM_2
#define PIN_BUTTON_HALT     GPIO_NUM_32  // external pull-up; reads 0 while pressed

#define SEND_EVERY  4   // samples (25 Hz at the IMU's 100 Hz)
#define PRINT_EVERY 10  // samples (10 Hz)

#define LIDAR_MIN_AMPLITUDE 100    // TF-Luna: weaker returns are unreliable
#define LIDAR_BAD_AMPLITUDE 65535  // TF-Luna: overexposed

static void show_ready(bool ready)
{
	gpio_set_level(PIN_LED_CALIBRATING, !ready);
	gpio_set_level(PIN_LED_READY, ready);
}

static void blink_calibrating(int times)
{
	for (int i = 0; i < times; i++) {
		gpio_set_level(PIN_LED_CALIBRATING, 0);
		vTaskDelay(pdMS_TO_TICKS(150));
		gpio_set_level(PIN_LED_CALIBRATING, 1);
		vTaskDelay(pdMS_TO_TICKS(150));
	}
}

static void on_pose_captured(void)
{
	blink_calibrating(1);
}

void app_main(void)
{
	gpio_config_t led_cfg = {
		.pin_bit_mask = (1ULL << PIN_LED_CALIBRATING) | (1ULL << PIN_LED_READY),
		.mode = GPIO_MODE_OUTPUT,
	};
	gpio_config(&led_cfg);
	show_ready(false);

	gpio_config_t button_cfg = {
		.pin_bit_mask = 1ULL << PIN_BUTTON_HALT,
		.mode = GPIO_MODE_INPUT,
	};
	gpio_config(&button_cfg);

	// Holding HALT/STOP through a reset runs the sensor calibration sweep before the usual
	// at-rest calibration
	bool calibrate_sensor = gpio_get_level(PIN_BUTTON_HALT) == 0;

	web_init();
	lidar_init();
	imu_init();

	if (calibrate_sensor) {
		blink_calibrating(3);
		calibration_run(on_pose_captured);
	}

	imu_calibrate();
	show_ready(true);

	for (int n = 1; ; n++) {
		imu_pose_t pose = imu_update();
		vec3_t p = pose.position;
		quat_t q = pose.orientation;

		lidar_data_t lidar = lidar_read();
		bool lidar_valid = lidar.distance_cm > 0 && lidar.amplitude >= LIDAR_MIN_AMPLITUDE && lidar.amplitude != LIDAR_BAD_AMPLITUDE;
		float distance = lidar_valid ? lidar.distance_cm / 100.0f : 0;  // m; 0 = no valid reading

		if (n % SEND_EVERY == 0) {
			// "px py pz qw qx qy qz d", parsed by web/script.js
			char text[112];
			snprintf(text, sizeof(text), "%.4f %.4f %.4f %.4f %.4f %.4f %.4f %.2f", p.x, p.y, p.z, q.w, q.x, q.y, q.z, distance);
			web_broadcast(text);
		}

		if (n % PRINT_EVERY == 0) {
			printf("Pos: x=%.3f y=%.3f z=%.3f m | Quat: w=%.4f x=%.4f y=%.4f z=%.4f | Dist: %.2f m\n",
				p.x, p.y, p.z, q.w, q.x, q.y, q.z, distance);
		}
	}
}

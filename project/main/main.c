
/*
 *   Source [main.c]
 */

#define IMU_DEMO
#ifdef IMU_DEMO

#include <stdio.h>
#include <stdbool.h>
#include "driver/gpio.h"
#include "imu.h"
#include "web.h"

#define PIN_LED_CALIBRATING GPIO_NUM_4
#define PIN_LED_READY       GPIO_NUM_2

#define SEND_EVERY  4   // samples (25 Hz at the IMU's 100 Hz)
#define PRINT_EVERY 10  // samples (10 Hz)

static void show_ready(bool ready)
{
	gpio_set_level(PIN_LED_CALIBRATING, !ready);
	gpio_set_level(PIN_LED_READY, ready);
}

void app_main(void)
{
	gpio_config_t led_cfg = {
		.pin_bit_mask = (1ULL << PIN_LED_CALIBRATING) | (1ULL << PIN_LED_READY),
		.mode = GPIO_MODE_OUTPUT,
	};
	gpio_config(&led_cfg);
	show_ready(false);

	web_init();
	imu_init();
	imu_calibrate();
	show_ready(true);

	for (int n = 1; ; n++) {
		imu_pose_t pose = imu_update();
		vec3_t p = pose.position;
		quat_t q = pose.orientation;

		if (n % SEND_EVERY == 0) {
			// "px py pz qw qx qy qz", parsed by web/index.html
			char text[96];
			snprintf(text, sizeof(text), "%.4f %.4f %.4f %.4f %.4f %.4f %.4f", p.x, p.y, p.z, q.w, q.x, q.y, q.z);
			web_broadcast(text);
		}

		if (n % PRINT_EVERY == 0) {
			printf("Pos: x=%.3f y=%.3f z=%.3f m | Quat: w=%.4f x=%.4f y=%.4f z=%.4f\n", p.x, p.y, p.z, q.w, q.x, q.y, q.z);
		}
	}
}

#endif /* IMU_DEMO */



// #define LIDAR_DEMO
#ifdef LIDAR_DEMO

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lidar.h"

void app_main(void)
{
	lidar_init();

	while (true) {
		vTaskDelay(pdMS_TO_TICKS(100));
		printf("Distance: %d cm\n", lidar_read().distance_cm);
	}
}

#endif /* LIDAR_DEMO */

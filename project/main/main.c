
/*
 *   Source [main.c]
 */

#define IMU_DEMO
#ifdef IMU_DEMO

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "imu.h"

#define PIN_LED_CALIBRATING GPIO_NUM_4
#define PIN_LED_READY       GPIO_NUM_2

#define SETTLE_TIME_S      1.0f  // let the sensor fusion converge before calibrating
#define CALIBRATION_TIME_S 2.0f  // device at rest; averaged into the baseline
#define PRINT_EVERY        10    // samples (10 Hz at the IMU's 100 Hz)

// Stationary detection (zero-velocity update)
#define STILL_MAX_ACCEL    0.2f   // m/s^2
#define STILL_MAX_RATE     0.1f   // rad/s
#define STILL_MIN_TIME     0.2f   // s both limits must hold before the device counts as stationary
#define BIAS_TRACKING_RATE 0.02f  // per-sample weight for re-estimating accel bias while stationary

static quat_t quat_multiply(quat_t a, quat_t b)
{
	return (quat_t){
		a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
		a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
		a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
		a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
	};
}

static quat_t quat_conjugate(quat_t q)         { return (quat_t){ q.w, -q.x, -q.y, -q.z }; }
static quat_t quat_add(quat_t a, quat_t b)     { return (quat_t){ a.w + b.w, a.x + b.x, a.y + b.y, a.z + b.z }; }
static quat_t quat_scale(quat_t q, float s)    { return (quat_t){ q.w * s, q.x * s, q.y * s, q.z * s }; }
static float  quat_dot(quat_t a, quat_t b)     { return a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z; }

static vec3_t vec_add(vec3_t a, vec3_t b)      { return (vec3_t){ a.x + b.x, a.y + b.y, a.z + b.z }; }
static vec3_t vec_sub(vec3_t a, vec3_t b)      { return (vec3_t){ a.x - b.x, a.y - b.y, a.z - b.z }; }
static vec3_t vec_scale(vec3_t v, float s)     { return (vec3_t){ v.x * s, v.y * s, v.z * s }; }
static float  vec_length(vec3_t v)             { return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z); }

// q * v * q^-1
static vec3_t quat_rotate(quat_t q, vec3_t v)
{
	quat_t r = quat_multiply(quat_multiply(q, (quat_t){ 0, v.x, v.y, v.z }), quat_conjugate(q));
	return (vec3_t){ r.x, r.y, r.z };
}

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

	imu_init();

	for (float t = 0; t < SETTLE_TIME_S; ) {
		t += imu_next_sample().dt;
	}

	// Calibration: average orientation and residual acceleration (the sensor's bias) at rest
	quat_t orientation_sum = { 0 };
	vec3_t accel_sum = { 0 };
	int count = 0;

	for (float t = 0; t < CALIBRATION_TIME_S; count++) {
		imu_sample_t sample = imu_next_sample();
		t += sample.dt;

		// q and -q are the same rotation; flip to agree with the running sum before adding
		float sign = (quat_dot(orientation_sum, sample.orientation) < 0) ? -1.0f : 1.0f;
		orientation_sum = quat_add(orientation_sum, quat_scale(sample.orientation, sign));
		accel_sum = vec_add(accel_sum, sample.linear_accel);
	}

	quat_t baseline = quat_scale(orientation_sum, 1.0f / sqrtf(quat_dot(orientation_sum, orientation_sum)));
	quat_t baseline_inverse = quat_conjugate(baseline);
	vec3_t accel_bias = vec_scale(accel_sum, 1.0f / count);

	show_ready(true);

	// Tracking: position is expressed in the device's frame as it was during calibration
	vec3_t position = { 0 }, velocity = { 0 }, prev_accel = { 0 };
	quat_t prev_orientation = baseline;
	float still_time = 0, motion_time = 0;
	bool moving = false;

	for (int n = 1; ; n++) {
		imu_sample_t sample = imu_next_sample();
		float dt = sample.dt;
		vec3_t accel = vec_sub(sample.linear_accel, accel_bias);

		// Angular rate from the change in orientation since the last sample
		quat_t step = quat_multiply(quat_conjugate(prev_orientation), sample.orientation);
		float angular_rate = 2.0f * vec_length((vec3_t){ step.x, step.y, step.z }) / dt;
		prev_orientation = sample.orientation;

		bool still_now = vec_length(accel) < STILL_MAX_ACCEL && angular_rate < STILL_MAX_RATE;
		still_time = still_now ? still_time + dt : 0;

		// Orientation change since calibration (current device frame -> baseline frame)
		quat_t rotation = quat_multiply(baseline_inverse, sample.orientation);

		if (still_time >= STILL_MIN_TIME) {
			if (moving) {
				// The true velocity is zero now, so whatever is left is integration error. Bias
				// error grows velocity error linearly, so the position picked up half of it
				// times the length of the motion; take that back out.
				position = vec_sub(position, vec_scale(velocity, 0.5f * motion_time));
				moving = false;
			}
			velocity = prev_accel = (vec3_t){ 0 };

			// At rest the measured linear acceleration is pure bias; track it as it drifts
			accel_bias = vec_add(accel_bias, vec_scale(accel, BIAS_TRACKING_RATE));
		} else {
			if (!moving) {
				moving = true;
				motion_time = 0;
			}
			motion_time += dt;

			// Trapezoidal integration in the baseline frame
			vec3_t baseline_accel = quat_rotate(rotation, accel);
			vec3_t next_velocity = vec_add(velocity, vec_scale(vec_add(prev_accel, baseline_accel), 0.5f * dt));
			position = vec_add(position, vec_scale(vec_add(velocity, next_velocity), 0.5f * dt));
			velocity = next_velocity;
			prev_accel = baseline_accel;
		}

		if (n % PRINT_EVERY == 0) {
			printf("Pos: x=%.3f y=%.3f z=%.3f m | dQuat: w=%.4f x=%.4f y=%.4f z=%.4f\n",
				position.x, position.y, position.z, rotation.w, rotation.x, rotation.y, rotation.z);
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



//#define WEB_DEMO
#ifdef WEB_DEMO

#include <stdio.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "esp_http_server.h"
#include "secrets.h"

#define MAX_CLIENTS 7

static httpd_handle_t server;

static void on_network_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
	if (base == WIFI_EVENT && (id == WIFI_EVENT_STA_START || id == WIFI_EVENT_STA_DISCONNECTED)) {
		esp_wifi_connect();
	} else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
		ip_event_got_ip_t *event = data;
		printf("WiFi connected - IP: " IPSTR "\n", IP2STR(&event->ip_info.ip));
	}
}

static void wifi_init(void)
{
	esp_err_t err = nvs_flash_init();
	if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		err = nvs_flash_init();
	}
	ESP_ERROR_CHECK(err);

	ESP_ERROR_CHECK(esp_netif_init());
	ESP_ERROR_CHECK(esp_event_loop_create_default());
	esp_netif_create_default_wifi_sta();

	wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&init_cfg));
	ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, on_network_event, NULL));
	ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, on_network_event, NULL));

	wifi_config_t wifi_cfg = {
		.sta = { .ssid = WIFI_SSID, .password = WIFI_PASSWORD },
	};
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
	ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
	ESP_ERROR_CHECK(esp_wifi_start());
}

// Called for every frame the browser sends (the server handles the handshake itself)
static esp_err_t ws_handler(httpd_req_t *req)
{
	// The page never sends anything, but drain any frame so the socket stays in sync
	uint8_t buf[32];
	httpd_ws_frame_t frame = { .payload = buf };
	return httpd_ws_recv_frame(req, &frame, sizeof(buf));
}

// Runs inside the server task (queued with httpd_queue_work); sends to every open WebSocket
static void send_counter(void *arg)
{
	char text[16];
	int len = snprintf(text, sizeof(text), "%d", (int)(intptr_t)arg);

	httpd_ws_frame_t frame = {
		.type    = HTTPD_WS_TYPE_TEXT,
		.payload = (uint8_t *)text,
		.len     = len,
	};

	int fds[MAX_CLIENTS];
	size_t count = MAX_CLIENTS;
	if (httpd_get_client_list(server, &count, fds) != ESP_OK) { return; }

	for (size_t i = 0; i < count; i++) {
		if (httpd_ws_get_fd_info(server, fds[i]) == HTTPD_WS_CLIENT_WEBSOCKET) {
			httpd_ws_send_frame_async(server, fds[i], &frame);
		}
	}
}

void app_main(void)
{
	wifi_init();

	httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
	cfg.max_open_sockets = MAX_CLIENTS;
	ESP_ERROR_CHECK(httpd_start(&server, &cfg));

	httpd_uri_t ws_uri = {
		.uri          = "/ws",
		.method       = HTTP_GET,
		.handler      = ws_handler,
		.is_websocket = true,
	};
	ESP_ERROR_CHECK(httpd_register_uri_handler(server, &ws_uri));

	int counter = 0;
	while (true) {
		vTaskDelay(pdMS_TO_TICKS(100));
		httpd_queue_work(server, send_counter, (void *)(intptr_t)counter++);
	}
}

#endif /* WEB_DEMO */



/*
 *   Source [imu.c]
 */

#include "imu.h"
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"

#define PIN_SDA        GPIO_NUM_21
#define PIN_SCL        GPIO_NUM_22
#define PIN_INT        GPIO_NUM_18
#define PIN_RESET      GPIO_NUM_19

#define I2C_ADDRESS    0x4A
#define I2C_SPEED_HZ   400000  // board has 4.7k pull-ups
#define I2C_TIMEOUT_MS 100

#define REPORT_INTERVAL_US 10000  // 100 Hz

#define SHTP_HEADER_SIZE   4
#define SHTP_MAX_PACKET    512
#define CHANNEL_EXECUTABLE 1
#define CHANNEL_CONTROL    2
#define CHANNEL_REPORTS    3

#define CMD_GET_FEATURE_RESPONSE 0xFC
#define CMD_SET_FEATURE          0xFD
#define EXEC_RESET_DONE          0x01

#define REPORT_LINEAR_ACCELERATION 0x04
#define REPORT_GAME_ROTATION       0x08
#define REPORT_TIMESTAMP_REBASE    0xFA
#define REPORT_BASE_TIMESTAMP      0xFB

#define QUEUE_SIZE 32

#define SETTLE_TIME_S      1.0f  // let the sensor fusion converge before calibrating
#define CALIBRATION_TIME_S 2.0f  // device at rest; averaged into the baseline

// Stationary detection (zero-velocity update)
#define STILL_MAX_ACCEL    0.2f   // m/s^2
#define STILL_MAX_RATE     0.1f   // rad/s
#define STILL_MIN_TIME     0.2f   // s both limits must hold before the device counts as stationary
#define BIAS_TRACKING_RATE 0.02f  // per-sample weight for re-estimating accel bias while stationary

typedef struct {
	quat_t orientation;   // device -> world (game rotation vector: gravity-aligned, no magnetometer)
	vec3_t linear_accel;  // gravity removed, device frame, m/s^2
	float dt;             // seconds since the previous sample, from the sensor's own sample clock
} imu_sample_t;

static i2c_master_dev_handle_t device;
static bool reset_done;

static quat_t latest_orientation = { 1, 0, 0, 0 };
static uint32_t accel_interval_us = REPORT_INTERVAL_US;  // actual rate, from the hub's feature response
static int last_accel_sequence = -1;

static imu_sample_t sample_queue[QUEUE_SIZE];
static int queue_head, queue_count;

// Tracking state
static quat_t heading_inverse;  // turns the world frame into the level frame
static vec3_t accel_bias;
static imu_pose_t pose;
static vec3_t velocity, prev_accel;
static quat_t prev_orientation;
static float still_time, motion_time;
static bool moving;

static quat_t quat_multiply(quat_t a, quat_t b)
{
	return (quat_t){
		a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
		a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
		a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
		a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
	};
}

static quat_t quat_conjugate(quat_t q)      { return (quat_t){ q.w, -q.x, -q.y, -q.z }; }
static quat_t quat_add(quat_t a, quat_t b)  { return (quat_t){ a.w + b.w, a.x + b.x, a.y + b.y, a.z + b.z }; }
static quat_t quat_scale(quat_t q, float s) { return (quat_t){ q.w * s, q.x * s, q.y * s, q.z * s }; }
static float  quat_dot(quat_t a, quat_t b)  { return a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z; }

static vec3_t vec_add(vec3_t a, vec3_t b)   { return (vec3_t){ a.x + b.x, a.y + b.y, a.z + b.z }; }
static vec3_t vec_sub(vec3_t a, vec3_t b)   { return (vec3_t){ a.x - b.x, a.y - b.y, a.z - b.z }; }
static vec3_t vec_scale(vec3_t v, float s)  { return (vec3_t){ v.x * s, v.y * s, v.z * s }; }
static float  vec_length(vec3_t v)          { return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z); }

// q * v * q^-1
static vec3_t quat_rotate(quat_t q, vec3_t v)
{
	quat_t r = quat_multiply(quat_multiply(q, (quat_t){ 0, v.x, v.y, v.z }), quat_conjugate(q));
	return (vec3_t){ r.x, r.y, r.z };
}

static bool packet_pending(void)
{
	return gpio_get_level(PIN_INT) == 0;
}

static float read_fixed_point(const uint8_t *bytes, int fraction_bits)
{
	int16_t raw = (int16_t)(bytes[0] | (bytes[1] << 8));
	return (float)raw / (float)(1 << fraction_bits);
}

static int report_size(uint8_t report_id)
{
	switch (report_id) {
	case REPORT_LINEAR_ACCELERATION: return 10;
	case REPORT_GAME_ROTATION:       return 12;
	case REPORT_TIMESTAMP_REBASE:
	case REPORT_BASE_TIMESTAMP:      return 5;
	default:                         return 0;
	}
}

static void queue_accel_sample(const uint8_t *report)
{
	// Sequence numbers count every report the hub produced, so gaps from dropped
	// reports still give the correct elapsed time
	uint8_t sequence = report[1];
	uint8_t steps = (last_accel_sequence < 0) ? 1 : (uint8_t)(sequence - last_accel_sequence);
	last_accel_sequence = sequence;

	if (queue_count == QUEUE_SIZE) { return; }

	sample_queue[(queue_head + queue_count++) % QUEUE_SIZE] = (imu_sample_t){
		.orientation = latest_orientation,
		.linear_accel = {
			read_fixed_point(&report[4], 8),
			read_fixed_point(&report[6], 8),
			read_fixed_point(&report[8], 8),
		},
		.dt = steps * accel_interval_us * 1e-6f,
	};
}

static void parse_reports(const uint8_t *data, int length)
{
	for (int i = 0; i < length; ) {
		const uint8_t *report = &data[i];
		int size = report_size(report[0]);
		if (size == 0 || i + size > length) { return; }

		if (report[0] == REPORT_GAME_ROTATION) {
			latest_orientation = (quat_t){
				.x = read_fixed_point(&report[4],  14),
				.y = read_fixed_point(&report[6],  14),
				.z = read_fixed_point(&report[8],  14),
				.w = read_fixed_point(&report[10], 14),
			};
		} else if (report[0] == REPORT_LINEAR_ACCELERATION) {
			queue_accel_sample(report);
		}

		i += size;
	}
}

static void parse_control(const uint8_t *data, int length)
{
	// The hub answers each Set Feature with the interval it actually chose
	if (length >= 9 && data[0] == CMD_GET_FEATURE_RESPONSE && data[1] == REPORT_LINEAR_ACCELERATION) {
		uint32_t interval = data[5] | (data[6] << 8) | (data[7] << 16) | ((uint32_t)data[8] << 24);
		if (interval > 0) { accel_interval_us = interval; }
	}
}

static void receive_packet(void)
{
	static uint8_t packet[SHTP_MAX_PACKET];

	if (i2c_master_receive(device, packet, SHTP_HEADER_SIZE, I2C_TIMEOUT_MS) != ESP_OK) { return; }

	int length = (packet[0] | (packet[1] << 8)) & 0x7FFF;
	if (length <= SHTP_HEADER_SIZE) { return; }
	if (length > SHTP_MAX_PACKET) { length = SHTP_MAX_PACKET; }

	if (i2c_master_receive(device, packet, length, I2C_TIMEOUT_MS) != ESP_OK) { return; }

	uint8_t channel = packet[2];
	const uint8_t *payload = &packet[SHTP_HEADER_SIZE];
	int payload_length = length - SHTP_HEADER_SIZE;

	if (channel == CHANNEL_REPORTS) {
		parse_reports(payload, payload_length);
	} else if (channel == CHANNEL_CONTROL) {
		parse_control(payload, payload_length);
	} else if (channel == CHANNEL_EXECUTABLE && payload[0] == EXEC_RESET_DONE) {
		reset_done = true;
	}
}

static void receive_or_wait(void)
{
	if (packet_pending()) {
		receive_packet();
	} else {
		vTaskDelay(1);
	}
}

static void enable_report(uint8_t report_id, uint32_t interval_us)
{
	static uint8_t sequence = 0;
	uint8_t packet[SHTP_HEADER_SIZE + 17] = { 0 };

	packet[0] = sizeof(packet);
	packet[2] = CHANNEL_CONTROL;
	packet[3] = sequence++;

	uint8_t *cmd = &packet[SHTP_HEADER_SIZE];
	cmd[0] = CMD_SET_FEATURE;
	cmd[1] = report_id;
	cmd[5] = interval_us & 0xFF;
	cmd[6] = (interval_us >> 8) & 0xFF;
	cmd[7] = (interval_us >> 16) & 0xFF;
	cmd[8] = (interval_us >> 24) & 0xFF;

	i2c_master_transmit(device, packet, sizeof(packet), I2C_TIMEOUT_MS);
}

static void setup_gpio(void)
{
	gpio_config_t reset_cfg = {
		.pin_bit_mask = 1ULL << PIN_RESET,
		.mode = GPIO_MODE_OUTPUT,
	};
	gpio_config(&reset_cfg);

	gpio_config_t int_cfg = {
		.pin_bit_mask = 1ULL << PIN_INT,
		.mode = GPIO_MODE_INPUT,
		.pull_up_en = GPIO_PULLUP_ENABLE,
	};
	gpio_config(&int_cfg);
}

static void setup_i2c(void)
{
	i2c_master_bus_config_t bus_cfg = {
		.i2c_port = I2C_NUM_0,
		.sda_io_num = PIN_SDA,
		.scl_io_num = PIN_SCL,
		.clk_source = I2C_CLK_SRC_DEFAULT,
		.glitch_ignore_cnt = 7,
		.flags.enable_internal_pullup = true,
	};
	i2c_master_bus_handle_t bus;
	ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));

	i2c_device_config_t dev_cfg = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address = I2C_ADDRESS,
		.scl_speed_hz = I2C_SPEED_HZ,
	};
	ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &dev_cfg, &device));
}

void imu_init(void)
{
	setup_gpio();
	setup_i2c();

	gpio_set_level(PIN_RESET, 0);
	vTaskDelay(pdMS_TO_TICKS(10));
	gpio_set_level(PIN_RESET, 1);

	while (!reset_done) {
		receive_or_wait();
	}

	enable_report(REPORT_GAME_ROTATION, REPORT_INTERVAL_US);
	enable_report(REPORT_LINEAR_ACCELERATION, REPORT_INTERVAL_US);
}

static imu_sample_t next_sample(void)
{
	while (queue_count == 0) {
		receive_or_wait();
	}

	imu_sample_t sample = sample_queue[queue_head];
	queue_head = (queue_head + 1) % QUEUE_SIZE;
	queue_count--;
	return sample;
}

void imu_calibrate(void)
{
	for (float t = 0; t < SETTLE_TIME_S; ) {
		t += next_sample().dt;
	}

	// Average orientation and residual acceleration (the sensor's bias) at rest
	quat_t orientation_sum = { 0 };
	vec3_t accel_sum = { 0 };
	int count = 0;

	for (float t = 0; t < CALIBRATION_TIME_S; count++) {
		imu_sample_t sample = next_sample();
		t += sample.dt;

		// q and -q are the same rotation; flip to agree with the running sum before adding
		float sign = (quat_dot(orientation_sum, sample.orientation) < 0) ? -1.0f : 1.0f;
		orientation_sum = quat_add(orientation_sum, quat_scale(sample.orientation, sign));
		accel_sum = vec_add(accel_sum, sample.linear_accel);
	}

	quat_t baseline = quat_scale(orientation_sum, 1.0f / sqrtf(quat_dot(orientation_sum, orientation_sum)));
	accel_bias = vec_scale(accel_sum, 1.0f / count);

	// The world frame's heading is arbitrary; remove the device's heading at calibration
	// (yaw about the vertical z axis) so the level frame's x starts along the device's x
	quat_t b = baseline;
	float yaw = atan2f(2.0f * (b.w * b.z + b.x * b.y), 1.0f - 2.0f * (b.y * b.y + b.z * b.z));
	heading_inverse = (quat_t){ cosf(0.5f * yaw), 0, 0, -sinf(0.5f * yaw) };

	pose = (imu_pose_t){ .orientation = quat_multiply(heading_inverse, baseline) };
	velocity = prev_accel = (vec3_t){ 0 };
	prev_orientation = baseline;
	still_time = motion_time = 0;
	moving = false;
}

imu_pose_t imu_update(void)
{
	imu_sample_t sample = next_sample();
	float dt = sample.dt;
	vec3_t accel = vec_sub(sample.linear_accel, accel_bias);

	// Angular rate from the change in orientation since the last sample
	quat_t step = quat_multiply(quat_conjugate(prev_orientation), sample.orientation);
	float angular_rate = 2.0f * vec_length((vec3_t){ step.x, step.y, step.z }) / dt;
	prev_orientation = sample.orientation;

	bool still_now = vec_length(accel) < STILL_MAX_ACCEL && angular_rate < STILL_MAX_RATE;
	still_time = still_now ? still_time + dt : 0;

	pose.orientation = quat_multiply(heading_inverse, sample.orientation);

	if (still_time >= STILL_MIN_TIME) {
		if (moving) {
			// The true velocity is zero now, so whatever is left is integration error. Bias
			// error grows velocity error linearly, so the position picked up half of it
			// times the length of the motion; take that back out.
			pose.position = vec_sub(pose.position, vec_scale(velocity, 0.5f * motion_time));
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

		// Trapezoidal integration in the level frame
		vec3_t level_accel = quat_rotate(pose.orientation, accel);
		vec3_t next_velocity = vec_add(velocity, vec_scale(vec_add(prev_accel, level_accel), 0.5f * dt));
		pose.position = vec_add(pose.position, vec_scale(vec_add(velocity, next_velocity), 0.5f * dt));
		velocity = next_velocity;
		prev_accel = level_accel;
	}

	return pose;
}

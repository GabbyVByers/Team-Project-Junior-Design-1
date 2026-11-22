
/*
 *   Source [imu.c]
 */

#include "imu.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#define PIN_SDA        GPIO_NUM_21
#define PIN_SCL        GPIO_NUM_22
#define PIN_INT        GPIO_NUM_18
#define PIN_RESET      GPIO_NUM_19

#define I2C_ADDRESS    0x4A
#define I2C_SPEED_HZ   100000
#define I2C_TIMEOUT_MS 100
#define I2C_STRETCH_US 13000   // the hub stretches the clock while it prepares data; the ESP32
                               // driver's default limit (2 ms) is too short. ~13.1 ms is the max.

#define REPORT_INTERVAL_US 10000  // 100 Hz

#define SHTP_HEADER_SIZE   4
#define SHTP_MAX_PACKET    512
#define CHANNEL_EXECUTABLE 1
#define CHANNEL_CONTROL    2
#define CHANNEL_REPORTS    3

#define REPORT_COMMAND_RESPONSE 0xF1
#define REPORT_COMMAND_REQUEST  0xF2
#define CMD_SET_FEATURE         0xFD
#define COMMAND_SAVE_DCD        0x06
#define COMMAND_ME_CALIBRATION  0x07
#define EXEC_RESET_DONE         0x01

#define COMMAND_TIMEOUT_MS 1000

#define REPORT_ACCELEROMETER    0x01
#define REPORT_GAME_ROTATION    0x08
#define REPORT_TIMESTAMP_REBASE 0xFA
#define REPORT_BASE_TIMESTAMP   0xFB

#define TICK_US        100  // unit of the hub's timestamp fields
#define QUEUE_SIZE     32
#define PENDING_SIZE   8

#define SETTLE_TIME_S      1.0f  // let the sensor fusion converge before calibrating
#define CALIBRATION_TIME_S 2.0f  // device at rest; averaged into the baseline

// Stationary detection (zero-velocity update). Uses only the raw accelerometer and the rotation
// rate, never the gravity-subtracted acceleration, which a few degrees of tilt error corrupt.
#define STILL_MAX_JITTER      0.1f   // m/s^2, raw reading vs its own short-term average
#define STILL_MAX_G_ERROR     0.5f   // m/s^2, raw magnitude vs gravity (loose: axis scale errors)
#define STILL_MAX_RATE        0.1f   // rad/s
#define STILL_MIN_TIME        0.2f   // s all limits must hold before the device counts as stationary
#define JITTER_SMOOTHING      0.1f   // per-sample weight of the short-term average
#define GRAVITY_TRACKING_RATE 0.02f  // per-sample weight for re-measuring gravity while stationary

static i2c_master_dev_handle_t device;
static uint8_t control_sequence;  // SHTP sequence number for packets we send on the control channel
static bool reset_done;
static int command_response = -1;  // command ID of the last command response received
static uint8_t command_status;
static volatile int64_t interrupt_time_us;  // when the hub last raised INT

// The two most recent orientations, and accel readings waiting for an orientation newer than them
static quat_t orientation_prev = { 1, 0, 0, 0 }, orientation_latest = { 1, 0, 0, 0 };
static int64_t orientation_prev_us, orientation_latest_us;
static vec3_t pending_accel[PENDING_SIZE];
static int64_t pending_accel_us[PENDING_SIZE];
static int pending_count;
static int64_t last_sample_us;
static quat_t last_sample_orientation;

static imu_sample_t sample_queue[QUEUE_SIZE];
static int queue_head, queue_count;

// Tracking state
static quat_t heading_inverse;  // turns the world frame into the level frame
static vec3_t gravity;          // accelerometer reading at rest, level frame (includes residual bias)
static imu_pose_t pose;
static vec3_t velocity, prev_accel;
static vec3_t accel_average;  // short-term average of the raw reading, device frame
static float still_time;

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
static quat_t quat_normalize(quat_t q)      { return quat_scale(q, 1.0f / sqrtf(quat_dot(q, q))); }

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

static void IRAM_ATTR on_interrupt(void *arg)
{
	interrupt_time_us = esp_timer_get_time();
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

static int32_t read_int32(const uint8_t *bytes)
{
	return (int32_t)(bytes[0] | (bytes[1] << 8) | (bytes[2] << 16) | ((uint32_t)bytes[3] << 24));
}

static int report_size(uint8_t report_id)
{
	switch (report_id) {
	case REPORT_ACCELEROMETER:    return 10;
	case REPORT_GAME_ROTATION:    return 12;
	case REPORT_TIMESTAMP_REBASE:
	case REPORT_BASE_TIMESTAMP:   return 5;
	default:                      return 0;
	}
}

// Pairs an accel reading with the orientation at that same moment and queues it
static void queue_sample(int64_t time_us, vec3_t accel)
{
	int64_t span = orientation_latest_us - orientation_prev_us;
	float f = (span > 0) ? (float)(time_us - orientation_prev_us) / (float)span : 1.0f;
	f = fminf(fmaxf(f, 0.0f), 1.0f);
	quat_t orientation = quat_normalize(quat_add(quat_scale(orientation_prev, 1.0f - f), quat_scale(orientation_latest, f)));

	if (last_sample_us == 0) {
		last_sample_us = time_us;
		last_sample_orientation = orientation;
		return;
	}
	if (queue_count == QUEUE_SIZE) { return; }

	float dt = (time_us - last_sample_us) * 1e-6f;
	quat_t step = quat_multiply(quat_conjugate(last_sample_orientation), orientation);
	float angle = 2.0f * vec_length((vec3_t){ step.x, step.y, step.z });

	sample_queue[(queue_head + queue_count++) % QUEUE_SIZE] = (imu_sample_t){
		.orientation = orientation,
		.accel = accel,
		.angular_rate = (dt > 0) ? angle / dt : 0,
		.dt = dt,
	};
	last_sample_us = time_us;
	last_sample_orientation = orientation;
}

static void on_accel(int64_t time_us, vec3_t accel)
{
	if (time_us <= orientation_latest_us) {
		queue_sample(time_us, accel);
		return;
	}

	// Wait for an orientation from after this reading so it can be interpolated
	if (pending_count == PENDING_SIZE) {
		queue_sample(pending_accel_us[0], pending_accel[0]);
		for (int i = 1; i < PENDING_SIZE; i++) {
			pending_accel[i - 1] = pending_accel[i];
			pending_accel_us[i - 1] = pending_accel_us[i];
		}
		pending_count--;
	}
	pending_accel[pending_count] = accel;
	pending_accel_us[pending_count++] = time_us;
}

static void on_orientation(int64_t time_us, quat_t orientation)
{
	// q and -q are the same rotation; stay in one hemisphere so interpolation takes the short way
	if (quat_dot(orientation, orientation_latest) < 0) {
		orientation = quat_scale(orientation, -1.0f);
	}
	orientation_prev = orientation_latest;
	orientation_prev_us = orientation_latest_us;
	orientation_latest = orientation;
	orientation_latest_us = time_us;

	int done = 0;
	while (done < pending_count && pending_accel_us[done] <= time_us) {
		queue_sample(pending_accel_us[done], pending_accel[done]);
		done++;
	}
	for (int i = done; i < pending_count; i++) {
		pending_accel[i - done] = pending_accel[i];
		pending_accel_us[i - done] = pending_accel_us[i];
	}
	pending_count -= done;
}

static void parse_reports(const uint8_t *data, int length, int64_t packet_interrupt_us)
{
	// Report time = interrupt time - base delta + report delay (all deltas in 100 us ticks)
	int64_t base_us = packet_interrupt_us;

	for (int i = 0; i < length; ) {
		const uint8_t *report = &data[i];
		int size = report_size(report[0]);
		if (size == 0 || i + size > length) { return; }

		int64_t time_us = base_us + TICK_US * (((report[2] >> 2) << 8) | report[3]);

		switch (report[0]) {
		case REPORT_BASE_TIMESTAMP:
			base_us = packet_interrupt_us - TICK_US * (int64_t)read_int32(&report[1]);
			break;
		case REPORT_TIMESTAMP_REBASE:
			base_us += TICK_US * (int64_t)read_int32(&report[1]);
			break;
		case REPORT_ACCELEROMETER:
			on_accel(time_us, (vec3_t){
				read_fixed_point(&report[4], 8),
				read_fixed_point(&report[6], 8),
				read_fixed_point(&report[8], 8),
			});
			break;
		case REPORT_GAME_ROTATION:
			on_orientation(time_us, (quat_t){
				.x = read_fixed_point(&report[4],  14),
				.y = read_fixed_point(&report[6],  14),
				.z = read_fixed_point(&report[8],  14),
				.w = read_fixed_point(&report[10], 14),
			});
			break;
		}

		i += size;
	}
}

static void receive_packet(void)
{
	static uint8_t packet[SHTP_MAX_PACKET];

	// INT can't fire again until this packet is read, so this is the edge that announced it
	int64_t packet_interrupt_us = interrupt_time_us;

	if (i2c_master_receive(device, packet, SHTP_HEADER_SIZE, I2C_TIMEOUT_MS) != ESP_OK) { return; }

	int length = (packet[0] | (packet[1] << 8)) & 0x7FFF;
	if (length <= SHTP_HEADER_SIZE) { return; }
	if (length > SHTP_MAX_PACKET) { length = SHTP_MAX_PACKET; }

	if (i2c_master_receive(device, packet, length, I2C_TIMEOUT_MS) != ESP_OK) { return; }

	uint8_t channel = packet[2];
	const uint8_t *payload = &packet[SHTP_HEADER_SIZE];
	int payload_length = length - SHTP_HEADER_SIZE;

	if (channel == CHANNEL_REPORTS) {
		parse_reports(payload, payload_length, packet_interrupt_us);
	} else if (channel == CHANNEL_CONTROL && payload_length >= 6 && payload[0] == REPORT_COMMAND_RESPONSE) {
		command_response = payload[2] & 0x7F;
		command_status = payload[5];  // R0
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

static void send_control(const uint8_t *payload, int length)
{
	uint8_t packet[SHTP_HEADER_SIZE + 17] = { 0 };
	packet[0] = SHTP_HEADER_SIZE + length;
	packet[2] = CHANNEL_CONTROL;
	packet[3] = control_sequence++;
	memcpy(&packet[SHTP_HEADER_SIZE], payload, length);

	i2c_master_transmit(device, packet, SHTP_HEADER_SIZE + length, I2C_TIMEOUT_MS);
}

static void enable_report(uint8_t report_id, uint32_t interval_us)
{
	uint8_t cmd[17] = { CMD_SET_FEATURE, report_id };
	cmd[5] = interval_us & 0xFF;
	cmd[6] = (interval_us >> 8) & 0xFF;
	cmd[7] = (interval_us >> 16) & 0xFF;
	cmd[8] = (interval_us >> 24) & 0xFF;
	send_control(cmd, sizeof(cmd));
}

// Sends an SH-2 command request (P0-P8 in params) and waits for its response; returns R0 (status)
// or -1 on timeout. Sensor reports that arrive meanwhile are still queued as usual.
static int run_command(uint8_t command, const uint8_t params[9])
{
	static uint8_t command_sequence;
	uint8_t request[12] = { REPORT_COMMAND_REQUEST, command_sequence++, command };
	memcpy(&request[3], params, 9);

	command_response = -1;
	send_control(request, sizeof(request));

	for (int waited = 0; command_response != command; ) {
		if (waited >= pdMS_TO_TICKS(COMMAND_TIMEOUT_MS)) { return -1; }
		if (packet_pending()) {
			receive_packet();
		} else {
			vTaskDelay(1);
			waited++;
		}
	}
	return command_status;
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
		.intr_type = GPIO_INTR_NEGEDGE,
	};
	gpio_config(&int_cfg);

	gpio_install_isr_service(0);
	gpio_isr_handler_add(PIN_INT, on_interrupt, NULL);
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
		.scl_wait_us = I2C_STRETCH_US,
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
	enable_report(REPORT_ACCELEROMETER, REPORT_INTERVAL_US);
}

imu_sample_t imu_next_sample(void)
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
		t += imu_next_sample().dt;
	}

	// Average orientation and the accelerometer reading in the world frame (gravity plus bias) at rest
	quat_t orientation_sum = { 0 };
	vec3_t gravity_sum = { 0 };
	int count = 0;

	for (float t = 0; t < CALIBRATION_TIME_S; count++) {
		imu_sample_t sample = imu_next_sample();
		t += sample.dt;
		accel_average = sample.accel;

		// q and -q are the same rotation; flip to agree with the running sum before adding
		float sign = (quat_dot(orientation_sum, sample.orientation) < 0) ? -1.0f : 1.0f;
		orientation_sum = quat_add(orientation_sum, quat_scale(sample.orientation, sign));
		gravity_sum = vec_add(gravity_sum, quat_rotate(sample.orientation, sample.accel));
	}

	quat_t baseline = quat_normalize(orientation_sum);

	// The world frame's heading is arbitrary; remove the device's heading at calibration
	// (yaw about the vertical z axis) so the level frame's x starts along the device's x
	quat_t b = baseline;
	float yaw = atan2f(2.0f * (b.w * b.z + b.x * b.y), 1.0f - 2.0f * (b.y * b.y + b.z * b.z));
	heading_inverse = (quat_t){ cosf(0.5f * yaw), 0, 0, -sinf(0.5f * yaw) };
	gravity = quat_rotate(heading_inverse, vec_scale(gravity_sum, 1.0f / count));

	pose = (imu_pose_t){ .orientation = quat_multiply(heading_inverse, baseline) };
	velocity = prev_accel = (vec3_t){ 0 };
	still_time = 0;
}

imu_pose_t imu_update(void)
{
	imu_sample_t sample = imu_next_sample();
	float dt = sample.dt;
	if (dt <= 0) { return pose; }

	// Rotate the raw reading into the level frame and remove gravity there, using an orientation
	// sampled at the same instant. The orientation's own tilt error (up to ~2.5 deg per the
	// datasheet) still leaks about 0.17 m/s^2 of gravity per degree; that is the main drift source.
	pose.orientation = quat_multiply(heading_inverse, sample.orientation);
	vec3_t accel = vec_sub(quat_rotate(pose.orientation, sample.accel), gravity);

	accel_average = vec_add(accel_average, vec_scale(vec_sub(sample.accel, accel_average), JITTER_SMOOTHING));
	float jitter = vec_length(vec_sub(sample.accel, accel_average));
	float g_error = fabsf(vec_length(sample.accel) - vec_length(gravity));

	bool still_now = jitter < STILL_MAX_JITTER && g_error < STILL_MAX_G_ERROR && sample.angular_rate < STILL_MAX_RATE;
	still_time = still_now ? still_time + dt : 0;

	if (still_time >= STILL_MIN_TIME) {
		// At rest the true velocity is zero and whatever is left is gravity/bias/tilt error;
		// track it (this also follows the sensor as it slowly corrects its own tilt error)
		velocity = prev_accel = (vec3_t){ 0 };
		gravity = vec_add(gravity, vec_scale(accel, GRAVITY_TRACKING_RATE));
	} else {
		// Trapezoidal integration in the level frame
		vec3_t next_velocity = vec_add(velocity, vec_scale(vec_add(prev_accel, accel), 0.5f * dt));
		pose.position = vec_add(pose.position, vec_scale(vec_add(velocity, next_velocity), 0.5f * dt));
		velocity = next_velocity;
		prev_accel = accel;
	}

	return pose;
}

void imu_enable_dynamic_calibration(bool accel, bool gyro, bool mag)
{
	// P3 = 0: Configure ME Calibration subcommand
	uint8_t params[9] = { accel, gyro, mag, 0 };
	run_command(COMMAND_ME_CALIBRATION, params);
}

bool imu_save_calibration(void)
{
	uint8_t params[9] = { 0 };
	return run_command(COMMAND_SAVE_DCD, params) == 0;
}

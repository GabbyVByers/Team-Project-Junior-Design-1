
/*
 *   Source [imu.c]
 */

#include "imu.h"
#include <stdbool.h>
#include <stdint.h>
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

static i2c_master_dev_handle_t device;
static bool reset_done;

static quat_t latest_orientation = { 1, 0, 0, 0 };
static uint32_t accel_interval_us = REPORT_INTERVAL_US;  // actual rate, from the hub's feature response
static int last_accel_sequence = -1;

static imu_sample_t sample_queue[QUEUE_SIZE];
static int queue_head, queue_count;

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

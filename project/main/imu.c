
/*
 *   Source [imu.c]
 */

#include "imu.h"
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

#define SIZE_LINEAR_ACCELERATION 10
#define SIZE_GAME_ROTATION       12
#define SIZE_TIMESTAMP           5

#define QUEUE_SIZE 32

static i2c_master_dev_handle_t device;
static bool reset_done;

static float qw = 1, qx, qy, qz;                         // latest orientation
static uint32_t accel_interval_us = REPORT_INTERVAL_US;  // actual rate, from the hub's feature response
static int last_accel_seq = -1;

static imu_sample_t queue[QUEUE_SIZE];
static int queue_head, queue_count;

static bool data_pending(void)
{
  return gpio_get_level(PIN_INT) == 0;
}

static float read_q(const uint8_t *bytes, int q)
{
  int16_t raw = (int16_t)(bytes[0] | (bytes[1] << 8));
  return (float)raw / (float)(1 << q);
}

static void push_accel_sample(const uint8_t *r)
{
  // Sequence numbers count every report the hub produced, so gaps from dropped
  // reports still give the correct elapsed time
  uint8_t seq = r[1];
  uint8_t steps = (last_accel_seq < 0) ? 1 : (uint8_t)(seq - last_accel_seq);
  last_accel_seq = seq;

  if (queue_count == QUEUE_SIZE) { return; }

  queue[(queue_head + queue_count++) % QUEUE_SIZE] = (imu_sample_t){
    .w = qw, .x = qx, .y = qy, .z = qz,
    .ax = read_q(&r[4], 8),
    .ay = read_q(&r[6], 8),
    .az = read_q(&r[8], 8),
    .dt = steps * accel_interval_us * 1e-6f,
  };
}

static void parse_reports(const uint8_t *data, int length)
{
  int i = 0;
  while (i < length) {
    const uint8_t *r = &data[i];
    int size;

    switch (r[0]) {
    case REPORT_BASE_TIMESTAMP:
    case REPORT_TIMESTAMP_REBASE:
      size = SIZE_TIMESTAMP;
      break;
    case REPORT_GAME_ROTATION:
      size = SIZE_GAME_ROTATION;
      if (i + size > length) { return; }
      qx = read_q(&r[4],  14);
      qy = read_q(&r[6],  14);
      qz = read_q(&r[8],  14);
      qw = read_q(&r[10], 14);
      break;
    case REPORT_LINEAR_ACCELERATION:
      size = SIZE_LINEAR_ACCELERATION;
      if (i + size > length) { return; }
      push_accel_sample(r);
      break;
    default:
      return;
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
    if (data_pending()) {
      receive_packet();
    } else {
      vTaskDelay(1);
    }
  }

  enable_report(REPORT_GAME_ROTATION, REPORT_INTERVAL_US);
  enable_report(REPORT_LINEAR_ACCELERATION, REPORT_INTERVAL_US);
}

bool imu_read(imu_sample_t *out)
{
  while (data_pending()) {
    receive_packet();
  }

  if (queue_count == 0) { return false; }

  *out = queue[queue_head];
  queue_head = (queue_head + 1) % QUEUE_SIZE;
  queue_count--;
  return true;
}

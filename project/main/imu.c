
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
#define I2C_SPEED_HZ   100000
#define I2C_TIMEOUT_MS 100

#define REPORT_INTERVAL_US 20000  // 50 Hz

#define SHTP_HEADER_SIZE   4
#define SHTP_MAX_PACKET    512
#define CHANNEL_EXECUTABLE 1
#define CHANNEL_CONTROL    2
#define CHANNEL_REPORTS    3

#define CMD_SET_FEATURE    0xFD
#define EXEC_RESET_DONE    0x01

#define REPORT_ACCELEROMETER    0x01
#define REPORT_ROTATION_VECTOR  0x05
#define REPORT_TIMESTAMP_REBASE 0xFA
#define REPORT_BASE_TIMESTAMP   0xFB

#define SIZE_ACCELEROMETER     10
#define SIZE_ROTATION_VECTOR   14
#define SIZE_TIMESTAMP         5

static i2c_master_dev_handle_t device;
static imu_data_t latest;
static bool reset_done;

static bool data_pending(void)
{
  return gpio_get_level(PIN_INT) == 0;
}

static float read_q(const uint8_t *bytes, int q)
{
  int16_t raw = (int16_t)(bytes[0] | (bytes[1] << 8));
  return (float)raw / (float)(1 << q);
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
    case REPORT_ACCELEROMETER:
      size = SIZE_ACCELEROMETER;
      if (i + size > length) { return; }
      latest.ax = read_q(&r[4], 8);
      latest.ay = read_q(&r[6], 8);
      latest.az = read_q(&r[8], 8);
      break;
    case REPORT_ROTATION_VECTOR:
      size = SIZE_ROTATION_VECTOR;
      if (i + size > length) { return; }
      latest.x = read_q(&r[4],  14);
      latest.y = read_q(&r[6],  14);
      latest.z = read_q(&r[8],  14);
      latest.w = read_q(&r[10], 14);
      break;
    default:
      return;
    }
    
    i += size;
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
  
  enable_report(REPORT_ROTATION_VECTOR, REPORT_INTERVAL_US);
  enable_report(REPORT_ACCELEROMETER, REPORT_INTERVAL_US);
}

imu_data_t imu_read(void)
{
  while (data_pending()) {
    receive_packet();
  }
  return latest;
}


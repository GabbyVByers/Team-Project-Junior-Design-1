
/*
 *   Source [imu.c]
 */

#include "imu.h"
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"

#define I2C_SDA_PIN             GPIO_NUM_21
#define I2C_SCL_PIN             GPIO_NUM_22
#define IMU_INTERRUPT_PIN       GPIO_NUM_18
#define IMU_RESET_PIN           GPIO_NUM_19
#define IMU_I2C_ADDRESS         0x4A
#define I2C_SPEED_HZ            100000
#define I2C_TIMEOUT_MS          100

#define HEADER_SIZE             4
#define PACKET_SIZE             512
#define CHANNEL_EXECUTABLE      1
#define CHANNEL_CONTROL         2
#define CHANNEL_SENSOR_REPORTS  3

#define COMMAND_SET_FEATURE     0xFD
#define EXECUTABLE_RESET_DONE   0x01

#define REPORT_BASE_TIMESTAMP   0xFB
#define REPORT_ACCELEROMETER    0x01
#define REPORT_ROTATION_VECTOR  0x05
#define BASE_TIMESTAMP_SIZE     5

typedef i2c_master_dev_handle_t I2C_HANDLE_T;

imu_data_t imu_data = { 0 };

static bool         IMU_READY  = false;
static I2C_HANDLE_T IMU_DEVICE = NULL;


bool imu_has_data(void)
{
  return (gpio_get_level(IMU_INTERRUPT_PIN) == 0);
}

static int16_t parse_int16(const uint8_t *bytes)
{
    return (int16_t)(bytes[0] | (bytes[1] << 8));
}

static float fixed_float(int16_t whole, int fraction)
{
    return (float)whole / (float)(1 << fraction);
}

static void imu_enable_report(uint8_t id, uint32_t ms)
{
  static uint8_t num = 0;
  uint8_t packet[HEADER_SIZE + 17] = { 0 };
  packet[0] = sizeof(packet);
  packet[2] = CHANNEL_CONTROL;
  packet[3] = num++;

  uint8_t *command = &packet[HEADER_SIZE];
  command[0] = COMMAND_SET_FEATURE;
  command[1] = id;
  command[5] = ms & 0xFF;
  command[6] = (ms >> 8) & 0xFF;
  command[7] = (ms >> 16) & 0xFF;
  command[8] = (ms >> 24) & 0xFF;

  i2c_master_transmit(IMU_DEVICE, packet, sizeof(packet), I2C_TIMEOUT_MS);
}

void imu_receive_packet(void)
{
  static uint8_t header[HEADER_SIZE]; // 4
  static uint8_t packet[PACKET_SIZE]; // 512

  if (i2c_master_receive(IMU_DEVICE, header, sizeof(header), I2C_TIMEOUT_MS) != ESP_OK) { return; }

  uint16_t length = (header[0] | header[1] << 8) & 0x7FFF;
  if (length < HEADER_SIZE) { return; }
  if (length > PACKET_SIZE) { length = PACKET_SIZE; }

  if (i2c_master_receive(IMU_DEVICE, packet, length, I2C_TIMEOUT_MS) != ESP_OK) { return; }

  uint8_t  channel = packet[2];
  uint8_t* payload = &packet[HEADER_SIZE];
  length = length - HEADER_SIZE;

  if (channel == CHANNEL_SENSOR_REPORTS) {
    int i = 0;
    while (i < length) {
      const uint8_t *report = &payload[i];
      switch (report[0]) {
      case REPORT_BASE_TIMESTAMP:
        i += BASE_TIMESTAMP_SIZE;
        break;
      case REPORT_ACCELEROMETER: {
        const int acc_size = 10;
        if ((i + acc_size) > length) return;
        imu_data.ax = fixed_float(parse_int16(&report[4]), 8); // acc x
        imu_data.ay = fixed_float(parse_int16(&report[6]), 8); // acc y
        imu_data.az = fixed_float(parse_int16(&report[8]), 8); // acc z
        i += acc_size;
        break;
      }
      case REPORT_ROTATION_VECTOR: {
        const int quat_size = 14;
        if ((i + quat_size) > length) return;
        imu_data.x = fixed_float(parse_int16(&report[4]),  14); // i
        imu_data.y = fixed_float(parse_int16(&report[6]),  14); // j
        imu_data.z = fixed_float(parse_int16(&report[8]),  14); // k
        imu_data.w = fixed_float(parse_int16(&report[10]), 14); // real
        i += quat_size;
        break;
      }
      default:
        return;
      }
    }
  }

  bool exe_chan = channel == CHANNEL_EXECUTABLE;
  bool exe_reset = payload[0] == EXECUTABLE_RESET_DONE;
  if (exe_chan && exe_reset) {
    IMU_READY = true;
  }
}

void imu_init(void)
{
  gpio_config_t reset_pin_config = {
    .pin_bit_mask = 1ULL << IMU_RESET_PIN,
    .mode = GPIO_MODE_OUTPUT,
  };

  gpio_config(&reset_pin_config);

  gpio_config_t interrupt_pin_config = {
    .pin_bit_mask = 1ULL << IMU_INTERRUPT_PIN,
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_ENABLE,
  };

  gpio_config(&interrupt_pin_config);

  i2c_master_bus_config_t bus_config = {
    .i2c_port = I2C_NUM_0,
    .sda_io_num = I2C_SDA_PIN,
    .scl_io_num = I2C_SCL_PIN,
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
  };

  i2c_master_bus_handle_t i2c_bus;
  i2c_new_master_bus(&bus_config, &i2c_bus);

  i2c_device_config_t device_config = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = IMU_I2C_ADDRESS,
    .scl_speed_hz = I2C_SPEED_HZ,
  };

  i2c_master_bus_add_device(i2c_bus, &device_config, &IMU_DEVICE);

  gpio_set_level(IMU_RESET_PIN, 0);
  vTaskDelay(pdMS_TO_TICKS(10));
  gpio_set_level(IMU_RESET_PIN, 1);

  while (!IMU_READY) {
    if (imu_has_data()) {
      imu_receive_packet();
    } else {
      vTaskDelay(1);
    }
  }

  imu_enable_report(REPORT_ROTATION_VECTOR, 20000);
  imu_enable_report(REPORT_ACCELEROMETER, 20000);
}


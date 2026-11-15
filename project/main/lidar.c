
/*
 *   Source [lidar.c]
 */

#include "lidar.h"
#include <stdint.h>
#include "driver/uart.h"

#define LIDAR_UART     UART_NUM_2
#define PIN_TX         17  // TX2 -> TF-Luna RX
#define PIN_RX         16  // RX2 <- TF-Luna TX
#define BAUD_RATE      115200
#define RX_BUFFER_SIZE 512

#define FRAME_HEADER   0x59
#define FRAME_SIZE     9   // 0x59 0x59 DistL DistH AmpL AmpH TempL TempH Checksum

static lidar_data_t latest;
static uint8_t frame[FRAME_SIZE];
static int frame_pos;

static void parse_frame(void)
{
  uint8_t sum = 0;
  for (int i = 0; i < FRAME_SIZE - 1; i++) { sum += frame[i]; }
  if (sum != frame[FRAME_SIZE - 1]) { return; }

  latest.distance_cm = frame[2] | (frame[3] << 8);
  latest.amplitude   = frame[4] | (frame[5] << 8);
  latest.temp_c      = (frame[6] | (frame[7] << 8)) / 8.0f - 256.0f;
}

static void receive_byte(uint8_t byte)
{
  if (frame_pos < 2 && byte != FRAME_HEADER) {
    frame_pos = 0;
    return;
  }

  frame[frame_pos++] = byte;

  if (frame_pos == FRAME_SIZE) {
    parse_frame();
    frame_pos = 0;
  }
}

void lidar_init(void)
{
  uart_config_t cfg = {
    .baud_rate  = BAUD_RATE,
    .data_bits  = UART_DATA_8_BITS,
    .parity     = UART_PARITY_DISABLE,
    .stop_bits  = UART_STOP_BITS_1,
    .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_DEFAULT,
  };
  ESP_ERROR_CHECK(uart_driver_install(LIDAR_UART, RX_BUFFER_SIZE, 0, 0, NULL, 0));
  ESP_ERROR_CHECK(uart_param_config(LIDAR_UART, &cfg));
  ESP_ERROR_CHECK(uart_set_pin(LIDAR_UART, PIN_TX, PIN_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

lidar_data_t lidar_read(void)
{
  uint8_t chunk[64];
  int count;

  while ((count = uart_read_bytes(LIDAR_UART, chunk, sizeof(chunk), 0)) > 0) {
    for (int i = 0; i < count; i++) { receive_byte(chunk[i]); }
  }
  return latest;
}

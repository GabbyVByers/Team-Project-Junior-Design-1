
/*
 *   Source [main.c]
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lidar.h"

void app_main(void)
{
  lidar_init();
  printf("Lidar Ready\n");

  while (true) {
    vTaskDelay(pdMS_TO_TICKS(10));

    lidar_data_t d = lidar_read();
    printf("Dist: %4d cm  Amp: %5d  Temp: %.1f C\n",
           d.distance_cm, d.amplitude, d.temp_c);
  }
}

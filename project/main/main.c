
/*
 *   Source [main.c]
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "imu.h"

void app_main(void)
{
  imu_init();
  printf("IMU Ready\n");

  while (true) {
    vTaskDelay(pdMS_TO_TICKS(10));

    imu_data_t d = imu_read();
    printf("Quat: [%6.3f, %6.3f, %6.3f, %6.3f] Acc: [%6.2f %6.2f %6.2f]\n",
           d.w, d.x, d.y, d.z, d.ax, d.ay, d.az);
  }
}


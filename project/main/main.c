
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

    while (imu_has_data()) {
      imu_receive_packet();
    }

    float w = imu_data.w;
    float x = imu_data.x;
    float y = imu_data.y;
    float z = imu_data.z;

    float ax = imu_data.ax;
    float ay = imu_data.ay;
    float az = imu_data.az;

    printf("Quat: [%6.3f, %6.3f, %6.3f, %6.3f] Acc: [%6.2f %6.2f %6.2f]\n", w, x, y, z, ax, ay, az);
  }
}


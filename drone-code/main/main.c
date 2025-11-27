#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "imu.h"



void app_main(void)
{
    imu_i2c_init();

    if (imu_init() != 0) {
        printf("IMU init FAILED\n");
        return;
    }

    xTaskCreate(imu_task, "imu_task", 4096, NULL, 5, NULL);
}

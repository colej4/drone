#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "imu.h"
#include "state_estimator.h"

#include "driver/timer.h"


#define TIMER_DIVIDER         80  //  Hardware timer clock divider

void init_timer()
{
    /* Select and initialize basic parameters of the timer */
    timer_config_t config = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .auto_reload = true,
    }; // default clock source is APB
    timer_init(TIMER_GROUP_0, TIMER_0, &config);

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);


    timer_start(TIMER_GROUP_0, TIMER_0);
}

void app_main(void)
{

    QueueHandle_t imu_data_queue = xQueueCreate(10, sizeof(timestamped_imu_data_t));
    imu_i2c_init();

    if (imu_init() != 0) {
        printf("IMU init FAILED\n");
        return;
    }

    init_timer();

    xTaskCreate(imu_task, "imu_task", 4096, (void*)&imu_data_queue, 5, NULL);
    xTaskCreate(state_estimator_task, "imu_task", 4096, (void*)&imu_data_queue, 5, NULL);
}
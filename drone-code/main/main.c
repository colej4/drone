#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "imu.h"
#include "state_estimator.h"
#include "math_helpers.h"
#include "ibus_protocol.h"
#include "control.h"

#include "driver/timer.h"


#define TIMER_DIVIDER         80  //  Hardware timer clock divider

#define HOME_ESC 0

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
    QueueHandle_t state_estimate_mailbox = xQueueCreate(1, sizeof(Vector3));
    QueueHandle_t ibus_mailbox = xQueueCreate(1, sizeof(IbusMessage));

    StateEstimatorConfig state_estimator_config = {
        .imu_data_queue = imu_data_queue,
        .state_estimate_mailbox = state_estimate_mailbox
    };

    imu_i2c_init();

    while (imu_init() != 0) {
        printf("IMU init FAILED\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    printf("IMU initialized successfully\n");

    init_timer();

    ibus_uart_init();

    xTaskCreate(ibus_task, "ibus_task", 4096, (void*)&ibus_mailbox, 5, NULL);
    xTaskCreate(imu_task, "imu_task", 4096, (void*)&imu_data_queue, 5, NULL);
    xTaskCreate(state_estimator_task, "state_estimator_task", 4096, (void*)&state_estimator_config, 5, NULL);
    #if HOME_ESC
    xTaskCreate(esc_home_task, "esc_home_task", 4096, (void*)&(ControlConfig){
        .ibus_mailbox = ibus_mailbox,
        .state_estimate_mailbox = state_estimate_mailbox
    }, 5, NULL);
    #else
    xTaskCreate(control_task, "control_task", 4096, (void*)&(ControlConfig){
        .ibus_mailbox = ibus_mailbox,
        .state_estimate_mailbox = state_estimate_mailbox
    }, 5, NULL);
    #endif
}
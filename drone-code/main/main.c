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

#include "wifi.h"
#include "wifi_logger.h"

//esp logging
#include "esp_log.h"
static const char* TAG = "main";

#define TIMER_DIVIDER         80  //  Hardware timer clock divider

#define HOME_ESC 0

#define WIFI_LOGGING 1



void init_timer()
{
    timer_config_t config = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .auto_reload = true,
    }; // default clock source is APB
    timer_init(TIMER_GROUP_0, TIMER_0, &config);

    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);


    timer_start(TIMER_GROUP_0, TIMER_0);
}

void app_main(void)
{
    //set log levels
    esp_log_level_set("*", ESP_LOG_VERBOSE);

    #if WIFI_LOGGING
    wifi_init_sta();
    // wait_for_wifi();
    log_stream_init_udp("192.168.0.113", 9000);
    #endif

    QueueHandle_t imu_data_queue = xQueueCreate(10, sizeof(timestamped_imu_data_t));
    QueueHandle_t state_estimate_mailbox = xQueueCreate(1, sizeof(Vector3));
    QueueHandle_t ibus_mailbox = xQueueCreate(1, sizeof(IbusMessage));

    StateEstimatorConfig state_estimator_config = {
        .imu_data_queue = imu_data_queue,
        .state_estimate_mailbox = state_estimate_mailbox
    };

    imu_i2c_init();

    while (imu_init() != 0) {
        ESP_LOGE(TAG, "IMU init FAILED");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_LOGI(TAG, "IMU initialized successfully");
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
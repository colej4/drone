#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "imu.hpp"
#include "state_estimator.hpp"
#include "math_helpers.hpp"
#include "ibus_protocol.hpp"
#include "control.hpp"
#include "driver/timer.h"

#include "wifi.hpp"
#include "wifi_logger.hpp"
#include "sd.hpp"
//esp logging
#include "esp_log.h"

static const char* TAG = "main";

#define TIMER_DIVIDER         80  //  Hardware timer clock divider

#define HOME_ESC 0

#define WIFI_LOGGING 0

#define SD_LOGGING 1



void init_timer()
{
    timer_config_t config = {};
    config.divider = TIMER_DIVIDER;
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en = TIMER_PAUSE;
    config.auto_reload = TIMER_AUTORELOAD_EN;
    // default clock source is APB
    timer_init(TIMER_GROUP_0, TIMER_0, &config);

    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);


    timer_start(TIMER_GROUP_0, TIMER_0);
}

extern "C" void app_main(void)
{
    //set log levels
    esp_log_level_set("*", ESP_LOG_INFO);
    //prevent spam in logs from sd card operation
    esp_log_level_set("control", ESP_LOG_VERBOSE);
    esp_log_level_set("state_estimator", ESP_LOG_VERBOSE);




    #if WIFI_LOGGING
    #if SD_LOGGING
    ESP_LOGE(TAG, "Cannot enable both WIFI_LOGGING and SD_LOGGING at the same time.");
    return;
    #endif
    wifi_init_sta();
    // wait_for_wifi();
    log_stream_init_udp("192.168.0.113", 9000);
    #endif
    #if SD_LOGGING
    ESP_ERROR_CHECK(mount_sd_and_start_logging());
    #endif

    

    QueueHandle_t imu_data_queue = xQueueCreate(10, sizeof(timestamped_imu_data_t));
    QueueHandle_t state_estimate_mailbox = xQueueCreate(1, sizeof(Vector3));
    QueueHandle_t ibus_mailbox = xQueueCreate(1, sizeof(IbusMessage));

    StateEstimatorConfig state_estimator_config = {};
    state_estimator_config.imu_data_queue = imu_data_queue;
    state_estimator_config.state_estimate_mailbox = state_estimate_mailbox;

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
    ControlConfig controlconfig = {};
    controlconfig.ibus_mailbox = ibus_mailbox;
    controlconfig.state_estimate_mailbox = state_estimate_mailbox;
    #if HOME_ESC
    xTaskCreate(esc_home_task, "esc_home_task", 4096, (void*)&controlconfig, 5, NULL);
    #else
    xTaskCreate(control_task, "control_task", 4096, (void*)&controlconfig, 5, NULL);
    #endif
}
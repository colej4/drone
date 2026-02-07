#ifndef STATE_ESTIMATOR_H
#define STATE_ESTIMATOR_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "math_helpers.hpp"

void state_estimator_task(void *arg);

typedef struct {
    QueueHandle_t imu_data_queue;
    QueueHandle_t state_estimate_mailbox;
} StateEstimatorConfig;

typedef struct {
    Vector3 orientation; // roll, pitch, yaw in radians
    Vector3 euler_rates; // angular rates in rad/s
} StateEstimate;

#endif
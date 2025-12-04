#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "imu.h"
#include "math_helpers.h"
#include "state_estimator.h"

void state_estimator_task(void *arg) {

    Vector3 orientation = {0.0f, 0.0f, 0.0f};

    StateEstimatorConfig* config = (StateEstimatorConfig*)arg;
    QueueHandle_t imu_data_queue = config->imu_data_queue;
    QueueHandle_t state_estimate_mailbox = config->state_estimate_mailbox;

    uint64_t last_timestamp = 0;
    int valid_accel_count = 0;

    float accel_roll = 0.0f, accel_pitch = 0.0f;

    while (1) {
        timestamped_imu_data_t ts_imu_data;
        if (xQueueReceive(imu_data_queue, &ts_imu_data, portMAX_DELAY) == pdTRUE) {
            imu_data_t imu_data = ts_imu_data.data;
            uint64_t timestamp = ts_imu_data.timestamp;
            if (last_timestamp == 0) {
                last_timestamp = timestamp;
                continue;
            }
            float dt = (float)(timestamp - last_timestamp) / 1e6f;
            // Simple state estimation logic (placeholder)
            orientation.x += imu_data.gx * dt; // assuming 1 ms timestep
            orientation.y += imu_data.gy * dt;
            orientation.z += imu_data.gz * dt;

            
            // Complementary filter with accelerometer
            accel_roll = 0.95 * accel_roll + 0.05 * atan2f(imu_data.ay, imu_data.az);
            accel_pitch = 0.95 * accel_pitch + 0.05 * atan2f(-imu_data.ax, sqrtf(imu_data.ay * imu_data.ay + imu_data.az * imu_data.az));


            float accel_mag = sqrtf(imu_data.ax * imu_data.ax + imu_data.ay * imu_data.ay + imu_data.az * imu_data.az);
            if (fabsf(accel_mag - G) < 0.3f && fabsf(accel_roll) < (0.1f) && fabsf(accel_pitch) < (0.1f)) {
                valid_accel_count++;
            } else {
                valid_accel_count = 0;
            }

            if (valid_accel_count > 10) {
                // Fuse accelerometer data
                orientation.x = 0.9997 * orientation.x + 0.0003 * accel_roll;
                orientation.y = 0.9997 * orientation.y + 0.0003 * accel_pitch;
            }

            last_timestamp = timestamp;

            xQueueOverwrite(state_estimate_mailbox, &orientation);

            // if (timestamp % 10 == 0 ) {
                // Print estimated orientation
                // printf("Estimated Orientation - Roll: %f, Pitch: %f, Yaw: %f @ %f seconds\n",
                //     orientation.x, orientation.y, orientation.z, (float)timestamp / 1e6f);
                // printf("Accel Roll: %f, Accel Pitch: %f\n", accel_roll, accel_pitch);
            // }
        }
    }
}

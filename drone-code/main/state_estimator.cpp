#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "imu.hpp"
#include "math_helpers.hpp"
#include "state_estimator.hpp"
#include "ekf.hpp"
#include "ekf_estimator.hpp"

//esp logging
#include "esp_log.h"
static const char* TAG = "state_estimator";

#define WINDOW_SIZE 4

void state_estimator_task(void *arg) {

    Vector3 orientation = {0.0f, 0.0f, 0.0f};
    Vector3 euler_rates[WINDOW_SIZE]{}; // moving average buffer for gyro rates
    int gyro_rate_index = 0;
    

    StateEstimatorConfig* config = (StateEstimatorConfig*)arg;
    QueueHandle_t imu_data_queue = config->imu_data_queue;
    QueueHandle_t state_estimate_mailbox = config->state_estimate_mailbox;

    uint64_t last_timestamp = 0;
    int valid_accel_count = 0;

    float accel_roll = 0.0f, accel_pitch = 0.0f;

    Eigen::Matrix<float, 7, 7> initial_covariance = Eigen::Matrix<float, 7, 7>::Identity() * 0.1f;
    Eigen::Matrix<float, 7, 1> initial_mean = (Eigen::Matrix<float, 7, 1>){1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    EKFStateEstimator* ekf_estimator =
        new EKFStateEstimator(
            initial_mean,
            initial_covariance,
            0
        );



    while (1) {
        timestamped_imu_data_t ts_imu_data;
        if (xQueueReceive(imu_data_queue, &ts_imu_data, portMAX_DELAY) == pdTRUE) {
            imu_data_t imu_data = ts_imu_data.data;
            uint64_t timestamp = ts_imu_data.timestamp;
            if (last_timestamp == 0) {
                last_timestamp = timestamp;
                ekf_estimator->last_gyro_timestamp_us = timestamp;
                continue;
            }
            float dt = (float)(timestamp - last_timestamp) / 1e6f;
            // Simple state estimation logic (placeholder)
            orientation.x += imu_data.gx * dt; // assuming 1 ms timestep
            orientation.y += imu_data.gy * dt;
            orientation.z += imu_data.gz * dt;

            Eigen::Vector3f predict_input = Eigen::Vector3f(imu_data.gx, imu_data.gy, imu_data.gz);
            ekf_estimator->predict(predict_input, timestamp);

            euler_rates[gyro_rate_index].x = imu_data.gx;
            euler_rates[gyro_rate_index].y = imu_data.gy;
            euler_rates[gyro_rate_index].z = imu_data.gz;
            gyro_rate_index = (gyro_rate_index + 1) % WINDOW_SIZE;
            Vector3 average_euler_rates = {0.0f, 0.0f, 0.0f};
            for (int i = 0; i < WINDOW_SIZE; i++) {
                average_euler_rates.x += euler_rates[i].x;
                average_euler_rates.y += euler_rates[i].y;
                average_euler_rates.z += euler_rates[i].z;
            }
            average_euler_rates.x /= WINDOW_SIZE;
            average_euler_rates.y /= WINDOW_SIZE;
            average_euler_rates.z /= WINDOW_SIZE;

            // Complementary filter with accelerometer
            accel_roll = 0.95 * accel_roll + 0.05 * atan2f(imu_data.ay, imu_data.az);
            accel_pitch = 0.95 * accel_pitch + 0.05 * atan2f(-imu_data.ax, sqrtf(imu_data.ay * imu_data.ay + imu_data.az * imu_data.az));


            float accel_mag = sqrtf(imu_data.ax * imu_data.ax + imu_data.ay * imu_data.ay + imu_data.az * imu_data.az);
            if (fabsf(accel_mag - GRAVITATIONAL_ACCELERATION) < 0.3f && fabsf(accel_roll) < (0.1f) && fabsf(accel_pitch) < (0.1f)) {
                valid_accel_count++;
            } else {
                valid_accel_count = 0;
            }

            if (valid_accel_count > 10) {
                // Fuse accelerometer data
                ekf_estimator->accelerometer_update(Eigen::Vector3f(imu_data.ax, imu_data.ay, imu_data.az));
                orientation.x = 0.9997 * orientation.x + 0.0003 * accel_roll;
                orientation.y = 0.9997 * orientation.y + 0.0003 * accel_pitch;
            }

            StateEstimate state_estimate;
            state_estimate.orientation = orientation;
            state_estimate.euler_rates = average_euler_rates;
            state_estimate.timestamp = timestamp;

            last_timestamp = timestamp;

            xQueueOverwrite(state_estimate_mailbox, &state_estimate);

            Vector3 ekf_orientation = ekf_estimator->get_orientation_euler();

            ESP_LOGV(TAG, "Roll: %f, Pitch: %f, Yaw: %f", orientation.x, orientation.y, orientation.z);

            ESP_LOGV(TAG, "EKF Roll: %f, EKF Pitch: %f, EKF Yaw: %f", ekf_orientation.x, ekf_orientation.y, ekf_orientation.z);
            
            // ESP_LOGV(TAG, "Orientation - Roll: %f, Pitch: %f, Yaw: %f | Euler Rates - gx: %f, gy: %f, gz: %f",
            //          orientation.x, orientation.y, orientation.z,
            //          euler_rates.x, euler_rates.y, euler_rates.z);

        }
    }
}

/*
* implementation of PID controller for controlling motor speed,
* takes setpoint and measurement values from an input queue,
* and provides output to another queue.
*/

#include <stdio.h>
#include <stdint.h> 
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/timer.h"

#include "pid.hpp"

//esp logging
#include "esp_log.h"
static const char* TAG = "pid";

PIDController* new_pid(float kP, float kI, float kD, float integral_bound, float derivative_ema_gain, float initial_setpoint, uint64_t timestamp) {
    PIDController* controller = (PIDController*) malloc(sizeof(PIDController));
    if (controller == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for PIDController");
        return NULL;
    }
    controller->kP = kP;
    controller->kI = kI;
    controller->kD = kD;
    controller->integral = 0.0f;
    controller->integral_bound = integral_bound;
    controller->derivative_ema_gain = derivative_ema_gain;
    controller->filtered_derivative = 0.0f;
    controller->previous_error = 0.0f;
    controller->previous_timestamp = timestamp;
    controller->setpoint = initial_setpoint;
    return controller;
}

float calculate_pid(PIDController* controller, float measurement, uint64_t timestamp) {
    float error = controller->setpoint - measurement;
    float elapsed_time = (float)(timestamp - controller->previous_timestamp) / 1e6f; //it is assumed timestamp is in micros
    if (elapsed_time > 1e-6f) {
        //compute proportional term
        float proportional_term = controller->kP * error;
        //compute integral term
        controller->integral += error * elapsed_time;
        //constrain integral term so |integral| < bound
        if (controller->integral > controller->integral_bound) {
            controller->integral = controller->integral_bound;
        }
        if (controller->integral < -1.0f * controller->integral_bound) {
            controller->integral = -1.0f * controller->integral_bound;
        }
        float integral_term = controller->kI * controller->integral;
        //compute derivative term
        float raw_derivative = (error - controller->previous_error) / elapsed_time;
        controller->filtered_derivative = controller->derivative_ema_gain * raw_derivative + (1.0 - controller->derivative_ema_gain) * controller->filtered_derivative;
        float derivative_term = controller->kD * controller->filtered_derivative;
        //update timestamp and error
        controller->previous_error = error;
        controller->previous_timestamp = timestamp;

        return proportional_term + integral_term + derivative_term;
    } else {
        return controller->kP * error + controller->kI * controller->integral;
    }
}

float calculate_pid_with_err(PIDController* controller, float error, uint64_t timestamp) {
    float elapsed_time = (float)(timestamp - controller->previous_timestamp) / 1e6f; //it is assumed timestamp is in micros
    if (elapsed_time > 1e-6f) {
        //compute proportional term
        float proportional_term = controller->kP * error;
        //compute integral term
        controller->integral += error * elapsed_time;
        //constrain integral term so |integral| < bound
        if (controller->integral > controller->integral_bound) {
            controller->integral = controller->integral_bound;
        }
        if (controller->integral < -1.0f * controller->integral_bound) {
            controller->integral = -1.0f * controller->integral_bound;
        }
        float integral_term = controller->kI * controller->integral;
        //compute derivative term
        float raw_derivative = (error - controller->previous_error) / elapsed_time;
        controller->filtered_derivative = controller->derivative_ema_gain * raw_derivative + (1.0 - controller->derivative_ema_gain) * controller->filtered_derivative;
        float derivative_term = controller->kD * controller->filtered_derivative;
        //update timestamp and error
        controller->previous_error = error;
        controller->previous_timestamp = timestamp;

        return proportional_term + integral_term + derivative_term;
    } else {
        return controller->kP * error + controller->kI * controller->integral;
    }
}


void vPID_controller(void *pvParameters) {
    //Read parameters and set initial values
    const TickType_t max_wait = pdMS_TO_TICKS(5);
    PIDConfig* pid_config = (PIDConfig *) pvParameters;
    uint64_t initial_timestamp;
    timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &initial_timestamp);
    PIDController* pid_controller = new_pid(
        pid_config->kP,
        pid_config->kI,
        pid_config->kD,
        pid_config->integral_bound,
        pid_config->derivative_ema_gain,
        pid_config->initial_setpoint,
        initial_timestamp//initial timestamp
    );
    //Status check for queues
    BaseType_t status;

    while(1){
        PIDMessage received_message;
        status = xQueueReceive(pid_config->input_queue, &received_message, max_wait);
        if (status) {
            if (received_message.type == SETPOINT) {
                pid_controller->setpoint = received_message.data;
            } else if (received_message.type == MEASUREMENT) {
                uint64_t timestamp;
                timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &timestamp);
                float output = calculate_pid(pid_controller, received_message.data, timestamp);
                //send output to output queue
                status = xQueueOverwrite(pid_config->output_mailbox, &output);
                if (!status) {
                    ESP_LOGE(TAG, "Failed to send PID output to queue in PID %d", pid_config->unique_id);
                }
            } else {
                ESP_LOGE(TAG, "Invalid Message Type in PID %d: %d", pid_config->unique_id, received_message.type);
            }
        } else {
            ESP_LOGW(TAG, "Failed to read data from queue (block time expired) in PID %d", pid_config->unique_id);
        }

    }
}
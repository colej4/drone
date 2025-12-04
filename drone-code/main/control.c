#include <stdio.h>
#include <stdint.h> 
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/timer.h"

#include "feedforward.h"
#include "control.h"
#include "pid.h"
#include "math_helpers.h"
#include "ibus_protocol.h"

#define MOTOR0_PIN 0
#define MOTOR1_PIN 1
#define MOTOR2_PIN 2
#define MOTOR3_PIN 3

#define KP 0.0
#define KI 0.0
#define KD 0.0
#define INTEGRAL_BOUND 0.0
#define DERIVATIVE_EMA_GAIN 0.8

#define CONTROLLER_YAW_SENSITIVITY 0.1

void control_task(void* arg) {

    //last_wake_time to ensure fixed frequency with vTaskDelayUntil
    TickType_t last_wake_time = xTaskGetTickCount();

    ControlConfig* config = (ControlConfig*)arg;
    QueueHandle_t ibus_mailbox = config->ibus_mailbox;
    QueueHandle_t state_estimate_mailbox = config-> state_estimate_mailbox;

    uint64_t initial_timestamp;
    uint64_t last_print_timestamp = 0;

    timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &initial_timestamp);
    PIDController* pid_controllers[2]; //roll and pitch controllers


    for(int i = 0; i < 2; i++) {
        pid_controllers[i] = new_pid(
        KP,
        KI,
        KD,
        INTEGRAL_BOUND,
        DERIVATIVE_EMA_GAIN,
        0.0,
        initial_timestamp//initial timestamp
    );
    
    }
    while(1) {
        //get input from RC transmitter
        IbusMessage controller_input;

        xQueuePeek(ibus_mailbox, &controller_input, 0);

        //get current state estimate
        Vector3 orientation_euler;

        xQueuePeek(state_estimate_mailbox, &orientation_euler, 0);

        //Convert RC transmitter input to reference orientation
        Quaternion ref_quat_headingless  = joystick_inputs_to_ref_quat_headingless(&controller_input);

        Quaternion current_orientation_quat = euler_to_quat(orientation_euler);

        Vector3 euler_error = euler_error_from_quats(ref_quat_headingless, current_orientation_quat);

        uint64_t timestamp;
        timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &timestamp);

        float roll_moment = calculate_pid_with_err(pid_controllers[0], euler_error.x, timestamp);
        float pitch_moment = calculate_pid_with_err(pid_controllers[1], euler_error.y, timestamp);
        float yaw_moment = controller_input.yaw * CONTROLLER_YAW_SENSITIVITY;
        float local_z_force = joystick_input_to_global_thrust(&controller_input) * thrust_multiplier_from_quat(current_orientation_quat);

        float control_inputs[4];

        calculate_control_input_from_moments(control_inputs, roll_moment, pitch_moment, yaw_moment, local_z_force);

        if(timestamp - last_print_timestamp > 500000) {
            last_print_timestamp = timestamp;
            printf("Moments & Forces: %0.3f roll, %0.3f pitch, %0.3f yaw, %0.3f thrust", roll_moment, pitch_moment, yaw_moment, local_z_force);
            printf("Voltages: %0.3f, %0.3f, %0.3f, %0.3f", control_inputs[0], control_inputs[1], control_inputs[2], control_inputs[3]);
        }

        xTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(1));
    }

}
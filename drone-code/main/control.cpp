#include <stdio.h>
#include <stdint.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/timer.h"
#include "driver/ledc.h"
#include "esp_err.h"

#include "feedforward.hpp"
#include "control.hpp"
#include "pid.hpp"
#include "math_helpers.hpp"
#include "ibus_protocol.hpp"
#include "state_estimator.hpp"

//esp logging
#include "esp_log.h"
static const char* TAG = "control";

// Motor GPIOs
#define MOTOR0_PIN 25
#define MOTOR1_PIN 26
#define MOTOR2_PIN 27
#define MOTOR3_PIN 14

// ESC / LEDC configuration
#define ESC_HZ              250
#define LEDC_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_TIMER          LEDC_TIMER_0
#define LEDC_RES            LEDC_TIMER_16_BIT

#define ESC_MIN_US          1000
#define ESC_MAX_US          2000
// Control configuration
#define KP_VEL 0.2
#define KI_VEL 0.08
#define KD_VEL 0.005
#define INTEGRAL_BOUND 0.15
#define DERIVATIVE_EMA_GAIN 1.0

#define KP_VEL_YAW 0.1
#define KI_VEL_YAW 0.0
#define KD_VEL_YAW 0.0

#define BATTERY_VOLTAGE 11.1f
#define CONTROLLER_YAW_SENSITIVITY 0.5f

#define KP_POS 18.0f // P controller to convert from angle to angular rate
#define KD_POS 0.0f

static bool full_estop = false;


static inline float clampf(float x, float lo, float hi)
{
    return (x < lo) ? lo : (x > hi) ? hi : x;
}

static inline uint32_t esc_us_to_duty(uint32_t pulse_us)
{
    const uint32_t period_us = 1000000UL / ESC_HZ;
    const uint32_t max_duty  = (1UL << LEDC_TIMER_16_BIT) - 1;  // 65535

    if (pulse_us > period_us) pulse_us = period_us;
    return (pulse_us * max_duty) / period_us;
}

static void esc_ledc_init(void)
{
    // Timer
    ledc_timer_config_t tcfg = {};
    tcfg.speed_mode       = LEDC_MODE;
    tcfg.timer_num        = LEDC_TIMER;
    tcfg.duty_resolution  = LEDC_RES;
    tcfg.freq_hz          = ESC_HZ;
    tcfg.clk_cfg          = LEDC_AUTO_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(&tcfg));

    const int gpios[4] = { MOTOR0_PIN, MOTOR1_PIN, MOTOR2_PIN, MOTOR3_PIN };
    const ledc_channel_t chs[4] = { LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2, LEDC_CHANNEL_3 };

    for (int i = 0; i < 4; i++) {
        ledc_channel_config_t ccfg = {};
        ccfg.gpio_num   = gpios[i];
        ccfg.speed_mode = LEDC_MODE;
        ccfg.channel    = chs[i];
        ccfg.timer_sel  = LEDC_TIMER;
        ccfg.duty       = 0;
        ccfg.hpoint     = 0;
        ccfg.intr_type  = LEDC_INTR_DISABLE;
        ESP_ERROR_CHECK(ledc_channel_config(&ccfg));
    }

    uint32_t duty_min = esc_us_to_duty(ESC_MIN_US);
    for (int i = 0; i < 4; i++) {
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, chs[i], duty_min));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, chs[i]));
    }
}

static inline void esc_write_us_4(const uint32_t us[4])
{
    const ledc_channel_t chs[4] = { LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2, LEDC_CHANNEL_3 };
    for (int i = 0; i < 4; i++) {
        uint32_t duty = esc_us_to_duty(us[i]);
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, chs[i], duty));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, chs[i]));
    }
}

void control_task(void* arg)
{
    // Initialize ESC PWM
    esc_ledc_init();

    TickType_t last_wake_time = xTaskGetTickCount();

    ControlConfig* config = (ControlConfig*)arg;
    QueueHandle_t ibus_mailbox = config->ibus_mailbox;
    QueueHandle_t state_estimate_mailbox = config->state_estimate_mailbox;

    uint64_t initial_timestamp;
    uint64_t last_print_timestamp = 0;

    timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &initial_timestamp);

    PIDController* pid_controllers[3]; // roll, pitch, yaw
    for (int i = 0; i < 3; i++) {
        pid_controllers[i] = new_pid(
            KP_VEL, KI_VEL, KD_VEL,
            INTEGRAL_BOUND,
            DERIVATIVE_EMA_GAIN,
            0.0,
            initial_timestamp
        );
    }
    pid_controllers[2]->kP = KP_VEL_YAW; // reduce P gain for yaw
    pid_controllers[2]->kI = KI_VEL_YAW; // reduce I gain for yaw
    pid_controllers[2]->kD = KD_VEL_YAW; // reduce D gain for yaw


    PIDController* pos_pid_controllers[2]; // roll, pitch position to rate
    for (int i = 0; i < 2; i++) {
        pos_pid_controllers[i] = new_pid(
            KP_POS, 0.0f, KD_POS,
            INTEGRAL_BOUND,
            DERIVATIVE_EMA_GAIN,
            0.0,
            initial_timestamp
        );
    }



    while (1) {
        // Get input from RC transmitter
        IbusMessage controller_input = {};
        (void)xQueuePeek(ibus_mailbox, &controller_input, 0);

        // Get current state estimate
        StateEstimate state_estimate = {};
        (void)xQueuePeek(state_estimate_mailbox, &state_estimate, 0);
        Vector3 orientation_euler = state_estimate.orientation;
        Vector3 euler_rates = state_estimate.euler_rates;

        ESP_LOGV(TAG, "Controller input: vra: %f, vrb: %f, roll: %f, pitch: %f, yaw: %f, throttle: %f",
                 controller_input.vra,
                 controller_input.vrb,
                 controller_input.roll,
                 controller_input.pitch,
                 controller_input.yaw,
                 controller_input.throttle);

        // Convert RC transmitter input to reference orientation
        Quaternion ref_quat_headingless = joystick_inputs_to_ref_quat_headingless(&controller_input);
        Quaternion current_orientation_quat = euler_to_quat(orientation_euler);
        Vector3 ref_euler = quat_to_euler(ref_quat_headingless);
        ref_euler.x = wrap_angle_pi(ref_euler.x);
        ref_euler.y = wrap_angle_pi(ref_euler.y);
        ref_euler.z = wrap_angle_pi(ref_euler.z);

        Vector3 euler_error;
        euler_error.x = wrap_angle_pi(ref_euler.x - orientation_euler.x);
        euler_error.y = wrap_angle_pi(ref_euler.y - orientation_euler.y);
        euler_error.z = wrap_angle_pi(ref_euler.z - orientation_euler.z);

        uint64_t timestamp;
        timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &timestamp);

        float roll_rate_setpoint = calculate_pid_with_err_and_derivative(pos_pid_controllers[0], euler_error.x, -euler_rates.x, timestamp);
        float pitch_rate_setpoint = calculate_pid_with_err_and_derivative(pos_pid_controllers[1], euler_error.y, -euler_rates.y, timestamp);
        // Yaw rate setpoint directly from joystick
        float yaw_rate_setpoint = controller_input.yaw * CONTROLLER_YAW_SENSITIVITY;

        float roll_rate_error = roll_rate_setpoint - euler_rates.x;
        float pitch_rate_error = pitch_rate_setpoint - euler_rates.y;
        float yaw_rate_error = yaw_rate_setpoint - euler_rates.z;

        float roll_moment  = calculate_pid_with_err(pid_controllers[0], roll_rate_error, timestamp);
        float pitch_moment = calculate_pid_with_err(pid_controllers[1], pitch_rate_error, timestamp);
        float yaw_moment   = calculate_pid_with_err(pid_controllers[2], yaw_rate_error, timestamp);

        float local_z_force =
            joystick_input_to_global_thrust(&controller_input) *
            thrust_multiplier_from_quat(current_orientation_quat);

        float control_inputs[4];
        calculate_control_input_from_moments(control_inputs, roll_moment, pitch_moment, yaw_moment, local_z_force);

        // Map "voltage commands" -> throttle fraction -> ESC microseconds
        // Assumes control_inputs[i] is in [0..BATTERY_VOLTAGE].
        uint32_t motor_us[4];
        for (int i = 0; i < 4; i++) {
            float v = clampf(control_inputs[i], 0.0f, BATTERY_VOLTAGE);
            float throttle = v / BATTERY_VOLTAGE;               // 0..1

            float us_f = (float)ESC_MIN_US + throttle * (float)(ESC_MAX_US - ESC_MIN_US);
            motor_us[i] = (uint32_t)(us_f + 0.5f);              // round to nearest
        }

        bool emergency_stop = false;
        if (controller_input.vra < 0.5f) {
            emergency_stop = true;
        }
        if (timestamp - state_estimate.timestamp > 20000) {
            //20 ms timeout for estop from stale state estimate
            emergency_stop = true;
        }
        if (timestamp - controller_input.timestamp > 100000) {
            //100 ms timeout for estop from no controller input
            emergency_stop = true;
        }

        if (fabs(orientation_euler.x) > M_PI / 4 || fabs(orientation_euler.y) > M_PI / 4) {
            //estop if we are tilted more than 45 degrees in any direction, likely indicates a crash
            full_estop = true;
        }
        if (emergency_stop || full_estop) {
            motor_us[0] = ESC_MIN_US;
            motor_us[1] = ESC_MIN_US;
            motor_us[2] = ESC_MIN_US;
            motor_us[3] = ESC_MIN_US;
        }

        esc_write_us_4(motor_us);


        if (timestamp - last_print_timestamp > 500000) {
            last_print_timestamp = timestamp;
            ESP_LOGD(TAG, "Target Orientation - Roll: %f, Pitch: %f",
                     ref_euler.x, ref_euler.y);
            ESP_LOGD(TAG, "Current Orientation - Roll: %f, Pitch: %f, Yaw: %f",
                   orientation_euler.x, orientation_euler.y, orientation_euler.z);
            ESP_LOGD(TAG, "Euler Error - Roll: %f, Pitch: %f, Yaw: %f",
                   euler_error.x, euler_error.y, euler_error.z);
            ESP_LOGD(TAG, "Moments & Forces: %0.3f roll, %0.3f pitch, %0.3f yaw, %0.3f thrust",
                   roll_moment, pitch_moment, yaw_moment, local_z_force);
            ESP_LOGD(TAG, "us: %lu, %lu, %lu, %lu",
                   motor_us[0], motor_us[1], motor_us[2], motor_us[3]);
        }

        //verbose log for sd card logging? not sure if this will overwhelm at 250hz
         ESP_LOGV(TAG, "TO (R, P): %f, %f | CO (R, P, Y): %f, %f, %f | EE (R, P, Y): %f, %f, %f | TR: %f, %f, %f | CR: %f. %f, %f | M & F: %0.3f roll, %0.3f pitch, %0.3f yaw, %0.3f thrust | us: %lu, %lu, %lu, %lu",
            ref_euler.x, ref_euler.y,
            orientation_euler.x, orientation_euler.y, orientation_euler.z,
            euler_error.x, euler_error.y, euler_error.z,
            roll_rate_setpoint, pitch_rate_setpoint, yaw_rate_setpoint,
            euler_rates.x, euler_rates.y, euler_rates.z,
            roll_moment, pitch_moment, yaw_moment, local_z_force,
            motor_us[0], motor_us[1], motor_us[2], motor_us[3]);
        

        // 250 Hz loop
        xTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(4));
    }
}



void esc_home_task(void* arg)
{
    // Initialize ESC PWM once
    esc_ledc_init();

    TickType_t last_wake_time = xTaskGetTickCount();

    ControlConfig* config = (ControlConfig*)arg;
    QueueHandle_t ibus_mailbox = config->ibus_mailbox;

    uint64_t initial_timestamp;
    uint64_t last_print_timestamp = 0;
    bool has_homed = false;

    timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &initial_timestamp);


    while (1) {
        // Get input from RC transmitter
        IbusMessage controller_input = {};
        (void)xQueuePeek(ibus_mailbox, &controller_input, 0);

        if (controller_input.throttle > 0.5f) {
            has_homed = true;
        }

        uint64_t timestamp;
        timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &timestamp);

        if (timestamp - last_print_timestamp > 500000) {
            last_print_timestamp = timestamp;
            ESP_LOGI(TAG, "Waiting to home, please increase joystick input after powering ESCs\n");
        }

        uint32_t motor_us[4];
        if (has_homed) {
            motor_us[0] = ESC_MIN_US;
            motor_us[1] = ESC_MIN_US;
            motor_us[2] = ESC_MIN_US;
            motor_us[3] = ESC_MIN_US;
        } else {
            motor_us[0] = ESC_MAX_US;
            motor_us[1] = ESC_MAX_US;
            motor_us[2] = ESC_MAX_US;
            motor_us[3] = ESC_MAX_US;
        }

        esc_write_us_4(motor_us);

        // 100 Hz loop
        xTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(10));
    }
}



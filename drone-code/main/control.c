#include <stdio.h>
#include <stdint.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/timer.h"
#include "driver/ledc.h"
#include "esp_err.h"

#include "feedforward.h"
#include "control.h"
#include "pid.h"
#include "math_helpers.h"
#include "ibus_protocol.h"

// ---------------- Motor GPIOs (MUST be output-capable) ----------------
// GPIO 34/35 are input-only on ESP32: do not use them for motor outputs.
#define MOTOR0_PIN 25
#define MOTOR1_PIN 26
#define MOTOR2_PIN 27
#define MOTOR3_PIN 14

// ---------------- ESC / LEDC configuration ----------------
#define ESC_HZ              50
#define LEDC_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_TIMER          LEDC_TIMER_0
#define LEDC_RES            LEDC_TIMER_16_BIT

#define ESC_MIN_US          1000
#define ESC_MAX_US          2000
// ---------------- Control configuration ----------------
#define KP 0.08
#define KI 0.0
#define KD 0.04
#define INTEGRAL_BOUND 0.0
#define DERIVATIVE_EMA_GAIN 0.8

#define BATTERY_VOLTAGE 11.1f
#define CONTROLLER_YAW_SENSITIVITY 0.01f


// If your calculate_control_input_from_moments outputs "voltages" in [0..BATTERY_VOLTAGE],
// map them to throttle fraction -> pulse width.
static inline float clampf(float x, float lo, float hi)
{
    return (x < lo) ? lo : (x > hi) ? hi : x;
}

static inline uint32_t esc_us_to_duty(uint32_t pulse_us)
{
    const uint32_t period_us = 1000000UL / ESC_HZ;              // 20000 us @ 50 Hz
    const uint32_t max_duty  = (1UL << LEDC_TIMER_16_BIT) - 1;  // 65535

    if (pulse_us > period_us) pulse_us = period_us;
    return (pulse_us * max_duty) / period_us;
}

static void esc_ledc_init(void)
{
    // Timer
    ledc_timer_config_t tcfg = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_RES,
        .freq_hz          = ESC_HZ,
        .clk_cfg          = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&tcfg));

    // Channels: one per motor
    const int gpios[4] = { MOTOR0_PIN, MOTOR1_PIN, MOTOR2_PIN, MOTOR3_PIN };
    const ledc_channel_t chs[4] = { LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2, LEDC_CHANNEL_3 };

    for (int i = 0; i < 4; i++) {
        ledc_channel_config_t ccfg = {
            .gpio_num   = gpios[i],
            .speed_mode = LEDC_MODE,
            .channel    = chs[i],
            .timer_sel  = LEDC_TIMER,
            .duty       = 0,
            .hpoint     = 0,
            .intr_type  = LEDC_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ccfg));
    }

    // Initialize motors to minimum pulse (safe baseline)
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
    // Initialize ESC PWM once (before loop)
    esc_ledc_init();

    // last_wake_time to ensure fixed frequency with vTaskDelayUntil
    TickType_t last_wake_time = xTaskGetTickCount();

    ControlConfig* config = (ControlConfig*)arg;
    QueueHandle_t ibus_mailbox = config->ibus_mailbox;
    QueueHandle_t state_estimate_mailbox = config->state_estimate_mailbox;

    uint64_t initial_timestamp;
    uint64_t last_print_timestamp = 0;

    timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &initial_timestamp);

    PIDController* pid_controllers[2]; // roll and pitch controllers
    for (int i = 0; i < 2; i++) {
        pid_controllers[i] = new_pid(
            KP, KI, KD,
            INTEGRAL_BOUND,
            DERIVATIVE_EMA_GAIN,
            0.0,
            initial_timestamp
        );
    }

    while (1) {
        // Get input from RC transmitter
        IbusMessage controller_input;
        (void)xQueuePeek(ibus_mailbox, &controller_input, 0);

        // Get current state estimate
        Vector3 orientation_euler;
        (void)xQueuePeek(state_estimate_mailbox, &orientation_euler, 0);

        // Convert RC transmitter input to reference orientation
        Quaternion ref_quat_headingless = joystick_inputs_to_ref_quat_headingless(&controller_input);
        Quaternion current_orientation_quat = euler_to_quat(orientation_euler);
        Vector3 euler_error = euler_error_from_quats(ref_quat_headingless, current_orientation_quat);

        uint64_t timestamp;
        timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &timestamp);

        float roll_moment  = calculate_pid_with_err(pid_controllers[0], euler_error.x, timestamp);
        float pitch_moment = calculate_pid_with_err(pid_controllers[1], euler_error.y, timestamp);
        float yaw_moment   = controller_input.yaw * CONTROLLER_YAW_SENSITIVITY;

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

        if (timestamp - last_print_timestamp > 500000) {
            last_print_timestamp = timestamp;
            printf("Target Orientation - Roll: %f, Pitch: %f\n",
                   ref_quat_headingless.x, ref_quat_headingless.y);
            printf("Current Orientation - Roll: %f, Pitch: %f, Yaw: %f\n",
                   orientation_euler.x, orientation_euler.y, orientation_euler.z);
            printf("Euler Error - Roll: %f, Pitch: %f, Yaw: %f\n",
                   euler_error.x, euler_error.y, euler_error.z);
            printf("Moments & Forces: %0.3f roll, %0.3f pitch, %0.3f yaw, %0.3f thrust\n",
                   roll_moment, pitch_moment, yaw_moment, local_z_force);
            printf("us: %lu, %lu, %lu, %lu\n",
                   motor_us[0], motor_us[1], motor_us[2], motor_us[3]);
            printf("vra: %f\n", controller_input.vra);
        }
        
        bool emergency_stop = false;
        if (controller_input.vra > 0.5f) {
            emergency_stop = true;
        }
        if (!emergency_stop) {
            // Write PWM pulses to ESCs
            esc_write_us_4(motor_us);
        }

        // 1 kHz loop (adjust to your desired controller rate)
        xTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(1));
    }
}



void esc_home_task(void* arg)
{
    // Initialize ESC PWM once (before loop)
    esc_ledc_init();

    // last_wake_time to ensure fixed frequency with vTaskDelayUntil
    TickType_t last_wake_time = xTaskGetTickCount();

    ControlConfig* config = (ControlConfig*)arg;
    QueueHandle_t ibus_mailbox = config->ibus_mailbox;
    QueueHandle_t state_estimate_mailbox = config->state_estimate_mailbox;

    uint64_t initial_timestamp;
    uint64_t last_print_timestamp = 0;
    bool has_homed = false;

    timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &initial_timestamp);


    while (1) {
        // Get input from RC transmitter
        IbusMessage controller_input;
        (void)xQueuePeek(ibus_mailbox, &controller_input, 0);

        if (controller_input.throttle > 0.5f) {
            has_homed = true;
        }

        uint64_t timestamp;
        timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &timestamp);

        if (timestamp - last_print_timestamp > 500000) {
            last_print_timestamp = timestamp;
            printf("Waiting to home, please increase joystick input after powering ESCs\n");
        }

        // Map "voltage commands" -> throttle fraction -> ESC microseconds
        // Assumes control_inputs[i] is in [0..BATTERY_VOLTAGE].
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

        // Write PWM pulses to ESCs
        esc_write_us_4(motor_us);

        // 1 kHz loop (adjust to your desired controller rate)
        xTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(1));
    }
}



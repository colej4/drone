#include <stdint.h> 
#include <stdio.h>

#include "feedforward.hpp"
#include "ibus_protocol.hpp"

//esp logging
#include "esp_log.h"
static const char* TAG = "feedforward";

//propellor contants and motor constants (exact ones in matlab) used to derive other constants
#define CT0 0.145
#define CQ0 0.0173

#define PROP_DIAM_METERS (5.0 * 0.0254) //5 inch prop in meters
#define RHO_AIR 1.225 //kg/m^3

#define KT (CT0 * RHO_AIR * powf(PROP_DIAM_METERS, 4) / powf(2 * M_PI, 2)) // N / (rad/s)^2 Should be 1.17e-6
#define KQ (CQ0 * RHO_AIR * powf(PROP_DIAM_METERS, 5) / powf(2 * M_PI, 2)) // Nm / (rad/s)^2 Should be 1.773e-8

#define MOTOR_KV 2200.0f // RPM per volt
#define I_PROP 3.8e-6 //kg*m^2, estimated from prop mass and geometry
#define MOTOR_RESISTANCE 0.1f //ohms

//constants for converting forces and moments to motor speed
#define MOTOR_LEVER_ARM 0.15556 //0.22 / sqrt(2) meters for 440mm rod
#define MAX_OMEGA_SQUARED 5234944.0f // approx (2288 rad/s)^2 (approximation of 2200kv motor at 10V)
#define OMEGA_SQUARED_OVER_FORCE (1.0 / KT) // proportionality constant between force and omega^2 (approximation from MATLAB) Should be 854359
#define OMEGA_SQUARED_OVER_Z_MOMENT (1.0 / (4.0 * KQ)) // proportionality constant between z moment and omega^2 (approximation from MATLAB) approx 1.41e7

//constants for converting motor speed to control input (not that good right now, only FF to get right stead state speed)
#define CONTROL_INPUT_OVER_OMEGA (60.0 / (2 * M_PI * MOTOR_KV)) //this is also motor ke, Should be 0.00434
#define CONTROL_INPUT_OVER_OMEGA_SQUARED  (MOTOR_RESISTANCE / CONTROL_INPUT_OVER_OMEGA * KQ) // Should be 4.08e-7

#define MAX_THRUST_NEWTONS 20.0f //maximum thrust in newtons
#define MIN_THRUST_NEWTONS 0.5f  //minimum thrust in newtons


//for controller
#define CONTROLLER_SENS 0.1 //max command (as fraction of upwards thrust)

Quaternion ref_quat_from_global_forces(Vector3 global_force_vec, float heading) {
    float force_norm = sqrtf(global_force_vec.x * global_force_vec.x +
                             global_force_vec.y * global_force_vec.y +
                             global_force_vec.z * global_force_vec.z);
    if (force_norm < 1e-3f) {
        return (Quaternion){1.0f, 0.0f, 0.0f, 0.0f};
    }

    Vector3 z_axis = (Vector3){0.0f, 0.0f, 1.0f};
    Vector3 force_dir = normalize(global_force_vec);
    Quaternion yawless_quat;

    if(fabs(1.0 - dot(force_dir, z_axis)) < 1e-4) {
        //force is aligned with z axis, return quat from heading only
        yawless_quat = (Quaternion){1.0f, 0.0f, 0.0f, 0.0f};
    } else {
        Vector3 rotation_axis = normalize(cross(z_axis, force_dir));
        float rotation_angle = acosf(dot(z_axis, force_dir));
        float half_angle = rotation_angle * 0.5f;
        float sin_half_angle = sinf(half_angle);
        yawless_quat = (Quaternion){
            cosf(half_angle),
            rotation_axis.x * sin_half_angle,
            rotation_axis.y * sin_half_angle,
            rotation_axis.z * sin_half_angle
        };
    }

    Quaternion yaw_quat = (Quaternion){
        cosf(heading * 0.5f),
        0.0f,
        0.0f,
        sinf(heading * 0.5f)
    };

    return quatmultiply(yaw_quat, yawless_quat);
}

//finds the reciprocal of the z component of a unit vector in the direction of a quat (essentially ratio between global f_z and local f_z)
float thrust_multiplier_from_quat(Quaternion quat) {
    float x = quat.x;
    float y = quat.y;

    float z = 1.0f - 2.0f * (x * x + y * y);
    if(z > 1e-6) {
        return 1.0f / z;
    } else {
        return 0.0f;
    }
}

static Vector3 joystick_inputs_to_forces(IbusMessage* message) {
    float force_z = message->throttle * 1.5f * G;
    float force_x = -CONTROLLER_SENS * message->roll * force_z;
    float force_y = -CONTROLLER_SENS * message->pitch * force_z;
    return (Vector3){force_x, force_y, force_z};
}

float joystick_input_to_global_thrust(IbusMessage* message) {
    float thrust = message->throttle * 2.0f * G;
    if (thrust > MAX_THRUST_NEWTONS) {
        ESP_LOGW(TAG, "Thrust command clamped from %f to %f", thrust, MAX_THRUST_NEWTONS);
        thrust = MAX_THRUST_NEWTONS;
    }
    if (thrust < MIN_THRUST_NEWTONS) {
        thrust = 0.0f;
    }
    return thrust;
}

Quaternion joystick_inputs_to_ref_quat_headingless(IbusMessage* message) {
    Vector3 forces = joystick_inputs_to_forces(message);
    //ignore heading for now, will take raw heading moment from joystick.
    return ref_quat_from_global_forces(forces, 0.0f);
}

Vector3 euler_error_from_quats(Quaternion q_ref, Quaternion q_meas) {
    Vector3 ref_euler = quat_to_euler(q_ref);
    Vector3 meas_euler = quat_to_euler(q_meas);

    Vector3 euler_error;
    euler_error.x = wrap_angle_pi(ref_euler.x - meas_euler.x);
    euler_error.y = wrap_angle_pi(ref_euler.y - meas_euler.y);
    euler_error.z = wrap_angle_pi(ref_euler.z - meas_euler.z);
    return euler_error;
}

//thrust is in local frame here, omega_squared_array of len 4
static void calculate_omega_squared(float* omega_squared_array, float m_x, float m_y, float m_z, float thrust_z) {
    float omega_squared_due_to_moments[4];

    omega_squared_due_to_moments[0] = (OMEGA_SQUARED_OVER_FORCE * (m_x + m_y) / (MOTOR_LEVER_ARM * 4.0) - OMEGA_SQUARED_OVER_Z_MOMENT * m_z);
    omega_squared_due_to_moments[1] = (OMEGA_SQUARED_OVER_FORCE * (-m_x + m_y) / (MOTOR_LEVER_ARM * 4.0)) + OMEGA_SQUARED_OVER_Z_MOMENT * m_z;
    omega_squared_due_to_moments[2] = (OMEGA_SQUARED_OVER_FORCE * (-m_x - m_y) / (MOTOR_LEVER_ARM * 4.0)) - OMEGA_SQUARED_OVER_Z_MOMENT * m_z;
    omega_squared_due_to_moments[3] = (OMEGA_SQUARED_OVER_FORCE * (m_x - m_y) / (MOTOR_LEVER_ARM * 4.0)) + OMEGA_SQUARED_OVER_Z_MOMENT * m_z;

    float omega_squared_due_to_thrust = OMEGA_SQUARED_OVER_FORCE * thrust_z / 4.0f;

    float min_omega_squared_due_to_moments = omega_squared_due_to_moments[0];
    for(int i = 0; i < 4; i++) {
        if (omega_squared_due_to_moments[i] < min_omega_squared_due_to_moments) {
            min_omega_squared_due_to_moments = omega_squared_due_to_moments[i];
        }
    }

    float saturation_factor = 1.0f;

    if (min_omega_squared_due_to_moments < 0.0f && fabs(min_omega_squared_due_to_moments) > omega_squared_due_to_thrust) {
        saturation_factor = omega_squared_due_to_thrust / (fabs(min_omega_squared_due_to_moments) + 1e-5f);
    }

    ESP_LOGV(TAG, "saturation_factor: %f, min_omega_squared_due_to_moments: %f, omega_squared_due_to_thrust: %f", saturation_factor, min_omega_squared_due_to_moments, omega_squared_due_to_thrust);

    omega_squared_array[0] = saturation_factor * omega_squared_due_to_moments[0] + omega_squared_due_to_thrust;
    omega_squared_array[1] = saturation_factor * omega_squared_due_to_moments[1] + omega_squared_due_to_thrust;
    omega_squared_array[2] = saturation_factor * omega_squared_due_to_moments[2] + omega_squared_due_to_thrust;
    omega_squared_array[3] = saturation_factor * omega_squared_due_to_moments[3] + omega_squared_due_to_thrust;
}

//requires control_input_array and omega_squared_array to be of length 4
static void calculate_control_input_from_omega_squared(float* control_input_array, float* omega_squared_array) {
    //find max omega_squared and clamp it
    float omega_array[4];
    float max_omega_squared_mag = fabs(omega_squared_array[0]);
    for(int i = 1; i < 4; i++) {
        if (fabs(omega_squared_array[i]) > max_omega_squared_mag) {
            max_omega_squared_mag = fabs(omega_squared_array[i]);
        }
    }
    //maybe should implement better saturation here?
    for (int i = 0; i < 4; i++) {
        if (omega_squared_array[i] < 0) omega_squared_array[i] = 0;
        if (omega_squared_array[i] > MAX_OMEGA_SQUARED) omega_squared_array[i] = MAX_OMEGA_SQUARED;
        omega_array[i] = sqrtf(omega_squared_array[i]);
    }


    for(int i = 0; i < 4; i++) {
        control_input_array[i] = omega_squared_array[i] * CONTROL_INPUT_OVER_OMEGA_SQUARED + omega_array[i] * CONTROL_INPUT_OVER_OMEGA;
    }
}

void calculate_control_input_from_moments(float* control_input_array, float m_x, float m_y, float m_z, float thrust_z) {
    float omega_squared_array[4];
    calculate_omega_squared(omega_squared_array, m_x, m_y, m_z, thrust_z);
    calculate_control_input_from_omega_squared(control_input_array, omega_squared_array);
}
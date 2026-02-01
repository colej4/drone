#include <stdint.h> 
#include <stdio.h>

#include "feedforward.h"
#include "ibus_protocol.h"


//constants for converting forces and moments to motor speed
#define MOTOR_LEVER_ARM 0.15556 //0.22 / sqrt(2) meters for 440mm rod
#define MAX_OMEGA_SQUARED 5234944.0f // approx (2288 rad/s)^2 (approximation of 2200kv motor at 10V)
#define OMEGA_SQUARED_OVER_FORCE 125000.00 // proportionality constant between force and omega^2 (approximation from MATLAB)
#define OMEGA_SQUARED_OVER_Z_MOMENT 287356.32 // proportionality constant between z moment and omega^2 (approximation from MATLAB)

//constants for converting motor speed to control input (not that good right now, only FF to get right stead state speed)
#define CONTROL_INPUT_OVER_OMEGA_SQUARED 6.863e-6
#define CONTROL_INPUT_OVER_OMEGA 0.0044

#define MAX_THRUST_NEWTONS 20.0f //maximum thrust in newtons
#define MIN_THRUST_NEWTONS 0.5f  //minimum thrust in newtons


//for controller
#define CONTROLLER_SENS 4.0 //force (in newtons) applied at max joystick input

Quaternion ref_quat_from_global_forces(Vector3 global_force_vec, float heading) {
    Vector3 z_axis = (Vector3){0.0f, 0.0f, 1.0f};
    Vector3 force_dir = normalize(global_force_vec);
    Quaternion yawless_quat;

    if(fabs(1.0 - dot(force_dir, z_axis)) < 1e-6) {
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
    float force_z = message->throttle * 2.0f * G;
    float force_x = -CONTROLLER_SENS * message->roll;
    float force_y = -CONTROLLER_SENS * message->pitch;
    return (Vector3){force_x, force_y, force_z};
}

float joystick_input_to_global_thrust(IbusMessage* message) {
    float thrust = message->throttle * 2.0f * G;
    if (thrust > MAX_THRUST_NEWTONS) {
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
    Quaternion q_meas_conj = (Quaternion){
        q_meas.w,
        -q_meas.x,
        -q_meas.y,
        -q_meas.z
    };

    Quaternion q_err = quatmultiply(q_ref, q_meas_conj);

    Vector3 euler_error;
    float theta = 2.0f * acosf(fmaxf(fminf(q_err.w, 1.0f), -1.0f));
    float sin_half_theta = sinf(theta / 2.0f);
    if (fabs(sin_half_theta) < 1e-6) {
        // printf("singularity in euler error calculation\n");
        euler_error = (Vector3){0.0f, 0.0f, 0.0f};
    } else {
        euler_error.x = (q_err.x / sin_half_theta) * theta;
        euler_error.y = (q_err.y / sin_half_theta) * theta;
        euler_error.z = (q_err.z / sin_half_theta) * theta;
    }
    return euler_error;
}

//thrust is in local frame here, omega_squared_array of len 4
static void calculate_omega_squared(float* omega_squared_array, float m_x, float m_y, float m_z, float thrust_z) {
    float f_0 = ((m_x + m_y) / MOTOR_LEVER_ARM + thrust_z) / 4.0f;
    float f_1 = ((-m_x + m_y) / MOTOR_LEVER_ARM + thrust_z) / 4.0f;
    float f_2 = ((-m_x - m_y) / MOTOR_LEVER_ARM + thrust_z) / 4.0f;
    float f_3 = ((m_x - m_y) / MOTOR_LEVER_ARM + thrust_z) / 4.0f;

    float omega_0_squared = OMEGA_SQUARED_OVER_FORCE * f_0 - OMEGA_SQUARED_OVER_Z_MOMENT * m_z;
    float omega_1_squared = OMEGA_SQUARED_OVER_FORCE * f_1 + OMEGA_SQUARED_OVER_Z_MOMENT * m_z;
    float omega_2_squared = OMEGA_SQUARED_OVER_FORCE * f_2 - OMEGA_SQUARED_OVER_Z_MOMENT * m_z;
    float omega_3_squared = OMEGA_SQUARED_OVER_FORCE * f_3 + OMEGA_SQUARED_OVER_Z_MOMENT * m_z;

    omega_squared_array[0] = omega_0_squared;
    omega_squared_array[1] = omega_1_squared;
    omega_squared_array[2] = omega_2_squared;
    omega_squared_array[3] = omega_3_squared;
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
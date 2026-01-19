#ifndef FEEDFORWARD_H
#define FEEDFORWARD_H

#include "math_helpers.h"
#include "ibus_protocol.h"
Quaternion ref_quat_from_global_forces(Vector3 global_force_vec, float heading);
Vector3 euler_error_from_quats(Quaternion q_ref, Quaternion q_meas);
void calculate_control_input_from_moments(float* control_input_array, float m_x, float m_y, float m_z, float thrust_z);
Quaternion joystick_inputs_to_ref_quat_headingless(IbusMessage* message);
float joystick_input_to_global_thrust(IbusMessage* message);
float thrust_multiplier_from_quat(Quaternion quat);

#endif
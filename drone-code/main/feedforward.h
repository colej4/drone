#include "math_helpers.h"
#include "ibus_protocol.h"
Quaternion ref_quat_from_global_forces(Vector3 global_force_vec, float heading);
void calculate_control_input_from_moments(float* control_input_array, float m_x, float m_y, float m_z, float thrust_z);
Quaternion joystick_inputs_to_ref_quat_headingless(IbusMessage* message);
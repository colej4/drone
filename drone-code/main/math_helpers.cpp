#include "math_helpers.hpp"
#include <functional>

Quaternion quatmultiply(Quaternion q1, Quaternion q2) {
    Quaternion result;
    result.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
    result.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
    result.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
    result.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;
    return result;
}

Quaternion euler_to_quat(Vector3 euler) {
    float cy = cosf(euler.z * 0.5f);
    float sy = sinf(euler.z * 0.5f);
    float cp = cosf(euler.y * 0.5f);
    float sp = sinf(euler.y * 0.5f);
    float cr = cosf(euler.x * 0.5f);
    float sr = sinf(euler.x * 0.5f);

    Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}


Vector3 quatrotate(Quaternion q, Vector3 v) {
    Quaternion v_quat = {0.0f, v.x, v.y, v.z};
    Quaternion q_conj = {q.w, -q.x, -q.y, -q.z};
    Quaternion result_quat = quatmultiply(quatmultiply(q, v_quat), q_conj);
    Vector3 result = {result_quat.x, result_quat.y, result_quat.z};
    return result;
}

Vector3 normalize(Vector3 v) {
    float norm = sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
    if (norm > 1e-6) {
        v.x /= norm;
        v.y /= norm;
        v.z /= norm;
    }
    return v;
}

float dot(Vector3 v1, Vector3 v2) {
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

Vector3 cross(Vector3 v1, Vector3 v2) {
    Vector3 result;
    result.x = v1.y * v2.z - v1.z * v2.y;
    result.y = v1.z * v2.x - v1.x * v2.z;
    result.z = v1.x * v2.y - v1.y * v2.x;
    return result;
}

int signum(int x) {
    if (x > 0) {
        return 1;
    } else if (x < 0) {
        return -1;
    } else {
        return 0;
    }
}

Vector3 quat_to_euler(Quaternion q) {
    const float pi = 3.14159265f;
    float norm = sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (norm > 1e-6f) {
        q.w /= norm;
        q.x /= norm;
        q.y /= norm;
        q.z /= norm;
    }

    float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    float roll = atan2f(sinr_cosp, cosr_cosp);

    float sinp = 2.0f * (q.w * q.y - q.z * q.x);
    if (sinp > 1.0f) sinp = 1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    float pitch = asinf(sinp);

    float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    float yaw = atan2f(siny_cosp, cosy_cosp);

    (void)pi;
    return (Vector3){roll, pitch, yaw};
}

float wrap_angle_pi(float angle) {
    const float pi = 3.14159265f;
    const float two_pi = 2.0f * pi;
    angle = fmodf(angle + pi, two_pi);
    if (angle < 0.0f) {
        angle += two_pi;
    }
    return angle - pi;
}
#ifndef MATH_HELPERS_H
#define MATH_HELPERS_H

#include <math.h>

#define G 9.81

typedef struct {
    float w;
    float x;
    float y;
    float z;
} Quaternion;

typedef struct {
    float x;
    float y;
    float z;
} Vector3;

Quaternion quatmultiply(Quaternion q1, Quaternion q2);
Quaternion euler_to_quat(Vector3 euler);
Vector3 quatrotate(Quaternion q, Vector3 v);
Vector3 normalize(Vector3 v);
float dot(Vector3 v1, Vector3 v2);
Vector3 cross(Vector3 v1, Vector3 v2);
int signum(int x);

#endif
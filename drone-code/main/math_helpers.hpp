#ifndef MATH_HELPERS_H
#define MATH_HELPERS_H

#include <math.h>
#include <functional>
#include <utility>

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
template<typename State, typename ControlInput, typename Func>
std::function<State(const State&, const ControlInput&)> rk4_step(Func&& f, float dt) {
    auto func = std::forward<Func>(f);
    return [func = std::move(func), dt](const State& x, const ControlInput& u) {
        const State k1 = func(x, u);
        const State k2 = func(x + (dt * 0.5f) * k1, u);
        const State k3 = func(x + (dt * 0.5f) * k2, u);
        const State k4 = func(x + dt * k3, u);

        return x + (dt * (1.0f / 6.0f)) * (k1 + 2.0f * k2 + 2.0f * k3 + k4);
    };
}

template<typename State, typename ControlInput, typename Func>
std::function<State(const State&, const ControlInput&)> euler_step(Func&& f, float dt) {
    auto func = std::forward<Func>(f);
    return [func = std::move(func), dt](const State& x, const ControlInput& u) {
        const State rate_of_change = func(x, u);
        return x + dt * rate_of_change;
    };
}

#endif
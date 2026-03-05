#include "Eigen/Dense"
#include <cmath>

#include "ekf.hpp"
#include "math_helpers.hpp"

#include "esp_log.h"

const char* TAG = "ekf_estimator";

#define PROCESS_NOISE Eigen::Matrix<float, 7, 7>::Identity()* 0.001f
#define ACCEL_MEASUREMENT_NOISE Eigen::Matrix<float, 3, 3>::Identity()

using State = Eigen::Vector<float, 7>;
using ControlInput = Eigen::Vector<float, 3>;


State state_rate_from_state_and_input(State state, ControlInput input) {
    //q0_dot = 1/2 * (-q1*(wx - bx) - q2*(wy - by) - q3*(wz - bz))
    float q0_dot = 0.5f * (-state(1) * (input(0) - state(4)) - state(2) * (input(1) - state(5)) - state(3) * (input(2) - state(6)));
    //q1_dot = 1/2 * ( q0*(wx - bx) + q2*(wz - bz) - q3*(wy - by))
    float q1_dot = 0.5f * ( state(0) * (input(0) - state(4)) + state(2) * (input(2) - state(6)) - state(3) * (input(1) - state(5)));
    //q2_dot = 1/2 * ( q0*(wy - by) - q1*(wz - bz) + q3*(wx - bx))
    float q2_dot = 0.5f * ( state(0) * (input(1) - state(5)) - state(1) * (input(2) - state(6)) + state(3) * (input(0) - state(4)));
    //q3_dot = 1/2 * ( q0*(wz - bz) + q1*(wy - by) - q2*(wx - bx))
    float q3_dot = 0.5f * ( state(0) * (input(2) - state(6)) + state(1) * (input(1) - state(5)) - state(2) * (input(0) - state(4)));
    //bx_dot, by_dot, bz_dot = 0

    return (State){
        q0_dot,
        q1_dot,
        q2_dot,
        q3_dot,
        0.0f,
        0.0f,
        0.0f
    };
}

Eigen::Matrix<float, 7, 7> jacobian_from_state_and_input(State state, ControlInput input, float dt) {
    Eigen::Matrix<float, 7, 7> I = Eigen::Matrix<float, 7, 7>::Identity();
    Eigen::Matrix<float, 7, 7> F;
    F.setZero();

    float p = input(0) - state(4);
    float q = input(1) - state(5);
    float r = input(2) - state(6);

    //partial derivatives of q_dot w.r.t. state given input
    F(0, 1) = -p;
    F(0, 2) = -q;
    F(0, 3) = -r;

    F(1, 0) = p;
    F(1, 2) = r;
    F(1, 3) = -q;

    F(2, 0) = q;
    F(2, 1) = -r;
    F(2, 3) = p;

    F(3, 0) = r;
    F(3, 1) = q;
    F(3, 2) = -p;

    F(0, 4) =  state(1);
    F(0, 5) =  state(2);
    F(0, 6) =  state(3);

    F(1, 4) = -state(0);
    F(1, 5) =  state(3);
    F(1, 6) = -state(2);

    F(2, 4) = -state(3);
    F(2, 5) = -state(0);
    F(2, 6) =  state(1);

    F(3, 4) =  state(2);
    F(3, 5) = -state(1);
    F(3, 6) = -state(0);
    

    F *= 0.5f * dt; 
    return I + F;
}

ControlInput accelerometer_measurement_from_state(State state) {
    ControlInput accel_meas;
    float q0 = state(0);
    float q1 = state(1);
    float q2 = state(2);
    float q3 = state(3);

    accel_meas(0) = 2.0f * (q1 * q3 - q0 * q2) * GRAVITATIONAL_ACCELERATION;
    accel_meas(1) = 2.0f * (q0 * q1 + q2 * q3) * GRAVITATIONAL_ACCELERATION;
    accel_meas(2) = (q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * GRAVITATIONAL_ACCELERATION;

    return accel_meas;
}

Eigen::Matrix<float, 3, 7> accelerometer_jacobian_from_state(State state) {
    Eigen::Matrix<float, 3, 7> H;
    H.setZero();

    float q0 = state(0);
    float q1 = state(1);
    float q2 = state(2);
    float q3 = state(3);

    //partial derivatives of accel measurement w.r.t. state
    H(0, 0) = -2.0f * q2 * GRAVITATIONAL_ACCELERATION;
    H(0, 1) = 2.0f * q3 * GRAVITATIONAL_ACCELERATION;
    H(0, 2) = -2.0f * q0 * GRAVITATIONAL_ACCELERATION;
    H(0, 3) = 2.0f * q1 * GRAVITATIONAL_ACCELERATION;

    H(1, 0) = 2.0f * q1 * GRAVITATIONAL_ACCELERATION;
    H(1, 1) = 2.0f * q0 * GRAVITATIONAL_ACCELERATION;
    H(1, 2) = 2.0f * q3 * GRAVITATIONAL_ACCELERATION;
    H(1, 3) = 2.0f * q2 * GRAVITATIONAL_ACCELERATION;

    H(2, 0) = 2.0f * q0 * GRAVITATIONAL_ACCELERATION;
    H(2, 1) = -2.0f * q1 * GRAVITATIONAL_ACCELERATION;
    H(2, 2) = -2.0f * q2 * GRAVITATIONAL_ACCELERATION;
    H(2, 3) = 2.0f * q3 * GRAVITATIONAL_ACCELERATION;

    return H;
}

class EKF_state_estimator {
    public:
        ExtendedKalmanFilter<7> ekf;
        std::function<State(State, ControlInput, float)> f;
        uint64_t last_gyro_timestamp_us;
        Eigen::Matrix<float, 7, 7> process_noise_covariance;
        Eigen::Matrix<float, 3, 3> accelerometer_measurement_noise_covariance;
        float imu_dt_s;

        EKF_state_estimator(State initial_mean, Eigen::Matrix<float, 7, 7> initial_covariance, uint32_t imu_rate_hz, uint64_t initial_timestamp_us)
            : ekf(initial_mean, initial_covariance) {
                this->imu_dt_s = 1.0f / (float)imu_rate_hz;
                this->last_gyro_timestamp_us = initial_timestamp_us;
                this->f = rk4_step<State, ControlInput>(state_rate_from_state_and_input);
                this->process_noise_covariance = PROCESS_NOISE;
                this->accelerometer_measurement_noise_covariance = ACCEL_MEASUREMENT_NOISE;
            }

        void predict(ControlInput control_input, uint64_t timestamp_us) {
            if (timestamp_us <= this->last_gyro_timestamp_us) {
                ESP_LOGW(TAG, "Non-increasing timestamp in EKF predict step");
                return;
            }
            uint64_t dt_us = timestamp_us - this->last_gyro_timestamp_us;
            float dt = (float)dt_us / 1e6f;
            this->last_gyro_timestamp_us = timestamp_us;
            ekf.predict(this->f,
                        jacobian_from_state_and_input,
                        this->process_noise_covariance * dt, 
                        control_input,
                        dt
                    );
            normalize_quaternion();
        }

        void normalize_quaternion() {
            float q0 = ekf.mean(0);
            float q1 = ekf.mean(1);
            float q2 = ekf.mean(2);
            float q3 = ekf.mean(3);
            float norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
            if (norm > 0.0f) {
                ekf.mean(0) /= norm;
                ekf.mean(1) /= norm;
                ekf.mean(2) /= norm;
                ekf.mean(3) /= norm;
            }
        }

        void accelerometer_update(ControlInput accel_measurement) {
            ekf.update<3>(
                accel_measurement,
                accelerometer_measurement_from_state,
                accelerometer_jacobian_from_state,
                this->accelerometer_measurement_noise_covariance
            );
            normalize_quaternion();
        }
};
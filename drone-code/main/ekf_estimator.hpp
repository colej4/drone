#ifndef EKF_ESTIMATOR_HPP
#define EKF_ESTIMATOR_HPP

#include <cstdint>
#include <functional>
#include "Eigen/Dense"

#include "ekf.hpp"

using EkfState = Eigen::Vector<float, 7>;
using EkfControlInput = Eigen::Vector<float, 3>;

class EKF_state_estimator {
public:
    ExtendedKalmanFilter<7> ekf;
    std::function<EkfState(EkfState, EkfControlInput, float)> f;
    uint64_t last_gyro_timestamp_us;
    Eigen::Matrix<float, 7, 7> process_noise_covariance;
    Eigen::Matrix<float, 3, 3> accelerometer_measurement_noise_covariance;
    float imu_dt_s;

    EKF_state_estimator(
        EkfState initial_mean,
        Eigen::Matrix<float, 7, 7> initial_covariance,
        uint32_t imu_rate_hz,
        uint64_t initial_timestamp_us
    );

    void predict(EkfControlInput control_input, uint64_t timestamp_us);
    void accelerometer_update(EkfControlInput accel_measurement);
    void normalize_quaternion();
};

#endif
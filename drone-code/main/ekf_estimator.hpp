#ifndef EKF_ESTIMATOR_HPP
#define EKF_ESTIMATOR_HPP

#include <cstdint>
#include <functional>
#include "Eigen/Dense"

#include "ekf.hpp"
#include "math_helpers.hpp"

using EkfState = Eigen::Vector<float, 7>;
using EkfControlInput = Eigen::Vector<float, 3>;

class EKFStateEstimator {
public:
    ExtendedKalmanFilter<7> ekf;
    std::function<EkfState(EkfState, EkfControlInput, float)> f;
    uint64_t last_gyro_timestamp_us;
    Eigen::Matrix<float, 7, 7> process_noise_covariance;
    Eigen::Matrix<float, 3, 3> accelerometer_measurement_noise_covariance;

    EKFStateEstimator(
        EkfState initial_mean,
        Eigen::Matrix<float, 7, 7> initial_covariance,
        uint64_t initial_timestamp_us
    );

    void predict(EkfControlInput control_input, uint64_t timestamp_us);
    void accelerometer_update(EkfControlInput accel_measurement);
    void normalize_quaternion();
    Vector3 get_orientation_euler();
};

#endif
#ifndef EKF_HPP
#define EKF_HPP

#include <functional>
#include <utility>
#include "Eigen/Dense"

template <int StateDim, typename T = float> class ExtendedKalmanFilter {
	public:
		Eigen::Vector<T, StateDim> mean;
		Eigen::Matrix<T, StateDim, StateDim> covariance;

		ExtendedKalmanFilter(Eigen::Vector<T, StateDim> initial_mean,
							 Eigen::Matrix<T, StateDim, StateDim> initial_covariance);

		template<typename ControlInput, typename StateFunc, typename JacobianFunc>
		void predict(
			StateFunc&& f,
			JacobianFunc&& F,
			Eigen::Matrix<T, StateDim, StateDim> process_noise,
			const ControlInput& u) {
			mean = std::forward<StateFunc>(f)(mean, u);
			Eigen::Matrix<T, StateDim, StateDim> F_jacobian = std::forward<JacobianFunc>(F)(mean, u);
			covariance = F_jacobian * covariance * F_jacobian.transpose() + process_noise;
		}

		template<int MeasDim, typename MeasFunc, typename JacobianFunc>
		void update(
			const Eigen::Vector<T, MeasDim>& measurement,
			MeasFunc&& h,
			JacobianFunc&& H,
			Eigen::Matrix<T, MeasDim, MeasDim> measurement_noise
		) {
			Eigen::Vector<T, MeasDim> y = measurement - std::forward<MeasFunc>(h)(mean); // innovation
			Eigen::Matrix<T, MeasDim, StateDim> H_jacobian = std::forward<JacobianFunc>(H)(mean);
			Eigen::Matrix<T, MeasDim, MeasDim> S = H_jacobian * covariance * H_jacobian.transpose() + measurement_noise; // innovation covariance
			Eigen::Matrix<T, StateDim, MeasDim> K = covariance * H_jacobian.transpose() * S.inverse(); // Kalman gain

			mean = mean + K * y;
			Eigen::Matrix<T, StateDim, StateDim> I = Eigen::Matrix<T, StateDim, StateDim>::Identity();
			covariance = (I - K * H_jacobian) * covariance;
		}
};

#endif
#ifndef IMU_POSTURE_EKF_HPP_
#define IMU_POSTURE_EKF_HPP_

#include <Eigen/Core>
#include <Eigen/Dense>

namespace pifod_ros2
{
    class ImuPostureEKF
    {
        public:
        /// @brief コンストラクタ
        ImuPostureEKF();

        /// @brief 加速度と角速度から姿勢を推定する
        /// @param angular 角速度[rad]
        /// @param linear_accel 加速度[m/s^2]
        /// @return オイラー角(roll, pitch, yaw)
        Eigen::Vector3f estimate(Eigen::Vector3f angular, Eigen::Vector3f linear_accel);

        private:
        Eigen::Matrix3f cov_;
        Eigen::Matrix3f estimation_noise_;
        Eigen::Matrix2f observation_noise_;
        Eigen::MatrixXf kalman_gain_;
        Eigen::Vector3f estimation_;
    };
    Eigen::Vector3f getInputMatrix(const float &angular_x, const float &angular_y, const float &angular_z);

    Eigen::Matrix<float, 3, 2> h();

    Eigen::Matrix3f jacob(const Eigen::Vector3f &input_matrix, const Eigen::Vector3f &estimation);

    Eigen::Vector3f predictX(const Eigen::Vector3f &input_matrix, const Eigen::Vector3f &estimation);

    Eigen::Matrix3f predictCov(const Eigen::Matrix3f &jacob, const Eigen::Matrix3f &cov, const Eigen::Matrix3f &est_noise);

    Eigen::Vector2f updateResidual(const Eigen::Vector2f &obs, const Eigen::Vector3f &est);

    Eigen::Matrix2f updateS(const Eigen::Matrix3f &cov_, const Eigen::Matrix2f &obs_noise);

    Eigen::Matrix<float, 3, 2> updateKalmanGain(const Eigen::Matrix2f &s, const Eigen::Matrix3f &cov);

    Eigen::Vector3f updateX(const Eigen::Vector3f &est, const Eigen::Matrix<float, 3, 2> &kalman_gain_, const Eigen::Vector2f &residual);

    Eigen::Matrix3f updateCov(const Eigen::Matrix<float, 3, 2> &kalman_gain, const Eigen::Matrix3f &cov);

    Eigen::Vector2f obsModel(const Eigen::Vector3f &linear_accel);
}

#endif
#include "pifod_ros2/imu_posture_ekf.hpp"

namespace pifod_ros2
{
    ImuPostureEKF::ImuPostureEKF()
    : estimation_(Eigen::Vector3f(0.0, 0.0, 0.0)),
    cov_(Eigen::Matrix3f::Identity()),
    estimation_noise_(Eigen::Matrix3f::Identity()),
    observation_noise_(Eigen::Matrix2f::Zero()),
    kalman_gain_(Eigen::Matrix<float, 3, 2>::Zero())
    {
        cov_(0, 0) = 0.0174*0.001;
        cov_(1, 1) = 0.0174*0.001;
        cov_(2, 2) = 0.0174*0.001;

        estimation_noise_(0, 0) = 0.0174*0.001;
        estimation_noise_(1, 1) = 0.0174*0.001;
        estimation_noise_(2, 2) = 0.0174*0.001;
    }

    Eigen::Vector3f ImuPostureEKF::estimate(Eigen::Vector3f angular, Eigen::Vector3f linear_accel)
    {
        const auto input_matrix = getInputMatrix(angular.x(), angular.y(), angular.z());

        const auto jacob_ = jacob(input_matrix, estimation_);
        estimation_ = predictX(input_matrix, estimation_);
        cov_ = predictCov(jacob_, cov_, estimation_noise_);
        const auto obs_model = obsModel(linear_accel);
        const auto residual = updateResidual(obs_model, estimation_);
        const auto s = updateS(cov_, observation_noise_);
        kalman_gain_ = updateKalmanGain(s, cov_);
        estimation_ = updateX(estimation_, kalman_gain_, residual);
        cov_ = updateCov(kalman_gain_, cov_);

        return estimation_;
    }

    Eigen::Vector3f getInputMatrix(const float &angular_x, const float &angular_y, const float &angular_z)
    {
        return Eigen::Vector3f(
            angular_x* 0.001,
            angular_y* 0.001,
            angular_z* 0.001
        );
    }

    Eigen::Matrix<float, 3, 2> h()
    {
        Eigen::Matrix<float, 3, 2> h_;
        h_.setZero();
        h_(0, 0) = 1.0;
        h_(1, 1) = 1.0;
        
        return h_;
    }

    Eigen::Matrix3f jacob(const Eigen::Vector3f &input_matrix, const Eigen::Vector3f &estimation)
    {
        auto cos_roll = cos(estimation.x());
        auto sin_roll = sin(estimation.x());
        auto cos_pitch = cos(estimation.y());
        auto sin_pitch = sin(estimation.y());

        auto m_11 = 1.0 + input_matrix.y() * ((cos_roll*sin_pitch)/cos_pitch) - input_matrix.z() * ((sin_roll*sin_pitch)/cos_pitch);
        auto m_12 = input_matrix.y()*(sin_roll/(cos_pitch*cos_pitch))+input_matrix.z()*((cos_roll/(cos_pitch*cos_pitch)));
        auto m_21 = -1.0*input_matrix.y()*sin_roll - input_matrix.z()*cos_roll;
        auto m_31 = input_matrix.y()*(cos_roll/cos_pitch) - input_matrix.z()*(sin_roll/cos_pitch);
        auto m_32 = input_matrix.y()*((sin_roll*sin_pitch)/(cos_pitch*cos_pitch))+input_matrix.z()*((cos_roll*sin_pitch)/(cos_pitch*cos_pitch));

        Eigen::Matrix3f mat;
        mat.setZero();
        mat(0, 0) = m_11;
        mat(0, 1) = m_12;
        mat(1, 0) = m_21;
        mat(1, 1) = 1.0;
        mat(2, 0) = m_31;
        mat(2, 1) = m_32;

        return mat;
    }

    Eigen::Vector3f predictX(const Eigen::Vector3f &input_matrix, const Eigen::Vector3f &estimation)
    {
        auto cos_roll = cos(estimation.x());
        auto sin_roll = sin(estimation.x());
        auto cos_pitch = cos(estimation.y());
        auto sin_pitch = sin(estimation.y());

        Eigen::Vector3f est;
        est(0) = estimation.x() + input_matrix.x() + input_matrix.y()*((sin_roll*sin_pitch)/cos_pitch)+input_matrix.z()*((cos_roll*sin_pitch)/cos_pitch);
        est(1) = estimation.y() + input_matrix.y() * cos_roll - input_matrix.z()*sin_roll;
        est(2) = estimation.z() + input_matrix.z() + input_matrix.y()*(sin_roll/cos_pitch) + input_matrix.z()*(cos_roll/cos_pitch);

        return est;
    }

    Eigen::Matrix3f predictCov(const Eigen::Matrix3f &jacob, const Eigen::Matrix3f &cov, const Eigen::Matrix3f &est_noise)
    {
        Eigen::Matrix3f t_jacob = jacob.transpose();
        Eigen::Matrix3f jacob_cov = jacob * cov;

        Eigen::Matrix3f new_cov;
        new_cov.setZero();

        Eigen::Matrix3f multiplied = jacob_cov * t_jacob;

        new_cov = multiplied + est_noise;

        return new_cov;
    }

    Eigen::Vector2f updateResidual(const Eigen::Vector2f &obs, const Eigen::Vector3f &est)
    {
        Eigen::Vector2f result;
        Eigen::Matrix<float, 2, 3> h_ = h().transpose();
        Eigen::Vector2f h_est = h_ * est;

        result(0) = obs.x() - h_est.x();
        result(1) = obs.y() - h_est.y();

        return result;
    }

    Eigen::Matrix2f updateS(const Eigen::Matrix3f &cov_, const Eigen::Matrix2f &obs_noise)
    {
        Eigen::Matrix<float, 2, 3> h_ = h().transpose();
        Eigen::Matrix<float, 2, 3> h_cov_ = h_ * cov_;
        Eigen::Matrix2f convert_cov_ = h_cov_ * h();

        return obs_noise + convert_cov_;
    }

    Eigen::Matrix<float, 3, 2> updateKalmanGain(const Eigen::Matrix2f &s, const Eigen::Matrix3f &cov)
    {
        auto h_ = h();

        Eigen::Matrix2f inverse_s = s.inverse();

        Eigen::Matrix<float, 3, 2> cov_and_h = cov * h_;

        return cov_and_h * inverse_s;
    }

    Eigen::Vector3f updateX(const Eigen::Vector3f &est, const Eigen::Matrix<float, 3, 2> &kalman_gain_, const Eigen::Vector2f &residual)
    {
        Eigen::Vector3f kalman_res = kalman_gain_ * residual;

        Eigen::Vector3f result;
        result.setZero();

        result(0) = est.x() + kalman_res.x();
        result(1) = est.y() + kalman_res.y();
        result(2) = est.z() + kalman_res.z();

        return result;
    }

    Eigen::Matrix3f updateCov(const Eigen::Matrix<float, 3, 2> &kalman_gain, const Eigen::Matrix3f &cov)
    {
        Eigen::Matrix3f i;
        i.setIdentity();

        Eigen::Matrix<float, 2, 3> h_ = h().transpose();

        Eigen::Matrix3f kalman_h = kalman_gain * h_;

        Eigen::Matrix3f i_k_h = i - kalman_h;

        return i_k_h * cov;
    }

    Eigen::Vector2f obsModel(const Eigen::Vector3f &linear_accel)
    {
        Eigen::Vector2f model;

        if(linear_accel.z() == 0.0)
        {
            if(linear_accel.y() > 0.0)
            {
                model(0) = acos(-1.0) / 2.0;
            }
            else
            {
                model(0) = -1.0 * acos(-1.0) / 2.0;
            }
        }
        else
        {
            model(0) = atan(linear_accel.y() / linear_accel.z());
        }

        if(sqrt(linear_accel.y()*linear_accel.y() + linear_accel.z()*linear_accel.z()) == 0.0)
        {
            if(-1.0*linear_accel.x() > 0.0)
            {
                model(1) = acos(-1.0) / 2.0;
            }
            else
            {
                model(1) = -1.0*acos(-1.0) / 2.0;
            }
        }
        else
        {
            model(1) = (-1.0 * linear_accel.x()) / atan(sqrt(linear_accel.y()*linear_accel.y() + linear_accel.z()*linear_accel.z()));
        }

        return model;
    }
}
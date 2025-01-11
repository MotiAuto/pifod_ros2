#include "pifod_ros2/icp_odom_utils.hpp"

namespace pifod_ros2
{
    ICPOdometer::ICPOdometer(const int max_iter, const double tolerance):
    max_iter_(max_iter),tolerance_(tolerance),target_(new pcl::PointCloud<pcl::PointXYZ>),
    translation_(Eigen::Vector3f()),rotation_(Eigen::Quaternionf())
    {

    }

    void ICPOdometer::setTarget(const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
    {
        pcl::fromROSMsg(*cloud, *target_);

        return;
    }

    void ICPOdometer::compute(const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cloud, *source);

        Eigen::Matrix3f r = Eigen::Matrix3f::Identity();
        Eigen::Vector3f t = Eigen::Vector3f::Zero();

        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_src = source;

        for(int i = 0; i < max_iter_; i++)
        {
            for(auto& point : *transformed_src)
            {
                auto p_vec = toEigenVec3(point);
                p_vec = r * p_vec + t;
                point.x = p_vec[0];
                point.y = p_vec[1];
                point.z = p_vec[2];
            }

            const auto matched_target = matching(transformed_src, target_);

            Eigen::MatrixXf A(3, 3);
            Eigen::VectorXf b(3);
            A.setZero();
            b.setZero();

            for(size_t i = 0; i < transformed_src->size(); i++)
            {
                const auto src_p = toEigenVec3(transformed_src->at(i));
                const auto tar_p = toEigenVec3(matched_target.at(i));

                A += src_p * src_p.transpose();
                b += src_p - tar_p;
            }

            Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
            r = svd.matrixU() * svd.matrixV().transpose();
            t = b;

            if(t.norm() < tolerance_)
            {
                break;
            }
        }

        translation_ = t;
        rotation_ = Eigen::Quaternionf(r);
    }

    Eigen::Vector3f ICPOdometer::getTranslation()
    {
        return translation_;
    }

    Eigen::Quaternionf ICPOdometer::getPosture()
    {
        return rotation_;
    }
}
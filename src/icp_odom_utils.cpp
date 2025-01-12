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

            const auto matched_target = matching(transformed_src, target_);

            Eigen::MatrixXf A(3, 3);
            A.setZero();
            Eigen::Vector3f src_center(0.0, 0.0, 0.0);
            Eigen::Vector3f tar_center(0.0, 0.0, 0.0);

            for(size_t i = 0; i < transformed_src->size(); i++)
            {
                const auto src_p = pifod_common_utils::toEigenVec3(transformed_src->at(i));
                const auto tar_p = pifod_common_utils::toEigenVec3(matched_target.at(i));

                src_center += src_p;
                tar_center += tar_p;
            }

            src_center /= transformed_src->size();
            tar_center /= matched_target.size();

            for(size_t i = 0; i < transformed_src->size(); i++)
            {
                const auto src_p = pifod_common_utils::toEigenVec3(transformed_src->at(i));
                const auto tar_p = pifod_common_utils::toEigenVec3(matched_target.at(i));

                A += src_p * tar_p.transpose();
            }

            Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
            r = svd.matrixU() * svd.matrixV().transpose();
            t = tar_center - r * src_center;

            if(t.norm() < tolerance_)
            {
                break;
            }
            
            transformed_src = transformPointCloud(transformed_src, r, t);
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
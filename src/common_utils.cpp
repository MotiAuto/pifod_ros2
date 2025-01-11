#include "pifod_ros2/common_utils.hpp"

namespace pifod_common_utils
{
    Eigen::Vector3f toEigenVec3(const float &x, const float &y, const float &z)
    {
        return Eigen::Vector3f(x, y, z);
    }

    Eigen::Vector3f toEigenVec3(const pcl::PointXYZ &p)
    {
        return Eigen::Vector3f(p.x, p.y, p.z);
    }
}
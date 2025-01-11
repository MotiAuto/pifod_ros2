#ifndef COMMON_UTILS_HPP_
#define COMMON_UTILS_HPP_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <pcl/point_types.h>

namespace pifod_ros2
{
    template<typename T>
    Eigen::Vector3d toEigenVec3(const T &x, const T &y, const T &z)
    {
        return Eigen::Vector3d(x, y, z);
    }

    Eigen::Vector3d toEigenVec3(const pcl::PointXYZ &p)
    {
        return Eigen::Vector3d(p.x, p.y, p.z);
    }
}

#endif
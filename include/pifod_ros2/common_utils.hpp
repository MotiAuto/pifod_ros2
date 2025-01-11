#ifndef COMMON_UTILS_HPP_
#define COMMON_UTILS_HPP_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <pcl/point_types.h>

Eigen::Vector3f toEigenVec3(const float &x, const float &y, const float &z);

Eigen::Vector3f toEigenVec3(const pcl::PointXYZ &p);

#endif
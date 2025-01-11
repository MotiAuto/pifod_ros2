#ifndef ICP_ODOM_UTILS_HPP_
#define ICP_ODOM_UTILS_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/filter_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "common_utils.hpp"

namespace pifod_ros2
{
    class ICPOdometer
    {
        public:
        /// @brief コンストラクタ
        /// @param max_iter ICPアルゴリズムの最大実行回数
        /// @param tolerance 収束判定
        ICPOdometer(const int max_iter=30, const double tolerance=1e-6);

        /// @brief ターゲット（初期）の点群をセットする
        /// @param cloud 点群
        void setTarget(const sensor_msgs::msg::PointCloud2::SharedPtr cloud);

        /// @brief ICPを実行する
        /// @param cloud 参照点群
        void compute(const sensor_msgs::msg::PointCloud2::SharedPtr cloud);

        /// @brief 結果の移動距離を取得する
        /// @return ３次元の移動距離
        Eigen::Vector3f getTranslation();

        /// @brief 結果の姿勢を取得する
        /// @return 四元数の姿勢
        Eigen::Quaternionf getPosture();

        private:
        pcl::PointCloud<pcl::PointXYZ>::Ptr target_;
        int max_iter_;
        double tolerance_;
        Eigen::Vector3f translation_;
        Eigen::Quaternionf rotation_;

        /// @brief KD-Treeによって最近傍探索しマッチングする
        /// @param source_cloud 参照元の点群
        /// @param target_cloud 探索する点群
        /// @return マッチングした点群を返す
        pcl::PointCloud<pcl::PointXYZ> matching(const pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud)
        {
            pcl::PointCloud<pcl::PointXYZ> matched;

            pcl::KdTreeFLANN<pcl::PointXYZ> kd_tree;
            kd_tree.setInputCloud(target_cloud);

            for(const auto &src_p : source_cloud->points)
            {
                std::vector<int> nearest_indices;
                std::vector<float> nearest_distances;

                if(kd_tree.nearestKSearch(src_p, 1, nearest_indices, nearest_distances) > 0)
                {
                    matched.push_back(target_cloud->at(nearest_indices[0]));
                }
            }

            return matched;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr transformPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const Eigen::Matrix3f &rotate, const Eigen::Vector3f &translate)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);

            for(const auto& p : cloud->points)
            {
                const Eigen::Vector3f eigen_p = pifod_common_utils::toEigenVec3(p);
                const Eigen::Vector3f rotated = rotate * eigen_p;
                const Eigen::Vector3f transformed = rotated + translate;
                
                transformed_cloud->push_back(pcl::PointXYZ(transformed.x(),transformed.y(),transformed.z()));
            }

            return transformed_cloud;
        }
    };
}

#endif
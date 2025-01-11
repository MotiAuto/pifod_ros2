#ifndef PIFOD_ROS2_HPP_
#define PIFOD_ROS2_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include <chrono>

#include "icp_odom_utils.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace pifod_ros2
{
    class PIFOD_ROS2 : public rclcpp::Node
    {
        public:
        explicit PIFOD_ROS2(const rclcpp::NodeOptions& option=rclcpp::NodeOptions());

        void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        void timer_callback();

        private:
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::shared_ptr<ICPOdometer> icp_;
        int icp_max_iter_;
        double icp_tolerance_;
        bool icp_setTargetFlag_;
        geometry_msgs::msg::PoseStamped icp_odom_;
        nav_msgs::msg::Path path_;
    };
}

#endif